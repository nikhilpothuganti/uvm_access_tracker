#include <linux/module.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>
#include <linux/sched.h>

#include "uvm_sampling_tracker.h"
#include "uvm_kvmalloc.h"
#include "uvm_va_range.h"
#include "uvm_va_space.h"

/* ktime_get_ns() calls ktime_get() which is EXPORT_SYMBOL_GPL; use the
 * non-GPL ktime_get_raw_ts64 path so proprietary modules can load. */
static inline u64 uvm_ktime_get_ns(void)
{
    struct timespec64 ts;
    ktime_get_raw_ts64(&ts);
    return timespec64_to_ns(&ts);
}

typedef struct
{
    int active;
    u32 sample_rate;
} uvm_sampling_tracker_defaults_t;

static uvm_sampling_tracker_defaults_t g_sampling_tracker_defaults = {
    .active = 1,
    .sample_rate = 1,
};

static void uvm_sampling_tracker_update_access_mask(uvm_page_record_t *rec, u32 access_type_mask)
{
    u32 old_mask;
    u32 new_mask;

    if (!access_type_mask)
        return;

    do {
        old_mask = READ_ONCE(rec->access_type_mask);
        new_mask = old_mask | access_type_mask;
        if (new_mask == old_mask)
            return;
    } while (cmpxchg(&rec->access_type_mask, old_mask, new_mask) != old_mask);
}

static void uvm_sampling_tracker_free_range_tracker(uvm_va_range_tracker_t *range_tracker)
{
    if (!range_tracker)
        return;

    uvm_kvfree(range_tracker->records);
    uvm_kvfree(range_tracker->bitmap);
    uvm_kvfree(range_tracker);
}

static uvm_va_range_tracker_t *uvm_sampling_tracker_alloc_range_tracker(uvm_va_range_t *va_range)
{
    uvm_va_range_tracker_t *range_tracker;
    unsigned long nr_pages = uvm_va_range_size(va_range) >> PAGE_SHIFT;
    unsigned long nr_record_pages = min(nr_pages, (unsigned long)UVM_TRACKER_RECORD_PAGES);
    size_t bitmap_bytes;
    size_t record_bytes;

    if (nr_pages == 0)
        return NULL;

    range_tracker = uvm_kvmalloc_zero(sizeof(*range_tracker));
    if (!range_tracker)
        return NULL;

    bitmap_bytes = BITS_TO_LONGS(nr_pages) * sizeof(range_tracker->bitmap[0]);
    record_bytes = nr_record_pages * sizeof(range_tracker->records[0]);

    range_tracker->bitmap = uvm_kvmalloc_zero(bitmap_bytes);
    if (!range_tracker->bitmap)
        goto error;

    range_tracker->records = uvm_kvmalloc_zero(record_bytes);
    if (!range_tracker->records)
        goto error;

    range_tracker->va_range = va_range;
    range_tracker->nr_pages = nr_pages;
    range_tracker->nr_record_pages = nr_record_pages;
    range_tracker->stride = DIV_ROUND_UP(nr_pages, nr_record_pages);
    if (range_tracker->stride == 0)
        range_tracker->stride = 1;

    return range_tracker;

error:
    uvm_sampling_tracker_free_range_tracker(range_tracker);
    return NULL;
}

int uvm_sampling_tracker_default_active(void)
{
    return READ_ONCE(g_sampling_tracker_defaults.active);
}

u32 uvm_sampling_tracker_default_sample_rate(void)
{
    return READ_ONCE(g_sampling_tracker_defaults.sample_rate);
}

void uvm_sampling_tracker_set_default_active(int active)
{
    WRITE_ONCE(g_sampling_tracker_defaults.active, !!active);
}

void uvm_sampling_tracker_set_default_sample_rate(u32 sample_rate)
{
    WRITE_ONCE(g_sampling_tracker_defaults.sample_rate, sample_rate);
}

void uvm_sampling_tracker_init_va_space(uvm_va_space_t *va_space)
{
    uvm_va_space_tracker_t *space_tracker;

    if (!va_space || va_space->sampling_tracker)
        return;

    space_tracker = uvm_kvmalloc_zero(sizeof(*space_tracker));
    if (!space_tracker) {
        pr_err("[tracker] failed to allocate VA space tracker for %px\n", va_space);
        return;
    }

    space_tracker->va_space = va_space;
    space_tracker->epoch = 1;
    space_tracker->pid = task_tgid_nr(current);
    space_tracker->active = uvm_sampling_tracker_default_active();
    space_tracker->sample_rate = uvm_sampling_tracker_default_sample_rate();
    space_tracker->gpu_id_val = UVM_ID_INVALID.val;

    va_space->sampling_tracker = space_tracker;
}

void uvm_sampling_tracker_destroy_va_space(uvm_va_space_t *va_space)
{
    if (!va_space)
        return;

    uvm_kvfree(va_space->sampling_tracker);
    va_space->sampling_tracker = NULL;
}

void uvm_sampling_tracker_init_va_range(uvm_va_range_t *va_range)
{
    if (!va_range || va_range->sampling_tracker || va_range->type != UVM_VA_RANGE_TYPE_MANAGED)
        return;

    va_range->sampling_tracker = uvm_sampling_tracker_alloc_range_tracker(va_range);
    if (!va_range->sampling_tracker) {
        pr_err("[tracker] failed to allocate range tracker for [0x%llx, 0x%llx]\n",
               va_range->node.start,
               va_range->node.end);
    }
}

void uvm_sampling_tracker_destroy_va_range(uvm_va_range_t *va_range)
{
    if (!va_range)
        return;

    uvm_sampling_tracker_free_range_tracker(va_range->sampling_tracker);
    va_range->sampling_tracker = NULL;
}

void uvm_sampling_tracker_reset_va_range(uvm_va_range_t *va_range)
{
    uvm_sampling_tracker_destroy_va_range(va_range);
    uvm_sampling_tracker_init_va_range(va_range);
}

void uvm_sampling_tracker_record(uvm_va_space_tracker_t *space_tracker,
                                 uvm_va_range_tracker_t *range_tracker,
                                 unsigned long addr,
                                 unsigned long base_va,
                                 u8 access_type)
{
    unsigned long page_index;
    unsigned long record_index;
    uvm_page_record_t *rec;
    u32 cur_epoch;
    u32 old_epoch;
    u32 access_type_mask;

    if (!space_tracker || !range_tracker || !READ_ONCE(space_tracker->active))
        return;
    if (addr < base_va)
        return;

    page_index = (addr - base_va) >> PAGE_SHIFT;
    if (page_index >= range_tracker->nr_pages)
        return;

    set_bit(page_index, range_tracker->bitmap);

    record_index = uvm_sampling_tracker_record_index(range_tracker, page_index);
    rec = &range_tracker->records[record_index];
    cur_epoch = READ_ONCE(space_tracker->epoch);
    old_epoch = READ_ONCE(rec->epoch);
    access_type_mask = uvm_sampling_tracker_access_type_mask(access_type);

    if (old_epoch == cur_epoch) {
        uvm_sampling_tracker_update_access_mask(rec, access_type_mask);
        return;
    }

    if (cmpxchg(&rec->epoch, old_epoch, cur_epoch) != old_epoch) {
        if (READ_ONCE(rec->epoch) == cur_epoch)
            uvm_sampling_tracker_update_access_mask(rec, access_type_mask);
        return;
    }

    WRITE_ONCE(rec->timestamp_ns, uvm_ktime_get_ns());
    WRITE_ONCE(rec->pid, (u32)task_tgid_nr(current));
    WRITE_ONCE(rec->access_type_mask, access_type_mask);
}

void uvm_sampling_tracker_init(void)
{
    uvm_sampling_tracker_set_default_active(1);
    uvm_sampling_tracker_set_default_sample_rate(1);

    if (uvm_sampling_procfs_init() != 0)
        pr_err("[tracker] failed to create procfs entry\n");
}

void uvm_sampling_tracker_destroy(void)
{
    uvm_sampling_procfs_destroy();
}
