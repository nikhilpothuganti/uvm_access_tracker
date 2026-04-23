#include <linux/module.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>
#include <linux/sched.h>
#include "uvm_sampling_tracker.h"

/* ktime_get_ns() calls ktime_get() which is EXPORT_SYMBOL_GPL; use the
 * non-GPL ktime_get_raw_ts64 path so proprietary modules can load. */
static inline u64 uvm_ktime_get_ns(void)
{
    struct timespec64 ts;
    ktime_get_raw_ts64(&ts);
    return timespec64_to_ns(&ts);
}

gpu_sampling_tracker_t *g_tracker = NULL;

void uvm_sampling_tracker_set_context(void *va_space, u32 gpu_id_val)
{
    if (!g_tracker)
        return;
    WRITE_ONCE(g_tracker->va_space,   va_space);
    WRITE_ONCE(g_tracker->gpu_id_val, gpu_id_val);
}

void uvm_sampling_tracker_record(unsigned long addr,
                                  unsigned long base_va,
                                  unsigned long region_end,
                                  u8 access_type)
{
    unsigned long      page_index;
    uvm_page_record_t *rec;
    u32                cur_epoch, old_epoch;

    if (!g_tracker || !g_tracker->active)
        return;
    if (addr < base_va || addr >= region_end)
        return;

    page_index = (addr - base_va) >> PAGE_SHIFT;

    /* ── Tier 1: fast bitmap (all 2 M pages) ─────────────────────────── */
    if (page_index >= g_tracker->num_pages)
        return;

    set_bit(page_index, g_tracker->bitmap);

    /* ── Tier 2: rich metadata (first 512 MB = 128 K pages) ──────────── */
    if (page_index >= UVM_TRACKER_RECORD_PAGES)
        return;

    cur_epoch = READ_ONCE(g_tracker->epoch);
    rec       = &g_tracker->records[page_index];
    old_epoch = READ_ONCE(rec->epoch);

    /*
     * Page already recorded this epoch: check if the incoming access is
     * more intrusive than what's stored and upgrade if so.
     *
     * UVM intrusion order: PREFETCH(0) < READ(1) < WRITE(2)
     *                       < ATOMIC_WEAK(3) < ATOMIC_STRONG(4)
     *
     * Example: page first faulted as READ (PTE restored RO), then the GPU
     * writes it — a second fault fires with access_type=WRITE.  The cmpxchg
     * below would fail (epoch already set), but without this upgrade the
     * record would permanently show "read" even though the page was written.
     *
     * Single-byte WRITE_ONCE on x86 is naturally atomic — no torn write.
     * A benign race between two upgrading CPUs leaves one valid value.
     */
    if (old_epoch == cur_epoch) {
        if (access_type > READ_ONCE(rec->access_type))
            WRITE_ONCE(rec->access_type, access_type);
        return;
    }

    /*
     * First access this epoch: atomically claim the slot.
     * Only one CPU succeeds; all others fall into the upgrade path above
     * on their next call once rec->epoch == cur_epoch.
     *
     * Minor benign race: a concurrent procfs reader may observe
     * rec->epoch == cur_epoch before the fields below are written.
     * Worst case the reader sees the previous epoch's pid/timestamp
     * for one read cycle — acceptable for a sampling tracker.
     */
    if (cmpxchg(&rec->epoch, old_epoch, cur_epoch) != old_epoch)
        return;

    WRITE_ONCE(rec->timestamp_ns, uvm_ktime_get_ns());
    WRITE_ONCE(rec->pid,          (u32)task_tgid_nr(current));
    WRITE_ONCE(rec->access_type,  access_type);
}

void uvm_sampling_tracker_init(void)
{
    size_t bitmap_bytes = BITS_TO_LONGS(UVM_TRACKER_BITMAP_PAGES) * sizeof(long);
    size_t record_bytes = UVM_TRACKER_RECORD_PAGES * sizeof(uvm_page_record_t);

    g_tracker = kzalloc(sizeof(*g_tracker), GFP_KERNEL);
    if (!g_tracker) {
        pr_err("[tracker] failed to allocate tracker struct\n");
        return;
    }

    g_tracker->bitmap = kzalloc(bitmap_bytes, GFP_KERNEL);
    if (!g_tracker->bitmap) {
        pr_err("[tracker] failed to allocate bitmap (%zu KB)\n",
               bitmap_bytes / 1024);
        goto err_free_tracker;
    }

    /*
     * records is ~3 MB — use vzalloc so it doesn't need to be physically
     * contiguous (kzalloc limit is ~4 MB but fragmentation can make it fail).
     */
    g_tracker->records = vzalloc(record_bytes);
    if (!g_tracker->records) {
        pr_err("[tracker] failed to allocate records (%zu KB)\n",
               record_bytes / 1024);
        goto err_free_bitmap;
    }

    g_tracker->num_pages   = UVM_TRACKER_BITMAP_PAGES;
    g_tracker->epoch       = 1;   /* 0 is the "never written" sentinel */
    g_tracker->active      = 1;
    g_tracker->sample_rate = 1;   /* default: track every block */

    uvm_sampling_procfs_init();
    pr_info("[tracker] initialized — bitmap %zu KB, records %zu KB\n",
            bitmap_bytes / 1024, record_bytes / 1024);
    return;

err_free_bitmap:
    kfree(g_tracker->bitmap);
err_free_tracker:
    kfree(g_tracker);
    g_tracker = NULL;
}

void uvm_sampling_tracker_destroy(void)
{
    uvm_sampling_procfs_destroy();
    if (!g_tracker)
        return;
    vfree(g_tracker->records);
    kfree(g_tracker->bitmap);
    kfree(g_tracker);
    g_tracker = NULL;
}
