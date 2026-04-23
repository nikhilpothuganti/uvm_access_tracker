#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/mutex.h>

#include "uvm_global.h"
#include "uvm_sampling_tracker.h"
#include "uvm_va_block.h"
#include "uvm_processors.h"

#define PROC_NAME "uvm_sampling_tracker"

static struct proc_dir_entry *tracker_proc;
static DEFINE_MUTEX(tracker_lock);

/* UVM fault access type names — must match UVM_FAULT_ACCESS_TYPE_* order */
static const char * const access_type_names[] = {
    "prefetch", "read", "write", "atomic_weak", "atomic_strong"
};

static void tracker_show_access_mask(struct seq_file *m, u32 access_mask)
{
    size_t i;
    bool first = true;

    if (access_mask == 0) {
        seq_puts(m, "none");
        return;
    }

    for (i = 0; i < ARRAY_SIZE(access_type_names); ++i) {
        if ((access_mask & (1U << i)) == 0)
            continue;

        if (!first)
            seq_putc(m, '|');

        seq_puts(m, access_type_names[i]);
        first = false;
    }
}

static void tracker_clear_range(uvm_va_range_tracker_t *range_tracker)
{
    if (range_tracker)
        bitmap_zero(range_tracker->bitmap, range_tracker->nr_pages);
}

/* =========================
   READ
   ========================= */

static int tracker_show(struct seq_file *m, void *v)
{
    uvm_va_space_t *va_space;
    unsigned long n_bitmap = 0;
    unsigned long n_rich = 0;
    unsigned long tracked_spaces = 0;
    unsigned long tracked_ranges = 0;

    mutex_lock(&tracker_lock);
    uvm_mutex_lock(&g_uvm_global.va_spaces.lock);

    list_for_each_entry(va_space, &g_uvm_global.va_spaces.list, list_node) {
        uvm_va_range_t *va_range;
        uvm_va_space_tracker_t *space_tracker = va_space->sampling_tracker;

        if (!space_tracker)
            continue;

        tracked_spaces++;
        uvm_va_space_down_read(va_space);

        uvm_for_each_va_range(va_range, va_space) {
            uvm_va_range_tracker_t *range_tracker = va_range->sampling_tracker;
            unsigned long page_index;
            u32 cur_epoch;

            if (!range_tracker)
                continue;

            tracked_ranges++;
            cur_epoch = READ_ONCE(space_tracker->epoch);

            for_each_set_bit(page_index, range_tracker->bitmap, range_tracker->nr_pages) {
                uvm_page_record_t *rec;

                n_bitmap++;
                rec = &range_tracker->records[uvm_sampling_tracker_record_index(range_tracker, page_index)];
                if (READ_ONCE(rec->epoch) == cur_epoch)
                    n_rich++;
            }
        }

        uvm_va_space_up_read(va_space);
    }

    seq_printf(m,
               "active_default=%d\nsample_rate_default=%u\ntracked_va_spaces=%lu\ntracked_ranges=%lu\n",
               uvm_sampling_tracker_default_active(),
               uvm_sampling_tracker_default_sample_rate(),
               tracked_spaces,
               tracked_ranges);
    seq_printf(m, "accessed=%lu (rich_metadata=%lu bitmap_only=%lu)\n",
               n_bitmap, n_rich, n_bitmap - n_rich);
    seq_puts(m, "---\n");

    list_for_each_entry(va_space, &g_uvm_global.va_spaces.list, list_node) {
        uvm_va_range_t *va_range;
        uvm_va_space_tracker_t *space_tracker = va_space->sampling_tracker;

        if (!space_tracker)
            continue;

        seq_printf(m,
                   "va_space=%px pid=%d epoch=%u active=%d sample_rate=%u gpu_id=%u\n",
                   va_space,
                   space_tracker->pid,
                   READ_ONCE(space_tracker->epoch),
                   READ_ONCE(space_tracker->active),
                   READ_ONCE(space_tracker->sample_rate),
                   READ_ONCE(space_tracker->gpu_id_val));

        uvm_va_space_down_read(va_space);

        uvm_for_each_va_range(va_range, va_space) {
            uvm_va_range_tracker_t *range_tracker = va_range->sampling_tracker;
            unsigned long page_index;
            u32 cur_epoch;

            if (!range_tracker)
                continue;

            cur_epoch = READ_ONCE(space_tracker->epoch);

            seq_printf(m,
                       "range=[0x%llx,0x%llx] pages=%lu record_pages=%lu stride=%lu\n",
                       va_range->node.start,
                       va_range->node.end,
                       range_tracker->nr_pages,
                       range_tracker->nr_record_pages,
                       range_tracker->stride);

            for_each_set_bit(page_index, range_tracker->bitmap, range_tracker->nr_pages) {
                uvm_page_record_t *rec;
                unsigned long long addr;

                rec = &range_tracker->records[uvm_sampling_tracker_record_index(range_tracker, page_index)];
                addr = va_range->node.start + ((unsigned long long)page_index << PAGE_SHIFT);

                if (READ_ONCE(rec->epoch) == cur_epoch) {
                    seq_printf(m,
                               "  page=%-8lu addr=0x%llx pid=%-6u ts_ns=%-20llu access=",
                               page_index,
                               addr,
                               READ_ONCE(rec->pid),
                               READ_ONCE(rec->timestamp_ns));
                    tracker_show_access_mask(m, READ_ONCE(rec->access_type_mask));
                    seq_putc(m, '\n');
                }
                else {
                    seq_printf(m, "  page=%-8lu addr=0x%llx\n", page_index, addr);
                }
            }
        }

        uvm_va_space_up_read(va_space);
    }

    uvm_mutex_unlock(&g_uvm_global.va_spaces.lock);
    mutex_unlock(&tracker_lock);
    return 0;
}

static int tracker_open(struct inode *inode, struct file *file)
{
    return single_open(file, tracker_show, NULL);
}

/* =========================
   WRITE
   ========================= */

static ssize_t tracker_write(struct file *file,
                             const char __user *buf,
                             size_t count,
                             loff_t *ppos)
{
    char kbuf[64];
    int val;
    uvm_va_space_t *va_space;

    if (count >= sizeof(kbuf))
        return -EINVAL;
    if (copy_from_user(kbuf, buf, count))
        return -EFAULT;

    kbuf[count] = '\0';

    mutex_lock(&tracker_lock);

    /* sample_rate <N> — 1=all blocks, N=1-in-N, 0=disable */
    if (strncmp(kbuf, "sample_rate", 11) == 0) {
        unsigned int uval;
        if (kstrtouint(kbuf + 12, 10, &uval) == 0) {
            uvm_sampling_tracker_set_default_sample_rate(uval);

            uvm_mutex_lock(&g_uvm_global.va_spaces.lock);
            list_for_each_entry(va_space, &g_uvm_global.va_spaces.list, list_node) {
                if (va_space->sampling_tracker)
                    WRITE_ONCE(va_space->sampling_tracker->sample_rate, uval);
            }
            uvm_mutex_unlock(&g_uvm_global.va_spaces.lock);

            pr_info("[tracker] sample_rate=%u\n", uval);
        }
    }
    /* enable <0|1> */
    else if (strncmp(kbuf, "enable", 6) == 0) {
        if (kstrtoint(kbuf + 7, 10, &val) == 0) {
            uvm_sampling_tracker_set_default_active(!!val);

            uvm_mutex_lock(&g_uvm_global.va_spaces.lock);
            list_for_each_entry(va_space, &g_uvm_global.va_spaces.list, list_node) {
                if (va_space->sampling_tracker)
                    WRITE_ONCE(va_space->sampling_tracker->active, !!val);
            }
            uvm_mutex_unlock(&g_uvm_global.va_spaces.lock);

            pr_info("[tracker] active=%d\n", !!val);
        }
    }
    /* clear — reset bitmaps and bump epochs without revoking GPU PTEs */
    else if (strncmp(kbuf, "clear", 5) == 0) {
        uvm_mutex_lock(&g_uvm_global.va_spaces.lock);
        list_for_each_entry(va_space, &g_uvm_global.va_spaces.list, list_node) {
            uvm_va_range_t *va_range;
            uvm_va_space_tracker_t *space_tracker = va_space->sampling_tracker;

            if (!space_tracker)
                continue;

            WRITE_ONCE(space_tracker->epoch, READ_ONCE(space_tracker->epoch) + 1);
            uvm_va_space_down_read(va_space);
            uvm_for_each_va_range(va_range, va_space)
                tracker_clear_range(va_range->sampling_tracker);
            uvm_va_space_up_read(va_space);
        }
        uvm_mutex_unlock(&g_uvm_global.va_spaces.lock);

        pr_info("[tracker] cleared all tracked ranges\n");
    }
    /*
     * revoke — start a new tracking epoch:
     *   1. bump epoch counters       (O(1) — invalidates all records)
     *   2. clear all range bitmaps
     *   3. unmap GPU PTEs            (subsequent accesses fault)
     */
    else if (strncmp(kbuf, "revoke", 6) == 0) {
        uvm_mutex_lock(&g_uvm_global.va_spaces.lock);
        list_for_each_entry(va_space, &g_uvm_global.va_spaces.list, list_node) {
            uvm_va_range_t *va_range;
            uvm_va_space_tracker_t *space_tracker = va_space->sampling_tracker;
            NV_STATUS status;
            uvm_gpu_id_t gpu_id;

            if (!space_tracker)
                continue;

            WRITE_ONCE(space_tracker->epoch, READ_ONCE(space_tracker->epoch) + 1);
            uvm_va_space_down_read(va_space);
            uvm_for_each_va_range(va_range, va_space)
                tracker_clear_range(va_range->sampling_tracker);
            uvm_va_space_up_read(va_space);

            if (READ_ONCE(space_tracker->gpu_id_val) == UVM_ID_INVALID.val)
                continue;

            gpu_id = uvm_gpu_id_from_value(READ_ONCE(space_tracker->gpu_id_val));
            status = uvm_tracking_revoke_epoch(va_space, gpu_id);
            if (status != NV_OK)
                pr_err("[tracker] revoke epoch failed for %px: 0x%x\n", va_space, status);
        }
        uvm_mutex_unlock(&g_uvm_global.va_spaces.lock);

        pr_info("[tracker] revoke completed across tracked VA spaces\n");
    }

    mutex_unlock(&tracker_lock);
    return count;
}

/* =========================
   PROC OPS
   ========================= */

static const struct proc_ops tracker_ops = {
    .proc_open    = tracker_open,
    .proc_read    = seq_read,
    .proc_lseek   = seq_lseek,
    .proc_release = single_release,
    .proc_write   = tracker_write,
};

/* =========================
   INIT / DESTROY
   ========================= */

int uvm_sampling_procfs_init(void)
{
    tracker_proc = proc_create(PROC_NAME, 0666, NULL, &tracker_ops);
    if (!tracker_proc)
        return -ENOMEM;
    pr_info("[tracker] /proc/%s created\n", PROC_NAME);
    return 0;
}

void uvm_sampling_procfs_destroy(void)
{
    if (tracker_proc)
        proc_remove(tracker_proc);
}
