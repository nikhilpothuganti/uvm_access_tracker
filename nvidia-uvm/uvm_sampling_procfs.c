#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/mutex.h>

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

/* =========================
   READ
   ========================= */

static int tracker_show(struct seq_file *m, void *v)
{
    unsigned long i;
    u32 cur_epoch;
    unsigned long n_bitmap = 0, n_rich = 0;

    mutex_lock(&tracker_lock);

    if (!g_tracker || !g_tracker->bitmap) {
        seq_puts(m, "tracker not initialized\n");
        mutex_unlock(&tracker_lock);
        return 0;
    }

    cur_epoch = READ_ONCE(g_tracker->epoch);

    seq_printf(m, "active=%d\nepoch=%u\npages=%lu\nsample_rate=%u\n",
               g_tracker->active, cur_epoch, g_tracker->num_pages,
               READ_ONCE(g_tracker->sample_rate));

    /*
     * Count before printing so the header has totals.
     * Single pass: both bitmap-only and record-backed pages.
     */
    for_each_set_bit(i, g_tracker->bitmap, g_tracker->num_pages) {
        n_bitmap++;
        if (i < UVM_TRACKER_RECORD_PAGES &&
            READ_ONCE(g_tracker->records[i].epoch) == cur_epoch)
            n_rich++;
    }

    seq_printf(m, "accessed=%lu (rich_metadata=%lu bitmap_only=%lu)\n",
               n_bitmap, n_rich, n_bitmap - n_rich);
    seq_puts(m, "---\n");

    for_each_set_bit(i, g_tracker->bitmap, g_tracker->num_pages) {
        if (i < UVM_TRACKER_RECORD_PAGES) {
            uvm_page_record_t *rec = &g_tracker->records[i];
            u32 epoch = READ_ONCE(rec->epoch);

            if (epoch == cur_epoch) {
                u8  at  = READ_ONCE(rec->access_type);
                u32 pid = READ_ONCE(rec->pid);
                u64 ts  = READ_ONCE(rec->timestamp_ns);
                const char *aname = (at < ARRAY_SIZE(access_type_names))
                                    ? access_type_names[at] : "unknown";

                seq_printf(m, "page=%-8lu pid=%-6u ts_ns=%-20llu access=%s\n",
                           i, pid, ts, aname);
                continue;
            }
        }

        /* Bitmap-only entry (beyond 512 MB or record from prior epoch) */
        seq_printf(m, "page=%-8lu\n", i);
    }

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
    int  val;

    if (count >= sizeof(kbuf))
        return -EINVAL;
    if (copy_from_user(kbuf, buf, count))
        return -EFAULT;

    kbuf[count] = '\0';

    mutex_lock(&tracker_lock);

    if (!g_tracker) {
        mutex_unlock(&tracker_lock);
        return -ENODEV;
    }

    /* sample_rate <N> — 1=all blocks, N=1-in-N, 0=disable */
    if (strncmp(kbuf, "sample_rate", 11) == 0) {
        unsigned int uval;
        if (kstrtouint(kbuf + 12, 10, &uval) == 0) {
            WRITE_ONCE(g_tracker->sample_rate, uval);
            pr_info("[tracker] sample_rate=%u\n", uval);
        }
    }
    /* enable <0|1> */
    else if (strncmp(kbuf, "enable", 6) == 0) {
        if (kstrtoint(kbuf + 7, 10, &val) == 0) {
            g_tracker->active = !!val;
            pr_info("[tracker] active=%d\n", g_tracker->active);
        }
    }
    /* clear — reset bitmap and bump epoch without revoking GPU PTEs */
    else if (strncmp(kbuf, "clear", 5) == 0) {
        g_tracker->epoch++;
        bitmap_zero(g_tracker->bitmap, g_tracker->num_pages);
        pr_info("[tracker] cleared (epoch now %u)\n", g_tracker->epoch);
    }
    /*
     * revoke — start a new tracking epoch:
     *   1. bump epoch counter         (O(1) — invalidates all records)
     *   2. clear bitmap               (O(N/64) — 256 KB memset)
     *   3. unmap GPU PTEs             (any subsequent access faults)
     *
     * The records array is NOT cleared; old entries become invisible
     * the moment epoch increments.
     */
    else if (strncmp(kbuf, "revoke", 6) == 0) {
        if (!g_tracker->va_space) {
            pr_err("[tracker] revoke: no va_space registered yet — "
                   "run a GPU workload first\n");
        } else {
            uvm_gpu_id_t   gpu_id   = { .val = g_tracker->gpu_id_val };
            uvm_va_space_t *va_space = (uvm_va_space_t *)g_tracker->va_space;
            NV_STATUS status;

            g_tracker->epoch++;
            bitmap_zero(g_tracker->bitmap, g_tracker->num_pages);

            status = uvm_tracking_revoke_epoch(va_space, gpu_id);
            if (status != NV_OK)
                pr_err("[tracker] revoke epoch failed: 0x%x\n", status);
            else
                pr_info("[tracker] revoke done (epoch now %u)\n",
                        g_tracker->epoch);
        }
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
