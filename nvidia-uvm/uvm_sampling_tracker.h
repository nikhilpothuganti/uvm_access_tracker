#ifndef UVM_SAMPLING_TRACKER_H
#define UVM_SAMPLING_TRACKER_H

#include <linux/bitmap.h>
#include <linux/types.h>

#define UVM_TRACKER_BITMAP_PAGES  (1UL << 21)  /* 2M pages — 256 KB bitmap  */
#define UVM_TRACKER_RECORD_PAGES  (1UL << 17)  /* 128K pages — 512 MB range */

/*
 * Per-page first-access record.
 *
 * Written at most once per epoch under a cmpxchg on the epoch field
 * (first-write-wins).  Readers check epoch == g_tracker->epoch before
 * trusting the other fields.
 *
 * Layout: 20 bytes, padded to 24 by the compiler for u64 alignment in arrays.
 */
typedef struct {
    u64 timestamp_ns;   /* ktime_get_ns() of first access this epoch */
    u32 pid;            /* tgid of the first faulting process         */
    u32 epoch;          /* epoch counter when this record was written */
    u8  access_type;    /* UVM_FAULT_ACCESS_TYPE_* value (cast to u8) */
    u8  _pad[3];
} uvm_page_record_t;

typedef struct {
    /*
     * Tier 1 — fast bitmap.
     * set_bit() on every fault, covers all 2 M pages.
     * Cleared on revoke (256 KB memset).
     */
    unsigned long      *bitmap;
    unsigned long       num_pages;      /* = UVM_TRACKER_BITMAP_PAGES */

    /*
     * Tier 2 — rich per-page first-access metadata.
     * Only written for the first access per page per epoch.
     * Never memset — epoch stamp makes stale entries invisible.
     * Covers up to UVM_TRACKER_RECORD_PAGES (128K pages = 512 MB).
     */
    uvm_page_record_t  *records;

    /*
     * Epoch counter.  Incremented on each revoke (O(1)).
     * Starts at 1 so that zero-initialised records are always "stale".
     */
    u32                 epoch;

    /* VA space and GPU context for the revoke sweep */
    void               *va_space;       /* uvm_va_space_t *, opaque   */
    u32                 gpu_id_val;

    int                 active;
    pid_t               pid;

    /*
     * Block sampling rate.  Only 1-in-sample_rate VA blocks suppress prefetch
     * and get page-level fault tracking.  The sampled block is chosen
     * deterministically by (block_start >> 21) % sample_rate == 0.
     *
     *   sample_rate == 1  →  every block is tracked  (default)
     *   sample_rate == 256 →  1 in 256 blocks tracked
     *   sample_rate == 0  →  no blocks tracked (effectively disables tracker)
     */
    u32                 sample_rate;
} gpu_sampling_tracker_t;

extern gpu_sampling_tracker_t *g_tracker;

/*
 * Returns true if the VA block whose allocation starts at block_start should
 * have prefetch suppressed and its faults recorded.  Safe to call from any
 * context; reads g_tracker fields with READ_ONCE.
 *
 * UVM_VA_BLOCK_BITS (21) is used directly to avoid a circular header
 * dependency on uvm_va_block_types.h.
 */
static inline bool uvm_sampling_tracker_block_sampled(unsigned long block_start)
{
    u32 rate;

    if (!g_tracker || !READ_ONCE(g_tracker->active))
        return false;

    rate = READ_ONCE(g_tracker->sample_rate);
    if (rate == 0)
        return false;
    if (rate == 1)
        return true;

    return ((block_start >> 21) % rate) == 0;
}

/*
 * Record one GPU fault.
 *   addr       — faulting virtual address
 *   base_va    — start of the tracked allocation (va_range->node.start)
 *   region_end — exclusive end of the allocation (va_range->node.end + 1)
 *   access_type — UVM_FAULT_ACCESS_TYPE_* cast to u8 by caller
 */
void uvm_sampling_tracker_record(unsigned long addr,
                                  unsigned long base_va,
                                  unsigned long region_end,
                                  u8 access_type);

/* Register the VA space and GPU targeted by revoke epochs. */
void uvm_sampling_tracker_set_context(void *va_space, u32 gpu_id_val);

void uvm_sampling_tracker_init(void);
void uvm_sampling_tracker_destroy(void);
int  uvm_sampling_procfs_init(void);
void uvm_sampling_procfs_destroy(void);

#endif /* UVM_SAMPLING_TRACKER_H */
