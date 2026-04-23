#ifndef UVM_SAMPLING_TRACKER_H
#define UVM_SAMPLING_TRACKER_H

#include "uvm_forward_decl.h"

#include <linux/bitmap.h>
#include <linux/types.h>

#define UVM_TRACKER_RECORD_PAGES  (1UL << 17)  /* 128K sampled metadata slots */

/*
 * Per-record first-access metadata.
 *
 * Each record represents one stride-sampled region within a VA range. The
 * record stores the first access metadata seen in the current epoch and a
 * bitmask of all access types observed for that region.
 */
typedef struct {
    u64 timestamp_ns;      /* ktime_get_ns() of first access this epoch */
    u32 pid;               /* tgid of the first faulting process         */
    u32 epoch;             /* epoch counter when this record was written */
    u32 access_type_mask;  /* bitmask of UVM_FAULT_ACCESS_TYPE_* bits    */
} uvm_page_record_t;

struct uvm_va_space_tracker_struct
{
    uvm_va_space_t *va_space;
    u32 epoch;
    pid_t pid;
    int active;
    u32 sample_rate;
    u32 gpu_id_val;
};

struct uvm_va_range_tracker_struct
{
    uvm_va_range_t     *va_range;
    unsigned long      *bitmap;
    unsigned long       nr_pages;
    uvm_page_record_t  *records;
    unsigned long       nr_record_pages;
    unsigned long       stride;
};

static inline u32 uvm_sampling_tracker_access_type_mask(u8 access_type)
{
    if (access_type >= 32)
        return 0;

    return 1U << access_type;
}

static inline unsigned long uvm_sampling_tracker_record_index(const uvm_va_range_tracker_t *range_tracker,
                                                              unsigned long page_index)
{
    unsigned long record_index;

    if (!range_tracker || range_tracker->nr_record_pages == 0)
        return 0;

    record_index = page_index / range_tracker->stride;
    if (record_index >= range_tracker->nr_record_pages)
        record_index = range_tracker->nr_record_pages - 1;

    return record_index;
}

/*
 * Returns true if the VA block whose allocation starts at block_start should
 * have prefetch suppressed and its faults recorded. Safe to call from any
 * context; reads the per-VA-space state locklessly.
 *
 * UVM_VA_BLOCK_BITS (21) is used directly to avoid a circular header
 * dependency on uvm_va_block_types.h.
 */
static inline bool uvm_sampling_tracker_block_sampled(uvm_va_space_tracker_t *space_tracker,
                                                      uvm_va_range_tracker_t *range_tracker,
                                                      unsigned long block_start)
{
    u32 rate;

    if (!space_tracker || !range_tracker || !READ_ONCE(space_tracker->active))
        return false;

    rate = READ_ONCE(space_tracker->sample_rate);
    if (rate == 0)
        return false;
    if (rate == 1)
        return true;

    return ((block_start >> 21) % rate) == 0;
}

static inline void uvm_sampling_tracker_set_context(uvm_va_space_tracker_t *space_tracker, u32 gpu_id_val)
{
    if (space_tracker)
        WRITE_ONCE(space_tracker->gpu_id_val, gpu_id_val);
}

int uvm_sampling_tracker_default_active(void);
u32 uvm_sampling_tracker_default_sample_rate(void);
void uvm_sampling_tracker_set_default_active(int active);
void uvm_sampling_tracker_set_default_sample_rate(u32 sample_rate);

void uvm_sampling_tracker_init_va_space(uvm_va_space_t *va_space);
void uvm_sampling_tracker_destroy_va_space(uvm_va_space_t *va_space);
void uvm_sampling_tracker_init_va_range(uvm_va_range_t *va_range);
void uvm_sampling_tracker_destroy_va_range(uvm_va_range_t *va_range);
void uvm_sampling_tracker_reset_va_range(uvm_va_range_t *va_range);

void uvm_sampling_tracker_record(uvm_va_space_tracker_t *space_tracker,
                                 uvm_va_range_tracker_t *range_tracker,
                                 unsigned long addr,
                                 unsigned long base_va,
                                 u8 access_type);

void uvm_sampling_tracker_init(void);
void uvm_sampling_tracker_destroy(void);
int  uvm_sampling_procfs_init(void);
void uvm_sampling_procfs_destroy(void);

#endif /* UVM_SAMPLING_TRACKER_H */
