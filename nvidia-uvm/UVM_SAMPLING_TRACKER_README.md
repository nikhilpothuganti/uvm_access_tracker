# UVM Access Sampling Tracker — End-to-End Explanation

The UVM sampling tracker is a lightweight, kernel-level subsystem built into the
NVIDIA UVM driver that records **which GPU-managed memory pages were accessed,
when, by whom, and how** — without requiring any changes to user applications.
It hooks into the existing GPU page-fault pipeline and exposes its data through
a `/proc` file.

---

## Table of Contents

1. [Key Data Structures](#1-key-data-structures)
2. [Lifecycle — Module Load to Unload](#2-lifecycle--module-load-to-unload)
3. [Per-VA-Space Initialization](#3-per-va-space-initialization)
4. [Per-VA-Range Initialization and Memory Layout](#4-per-va-range-initialization-and-memory-layout)
5. [Fault-Time Recording](#5-fault-time-recording)
6. [Prefetch Suppression](#6-prefetch-suppression)
7. [Lock-Free Record Update](#7-lock-free-record-update)
8. [The procfs Control Interface](#8-the-procfs-control-interface)
9. [The `revoke` Command and Epoch Mechanism](#9-the-revoke-command-and-epoch-mechanism)
10. [Teardown](#10-teardown)
11. [Data Flow Diagram](#11-data-flow-diagram)

---

## 1. Key Data Structures

### `uvm_page_record_t` — one sampled metadata slot

```c
// uvm_sampling_tracker.h
typedef struct {
    u64 timestamp_ns;      // ktime of first access this epoch
    u32 pid;               // tgid of the faulting process
    u32 epoch;             // epoch when this record was written
    u32 access_type_mask;  // bitmask: prefetch|read|write|atomic_weak|atomic_strong
} uvm_page_record_t;
```

Each record covers a *stride*-sized stripe of pages (explained in §4). It stores
only the **first** access within the current epoch; subsequent accesses from the
same epoch only OR their access type into `access_type_mask`.

### `uvm_va_range_tracker_t` — per VA range state

```c
// uvm_sampling_tracker.h
struct uvm_va_range_tracker_struct {
    uvm_va_range_t     *va_range;
    unsigned long      *bitmap;          // one bit per page — set on first access
    unsigned long       nr_pages;        // total pages in this range
    uvm_page_record_t  *records;         // capped at UVM_TRACKER_RECORD_PAGES (128K)
    unsigned long       nr_record_pages; // actual number of record slots allocated
    unsigned long       stride;          // pages per record slot
};
```

### `uvm_va_space_tracker_t` — per VA space (per process) control

```c
// uvm_sampling_tracker.h
struct uvm_va_space_tracker_struct {
    uvm_va_space_t *va_space;
    u32  epoch;         // incremented on each "clear" or "revoke"
    pid_t pid;          // tgid of the process that opened /dev/nvidia-uvm
    int  active;        // 0 = tracker disabled for this VA space
    u32  sample_rate;   // 1 = track all blocks, N = track 1-in-N blocks, 0 = off
    u32  gpu_id_val;    // last GPU that serviced a fault here
};
```

---

## 2. Lifecycle — Module Load to Unload

```
insmod nvidia-uvm.ko
    └─► uvm_sampling_tracker_init()           [uvm.c:1069]
            ├── set global defaults: active=1, sample_rate=1
            └── uvm_sampling_procfs_init()
                    └── proc_create("uvm_sampling_tracker", 0666, ...)
                            → /proc/uvm_sampling_tracker is now live

... (driver in use) ...

rmmod nvidia-uvm.ko
    └─► uvm_sampling_tracker_destroy()        [uvm.c:1120]
            └── uvm_sampling_procfs_destroy()
                    └── proc_remove(tracker_proc)
```

```c
// uvm_sampling_tracker.c
void uvm_sampling_tracker_init(void)
{
    uvm_sampling_tracker_set_default_active(1);
    uvm_sampling_tracker_set_default_sample_rate(1);

    if (uvm_sampling_procfs_init() != 0)
        pr_err("[tracker] failed to create procfs entry\n");
}
```

The global defaults live in a single static struct and are accessed with
`READ_ONCE` / `WRITE_ONCE` to avoid torn reads across CPUs:

```c
static uvm_sampling_tracker_defaults_t g_sampling_tracker_defaults = {
    .active = 1,
    .sample_rate = 1,
};
```

---

## 3. Per-VA-Space Initialization

When a process opens `/dev/nvidia-uvm` (i.e., calls `cuInit()` or equivalent),
the UVM driver creates a `uvm_va_space_t` and immediately initializes a tracker
for it:

```c
// uvm_va_space.c:192
uvm_sampling_tracker_init_va_space(va_space);
```

```c
// uvm_sampling_tracker.c
void uvm_sampling_tracker_init_va_space(uvm_va_space_t *va_space)
{
    uvm_va_space_tracker_t *space_tracker;

    if (!va_space || va_space->sampling_tracker)
        return;

    space_tracker = uvm_kvmalloc_zero(sizeof(*space_tracker));
    ...
    space_tracker->va_space    = va_space;
    space_tracker->epoch       = 1;           // epoch 0 is the "stale" sentinel
    space_tracker->pid         = task_tgid_nr(current);
    space_tracker->active      = uvm_sampling_tracker_default_active();
    space_tracker->sample_rate = uvm_sampling_tracker_default_sample_rate();
    space_tracker->gpu_id_val  = UVM_ID_INVALID.val;

    va_space->sampling_tracker = space_tracker;
}
```

The epoch starts at 1 because every `uvm_page_record_t` is zero-initialized,
meaning `rec->epoch == 0`. Any record whose epoch is 0 is automatically
considered stale relative to the live epoch (≥ 1).

---

## 4. Per-VA-Range Initialization and Memory Layout

When `cudaMalloc` (or any UVM managed allocation) creates a `uvm_va_range_t`,
the tracker allocates its bitmap and record array:

```c
// uvm_va_range.c:150
uvm_sampling_tracker_init_va_range(va_range);
```

```c
// uvm_sampling_tracker.c
static uvm_va_range_tracker_t *uvm_sampling_tracker_alloc_range_tracker(uvm_va_range_t *va_range)
{
    unsigned long nr_pages        = uvm_va_range_size(va_range) >> PAGE_SHIFT;
    unsigned long nr_record_pages = min(nr_pages, (unsigned long)UVM_TRACKER_RECORD_PAGES);
    //                                            ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //                                            capped at 128K slots (1 << 17)

    range_tracker->bitmap  = uvm_kvmalloc_zero(BITS_TO_LONGS(nr_pages) * sizeof(long));
    range_tracker->records = uvm_kvmalloc_zero(nr_record_pages * sizeof(uvm_page_record_t));

    range_tracker->nr_pages        = nr_pages;
    range_tracker->nr_record_pages = nr_record_pages;
    range_tracker->stride          = DIV_ROUND_UP(nr_pages, nr_record_pages);
    ...
}
```

### Memory layout for a 1 GB allocation (262144 pages, stride = 2)

```
VA range: [0x200000000, 0x240000000)  (1 GB)

Bitmap  (262144 bits = 32768 bytes):
  bit 0      → page 0      (addr 0x200000000)
  bit 1      → page 1      (addr 0x200001000)
  ...
  bit 262143 → page 262143 (addr 0x23ffff000)

Records (128K entries × 20 bytes = 2.5 MB):
  record[0]  covers pages 0–1   (stride = 2)
  record[1]  covers pages 2–3
  ...
  record[131071] covers pages 262142–262143
```

The record index for any page is:

```c
// uvm_sampling_tracker.h
static inline unsigned long uvm_sampling_tracker_record_index(
        const uvm_va_range_tracker_t *range_tracker, unsigned long page_index)
{
    unsigned long record_index = page_index / range_tracker->stride;
    if (record_index >= range_tracker->nr_record_pages)
        record_index = range_tracker->nr_record_pages - 1;
    return record_index;
}
```

For ranges smaller than 128K pages (e.g., a 256 MB allocation = 65536 pages),
stride is 1, so every page has its own record slot — full resolution.

---

## 5. Fault-Time Recording

The central integration point is inside the GPU replayable fault handler, which
processes batches of GPU page faults. For every fault entry in the current block:

```c
// uvm_gpu_replayable_faults.c:1158–1193

// 1. Stamp the GPU id so "revoke" knows which GPU's PTEs to unmap.
if (va_space->sampling_tracker)
    uvm_sampling_tracker_set_context(va_space->sampling_tracker, gpu->id.val);

// 2. Check whether this 2 MB block is in the sample.
if (uvm_sampling_tracker_block_sampled(va_space->sampling_tracker,
                                       va_block->va_range->sampling_tracker,
                                       va_block->start))
    // 3. Record the fault.
    uvm_sampling_tracker_record(
            va_space->sampling_tracker,
            va_block->va_range->sampling_tracker,
            current_entry->fault_address,        // exact faulting address
            va_block->va_range->node.start,      // range base VA
            (u8)current_entry->fault_access_type);
```

### Block sampling decision

```c
// uvm_sampling_tracker.h
static inline bool uvm_sampling_tracker_block_sampled(
        uvm_va_space_tracker_t *space_tracker,
        uvm_va_range_tracker_t *range_tracker,
        unsigned long block_start)
{
    u32 rate = READ_ONCE(space_tracker->sample_rate);

    if (!READ_ONCE(space_tracker->active)) return false;
    if (rate == 0)  return false;
    if (rate == 1)  return true;   // fast path: track everything

    // UVM VA blocks are exactly 2 MB = 1 << 21 bytes.
    // Divide the block number by the sample rate and keep only
    // those whose remainder is 0.
    return ((block_start >> 21) % rate) == 0;
}
```

With `sample_rate = 4`, only every 4th 2 MB block is tracked. This reduces
overhead for very large allocations while still giving a representative sample.

---

## 6. Prefetch Suppression

For sampled blocks, the tracker suppresses the UVM prefetch engine so that only
**explicitly faulted pages** get migrated. This ensures every individual page
access causes its own fault and gets its own record — without prefetch silently
pulling in neighbors and polluting the access data.

```c
// uvm_va_block.c:9488–9497

// For sampled blocks, suppress all prefetch effects. The hint is consumed
// in two places below: region/access_type expansion and the
// new_residency_mask OR in the per-processor migration loop.
// Both must be suppressed so that prefetch pages are never silently
// migrated or mapped — every page must fault individually for
// page-level tracking granularity.
if (uvm_sampling_tracker_block_sampled(va_block->va_range->va_space->sampling_tracker,
                                       va_block->va_range->sampling_tracker,
                                       va_block->start))
    prefetch_hint = UVM_PERF_PREFETCH_HINT_NONE();
```

---

## 7. Lock-Free Record Update

`uvm_sampling_tracker_record()` is designed to be called from concurrent fault
handlers with **no spinlocks**. It uses `READ_ONCE`, `WRITE_ONCE`, and `cmpxchg`
for all shared state.

```c
// uvm_sampling_tracker.c
void uvm_sampling_tracker_record(uvm_va_space_tracker_t *space_tracker,
                                 uvm_va_range_tracker_t *range_tracker,
                                 unsigned long addr,
                                 unsigned long base_va,
                                 u8 access_type)
{
    unsigned long page_index   = (addr - base_va) >> PAGE_SHIFT;
    unsigned long record_index;
    uvm_page_record_t *rec;
    u32 cur_epoch, old_epoch, access_type_mask;

    if (!READ_ONCE(space_tracker->active)) return;
    if (page_index >= range_tracker->nr_pages) return;

    // Mark this page as "seen" in the bitmap — atomic bit set.
    set_bit(page_index, range_tracker->bitmap);

    record_index      = uvm_sampling_tracker_record_index(range_tracker, page_index);
    rec               = &range_tracker->records[record_index];
    cur_epoch         = READ_ONCE(space_tracker->epoch);
    old_epoch         = READ_ONCE(rec->epoch);
    access_type_mask  = uvm_sampling_tracker_access_type_mask(access_type);

    if (old_epoch == cur_epoch) {
        // Same epoch — just OR the new access type in atomically.
        uvm_sampling_tracker_update_access_mask(rec, access_type_mask);
        return;
    }

    // New epoch: race to claim this record.
    if (cmpxchg(&rec->epoch, old_epoch, cur_epoch) != old_epoch) {
        // Lost the race. If winner already wrote this epoch, merge our type.
        if (READ_ONCE(rec->epoch) == cur_epoch)
            uvm_sampling_tracker_update_access_mask(rec, access_type_mask);
        return;
    }

    // Won the race — write first-access metadata for this epoch.
    WRITE_ONCE(rec->timestamp_ns, uvm_ktime_get_ns());
    WRITE_ONCE(rec->pid,          (u32)task_tgid_nr(current));
    WRITE_ONCE(rec->access_type_mask, access_type_mask);
}
```

The `access_type_mask` OR is itself lock-free:

```c
static void uvm_sampling_tracker_update_access_mask(uvm_page_record_t *rec, u32 access_type_mask)
{
    u32 old_mask, new_mask;
    do {
        old_mask = READ_ONCE(rec->access_type_mask);
        new_mask = old_mask | access_type_mask;
        if (new_mask == old_mask) return;   // no new bits; nothing to do
    } while (cmpxchg(&rec->access_type_mask, old_mask, new_mask) != old_mask);
}
```

### Timeline for a single record, two CPUs racing

```
CPU A (write fault, epoch 2)       CPU B (read fault, epoch 2)
──────────────────────────         ──────────────────────────
reads rec->epoch = 1 (old)         reads rec->epoch = 1 (old)
cmpxchg(1→2) WINS                  cmpxchg(1→2) FAILS
writes timestamp, pid              sees rec->epoch == 2
writes mask = WRITE (0x4)          OR's in READ  (0x2)
                                   mask = WRITE|READ (0x6)
```

---

## 8. The procfs Control Interface

### Reading: `cat /proc/uvm_sampling_tracker`

```
active_default=1
sample_rate_default=1
tracked_va_spaces=1
tracked_ranges=3
accessed=42 (rich_metadata=38 bitmap_only=4)
---
va_space=0xffff8881234a0000 pid=12345 epoch=3 active=1 sample_rate=1 gpu_id=0
range=[0x200000000,0x240000000] pages=262144 record_pages=131072 stride=2
  page=0        addr=0x200000000 pid=12345  ts_ns=1714000000000000000  access=write
  page=2        addr=0x200002000 pid=12345  ts_ns=1714000000000010000  access=read|write
  page=4        addr=0x200004000 ...
```

- **rich metadata** = bitmap bit set AND `rec->epoch == cur_epoch` (full record available)
- **bitmap only** = bit set but `rec->epoch != cur_epoch` (page was accessed in a previous epoch; only the fact of access is retained, not timestamp/pid)

### Writing: `echo <cmd> > /proc/uvm_sampling_tracker`

| Command | Effect |
|---|---|
| `enable 1` | Enable tracking for all VA spaces |
| `enable 0` | Disable tracking (recording skipped, no overhead) |
| `sample_rate N` | Track 1-in-N blocks (1 = all, 0 = off) |
| `clear` | Bump epoch + zero bitmaps. Old records become stale. GPU PTEs untouched — next access need not fault to be tracked; it must fault naturally. |
| `revoke` | Same as `clear`, then strip GPU write PTEs so every subsequent write causes a fresh fault and gets a new record. |

```c
// uvm_sampling_procfs.c — "clear" handler
else if (strncmp(kbuf, "clear", 5) == 0) {
    list_for_each_entry(va_space, &g_uvm_global.va_spaces.list, list_node) {
        uvm_va_space_tracker_t *space_tracker = va_space->sampling_tracker;

        // O(1): incrementing epoch instantly invalidates all 128K records.
        WRITE_ONCE(space_tracker->epoch, READ_ONCE(space_tracker->epoch) + 1);

        uvm_va_space_down_read(va_space);
        uvm_for_each_va_range(va_range, va_space)
            bitmap_zero(range_tracker->bitmap, range_tracker->nr_pages);
        uvm_va_space_up_read(va_space);
    }
}
```

---

## 9. The `revoke` Command and Epoch Mechanism

`revoke` is the most powerful command. It:

1. Bumps each VA space's epoch (instantly invalidates all existing records)
2. Zeros all bitmaps
3. Calls `uvm_tracking_revoke_epoch()` to strip GPU write permissions

```c
// uvm_va_block.c:11011
NV_STATUS uvm_tracking_revoke_epoch(uvm_va_space_t *va_space, uvm_gpu_id_t gpu_id)
{
    uvm_va_space_down_read(va_space);
    uvm_for_each_va_range(va_range, va_space) {
        if (va_range->type != UVM_VA_RANGE_TYPE_MANAGED) continue;

        for_each_va_block_in_va_range(va_range, va_block) {
            uvm_mutex_lock(&va_block->lock);
            // Downgrade GPU mappings from RW → RO (or unmap).
            // Next GPU write to any page will fault.
            status = uvm_va_block_revoke_write_for_tracking(va_block, block_context, gpu_id);
            uvm_mutex_unlock(&va_block->lock);
        }
    }
    uvm_va_space_up_read(va_space);
}
```

### Why the epoch pattern is O(1)

Instead of zeroing 128K records (2.5 MB of writes), bumping a single `u32`
immediately makes every record appear stale. The next fault that writes to a
record wins it with `cmpxchg` and refreshes it for the new epoch.

```
Epoch 2 (active)            After "revoke" → Epoch 3

record[0].epoch = 2  ✓      record[0].epoch = 2  ✗ (stale vs 3)
record[1].epoch = 2  ✓      record[1].epoch = 2  ✗
record[2].epoch = 1  ✗      record[2].epoch = 1  ✗
```

No memory is zeroed; the old epoch numbers simply no longer match.

---

## 10. Teardown

```
Process exits / cuCtxDestroy()
    └─► uvm_va_space_destroy()
            ├── uvm_sampling_tracker_destroy_va_space(va_space)   [uvm_va_space.c:223]
            │       └── uvm_kvfree(va_space->sampling_tracker)
            └── for each va_range:
                    uvm_sampling_tracker_destroy_va_range(va_range) [uvm_va_range.c:490]
                            ├── uvm_kvfree(range_tracker->records)
                            ├── uvm_kvfree(range_tracker->bitmap)
                            └── uvm_kvfree(range_tracker)
```

---

## 11. Data Flow Diagram

```
┌──────────────────────────────────────────────────────────────────────────┐
│  insmod nvidia-uvm.ko                                                    │
│    uvm_sampling_tracker_init()  →  /proc/uvm_sampling_tracker created   │
└───────────────────────┬──────────────────────────────────────────────────┘
                        │
        ┌───────────────▼──────────────────┐
        │  process opens /dev/nvidia-uvm   │
        │  uvm_sampling_tracker_init_va_space()                            │
        │    alloc uvm_va_space_tracker_t                                  │
        │    epoch=1, active=1, sample_rate=1                              │
        └───────────────┬──────────────────┘
                        │
        ┌───────────────▼──────────────────┐
        │  cudaMalloc(ptr, size)           │
        │  uvm_sampling_tracker_init_va_range()                            │
        │    alloc bitmap  (nr_pages bits)                                 │
        │    alloc records (≤128K slots)                                   │
        │    compute stride = ceil(nr_pages / nr_record_pages)             │
        └───────────────┬──────────────────┘
                        │
        ┌───────────────▼──────────────────┐
        │  GPU reads/writes ptr[i]         │
        │  → GPU page fault                │
        │  fault handler:                  │
        │    set_context(gpu_id)           │
        │    block_sampled()?              │
        │      NO  → normal UVM service    │
        │      YES → record(addr, type)    │
        │              set_bit(page_index) │
        │              cmpxchg epoch       │
        │              write ts/pid/mask   │
        │    prefetch suppressed for       │
        │    sampled blocks                │
        └───────────────┬──────────────────┘
                        │
        ┌───────────────▼──────────────────┐
        │  cat /proc/uvm_sampling_tracker  │
        │    iterate all VA spaces/ranges  │
        │    for_each_set_bit in bitmap:   │
        │      if rec->epoch == cur_epoch: │
        │        print addr, pid, ts, mask │
        │      else:                       │
        │        print addr only           │
        └───────────────┬──────────────────┘
                        │
        ┌───────────────▼──────────────────┐
        │  echo revoke > /proc/...         │
        │    bump epoch  (O(1) invalidate) │
        │    zero bitmaps                  │
        │    revoke GPU write PTEs         │
        │      → next write faults again   │
        └──────────────────────────────────┘
```

---

## Summary

| Step | Where | What happens |
|---|---|---|
| Module load | `uvm.c` | Global defaults set, `/proc/uvm_sampling_tracker` created |
| `cuInit()` / open `/dev/nvidia-uvm` | `uvm_va_space.c` | `uvm_va_space_tracker_t` allocated, epoch starts at 1 |
| `cudaMalloc()` | `uvm_va_range.c` | Bitmap + record array allocated with stride compression |
| GPU fault | `uvm_gpu_replayable_faults.c` | Block sampling check, then `uvm_sampling_tracker_record()` |
| Record | `uvm_sampling_tracker.c` | Lock-free epoch + `cmpxchg`; bitmap bit set; metadata written |
| Prefetch | `uvm_va_block.c` | Suppressed for sampled blocks to preserve fault granularity |
| `cat /proc/...` | `uvm_sampling_procfs.c` | Walk all VA spaces, print per-page records |
| `echo revoke > /proc/...` | `uvm_sampling_procfs.c` | O(1) epoch bump + GPU PTE revocation → fresh fault cycle |
| Process exit | `uvm_va_space.c`, `uvm_va_range.c` | Bitmap, records, and tracker structs freed |
