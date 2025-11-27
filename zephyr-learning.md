---
applyTo: zephyr-learning.md
---

> **Description**
This is the note I use to document all my learnings about Zephyr RTOS. You must keep it updated automatically whenever I share new learnings or insights about Zephyr.

> 🧠 **Learning Requirement**
> I will write down my learnings about Zephyr in our conversations, you must:
> 1. If I shared a link, learn from the link and search on the web for more information to validate my understanding.
> 2. Point out any misconceptions or incomplete explanations in my learnings.

> 📝 **Formatting Requirement**
> Keep this document human-readable with clear structure, color/emojis for emphasis, and friendly callouts every time it is updated.

---

# Zephyr Learning Reference

This file contains detailed learning notes about Zephyr RTOS concepts, systems, and features.

> ⚠️ **Important:** Preserve technical accuracy and keep section numbering aligned with the rest of the engineering notes when updating this file.

## Terms
**Subsystem** is a higher-level OS component (logging, power management, networking, sensors, etc.) that sits on top of the kernel, often orchestrates multiple drivers, and exposes a coherent API plus Kconfig knobs for applications to use—it’s not just “kernel minus app and drivers”; drivers usually belong to or are managed by subsystems.

## 1. Build and Configuration System

The Zephyr build process consists of two main phases:

### 1.1 Configuration Phase (CMake-driven)
- Device Tree Processing:
  - Input: DTS (`.dts`), DTSI (`.dtsi`), and bindings (`.yaml`)
  - Output: 
    - Final devicetree: `build/zephyr/zephyr.dts`
    - Generated header: `build/zephyr/include/generated/zephyr/devicetree_generated.h`

- Kconfig Processing:
  - Purpose: Enable/disable features without code changes
  - Sources and Hierarchy (in order of precedence):
    1. Board default config (`boards/<vendor>/<board>/<board>_defconfig`)
    2. CMake cache entries with CONFIG_ prefix
    3. Application config (`prj.conf` by default, or CONF_FILE)
    4. Overlay configs via `-DOVERLAY_CONFIG`
    5. Board-specific configs from `boards/<board>`
    6. SoC/Arch specific Kconfig
    7. Subsystem/driver Kconfig files
    8. `Kconfig.zephyr` (top-level)
    - 🔔 **Important:** Earlier entries override later ones; double-check overlay ordering when troubleshooting unexpected `CONFIG_` values.
  
  - Configuration Tools:
    - `menuconfig`: Terminal-based configuration interface
    - `guiconfig`: GUI-based configuration interface
    - `traceconfig`: Generates detailed symbol value origins

  - Output Files:
    - `zephyr/.config`: Raw merged configuration
    - `build/zephyr/include/generated/autoconf.h`: Generated header with config macros
    - `build/zephyr/kconfig/` : Contains trace files if traceconfig used

### 1.2 Build Phase (Make/Ninja-driven)
- Key Operations:
  - Offset generation
  - System call boilerplate
  - Dependency handling
  - Binary generation

- Output Files:
  - `zephyr.bin`: Raw binary format (direct memory dump)
  - `zephyr.hex`: Intel HEX format with address info and checksums (for flashing)
  - `zephyr.elf`: Contains debug symbols (for debugging)

- Binary Types:
  - Non-fixed size: Relocatable in memory (for dynamic loading)
  - Fixed size: Linked to specific addresses (typical for bare metal)

## 2. Device Tree Bindings

### 2.1 Core Concepts
1. Binding files (`.yaml`) define:
   - Required and optional properties for DT nodes
   - Property types, constraints, and validation rules
   - Location: `dts/bindings/<vendor>/<device>.yaml` matching `compatible` string
   - Example: `compatible = "nordic,nrf52840-gpio"` → `dts/bindings/gpio/nordic,nrf52840-gpio.yaml`

2. Node Matching Process:
   - Nodes can have multiple strings in `compatible` property
   - Build system searches bindings in order listed in `compatible`
   - Uses first matching binding found
   - Example: `compatible = "vendor,specific-model", "vendor,generic-model"`
     - Tries `vendor,specific-model` first
     - Falls back to `vendor,generic-model` if specific not found

3. Child Bindings:
   - When a node has no matching `compatible`:
     - Build system checks parent node's binding file
     - Parent's `child-binding` section defines properties for children
   - Common in bus devices (I2C/SPI):
     - Parent: I2C controller with binding
     - Children: I2C devices using parent's child-binding
   - Example:
     ```yaml
     # i2c-controller.yaml
     child-binding:
       properties:
         reg:
           required: true
         clock-frequency:
           type: int
     ```

### 2.2 Key Features
- Build System Processing:
  - Validates node properties against binding schema
  - Generates C header with devicetree macros
  - Outputs: 
    1. `build/zephyr/zephyr.dts`: Final devicetree
    2. `build/zephyr/include/generated/devicetree_generated.h`: C macros for DT access

### 2.3 Practical Usage
1. Find bindings:
   - Check node's `compatible` property
   - Search in `dts/bindings/` matching vendor/device structure
   - Look for parent's binding if no direct match

2. Access DT data:
   - Use generated macros in C code:
     - `DT_NODELABEL(label)`: Reference node by label
     - `DT_PROP(node_id, property)`: Get property value
     - `DT_INST(n, compatible)`: Get nth instance of compatible
   - 📝 **TODO:** Add a short worked example (board overlay → generated macro usage) to illustrate the full lookup path.

## 3. Scheduling, Threading, and Interrupts

### 3.1 Thread Management

#### 3.1.1 Stack Management
- Every thread has a fixed-size stack
  - Regular threads: Use `K_THREAD_STACK_DEFINE` for kernel-only stacks
  - User threads: Must use "user-capable" stacks
    - Two-stack model: User stack + small privileged kernel stack
  - ISRs: Use separate dedicated stack
- Stack Features:
  - Stack overflow detection
  - Stack usage measurement
  - Stack sentinel: Catches stack corruptions

#### 3.1.2 Thread Priority
- Lower numerical value = Higher priority
- Two categories:
  1. Cooperative threads (negative priority)
     - Cannot be preempted once running, except by meta-IRQ threads
     - Must explicitly yield control
  2. Preemptible threads (non-negative priority)
     - Can be preempted by higher priority threads
- Dynamic priority: Can be changed at runtime

#### 3.1.3 Scheduling Behavior
- Scheduling Points:
  - Thread yields/suspends/waits
  - Other threads become ready
  - Return from ISR
  - Time slice expiration (if enabled)

- Thread Selection:
  1. Highest priority ready thread gets selected
  2. For equal priority: longest waiting thread runs
  3. With earliest-deadline-first: both priority AND deadline must match

- Time Slicing:
  - Allows sharing CPU among equal priority threads
  - No guaranteed execution time
  - Thread may run longer if higher priority thread runs at slice expiration

- Preemption Control:
  - `k_sched_lock()`: Prevents preemption
  - Lock persists across thread suspension/resumption
  - System allows other threads if locked thread becomes unready

#### 3.1.4 Thread-Specific Data vs TLS
- Thread-specific data:
  - Managed by kernel
  - Set/get via k_thread_custom_data APIs
  - Single word (32/64 bit) per thread
  - Fast access, limited size
  - Use case: Thread identification, state flags
- Thread Local Storage (TLS):
  - Compiler/architecture supported
  - Multiple variables per thread
  - Accessed like global variables
  - More memory overhead
  - Use case: Thread-local variables in C

#### 3.1.5 Thread Statistics
- Runtime stats: CPU usage, execution cycles
- Stack usage monitoring
- State tracking (running, ready, blocked)

#### 3.1.6 System Threads

##### 3.1.6.1 Overview
System threads are automatically spawned by the kernel during system initialization. These threads provide core OS functionality and cannot be manually created or destroyed by applications.

##### 3.1.6.2 Main Thread
- **Purpose**: Executes the application's `main()` function
- **Priority**: Default is 0 (highest preemptible priority)
  - Configurable via `CONFIG_MAIN_THREAD_PRIORITY`
- **Stack**: Configurable via `CONFIG_MAIN_STACK_SIZE`
- **Lifecycle**:
  1. Kernel completes initialization
  2. Main thread starts
  3. Calls application's `main()` function
  4. If `main()` returns, thread terminates (unusual for embedded)
- **Use case**: Application entry point, initialization code

##### 3.1.6.3 Idle Thread
- **Purpose**: Runs when no other work is available; handles power management
- **Priority**: Lowest priority (numerically highest value)
- **Always ready**: Never sleeps or blocks - always ready to run
- **Multi-core**: In SMP systems, one idle thread exists per CPU core
- **Behavior**:
  - Executes infinite loop
  - Calls `k_cpu_idle()` or `k_cpu_atomic_idle()` to enter low-power state
  - Activates power management if `CONFIG_PM` enabled
  - Can execute custom idle hooks (functions run during idle time)
- **Configuration**:
  - Stack size: `CONFIG_IDLE_STACK_SIZE`
  - Cannot replace the thread itself
  - Can register idle hooks for custom idle-time processing
- **Use case**: Power saving, background maintenance tasks via hooks

##### 3.1.6.4 Other System Threads
Beyond main and idle threads, Zephyr creates additional system threads based on enabled features:
- **System Workqueue Thread(s)**: Execute deferred work items
- **Logging Thread**: Process log messages asynchronously
- **Shell Thread**: Handle shell command processing
- **Network Threads**: Handle network stack processing (RX thread, TX thread, management thread)
  - 📝 **TODO:** Document shell/log thread stack requirements once sizing guidance is finalized.

##### 3.1.6.5 Key Characteristics
- **Automatic creation**: Applications don't create these threads
- **Essential services**: Provide core OS functionality
- **Configuration-driven**: Existence depends on enabled Kconfig options
- **Non-terminable**: Most system threads run forever (except main)

#### 3.1.7 Workqueue Threads

##### 3.1.7.1 Overview
Workqueues provide a mechanism for deferring work execution to a dedicated thread context. Each workqueue has its own thread that processes work items sequentially in FIFO order.

##### 3.1.7.2 Core Concepts
- **Work Items**: Encapsulate a function to be executed by a workqueue thread
  - Each work item specifies its worker function
  - Processed in FIFO order
  - Cannot be reinitialized while running or pending, but can be resubmitted

- **Multiple Workqueues**:
  - Any number of workqueues can be defined
  - Each has its own dedicated thread
  - Priority is configurable at creation time

- **System Workqueue**:
  - Always available for application use
  - Applications can use system workqueue or create custom ones

##### 3.1.7.3 Work Item Types

**Standard Work**:
- Submit to system queue or specific queue
- Executed immediately when workqueue thread is scheduled
- Can be cancelled if still pending in queue (not yet running)

**Delayable Work**:
- Work item processed only after a specified timeout
- After timeout expires, work item is submitted to the workqueue
- Can be cancelled before timeout expires
- After timeout, it becomes standard work (cancellation depends on whether it's still pending or already running)

**Triggered Work**:
- Work item submitted when specific poll events occur
- Specify list of events to monitor
- When any event triggers, work item is submitted to workqueue
- Can be cancelled before event triggers
- After event triggers, it becomes standard work (cancellation depends on whether it's still pending or already running)
- Useful for event-driven deferred processing

##### 3.1.7.4 Work Item Lifecycle
- **Pending**: Queued but not yet executing, OR currently executing, OR both
- **Running**: Currently being processed by workqueue thread
- **Resubmission**: Work can be resubmitted while running (becomes both running AND pending)
- **Busy**: Work is in any active state (pending, running, canceling, etc.)
- **Constraint**: Cannot reinitialize work item while it's busy (running or pending)

### 3.2 Interrupts

#### 3.2.1 Interrupt Context
- ISRs execute in the kernel's interrupt context using dedicated interrupt stack memory (architecture-dependent).
- Stack sizing must account for the deepest expected nesting level if interrupt nesting is enabled.
- Most kernel APIs are thread-only; `k_is_in_isr()` helps code paths adapt their behavior in interrupt context.

#### 3.2.2 Preventing Interruptions
- `irq_lock()` masks interrupts for the current thread and prevents preemption until the lock is released; the lock is suspended if the thread blocks and reinstated when it resumes.
- Locking can be nested; interrupts resume only after matching unlocks.
- Disabling a specific IRQ masks that interrupt system-wide, preventing it from preempting any thread until re-enabled.

#### 3.2.3 Offloading ISR Work
- ISRs should finish quickly to keep latency predictable.
- Lengthy work should be deferred to threads via kernel objects (e.g., FIFO, semaphore) or workqueues.
- Offloading typically causes a single context switch after the ISR completes, letting processing continue in thread context.

#### 3.2.4 Zero-Latency Interrupts
- Zero-latency interrupts bypass interrupt locking and must be declared as direct ISRs with the zero-latency flag.
- They must not interact with kernel APIs or shared data inspected by normal contexts; behavior is otherwise undefined.
- Intended for handling critical hardware events with the lowest possible latency.

#### 3.2.5 Shared Interrupt Lines
- Enabling shared interrupts lets multiple ISR/argument pairs attach to one interrupt line.
- When triggered, each registered client ISR runs in registration order (subject to the configured client limit).
- Dynamic interrupts can be disconnected at runtime when both shared and dynamic interrupt support are enabled.

### 3.3 Symmetric Multiprocessing (SMP)

#### 3.3.1 Overview
- `CONFIG_SMP` enables multiple physical CPUs to run Zephyr threads concurrently; `CONFIG_MP_MAX_NUM_CPUS` reflects the number of usable CPUs.
- Applications do not need special APIs to benefit: any runnable threads can execute in parallel on different cores.

#### 3.3.2 Synchronization
- `k_spin_lock()` provides cross-core mutual exclusion by masking interrupts locally and spinning until the shared lock variable is free; spinlocks are **not recursive** (re-acquiring the same lock while held deadlocks) though distinct spinlocks can be nested.
- `irq_lock()`/`irq_unlock()` keep legacy semantics via a **single global spinlock with nesting** so only one thread across all CPUs can hold it; other CPUs spin until the lock is released, so SMP code should prefer spinlocks for fine-grained critical sections.
- Traditional IPC primitives (mutexes, semaphores, queues) retain their behavior under SMP and remain appropriate for blocking synchronization.

#### 3.3.3 CPU Masking
- `CONFIG_SCHED_CPU_MASK` lets applications constrain where a thread may run; by default threads are eligible for all CPUs.
- APIs such as `k_thread_cpu_mask_disable()` / `k_thread_cpu_mask_enable()` adjust per-thread affinity but require the target thread to be non-runnable.
- Because the scheduler must scan all ready threads when CPU masks are enabled, this feature is available only with the `CONFIG_SCHED_SIMPLE` backend (no per-CPU queues or scalable/multi-queue schedulers).

#### 3.3.4 SMP Boot Process
- System boot mirrors uniprocessor flow: all kernel and device initialization occurs on the primary CPU while other CPUs remain halted.
- Just before invoking application `main()`, the kernel calls `z_smp_init()` to start secondary CPUs via `arch_cpu_start()`, providing each with a stack region and a `smp_init_top()` entry point.
- Auxiliary CPUs run `smp_init_top()`, set up per-CPU timer state, wait for a shared “start flag,” then enter the scheduler via `z_swap()` so regular thread scheduling can proceed on all cores simultaneously.

#### 3.3.5 Interprocessor Interrupts (IPIs)
- Architectures should provide `arch_sched_broadcast_ipi()` (and optionally `arch_sched_directed_ipi()` when `CONFIG_ARCH_HAS_DIRECTED_IPIS`) so CPUs can request remote schedulers to reschedule or wake from low-power idle.
- IPIs allow features like `k_thread_abort()` to synchronize with threads running on other CPUs and enable true low-power idle by waking sleeping cores when new work appears.
- If IPIs are unavailable, Zephyr falls back to a polling idle loop: correct but less power efficient because idle CPUs spin checking scheduler state instead of using deep idle states.

##### 3.3.5.1 IPI Cascades
- With CPU masks, a single IPI round may not produce a valid assignment of ready threads to CPUs; enabling `CONFIG_SCHED_IPI_CASCADE` lets the kernel send follow-up IPIs until a valid set is scheduled.
- Cascades incur overhead (extra scheduling checks, more interrupts, potential short-lived thread migrations) so they are disabled by default and should be enabled only when tighter affinity guarantees are required.
  - ⚠️ **Important:** Verify that tracing is enabled when experimenting with cascades; silent stalls are difficult to debug without scheduler instrumentation.

##### 3.3.5.2 IPI Work Items
- `k_ipi_work_add()` queues ISR-context callbacks to run on selected CPUs after the next signaled IPI; `k_ipi_work_signal()` triggers delivery and `k_ipi_work_wait()` synchronizes with completion (single waiter enforced).
- Useful for cross-CPU coordination when ISR-level actions must run on remote cores, e.g., propagating peripheral status updates.

## 4. Memory Management

### 4.1 Heap Allocators
- `k_heap` is the synchronized heap object (blocking `k_heap_alloc()` with timeout + `k_heap_free()`). It can be defined with `K_HEAP_DEFINE()` or initialized over any contiguous buffer via `k_heap_init()` (external RAM, non-cacheable region, etc.).
- `sys_heap` is the low-level allocator that backs every `k_heap`. It has the same semantics but **no locking**; callers must serialize access (useful in ISRs or custom synchronization contexts). `CONFIG_SYS_HEAP_ALLOC_LOOPS` bounds the search work per allocation.
- The **system heap** is simply the kernel-managed `k_heap` that powers `k_malloc()`/`k_free()`. Size is set by `CONFIG_HEAP_MEM_POOL_SIZE` plus any `HEAP_MEM_POOL_ADD_SIZE_*` contributors (which enforce a minimum unless `CONFIG_HEAP_MEM_POOL_IGNORE_MIN` overrides it). Only one system heap exists and it is not directly addressable.
- All heap pointers are pointer-size aligned. `k_heap_alloc()` accepts `K_NO_WAIT`, finite `k_timeout_t`, or `K_FOREVER`, making it more flexible than standard `malloc()`.
- Instrumentation: the optional heap listener API can track allocations, and the kernel logs allocation failures if `CONFIG_SYS_HEAP_VALIDATE` or tracing options are enabled.

### 4.2 Multi-Heap and Attribute-Based Allocation
- `sys_multi_heap` wraps multiple `sys_heap` instances so discontiguous regions can be treated as a single pool. Each allocation call passes an opaque configuration pointer. During initialization you register a callback that inspects that opaque value (e.g., cacheability enum) and chooses which underlying heap to use.
- The `shared_multi_heap_*` helpers implement the common pattern: platform code collects memory regions (often from Devicetree) into `shared_multi_heap_region` structs, tags them with attributes like `SMH_REG_ATTR_CACHEABLE` or `SMH_REG_ATTR_NON_CACHEABLE`, and calls `shared_multi_heap_add()`.
- Drivers/apps then call `shared_multi_heap_alloc(attr, size)` (or aligned/realloc variants). The framework routes the request to a region whose `.attr` matches the opaque value and still has space; fallback order is up to the registered chooser.
- Attributes are extensible: define your own constants/enums when you need tags such as “external RAM”, “DMA-capable”, or “CPU affinity”.

### 4.3 Memory Slabs
- `k_mem_slab` provides fixed-size block allocation backed by an aligned buffer defined at build or runtime (`K_MEM_SLAB_DEFINE`, `k_mem_slab_init`). Blocks must be at least 4N bytes and the buffer is aligned to the same N so every block satisfies the hardware alignment requirement.
- The slab tracks free blocks via an embedded linked list (first word of each free block) and supports blocking allocation with per-thread timeouts; waiting threads are serviced in priority + wait-time order.
- Slabs avoid fragmentation entirely and shine for IPC buffers, networking packets, or other uniform payload sizes. Multiple slabs with different block sizes can coexist, giving deterministic allocation without touching the general heap.

### 4.4 Memory Blocks Allocator (`sys_mem_blocks`)
- Similar goal (fixed-size blocks) but metadata lives **outside** the backing buffer, so the buffer can reside in power-gated or DMA-only memory. Allocation/free use a bitmap (`sys_bitarray`) instead of linked lists.
- Must be declared at compile time via `SYS_MEM_BLOCKS_DEFINE*`; initialization happens in place. You can supply your own external buffer (`SYS_MEM_BLOCKS_DEFINE_WITH_EXT_BUF`) if the default placement is not suitable.
- Multiple blocks can be allocated/freed in one call, and the returned blocks are not guaranteed to be contiguous—perfect for scatter-gather DMA descriptors.
- `sys_multi_mem_blocks` mirrors `sys_multi_heap`: register a chooser callback to hand out blocks from whichever allocator meets the caller’s opaque configuration parameter.

### 4.5 Demand Paging
- Enable with `CONFIG_DEMAND_PAGING` plus architecture support. Physical memory is divided into page frames; page faults trigger the paging core to fetch data from a backing store when the referenced page is absent.
- Components:
  - **Eviction algorithm** (`k_mem_paging_eviction_*` hooks). Zephyr ships NRU (demo) and LRU (recommended) implementations; you can provide your own to control which page frame gets freed next.
  - **Backing store driver** (`k_mem_paging_backing_store_*` hooks) responsible for reserving backing locations, copying data to/from `K_MEM_SCRATCH_PAGE`, and acknowledging completion.
  - **Optional stats** via `k_mem_paging_stats_get()` plus timing histograms when `CONFIG_DEMAND_PAGING_TIMING_HISTOGRAM*` is set.
- APIs like `k_mem_page_in()` / `k_mem_page_out()` let applications prefetch or evict pages deliberately to hide latency or free RAM.

### 4.6 Virtual Memory
- Requires `CONFIG_MMU` and page-size settings (`CONFIG_MMU_PAGE_SIZE`, `CONFIG_KERNEL_VM_BASE`, `CONFIG_KERNEL_VM_SIZE`, `CONFIG_KERNEL_VM_OFFSET`). Without demand paging, every virtual mapping must still be backed by physical RAM.
- Boot mappings typically mirror physical layout: `.text` read-only/exec, `.rodata` read-only, data/stack sections kernel-only, with optional `CONFIG_KERNEL_DIRECT_MAP` to expose MMIO regions.
- `k_mem_map()` provides anonymous page-aligned mappings from the free virtual region (`K_MEM_VM_FREE_START` up to `K_MEM_VIRT_RAM_END`). Guard pages are automatically inserted before and after each mapping to trap underruns/overruns. Always pass the same size to `k_mem_unmap()` when freeing the region; the API does not validate mismatched lengths.
- Virtual memory alone does **not** extend usable RAM. To oversubscribe memory you must pair VM with demand paging so unmapped regions can spill to backing store.

## 5. Device Driver Model

### 5.1 Device Objects and Lifecycle
- Each driver registers a `struct device` that stores the instance name, state, optional power-management data, and a pointer to an API vtable.
- Device objects are placed in a linker section and brought up automatically during kernel initialization; applications never malloc them.
- `device_is_ready()` is the canonical guard before using a peripheral because it checks both initialization status and dependencies declared via `DT_REQUIRES`/`zephyr,depends-on`.
- Applications typically look up devices with `DEVICE_DT_GET()` / `DEVICE_DT_GET_ANY()` / `DEVICE_DT_GET_ONE()` macros so the compiler can catch missing instances.

### 5.2 API Structures and Callbacks
- Drivers expose type-specific API structs (e.g., `struct gpio_driver_api`, `struct i2c_driver_api`) containing function pointers that implement standard operations.
- Inline helpers in `<zephyr/drivers/...>` modules fetch the API pointer from `struct device` and call the appropriate method, keeping application code portable.
- Minimal example:
  ```c
  struct my_sensor_api {
      int (*sample_fetch)(const struct device *dev);
      int (*channel_get)(const struct device *dev, enum sensor_channel ch,
                         struct sensor_value *val);
  };

  static const struct my_sensor_api my_api = {
      .sample_fetch = my_sensor_sample_fetch,
      .channel_get = my_sensor_channel_get,
  };
  ```
- Applications never touch the callbacks directly—use the inline wrappers (`sensor_sample_fetch(dev)`) to stay aligned with future API changes.

### 5.3 Initialization Levels and Priorities
- `DEVICE_DT_DEFINE()` / `DEVICE_DT_INST_DEFINE()` bind a driver to a devicetree node and schedule its init function into one of Zephyr's init levels.
- Levels run in this order: `PRE_KERNEL_1` (hardware, clocks), `PRE_KERNEL_2` (dependent on early drivers), `POST_KERNEL` (most drivers), `APPLICATION` (app-defined setup), plus optional `SMP` stages.
- Each device selects a small integer priority inside its level; smaller numbers run earlier. Use this to satisfy ordering constraints without hard-coding dependencies in CMake.
- `DEVICE_DT_DEFINE(node, init_fn, pm_control_fn, data, config, level, prio, api);`
  ties everything together: `init_fn` reads devicetree properties, config structures hold constant register information, and `data` holds mutable runtime state.

### 5.4 Devicetree-Driven Instantiation
- Devicetree nodes describe every driver instance. Status must be `okay` for code to be generated; overlays are the usual way to enable or tweak peripherals.
- `DEVICE_DT_INST_DEFINE(n, ...)` expands over every `status = "okay"` instance of the compatible, pulling values directly from `devicetree_generated.h`.
- Macro helpers:
  - `DT_INST_REG_ADDR(n)`, `DT_INST_PROP(n, prop)` supply base addresses and configuration constants.
  - `DT_INST_FOREACH_STATUS_OKAY(fn)` or `LISTIFY(DT_NUM_INST_STATUS_OKAY(compat), ...)` iterate over instances when registering tables or work items.
- Example overlay snippet enabling an SPI sensor:
  ```dts
  &spi1 {
      status = "okay";
      bme280@76 {
          compatible = "bosch,bme280";
          reg = <0x76>;
          label = "BME280_SPI";
          spi-max-frequency = <8000000>;
      };
  };
  ```
  Paired driver code uses `DEVICE_DT_INST_DEFINE(0, bme280_init, NULL, &data0, &cfg0,
  POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &bme280_api);` to materialize the instance.

### 5.5 Power Management Hooks
- Drivers can opt into device PM via the `pm_device` sub-structure embedded in `struct device`; Zephyr calls the registered `pm_device_action_t` handler when transitioning between `PM_DEVICE_ACTION_SUSPEND`, `RESUME`, etc.
- Runtime PM (`CONFIG_PM_DEVICE_RUNTIME`) lets drivers automatically suspend after inactivity; they must call `pm_device_runtime_get_sync()` / `pm_device_runtime_put()` around critical sections.
- SoC-level power states (idle, deep sleep) can cascade into driver callbacks, so drivers must leave hardware quiescent when asked to suspend.
- When no custom PM handling is required, pass `NULL` for the control function in `DEVICE_DT_DEFINE()` and Zephyr skips the hooks.

### 5.6 Simple Usage Examples
- Fetching a device and toggling GPIO:
  ```c
  const struct device *led = DEVICE_DT_GET(DT_ALIAS(led0));

  if (!device_is_ready(led)) {
      return -ENODEV;
  }

  gpio_pin_configure_dt(&led0_spec, GPIO_OUTPUT_ACTIVE);
  gpio_pin_toggle_dt(&led0_spec);
  ```
- Creating a lightweight custom driver (single instance):
  ```c
  static int dummy_init(const struct device *dev) {
      const struct dummy_config *cfg = dev->config;
      struct dummy_data *data = dev->data;
      data->base = cfg->base;
      return 0;
  }

  static struct dummy_data dummy0_data;
  static const struct dummy_config dummy0_cfg = {
      .base = DT_INST_REG_ADDR(0),
  };

  DEVICE_DT_INST_DEFINE(0, dummy_init, NULL,
                        &dummy0_data, &dummy0_cfg,
                        POST_KERNEL, 50, &dummy_api);
  ```
- 💡 **Tip:** Keep configuration structs `const` so they land in flash, and gate optional features behind Kconfig symbols to avoid initializing unused hardware.

## 6. OS Services

### 6.1 Workqueues and Deferred Execution
- `k_work` lets ISRs and threads schedule callbacks on a worker thread so long-running tasks stay out of interrupt context.
- The system workqueue (`k_sys_work_q`) is always available; create custom queues with `K_THREAD_STACK_DEFINE` + `k_work_queue_start()` when you need dedicated priority or stack sizing.
- Delayable work wraps a timer (`k_work_delayable`) while triggered work (`k_work_poll`) reacts to poll events; both ultimately enqueue standard work items once ready.
- Example: reschedule a sensor read outside the ISR that caught the interrupt.
  ```c
  static void sensor_work_handler(struct k_work *work)
  {
      ARG_UNUSED(work);
      sensor_sample_fetch(sensor_dev);
      sensor_channel_get(sensor_dev, SENSOR_CHAN_ACCEL_XYZ, &val);
  }

  K_WORK_DEFINE(sensor_work, sensor_work_handler);

  void gpio_callback(const struct device *dev, struct gpio_callback *cb,
                     uint32_t pins)
  {
      k_work_submit(&sensor_work);
  }
  ```
- 🔔 **Important:** A work item cannot be reinitialized while busy; use `k_work_busy_get()` before touching shared state to avoid race conditions when ISRs resubmit work rapidly.

### 6.2 Logging Subsystem
- `CONFIG_LOG` spawns a logging thread that dequeues log messages emitted through `LOG_MODULE_REGISTER()` macros.
- Backends (RTT, UART, memory) are enabled via `CONFIG_LOG_BACKEND_*`; they subscribe to the centralized circular buffer managed by `log_core`.
- `LOG_INF/WARN/ERR` are deferred by default; `CONFIG_LOG_MODE_IMMEDIATE` forces synchronous emission but increases ISR latency.
- Example module setup:
  ```c
  #include <zephyr/logging/log.h>
  LOG_MODULE_REGISTER(app, CONFIG_APP_LOG_LEVEL);

  void do_init(void)
  {
      LOG_INF("Init complete (clk=%u Hz)", SystemCoreClock);
  }
  ```
- 📝 **TODO:** Document how `CONFIG_LOG_PROCESS_THREAD_STACK_SIZE` scales with enabled backends once sizing data is collected.

### 6.3 Shell Service
- `CONFIG_SHELL` launches a shell thread that pulls commands from UART, RTT, or Bluetooth transports; each transport is configured via `CONFIG_SHELL_BACKEND_*` options.
- Commands are registered with `SHELL_CMD_REGISTER()` or `SHELL_STATIC_SUBCMD_SET_CREATE()`, producing a hierarchical command tree with built-in help.
- Shell handlers run in the shell thread’s context, so blocking operations stall user input; offload lengthy work to a workqueue or dedicated thread.
- Minimal command example:
  ```c
  static int cmd_led(const struct shell *shell, size_t argc, char **argv)
  {
      ARG_UNUSED(argc);
      ARG_UNUSED(argv);
      gpio_pin_toggle_dt(&led0_spec);
      shell_print(shell, "LED toggled");
      return 0;
  }

  SHELL_CMD_REGISTER(led, NULL, "Toggle status LED", cmd_led);
  ```

### 6.4 Kernel IPC Primitives
- Zephyr offers multiple IPC objects with distinct semantics:
  - `k_msgq`: fixed-size messages copied into an internal ring buffer; great for small telemetry packets.
  - `k_fifo` / `k_lifo`: pass pointers to application-defined nodes, avoiding copies and enabling zero-copy producer/consumer patterns.
  - `k_mbox`: multi-word messages with optional asynchronous notifications; intended for multi-core SoCs.
  - `k_pipe`: streaming byte pipes that support partial reads/writes with timeouts.
- All IPC objects support timeouts via `k_timeout_t`, so callers can use `K_NO_WAIT`, finite waits, or `K_FOREVER` to block indefinitely.
- Example message queue setup:
  ```c
  K_MSGQ_DEFINE(sensor_q, sizeof(struct sensor_packet), 8, 4);

  void producer_thread(void)
  {
      struct sensor_packet pkt = read_sensor();
      k_msgq_put(&sensor_q, &pkt, K_NO_WAIT);
  }

  void consumer_thread(void)
  {
      struct sensor_packet pkt;
      while (k_msgq_get(&sensor_q, &pkt, K_FOREVER) == 0) {
          process_packet(&pkt);
      }
  }
  ```

### 6.5 Timers and System Clock Services
- `k_timer` wraps the kernel tick source, running callbacks either in ISR context (expiry handler) or thread context (stop handler). Use `k_timer_start()` with a start delay and optional period to model periodic jobs without spinning.
- `k_work_schedule()` is built atop timers, so aligning timer periods with work scheduling reduces jitter.
- `k_uptime_get()` / `k_cycle_get_32()` expose millisecond and raw cycle counters respectively; prefer uptime for portable code and cycle counts for profiling on a known clock rate.
- Example periodic timer kicking work:
  ```c
  static void telemetry_expiry(struct k_timer *timer)
  {
      ARG_UNUSED(timer);
      k_work_submit(&telemetry_work);
  }

  K_TIMER_DEFINE(telemetry_timer, telemetry_expiry, NULL);

  void main(void)
  {
      k_timer_start(&telemetry_timer, K_SECONDS(1), K_SECONDS(5));
  }
  ```
- ⚠️ **Important:** Timer handlers run in interrupt context—avoid blocking APIs there. Instead, trigger lightweight actions or hand work to a queue as shown above.

## 7. Power Management (PM)

> 💭 **My PM Mental Model**
>
> Power management spans four cooperating domains:
> 1. **System state management** orchestrates whole-platform states (Sleep, Hibernate, Shutdown etc). When the policy manager initiates one of these transitions, every device driver must honor the requested system state or explicitly veto it, otherwise the transition stalls.
> 2. **Device state management** lets peripherals drop to lower-power modes even while the system stays active, typically via system-managed PM hooks or runtime PM refcounts.
> 3. **Processor performance/state management** treats CPUs as special devices: while active (ACPI C0) they can shift performance via P-states, and when idle they entry/exit architectural C-states through Zephyr’s idle + CPU frequency subsystems.
> 4. **Thermal & power limit enforcement** cuts across all layers. Thermal trips may force a system suspend/shutdown, while board-level power governors can throttle CPUs or devices despite their local policies.
>
> Keeping these domains conceptually separate helps map Zephyr’s modular APIs (system PM core, device PM, CPU freq/SMP idle, thermal inputs) to the real-world triggers that drive them.

Interfaces and APIs in Zephyr's PM stack are intentionally SoC-agnostic: the core framework exposes uniform hooks to the kernel and applications, while SoC-specific code plugs in the actual state transitions, residency data, and wake-up plumbing. That split keeps higher layers portable even when the underlying silicon has wildly different sleep modes.

### 7.1 System Power Management
Reference: [System PM docs](https://docs.zephyrproject.org/latest/services/pm/system.html)

#### 7.1.1 Layered View & Coordination
- **Application layer:**
  - Registers deadlines through `pm_policy_event_register()` and can block specific system states via `pm_policy_state_lock_get()` / `pm_policy_state_lock_put()` so long as the locks are released once the critical section ends.
  - Can supply a custom policy callback (application-based mode) that overrides the default residency engine whenever it returns a target state.
- **Kernel / idle thread:**
  - When the scheduler goes idle (no runnable work), the idle thread asks the PM policy which `pm_state` to enter.
  - The default residency policy computes `next_scheduled_event` by combining kernel-managed timeouts (next timer, sleep, or delayed work) with any registered application deadlines. It then selects the deepest state whose minimum residency plus exit latency fits before that deadline.
  - If the application provides a policy callback, the kernel simply forwards the computed `next_scheduled_event` and obeys the callback’s chosen state; the residency engine is bypassed.

#### 7.1.2 End-to-End Flow
1. **Idle entry:** Scheduler finds no runnable threads, so the idle thread runs with `CONFIG_PM` enabled.
2. **Deadline gathering:**
   - Kernel determines the earliest internal timeout.
   - Applications/subsystems optionally inject deadlines via `pm_policy_event_register()`.
   - The minimum of those values becomes `next_scheduled_event`.
3. **Policy decision:**
   - Residency mode: choose the deepest `pm_state` whose `min_residency + exit_latency <= time_until(next_scheduled_event)`.
   - Application mode: invoke the registered callback with `next_scheduled_event`; the callback returns the desired `pm_state` (or `PM_STATE_ACTIVE` to skip entry).
4. **State locks:** Before committing, the kernel checks whether `pm_policy_state_lock_get()` (and DT constraints) forbid the candidate state. If locked, it backs off to the next shallower level.
5. **Transition + wake:** The SoC-specific backend executes the transition. A wake event or timeout brings the CPU back to active state and resumes normal scheduling.

#### 7.1.3 Key Takeaways
- System PM only evaluates states when the idle thread runs; if work is pending, no state transition occurs.
- Residency policy balances two clocks: kernel-owned timers and app-specified deadlines. There is currently **no API for apps to declare an exit-latency requirement**, so they must use deadlines/state locks to guard critical sections.
- Applications can block unwanted transitions entirely by calling `pm_policy_state_lock_get()`; the lock stays in effect until `pm_policy_state_lock_put()` releases it, so ensure every acquisition is matched with a release once the critical section ends.
- Application policies can completely override residency heuristics, enabling product-specific heuristics (battery level, thermal trips, etc.) while reusing the same kernel plumbing.
- Wake sources remain an application responsibility—ensure at least one interrupt-capable source stays powered for every low-power state you plan to enter.

#### 7.1.4 Key Macros, Data Structures & Callbacks
- **`CONFIG_PM`** – master Kconfig switch enabling the system PM framework and idle policy hooks.
- **`pm_state` / `struct pm_state_info`** – enumerations + metadata (min residency, exit latency) describing each platform-supported system state. Quick reference of built-in targets:

  | State | Resume Analogy | Notes |
  | --- | --- | --- |
  | `PM_STATE_ACTIVE` | ACPI G0/S0 | Baseline run state. |
  | `PM_STATE_RUNTIME_IDLE` | ACPI S0ix | CPU idle, devices unchanged. |
  | `PM_STATE_SUSPEND_TO_IDLE` | ACPI S1 | Context retained; clocks gated. |
  | `PM_STATE_STANDBY` | ACPI S2 | Non-boot CPUs off; longer latency. |
  | `PM_STATE_SUSPEND_TO_RAM` | ACPI S3 | DRAM self-refresh; boot ROM assist on resume. |
  | `PM_STATE_SUSPEND_TO_DISK` | ACPI S4 | RAM persisted to NVM; near-complete power loss. |
  | `PM_STATE_SOFT_OFF` | ACPI G2/S5 | Cold boot required. |
- **`pm_policy_state_lock_get()` / `_put()`** – per-state lock API that applications or drivers use to veto specific transitions during sensitive operations.
- **`pm_policy_event_register()`** – lets apps inject “wake by” deadlines into the residency engine so latency-sensitive work is not delayed.
- **`pm_policy_next_state()`** (optional override) – application-supplied callback that replaces the residency engine when custom logic needs full control.

### 7.2 Device Power Management
Reference: [Device PM docs](https://docs.zephyrproject.org/latest/services/pm/device.html)

#### 7.2.1 System-Managed Device PM

#### 7.2.1.1 Layered View & Coordination
- **Policy core:** Enabled with `CONFIG_PM_DEVICE` + `CONFIG_PM_DEVICE_SYSTEM_MANAGED`. When the system PM policy chooses a low-power CPU state (and the state’s DT node doesn't specify `zephyr,pm-device-disabled`), the core sequences every eligible device, honoring `CONFIG_PM_NEED_ALL_DEVICES_IDLE` before entering the state.
- **Device drivers:** Opt in by supplying a `pm_device_action_t` callback when defining the device. Drivers may flag ongoing work via `pm_device_busy_set()` so the policy waits before suspending them.
- **Applications:** Normally stay hands-off; their role is indirect—busy refcounts from drivers and state vetoes keep the platform awake if critical work is running. Apps can still monitor results (e.g., logging) but should not micromanage suspend/resume in this mode.

#### 7.2.1.2 End-to-End Flow
1. **Eligibility pass:** During boot, devices that initialized successfully and expose `pm_device_action_t` are enrolled for system-managed PM unless runtime PM has exclusive control.
2. **State selection:** Scheduler idles (meaning every CPU’s idle thread is running because no runnable work remains), so the system PM policy picks a target CPU state. If `CONFIG_PM_NEED_ALL_DEVICES_IDLE=y`, the policy verifies no device has an outstanding busy flag.
3. **Suspend walk:** Devices are traversed in initialization order, each receiving `PM_DEVICE_ACTION_SUSPEND`. Any device may abort the transition by returning an error (commonly `-EBUSY`).
4. **CPU state entry:** If all devices suspend cleanly, the SoC enters the selected sleep state.
5. **Resume walk:** On wake, devices resume in reverse order via `PM_DEVICE_ACTION_RESUME`, after which normal scheduling resumes.

#### 7.2.1.3 Key Takeaways
- Designed for “follow the system” peripherals that can mirror SoC sleep/wake with minimal logic; anything requiring bespoke timing should migrate to runtime PM instead.
- Suspend/resume callbacks run in idle-thread context, so keep them non-blocking or offload heavy work after checking `k_can_yield()`.
- A single device can veto entry either by returning an error from the suspend callback or by leaving a busy flag set; be disciplined about clearing `pm_device_busy_set()` once the protected transaction ends.
- Devicetree gates participation per state via `zephyr,pm-device-disabled`, letting you keep particular low-power modes from touching fragile hardware.

#### 7.2.1.4 Key Macros, Data Structures & Callbacks
- **`CONFIG_PM_DEVICE`, `CONFIG_PM_DEVICE_SYSTEM_MANAGED`** – enable the framework and tie device sequencing to system PM states.
- **`pm_device_action_t`** – driver-provided callback invoked with actions such as `PM_DEVICE_ACTION_SUSPEND`, `RESUME`, `TURN_OFF`, etc.
- **`pm_device_busy_set()` / `pm_device_busy_clear()`** – let a driver declare in-progress work so the system policy delays low-power entry until cleared (requires `CONFIG_PM_NEED_ALL_DEVICES_IDLE`).
- **Devicetree `zephyr,pm-device-disabled` property** – opt-out toggle on individual low-power states to prevent the PM core from touching devices when that state is selected.

#### 7.2.2 Runtime Device PM
Reference: [Device runtime PM docs](https://docs.zephyrproject.org/latest/services/pm/device_runtime.html)

##### 7.2.2.1 Layered View & Coordination
- **Applications & subsystems:** Invoke their usual functional APIs (e.g. `sensor_sample_fetch()`) and can optionally enable/disable runtime PM per device by the driver, but applications are not supposed to call `pm_device_runtime_get()/put()` directly.
- **Runtime PM core:** Enabled through `CONFIG_PM_DEVICE_RUNTIME`, it tracks each device’s usage count, owns the `PM_DEVICE_STATE_*` state machine, and issues `PM_DEVICE_ACTION_*` when reference counts cross 0 ↔ 1. It also propagates requests to parent power domains when the child declares `power-domains` in Devicetree.
- **Device drivers:** Register a `pm_device_action_t` handler, call `pm_device_runtime_enable()` (manually or via the `zephyr,pm-device-runtime-auto` flag), wrap hardware access in `pm_device_runtime_get()/put()` (sync or async), and decide when autosuspend delays are appropriate.

##### 7.2.2.2 End-to-End Flow
1. **Enable runtime PM:** During init, the driver calls `pm_device_runtime_enable()` (optionally after `pm_device_init_suspended()` if the hardware already sits in low power). Boards can globally default to enabled via `CONFIG_PM_DEVICE_RUNTIME_DEFAULT_ENABLE` or per-node with `zephyr,pm-device-runtime-auto`.
2. **Client requests hardware:** Before touching registers, the driver invokes `pm_device_runtime_get()`. A 0 → 1 transition triggers `PM_DEVICE_ACTION_RESUME`, bringing the device to `PM_DEVICE_STATE_ACTIVE`.
3. **Work completion:** After the operation, the driver calls `pm_device_runtime_put()` to decrement the usage count immediately, or `pm_device_runtime_put_async(dev, delay)` to defer suspension by a grace period.
4. **Suspend transition:** When the refcount reaches zero, the core marks the device `PM_DEVICE_STATE_SUSPENDING` (async path) or directly `PM_DEVICE_STATE_SUSPENDED` (sync path) and issues `PM_DEVICE_ACTION_SUSPEND`. The resume/suspend callbacks run serially in the runtime PM work context.
5. **Interaction with system PM:** Once runtime PM is enabled on a device, system-managed PM stops touching it during CPU low-power entry. The device’s availability is therefore controlled entirely by its runtime refcount, which shortens system suspend/resume because the idle thread no longer has to walk those drivers.

##### 7.2.2.3 Key Takeaways
- Runtime PM is reference-counted: every successful `pm_device_runtime_get()` must be paired with a `put()` (sync or async). Forgetting to `put()` pins the hardware on; extra `put()` calls can suspend hardware mid-transaction.
- Driver callbacks must stay non-blocking unless you move runtime PM work off the system workqueue via `CONFIG_PM_DEVICE_RUNTIME_USE_DEDICATED_WQ` or set `CONFIG_PM_DEVICE_DRIVER_NEEDS_DEDICATED_WQ` for that driver.
- The runtime PM core does **not** automatically manage arbitrary parent/child dependencies—drivers must `get()/put()` their dependencies-drivers manually.
- Runtime-managed devices are invisible to system-managed PM transitions, which is why Zephyr recommends runtime PM for anything that needs bespoke timing or long-lived wakeups; system-managed PM should only cover “follow the SoC” peripherals.
- Autosuspend (`pm_device_runtime_put_async()` with a delay) prevents thrashing on chatty peripherals but still funnels through the runtime PM workqueue, so treat suspend handlers like workqueue callbacks: avoid blocking and offload heavy operations elsewhere when possible.

##### 7.2.2.4 Key Macros, Data Structures & Callbacks
- **Kconfig switches:** `CONFIG_PM_DEVICE_RUNTIME` (master enable), `CONFIG_PM_DEVICE_RUNTIME_DEFAULT_ENABLE` (enable all devices by default), `CONFIG_PM_DEVICE_RUNTIME_ASYNC` (allow async put paths), `CONFIG_PM_DEVICE_RUNTIME_USE_DEDICATED_WQ` / `CONFIG_PM_DEVICE_DRIVER_NEEDS_DEDICATED_WQ` (move runtime PM work off the system queue when suspend/resume must block).
- **Devicetree hooks:** `zephyr,pm-device-runtime-auto` auto-enables runtime PM right after init.
- **APIs:** `pm_device_runtime_enable()/disable()` flip runtime PM at runtime, `pm_device_init_suspended()` advertises that the hardware already booted in a low-power state, `pm_device_runtime_get()/put()` implement the synchronous refcounting path, and `pm_device_runtime_put_async()` schedules deferred suspension. All callbacks funnel into the driver’s `pm_device_action_t` handler, which must handle `PM_DEVICE_ACTION_RESUME/SUSPEND/TURN_ON/TURN_OFF` to reflect the requested state in hardware.
 - **Key enum reference:**

   | `pm_device_action` value | When the core issues it | Driver expectation |
   | --- | --- | --- |
   | `PM_DEVICE_ACTION_RESUME` | Refcount rises from 0 → 1 (or policy explicitly wakes the device). | Re-enable clocks/IRQs, restore registers, make hardware usable, then return 0.
   | `PM_DEVICE_ACTION_SUSPEND` | Refcount drops to 0 and no autosuspend delay is pending. | Flush in-flight work, gate clocks, put hardware into its low-power state.
   | `PM_DEVICE_ACTION_TURN_ON` | Power domain or SoC backend just supplied power before drivers run. | Perform any power-on initialization required before the device can resume.
   | `PM_DEVICE_ACTION_TURN_OFF` | Domain/policy is about to remove power entirely. | Save context (if possible) and place hardware in a safe, power-loss-tolerant state.

   | `pm_device_state` value | Meaning | Who sets it |
   | --- | --- | --- |
   | `PM_DEVICE_STATE_ACTIVE` | Device is fully usable and clocked. | Driver returns success from RESUME/TURN_ON handlers.
   | `PM_DEVICE_STATE_LOW_POWER` | Device is in a vendor-defined retention state but can wake quickly. | Drivers that support intermediate modes set this via `pm_device_runtime_set_state()`.
   | `PM_DEVICE_STATE_SUSPENDED` | Hardware context retained but interface quiesced; waits for resume. | Runtime PM core after a successful SUSPEND.
   | `PM_DEVICE_STATE_OFF` | Rails removed; only a full TURN_ON restores operation. | Core after TURN_OFF completes.
   | `PM_DEVICE_STATE_SUSPENDING` / `PM_DEVICE_STATE_RESUMING` | Transitional markers while suspend/resume work is in-flight (mainly async puts). | Runtime PM core while dispatching workqueue jobs.
   | `PM_DEVICE_STATE_TURNING_ON` / `PM_DEVICE_STATE_TURNING_OFF` | Transitional markers while domain power is changing. | Runtime PM or power-domain core coordinating dependencies.
   | `PM_DEVICE_STATE_ERROR` | Last PM operation failed; framework will stop issuing new ones until recovered. | Runtime PM core when a callback returns failure; driver should log and attempt recovery before re-enabling runtime PM.

#### 7.2.3 Power Domains
Reference: [Power domain docs](https://docs.zephyrproject.org/latest/services/pm/power_domain.html)

- **Concept:** A power domain models a shared rail/regulator/SoC region supplying multiple devices. The domain itself is a Zephyr device; consumers reference it with `power-domains = <&domain>` in Devicetree so the kernel knows which peripherals share that rail.
- **Notifications:** When the rail turns off/on, the domain driver must notify every child via `PM_DEVICE_ACTION_TURN_OFF` / `PM_DEVICE_ACTION_TURN_ON`, keeping device PM state aligned with hardware reality.
- **Runtime PM integration:** Child devices typically rely on runtime PM to increment/decrement their usage counts. Those calls propagate to the parent domain, keeping its refcount accurate. Once all children are idle (usage reaches zero), the domain can suspend, drop power, and broadcast TURN_OFF; the next child request automatically powers it back up.
- **Why use it:** Power domains provide a clean, declarative way to model shared rails, ensuring all devices hear about power removal/restoration and preventing ad-hoc GPIO toggles that desynchronize drivers from actual hardware state.

**Example: two sensors on a shared I2C rail**
1. Both sensors A and B are active, so each driver calls `pm_device_runtime_get()`. Their usage counts move to 1, and the I2C domain's aggregate usage becomes 2, keeping the rail on.
2. Sensor A becomes idle and calls `pm_device_runtime_put()`. Its usage drops to 0, but B is still active, so the domain’s refcount stays at 1 and power remains applied.
3. When sensor B also calls `pm_device_runtime_put()`, the domain refcount hits 0. The power-domain driver suspends, shuts the rail off, and sends `PM_DEVICE_ACTION_TURN_OFF` to both sensors. The next `get()` from either sensor automatically powers the rail back up.

#### 7.2.4 Resource Management (On-Off Manager)

##### 7.2.4.1 Concept & Purpose
- The Resource Management (RM) On-Off Manager is a lightweight coordination primitive for any binary resource (ON/OFF plus transition/Error states) with multiple clients.
- It centralizes refcounting, asynchronous transitions, error recovery, and optional notifications so drivers/services can reuse a proven state machine instead of rolling custom logic.
- Each resource owner supplies callbacks to start/stop/reset the underlying hardware or service; RM orchestrates when to invoke them based on client demand.

##### 7.2.4.2 Core Model & Operations
- **States:** `off`, `on`, `error`, and “transition in progress.”
- **Client APIs:**
  - `onoff_request()` increments the refcount; a 0→1 transition triggers the owner’s `start` callback (sync or async via Zephyr’s async notification helpers).
  - `onoff_release()` decrements the refcount; a 1→0 transition triggers `stop`.
  - `onoff_reset()` recovers from `error` back to `off`.
  - `onoff_cancel()` lets a client abandon a pending request, but it does **not** abort the underlying transition—only the resource owner decides that.
- RM tracks **only counts**, not which client owns which request, so callers must know whether their request succeeded before issuing `release()`.

##### 7.2.4.3 RM vs Device Power Management
- **Device PM** operates on `struct device`, integrates tightly with system power states/power domains, and uses `pm_device_runtime_get/put()` plus `pm_device_action_t` callbacks.
- **RM/On-Off** is agnostic to the device model and can guard any logical resource (clock, rail, service) whether or not it is a Zephyr device. It has zero implicit knowledge of system PM or Devicetree metadata.
- Practical heuristic: *Device PM* handles framework-managed devices that participate in global power policy; *RM* provides a reusable on/off + refcount engine you can embed anywhere, including inside a Device PM callback if that helps structure the driver.

##### 7.2.4.4 When to Use Which
- **Reach for RM when:**
  - You need to share a binary resource (internal PLL, shared regulator, “turbo mode” flag, logging backend) among several clients without building a full device.
  - Enable/disable sequences might be asynchronous and you want standardized completion signaling.
  - You prefer a compact helper to manage refcounts, retries, and error recovery.
- **Stick with Device PM when:**
  - The resource is already a first-class Zephyr device and must honor system-level sleep, power domains, and `pm_device_state` semantics.

##### 7.2.4.5 Practical Notes
- RM callbacks may run in interrupt context if clients make requests there, so they should be non-blocking or offload heavy work.
- Monitoring hooks can observe transitions or failures, enabling logging or watchdog-style resets without tightly coupling to the resource owner.
- Because RM and Device PM are orthogonal, it’s common to see RM manage an internal sub-resource while the encompassing Device PM handler coordinates with the rest of the system.

### 7.3 CPU Frequency Scaling

#### 7.3.1 Key Terms
- **Metrics:** Measurements the policy observes to decide which P-state to select (CPU load, junction temperature, thermal headroom, etc.).
- **P-state policy:** Decision logic that chooses the next P-state by comparing metrics to thresholds or heuristics.
- **P-state driver:** SoC-specific mechanism that parses P-states from Devicetree, exposes `cpu_freq_pstate_set(index)` (or equivalent), and actually reprograms the clock/PLL and voltage rails as needed.
- **CPU frequency scaling subsystem:** Core Zephyr framework that owns the P-state list, exposes change/query APIs, may run a periodic `k_timer`, and assumes all CPUs share one frequency unless `CONFIG_CPU_FREQ_PER_CPU_SCALING` opts into per-core control.

#### 7.3.2 Layered View & Coordination
1. **Hardware + Devicetree (Capabilities):** The SoC advertises supported performance states in silicon; Devicetree captures them (e.g., `zephyr,pstate` lists) without prescribing behavior.
2. **P-state Driver (Mechanism):** Reads `zephyr,pstate`, builds the internal table, validates requested indices, and reprograms PLL/voltage safely from timer/IRQ context. It never decides *when* to change—just executes transitions.
3. **CPU Freq Subsystem Core (Coordinator):** Stores the shared P-state table, exposes public APIs, and often owns a policy `k_timer`. If nothing calls it, the CPU stays at the driver’s initial P-state.
4. **P-state Policy (Decision Maker):** Kernel module (generic or SoC-specific) that periodically gathers metrics, applies thresholds, and requests changes via `cpu_freq_pstate_set(target_idx)` when the desired state differs from the current one. Policies can be registered with the subsystem’s timer or manage their own scheduling.
5. **Application Layer (Consumer):** Normally agnostic to P-states; it just runs workloads while the policy adjusts frequency under the hood. Apps may optionally force a fixed P-state for tests or feed hints into higher-level power managers.

#### 7.3.3 Typical End-to-End Flow
1. **Boot:** Driver parses Devicetree, programs an initial P-state, and hands the table to the CPU freq subsystem, which may start its periodic timer.
2. **Timer Tick:** Subsystem fires the policy callback (if any).
3. **Policy Evaluation:** Policy samples metrics (CPU utilization, `Tj`, surface temps, etc.), compares them to configured thresholds, and computes the target P-state index.
4. **State Transition:** When the target differs from the current state, the policy calls `cpu_freq_pstate_set(index)`.
5. **Hardware Update:** The P-state driver reconfigures the clock/PLL (and voltage rails if supported). The CPU now runs at the new frequency until the next policy decision.

#### 7.3.4 Key Takeaways
- Devicetree documents *capabilities* (available P-states), not the governance logic.
- Automatic scaling only happens if a policy (or application code) actively requests changes; otherwise the CPU remains at the initial P-state.
- Metrics are pluggable: any signal measurable in kernel context—performance counters, thermal sensors, workload hints—can feed into the policy.
- Align policy timer context with driver requirements: since `cpu_freq_pstate_set()` must be IRQ-safe, lightweight policies can run directly from the subsystem timer, while heavier analytics should defer work to a thread before invoking the driver.

