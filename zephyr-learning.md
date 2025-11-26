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

## 7. Power Management

Interfaces and APIs in Zephyr's PM stack are intentionally SoC-agnostic: the core framework exposes uniform hooks to the kernel and applications, while SoC-specific code plugs in the actual state transitions, residency data, and wake-up plumbing. That split keeps higher layers portable even when the underlying silicon has wildly different sleep modes.

### 7.1 System Power Management
Reference: [System PM docs](https://docs.zephyrproject.org/latest/services/pm/system.html)

1. **Kernel idle flow:** When the scheduler has nothing runnable and `CONFIG_PM` is enabled, the kernel enters idle and asks the PM subsystem to suspend for a requested duration. The policy manager compares that time window against each supported power state and picks the best fit, then restores execution when the wake event fires.
2. **Wake-up events:** Applications are responsible for provisioning at least one interrupt-capable wake source (RTC, counter, GPIO, modem peripheral, etc.). Availability depends on the SoC and the chosen state, so confirm which peripherals remain powered before relying on them.
3. **Key concepts:**
   - **Power states (`pm_state`):** Each state specifies power draw, context retention, and exit latency (see API docs for authoritative definitions). Quick reference:

     | State | Power/Retention Summary | Resume Notes & Analogy |
     | --- | --- | --- |
     | `PM_STATE_ACTIVE` | Fully powered system; CPUs + peripherals running. | Immediate resume; baseline run state (ACPI G0/S0).
     | `PM_STATE_RUNTIME_IDLE` | CPUs drop into deepest idle, devices stay in their current states. | Fast wake; similar to ACPI S0ix for short naps.
     | `PM_STATE_SUSPEND_TO_IDLE` | All cores idle and many peripherals clock-gated, but CPU context retained. | Straight-line resume without reinit (ACPI S1).
     | `PM_STATE_STANDBY` | Non-boot CPUs powered off, remaining logic and peripherals in deeper savings. | Higher latency than suspend-to-idle; maps to ACPI S2.
     | `PM_STATE_SUSPEND_TO_RAM` | DRAM in self-refresh, most silicon off; CPU/device context saved in RAM. | Requires boot ROM assist before kernel resumes (ACPI S3).
     | `PM_STATE_SUSPEND_TO_DISK` | RAM contents flushed to NVM; virtually all power removed. | Wake copies state back from storage before continuing (ACPI S4).
     | `PM_STATE_SOFT_OFF` | Minimal leakage; no CPU/RAM context retained. | Cold boot path (ACPI G2/S5).

   - **Power policy manager:** Decides which state to enter based on availability, residency, and latency constraints.
     - *Residency-based mode* (default): choose the deepest state whose `min_residency + exit_latency` fits within the kernel’s requested suspend time.
     - *Application-defined mode*: implement `pm_policy_next_state()` to select states using custom heuristics.
   - **State locks/constraints:** APIs such as `pm_policy_state_lock_get()` and DT-defined constraints let drivers/applications block unsafe states (e.g., while DMA or networking is active).

### 7.2 Device Power Management
Reference: [Device PM docs](https://docs.zephyrproject.org/latest/services/pm/device.html)

#### 7.2.1 System-Managed Device PM
- **Enablement:** Requires `CONFIG_PM_DEVICE=y` and `CONFIG_PM_DEVICE_SYSTEM_MANAGED=y`. For a given CPU power state, the PM core only runs device callbacks if that state’s Devicetree node omits the `zephyr,pm-device-disabled` flag.
- **Eligibility:** A device participates when (1) it initialized successfully and lives on the global device list, (2) its driver implements `pm_device_action_t` callbacks, and (3) it is not exclusively controlled by runtime PM.
- **Suspend/resume flow:** After the system PM policy selects a low-power CPU state, the framework walks devices in initialization order, calling `PM_DEVICE_ACTION_SUSPEND`, then enters the SoC sleep state. Upon wake, devices resume in reverse order via `PM_DEVICE_ACTION_RESUME`.
- **Blocking entry:** A single device can veto the transition by returning an error (commonly `-EBUSY`) from its suspend callback. With `CONFIG_PM_NEED_ALL_DEVICES_IDLE=y`, any device that previously called `pm_device_busy_set()` keeps the system active until it clears the busy flag.
- **Callback constraints:** Suspend/resume handlers may run in the idle thread context, so they must remain non-blocking—avoid sleeping APIs unless you first check `k_can_yield()` and offload work elsewhere.
- **Usage notes:** Best for hardware that simply mirrors SoC sleep/wake without independent policies and can quiesce immediately. For general-purpose devices needing fine-grained control or blocking operations, Zephyr recommends adopting device runtime PM instead of the system-managed approach.

#### 7.2.2 Runtime Device PM
- **Enablement:** Turn on `CONFIG_PM_DEVICE_RUNTIME`. You can globally enable runtime PM with `CONFIG_PM_DEVICE_RUNTIME_DEFAULT_ENABLE`, mark specific devices with the `zephyr,pm-device-runtime-auto` DT property, or call `pm_device_runtime_enable()` per device.
- **Driver responsibility:** Drivers call `pm_device_runtime_get()` / `pm_device_runtime_put()` (or `_async`) to bump their usage counts whenever they need hardware powered, independent of the current system power state. Applications do not directly suspend/resume devices but may enable/disable runtime PM for them.
- **State machine:** Devices start in `PM_DEVICE_STATE_SUSPENDED`. First `get()` resumes the hardware (`PM_DEVICE_STATE_ACTIVE`). When the usage count drops to zero, the device transitions back to suspended—either synchronously or via a `PM_DEVICE_STATE_SUSPENDING` intermediate state for async work.

- **Power domains:** ❓ *Open question*—docs state that devices under a DT `power-domains` node automatically request/release the parent domain on runtime `get()`/`put()`. Need to study Zephyr power-domain support to confirm sequencing and constraints before relying on this behavior.
- **Best practices:** Keep suspend/resume callbacks short; defer lengthy work to a thread if `pm_device_runtime_put_async()` is used. Favor runtime PM for peripherals that make autonomous power decisions (e.g., sensors, radios) regardless of system-wide sleep states.

##### Runtime PM Roles & Responsibilities
1. **Application layer**
  - Calls driver/subsystem functional APIs such as `sensor_sample_fetch()` or `uart_tx()` without touching PM directly.
  - May offer hints about expected idle windows, but in a clean design it never invokes `pm_device_runtime_get()/put()` itself.
2. **Runtime PM core**
  - Provides APIs like `pm_device_runtime_get()`, `pm_device_runtime_put()`, and their async/delayed variants.
  - Maintains each device’s usage refcount and drives the official PM state machine (`PM_DEVICE_STATE_ACTIVE`, `SUSPENDING`, `SUSPENDED`, etc.).
  - On a 0 → 1 refcount transition it issues `PM_DEVICE_ACTION_RESUME` (plus `TURN_ON` when a power domain is involved) before marking the device ACTIVE.
  - When the refcount returns to zero it marks the device eligible for low power, runs any autosuspend policy (immediate or delayed), transitions through `SUSPENDING`, and calls `PM_DEVICE_ACTION_SUSPEND` so the driver ends in `PM_DEVICE_STATE_SUSPENDED` or OFF if power domains participate.
3. **Device driver / subsystem**
  - Wraps every hardware-touching API with `pm_device_runtime_get()` before use and `pm_device_runtime_put()` afterward so the PM core knows when the hardware is busy.
  - Uses `pm_device_runtime_put_async(dev, delay)` to request autosuspend after a grace period when immediate suspend would thrash performance.
  - Implements the `pm_device_action_t` callback to translate core actions into register writes, clock enables, or regulator toggles for that peripheral.

##### Design Principles & Flow
- Runtime-managed devices are isolated from system PM transitions: once runtime PM is enabled, they are no longer suspended/resumed automatically during CPU low-power entry—they depend solely on driver `get()/put()` calls.
- `pm_device_runtime_get()` and `pm_device_runtime_put()` are blocking operations that adjust the usage count and, when crossing 0/1, invoke `PM_DEVICE_ACTION_RESUME/SUSPEND` as shown in the flow diagram above. Long suspend/resume handlers therefore add latency to the caller.
- To avoid blocking the caller, drivers can use `pm_device_runtime_put_async()`, but must ensure the work runs in a context that tolerates suspension latency. By default these async operations execute on the system workqueue, so suspend handlers must not block and thereby stall other system work. Set `CONFIG_PM_DEVICE_RUNTIME_USE_DEDICATED_WQ=y` to move runtime PM work to its own queue when blocking is unavoidable.
- If asynchronous support is unnecessary (or memory-constrained), disable it via `CONFIG_PM_DEVICE_RUNTIME_ASYNC=n` to reduce complexity once all drivers rely on synchronous paths.

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
