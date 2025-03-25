# zig-miros

A zig port of the state-machine.com tiny real-time operating system
[MiROS](https://github.com/QuantumLeaps/MiROS).

Includes a dummy example application for the rp2040 using
[Microzig](https://github.com/ZigEmbeddedGroup/microzig) for its HAL and toolchain.

## Using miros:

1. Add miros as a dependency
2. Define a `systick_handler` that calls `miros.tick()`, and, with interrupts disabled,
   `miros.sched()`.
3. Define `OS_onIdle()`. It should `asm volatile ("wfi")` to not waste CPU.
4. Define `OS_onStartup()`. It must enable the **SysTick** interrupts, with the highest priority,
   and set its interrupt period. Note that `miros.delay` is in ticks.
5. Define your tasks, with the type `fn() callconv(.c) noreturn`.
6. Create a stack (A slice of u32 with `align(8)`) for each task and for the idle task.
7. Initialize the OS with `miros.init(&idle_stack)`.
8. Initialize all your threads wiith `thread.init(&stack)`.
9. Start all the threads, defining its priority and its handler with `thread.start(prio, &task)`.
10. Run `miros.run()`.
