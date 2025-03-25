const std = @import("std");
const microzig = @import("microzig");
const rp2xxx = microzig.hal;
const time = rp2xxx.time;

const led = rp2xxx.gpio.num(25);
const led1 = rp2xxx.gpio.num(14);
const led2 = rp2xxx.gpio.num(15);
const uart = rp2xxx.uart.instance.num(0);
const baud_rate = 115200;
const uart_tx_pin = rp2xxx.gpio.num(0);

const peripherals = microzig.chip.peripherals;

pub const microzig_options: microzig.Options = .{
    .log_level = .debug,
    .logFn = rp2xxx.uart.logFn,
    // Set the handlers
    .interrupts = switch (rp2xxx.compatibility.chip) {
        .RP2040 => .{
            .PendSV = pendsv_handler,
            .SysTick = systick_handler,
        },
        .RP2350 => .{
            .PendSV = pendsv_handler,
            .SysTick = systick_handler,
        },
    },
};

const OSThread = extern struct {
    const Self = @This();
    sp: *volatile u32 = undefined,
    spi: usize = 0,
    timeout: u32 = 0,
    priority: u8 = 0,
    stack: [*]u32 = undefined,
    // TODO: Make sure sp is 8 byte aligned as well?
    // Maybe by making sure the size is a multiple of 8

    fn init(self: *Self, stack: []u32) void {
        self.spi = stack.len;
        self.sp = &self.stack[self.spi];
        self.stack = stack.ptr;
    }

    fn push(self: *Self, v: anytype) void {
        // Check for overflow
        if (self.spi == 0)
            @panic("Stack overflow");
        self.spi -= 1;
        self.sp = &self.stack[self.spi];
        switch (@typeInfo(@TypeOf(v))) {
            .comptime_int => self.stack[self.spi] = @as(u32, v),
            .int => self.stack[self.spi] = @bitCast(v),
            .pointer => self.stack[self.spi] = @intFromPtr(v),
            else => {
                std.log.warn("Type is {any}", .{@TypeOf(v)});
                unreachable;
            },
        }
    }

    fn start(self: *Self, prio: u8, handler: *const fn () callconv(.c) noreturn) void {
        // Error if there is already a thread at this priority
        if (OS_threads[prio] != null)
            @panic("thread with this priority already exists");

        // Push xPSR
        self.push(1 << 24);
        // Push handler
        self.push(handler);
        // Push fake LR, R12, R3, R2, R1, R0, R11, R10, R9, R8, R7, R6, R5, R4
        for ([_]u32{ 13, 12, 3, 2, 1, 0, 11, 10, 9, 8, 7, 6, 5, 4 }) |i| {
            self.push(i);
        }

        self.priority = prio;

        // Add to thread list
        OS_threads[prio] = self;
        // Unless the idle thread, add to ready set or delay set as appropriate
        if (prio != 0) {
            const bit = (@as(u32, 1) << @truncate(prio - 1));
            if (self.timeout == 0)
                OS_readySet |= bit
            else
                OS_delayedSet |= bit;
        }
    }
};

pub fn OS_onIdle() void {
    asm volatile ("wfi");
}

pub fn OS_init(stack: []u32) void {
    peripherals.PPB.SHPR3.modify(.{
        // Set priority of PendSV low so it happens last after SysTick
        .PRI_14 = 3,
        // TODO: Should this be set in client code?
        // Set priority of SysTick to the highest (which should be the default)
        .PRI_15 = 0,
    });

    idle_thread.init(stack);
    idle_thread.start(0, idle_task);
}

pub fn OS_run() noreturn {
    // Enable the SysTick interrupt manually
    peripherals.PPB.SYST_CSR.modify(.{ .ENABLE = 1, .TICKINT = 1, .CLKSOURCE = 1 });
    // RVR = Reload Value Register
    // 1ms (125MHz / 125KMz)
    peripherals.PPB.SYST_RVR.modify(.{ .RELOAD = 125_000 - 1 });

    // Kick off the 'first' scheduling of tasks
    OS_sched();
    unreachable;
}

// Must be called from an interrupt handler, where interrupts are disabled
fn OS_sched() void {
    // If nothing is ready, schedule the idle thread
    if (OS_readySet == 0) {
        OS_next = OS_threads[0].?;
    } else {
        const idx = get_top_bit_index(OS_readySet);
        OS_next = OS_threads[idx].?;
    }

    // If we have a new thread to schedule, make PendSV execute
    if (OS_next != OS_curr) {
        peripherals.PPB.ICSR.modify(.{ .PENDSVSET = 1 });
    }
}

fn OS_delay(ticks: u32) void {
    // disable interrupts
    const cs = microzig.interrupt.enter_critical_section();
    defer cs.leave();
    if (OS_curr == OS_threads[0]) @panic("cannot call OS_delay from idle thread");

    const bit = (@as(u32, 1) << @truncate(OS_curr.priority - 1));
    // Set current thread as blocked
    OS_curr.timeout = ticks;
    // Clear ready bit
    OS_readySet &= ~bit;
    // Set delayed bit
    OS_delayedSet |= bit;
    // Call the scheduler
    OS_sched();
}

fn OS_tick() void {
    var s = OS_delayedSet;
    while (s != 0) {
        const idx = get_top_bit_index(s);
        const t = OS_threads[idx].?;
        const bit = (@as(u32, 1) << @truncate(t.priority - 1));
        if (t.timeout == 0)
            @panic("invalid thread in delayedSet");

        // Lower timeout
        t.timeout -= 1;

        // If the timeout is done, set ready & clear delayed
        if (t.timeout == 0) {
            OS_readySet |= bit;
            OS_delayedSet &= ~bit;
        }
        s &= ~bit;
    }
}

fn get_top_bit_index(v: u32) usize {
    // NOTE: There is no clz instruction on Thumb before Thumb2
    return 32 - @clz(v);
}

var stack1: [0x800]u32 = @splat(0xdeadbeef);
var stack2: [0x800]u32 = undefined;
var stack3: [0x800]u32 = undefined;
// Allow setting up a startup delay
var thread1 = OSThread{ .timeout = 500 };
var thread2 = OSThread{};
var thread3 = OSThread{};

export var OS_curr: *OSThread = undefined;
export var OS_next: *OSThread = undefined;

// TODO: Encapsulate
var OS_threads: [32 + 1]?*OSThread = undefined;
var OS_readySet: u32 = 0;
var OS_delayedSet: u32 = 0;

// We'd like to use .naked here, but we don't allow that in the vector table
pub fn pendsv_handler() callconv(.c) void {
    // TODO: This zig part of this modifies sp, do we have to account for it before we push?
    asm volatile (
        \\
        // HACK: Account for the stack adjustment in the prologue
        \\ add sp, #8
        // Disable interrupts
        \\ cpsid i
        // Only save registers after we have a thread
        \\ ldr r1, =OS_curr
        \\ ldr r1, [r1]
        \\ cmp r1, #0
        \\ beq PendSV_restore
        // Push registers R4-R11 on the stack
        // \\push {r4-r11}
        \\ push {r4-r7}
        \\ mov r4, r8
        \\ mov r5, r9
        \\ mov r6, r10
        \\ mov r7, r11
        \\ push {r4-r7}
        // OS_curr->sp = sp
        // NOTE: R1 still holds OS_curr
        \\ mov r0, sp
        \\ str r0, [r1]
        \\PendSV_restore:
        // sp = OS_next->sp
        \\ ldr r1, =OS_next
        \\ ldr r1, [r1]
        \\ ldr r0, [r1]
        \\ mov sp, r0
        // OS_curr = OS_next
        \\ ldr r1, =OS_next
        \\ ldr r1, [r1]
        \\ ldr r2, =OS_curr
        \\ str r1, [r2]
        // Pop registers
        \\ mov r0, sp
        \\ adds r0, #(4*4)
        \\ ldmia r0!, {r4-r7}
        \\ mov r11, r7
        \\ mov r10, r6
        \\ mov r9, r5
        \\ mov r8, r4
        \\ pop {r4-r7}
        \\ add sp, sp, #(4*4)
        // Enable interrupts
        \\ cpsie i
        \\ bx lr
        // No return value
        :
        // inputs
        : [OS_curr] "" (OS_curr),
          [OS_next] "" (OS_next),
          // no clobbers since we return
    );
}

pub fn systick_handler() callconv(.c) void {
    OS_tick();
    const cs = microzig.interrupt.enter_critical_section();
    defer cs.leave();

    OS_sched();
}

fn task1() callconv(.c) noreturn {
    while (true) {
        std.log.info("In task 1", .{});
        led1.toggle();
        OS_delay(500);
    }
}

fn task2() callconv(.c) noreturn {
    while (true) {
        // std.log.info("In task 2", .{});
        led2.toggle();
        OS_delay(1000);
    }
}

fn task3() callconv(.c) noreturn {
    while (true) {
        // std.log.info("In task 3", .{});
        led.toggle();
        OS_delay(750);
    }
}

fn idle_task() callconv(.c) noreturn {
    while (true) {
        OS_onIdle();
    }
}

var idle_stack: [0x100]u32 = undefined;
var idle_thread = OSThread{};

pub fn main() noreturn {
    // init uart logging
    uart_tx_pin.set_function(.uart);
    uart.apply(.{ .baud_rate = baud_rate, .clock_config = rp2xxx.clock_config });
    rp2xxx.uart.init_logger(uart);

    inline for (.{ led, led1, led2 }) |l| {
        l.set_function(.sio);
        l.set_direction(.out);
    }

    OS_init(&idle_stack);

    // Assign stack to threads
    thread1.init(&stack1);
    thread2.init(&stack2);
    thread3.init(&stack3);

    // Initialize stack and add to thread pool
    thread1.start(1, &task1);
    thread2.start(5, &task2);
    thread3.start(13, &task3);

    OS_run();
}
