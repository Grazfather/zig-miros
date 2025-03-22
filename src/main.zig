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

fn OSThread_size(comptime size: usize) type {
    return extern struct {
        const Self = @This();
        sp: *volatile u32 = undefined,
        spi: usize = size,
        // TODO: Store out of band?
        stack: [size]u32 align(8) = @splat(0xdeadbeef),
        // TODO: Make sure sp is 8 byte aligned as well?
        // Maybe by making sure the size is a multiple of 8

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

        fn pop(self: *Self) !usize {
            const v = self.stack[self.spi];
            // Check for underflow
            if (self.spi == size - 1)
                @panic("Stack underflow");
            self.spi += 1;
            return v;
        }

        fn start(self: *Self, handler: *const fn () callconv(.c) noreturn) void {
            // Push xPSR
            self.push(1 << 24);
            // Push handler
            self.push(handler);
            // Push fake LR, R12, R3, R2, R1, R0, R11, R10, R9, R8, R7, R6, R5, R4
            for ([_]u32{ 13, 12, 3, 2, 1, 0, 11, 10, 9, 8, 7, 6, 5, 4 }) |i| {
                self.push(i);
            }

            // Add to thread list
            // -- Error if there are too many threads
            if (OS_threadNum >= 32)
                @panic("oops");
            OS_threads[OS_threadNum] = self;
            OS_threadNum += 1;
        }
    };
}
const OSThread = OSThread_size(0x1000);
var thread1 = OSThread{};
var thread2 = OSThread{};

pub export var OS_curr: *OSThread = undefined;
pub export var OS_next: *OSThread = undefined;

// TODO: Encapsulate
var OS_threads: [32 + 1]*OSThread = undefined;
var OS_threadNum: usize = 0;
var OS_currIdx: usize = 0;

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
    // TODO: tick ctr increment
    const cs = microzig.interrupt.enter_critical_section();
    defer cs.leave();

    OS_sched();
}

const sleep_ms = rp2xxx.time.sleep_ms;
fn task1() callconv(.c) noreturn {
    while (true) {
        std.log.info("In task 1", .{});
        led1.toggle();
        // TODO: Fix sleep function not working (messing up timers?)
        // sleep_ms(250);

        // busy loop to not involve other timer stuff
        var i: u32 = 0;
        for (0..500000) |_| {
            i += 1;
        }
    }
}

fn task2() callconv(.c) noreturn {
    while (true) {
        std.log.info("In task 2", .{});
        led2.toggle();
        // sleep_ms(1000);
        // busy loop to not involve other timer stuff
        var i: u32 = 0;
        for (0..1000000) |_| {
            i += 1;
        }
    }
}

// Must be called from an interrupt handler, where interrupts are disabled
fn OS_sched() void {
    // TODO: Schedule threads
    if (OS_curr == &thread1)
        OS_next = &thread2
    else
        OS_next = &thread1;

    // If we have a new thread to schedule, make PendSV execute
    if (OS_next != OS_curr) {
        peripherals.PPB.ICSR.modify(.{ .PENDSVSET = 1 });
    }
}

pub fn main() !void {
    // init uart logging
    uart_tx_pin.set_function(.uart);
    uart.apply(.{ .baud_rate = baud_rate, .clock_config = rp2xxx.clock_config });
    rp2xxx.uart.init_logger(uart);

    inline for (.{ led, led1, led2 }) |l| {
        l.set_function(.sio);
        l.set_direction(.out);
    }

    // Initialize stack for each task
    thread1.start(&task1);
    thread2.start(&task2);

    // Schedule a task for next interrupt
    OS_next = &thread2;

    std.log.info("task1 at {*}", .{&task1});
    std.log.info("thread1 at {*}", .{&thread1});

    std.log.info("task2 at {*}", .{&task2});
    std.log.info("thread2 at {*}", .{&thread2});

    std.log.info("systick handler at {*}", .{&systick_handler});
    std.log.info("pendsv handler at {*}", .{&pendsv_handler});

    // TODO: Set in some OS init function
    peripherals.PPB.SHPR3.modify(.{
        // Set priority of PendSV low so it happens last after SysTick
        .PRI_14 = 3,
        // Set priority of SysTick to the highest (which should be the default)
        .PRI_15 = 0,
    });

    // NOTE: To manually kick off interrupt in pendsv: PENDSVSET: write bit 28 of ICSR
    // x/wx 0xe000ed04
    // set {int}0xe000ed04 = 0x1....
    // e.g. set to 0x10000000 (plus preserve old bits)
    // Enable it SysTick interrupt manually
    peripherals.PPB.SYST_CSR.modify(.{ .ENABLE = 1, .TICKINT = 1, .CLKSOURCE = 1 });
    // RVR = Reload Value Register
    // 1ms (125MHz / 125KMz)
    peripherals.PPB.SYST_RVR.modify(.{ .RELOAD = 125_000 - 1 });

    while (true) {
        asm volatile ("wfi");
    }
}
