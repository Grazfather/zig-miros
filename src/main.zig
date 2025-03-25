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

const miros = @import("./miros.zig");

pub const microzig_options: microzig.Options = .{
    .log_level = .debug,
    .logFn = rp2xxx.uart.logFn,
    // Set the handlers
    .interrupts = switch (rp2xxx.compatibility.chip) {
        .RP2040 => .{
            .PendSV = miros.pendsv_handler,
            .SysTick = systick_handler,
        },
        .RP2350 => .{
            .PendSV = miros.pendsv_handler,
            .SysTick = systick_handler,
        },
    },
};

var stack1: [0x800]u32 = @splat(0xdeadbeef);
var stack2: [0x800]u32 = undefined;
var stack3: [0x800]u32 = undefined;
// Allow setting up a startup delay
var thread1 = miros.OSThread{ .timeout = 500 };
var thread2 = miros.OSThread{};
var thread3 = miros.OSThread{};

pub fn systick_handler() callconv(.c) void {
    miros.OS_tick();
    const cs = microzig.interrupt.enter_critical_section();
    defer cs.leave();

    miros.OS_sched();
}

fn task1() callconv(.c) noreturn {
    while (true) {
        std.log.info("In task 1", .{});
        led1.toggle();
        miros.OS_delay(500);
    }
}

fn task2() callconv(.c) noreturn {
    while (true) {
        std.log.info("In task 2", .{});
        led2.toggle();
        miros.OS_delay(1000);
    }
}

fn task3() callconv(.c) noreturn {
    while (true) {
        std.log.info("In task 3", .{});
        led.toggle();
        miros.OS_delay(750);
    }
}

export fn OS_onStartup() void {
    // Set the priority of SysTick to the highest (which should be the default)
    peripherals.PPB.SHPR3.modify(.{ .PRI_15 = 0 });
    // Enable the SysTick interrupt manually
    peripherals.PPB.SYST_CSR.modify(.{ .ENABLE = 1, .TICKINT = 1, .CLKSOURCE = 1 });
    // RVR = Reload Value Register
    // 1ms (125MHz / 125KMz)
    peripherals.PPB.SYST_RVR.modify(.{ .RELOAD = 125_000 - 1 });
}

export fn OS_onIdle() void {
    asm volatile ("wfi");
}

var idle_stack: [0x100]u32 = undefined;

pub fn main() noreturn {
    // init uart logging
    uart_tx_pin.set_function(.uart);
    uart.apply(.{ .baud_rate = baud_rate, .clock_config = rp2xxx.clock_config });
    rp2xxx.uart.init_logger(uart);

    inline for (.{ led, led1, led2 }) |l| {
        l.set_function(.sio);
        l.set_direction(.out);
    }

    miros.OS_init(&idle_stack);

    // Assign stack to threads
    thread1.init(&stack1);
    thread2.init(&stack2);
    thread3.init(&stack3);

    // Initialize stack and add to thread pool
    thread1.start(1, &task1);
    thread2.start(5, &task2);
    thread3.start(13, &task3);

    miros.OS_run();
}
