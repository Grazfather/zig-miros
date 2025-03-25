const std = @import("std");

extern fn OS_onStartup() void;
extern fn OS_onIdle() void;

pub const Thread = extern struct {
    const Self = @This();
    sp: *volatile u32 = undefined,
    spi: usize = 0,
    timeout: u32 = 0,
    priority: u8 = 0,
    stack: [*]u32 = undefined,
    // TODO: Make sure sp is 8 byte aligned as well?
    // Maybe by making sure the size is a multiple of 8

    pub fn init(self: *Self, stack: []u32) void {
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

    pub fn start(self: *Self, prio: u8, handler: *const fn () callconv(.c) noreturn) void {
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

pub fn init(stack: []u32) void {
    // Set PendSV priority to the lowest possible
    const SHPR3: *volatile u32 = @ptrFromInt(0xE000ED20);
    SHPR3.* |= 0xFF << 16;

    idle_thread.init(stack);
    idle_thread.start(0, idle_task);
}

pub fn run() noreturn {
    // Must be provided by client;
    OS_onStartup();
    // Kick off the 'first' scheduling of tasks
    sched();
    unreachable;
}

// Must be called from an interrupt handler, where interrupts are disabled
pub fn sched() void {
    // If nothing is ready, schedule the idle thread
    if (OS_readySet == 0) {
        OS_next = OS_threads[0].?;
    } else {
        const idx = get_top_bit_index(OS_readySet);
        OS_next = OS_threads[idx].?;
    }

    // If we have a new thread to schedule, make PendSV execute
    if (OS_next != OS_curr) {
        const ICSR: *volatile u32 = @ptrFromInt(0xE000ED04);
        ICSR.* = 1 << 28;
    }
}

pub fn delay(ticks: u32) void {
    // disable interrupts
    asm volatile ("cpsid i");
    if (OS_curr == OS_threads[0]) @panic("cannot call OS_delay from idle thread");

    const bit = (@as(u32, 1) << @truncate(OS_curr.priority - 1));
    // Set current thread as blocked
    OS_curr.timeout = ticks;
    // Clear ready bit
    OS_readySet &= ~bit;
    // Set delayed bit
    OS_delayedSet |= bit;
    // Call the scheduler
    sched();
    // enable interrupts
    asm volatile ("cpsie i");
}

pub fn tick() void {
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

fn idle_task() callconv(.c) noreturn {
    while (true) {
        OS_onIdle();
    }
}

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

var OS_threads: [32 + 1]?*Thread = undefined;
var OS_readySet: u32 = 0;
var OS_delayedSet: u32 = 0;

export var OS_curr: *Thread = undefined;
export var OS_next: *Thread = undefined;

// NOTE: Client code provides the stack
var idle_thread = Thread{};
