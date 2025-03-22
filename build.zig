const std = @import("std");
const microzig = @import("microzig");

const MicroBuild = microzig.MicroBuild(.{ .rp2xxx = true });

pub fn build(b: *std.Build) void {
    const mz_dep = b.dependency("microzig", .{});
    const mz = MicroBuild.init(b, mz_dep) orelse unreachable;

    const optimize = b.standardOptimizeOption(.{});
    const firmware = mz.add_firmware(.{
        .name = "miros",
        .target = mz.ports.rp2xxx.boards.raspberrypi.pico,
        .optimize = optimize,
        .root_source_file = b.path("src/main.zig"),
    });

    mz.install_firmware(firmware, .{});
    mz.install_firmware(firmware, .{ .format = .elf });
}
