const std = @import("std");
const Image = @import("Image.zig");
const assert = std.debug.assert;

const Coordinate = struct{
    x: usize = 0,
    y: usize = 0
};


inline fn coordinate_to_idx(coord: Coordinate, width: usize) usize{
    return (coord.x * width) + coord.y;
}

/// Source is required to be grayscale
pub fn local_binary_pattern(image: *Image, lbp: []u8) !void{
    assert(lbp.len == image.data_gray.len);
    const padding = 1;

    const padded_image = try image.pad_gray(padding);
    defer image.smp.free(padded_image);

    const padded_width = image.width + (2 * padding);
    const shifts = [9]u3{ 0, 1, 2, 7, 0, 3, 6, 5, 4 };


    for(padding..image.height + padding )|x|{
        for(padding..image.width + padding)|y|{

            const threshold = padded_image[coordinate_to_idx(.{.x = x, .y = y}, padded_width)];
            var code: u8 = 0;
            inline for(0..3)|i|{
                inline for(0..3)|j|{
                    if(i == padding and j == padding) {
                        continue;
                    }

                    const neighbor_x = x + i - padding;
                    const neighbor_y = y + j - padding;

                    const neighbor_idx = coordinate_to_idx(
                        .{.x = neighbor_x, .y = neighbor_y},
                        padded_width
                    );

                   if (padded_image[neighbor_idx] >= threshold) {
                        code |= @as(u8, 1) << shifts[i * 3 + j];
                    }
                }
                const lbp_idx = coordinate_to_idx(
                    .{.x = x - padding, .y = y-padding},
                    image.width
                );
                lbp[lbp_idx] = code;
            }
        }
    }
}