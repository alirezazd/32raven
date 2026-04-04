#!/usr/bin/env python3

# /// script
# dependencies = [
#     "pillow",
# ]
# ///

from __future__ import annotations

import argparse
import pathlib
import re
from typing import Iterable

from PIL import Image, ImageDraw, ImageFilter, ImageOps


DEFAULT_VISIBLE_WIDTH = 72
DEFAULT_VISIBLE_HEIGHT = 40
DEFAULT_CANVAS_WIDTH = 128
DEFAULT_CANVAS_HEIGHT = 64
DEFAULT_OFFSET_X = 28
DEFAULT_OFFSET_Y = 24


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert a PNG into a tiny OLED asset set: visible monochrome image, "
            "preview image, full controller canvas image, and optional C++ header."
        )
    )
    parser.add_argument("input", type=pathlib.Path, help="Input PNG path")
    parser.add_argument(
        "--mono-out",
        type=pathlib.Path,
        help="Visible monochrome PNG output path",
    )
    parser.add_argument(
        "--preview-out",
        type=pathlib.Path,
        help="OLED-emulated preview PNG output path",
    )
    parser.add_argument(
        "--frame-out",
        type=pathlib.Path,
        help="Full controller-canvas monochrome PNG output path",
    )
    parser.add_argument(
        "--header-out",
        type=pathlib.Path,
        help="Generated C++ header output path",
    )
    parser.add_argument(
        "--name",
        default="boot_logo",
        help="Namespace/asset name used in the generated header",
    )
    parser.add_argument(
        "--visible-width",
        type=int,
        default=DEFAULT_VISIBLE_WIDTH,
        help="Visible OLED width in pixels",
    )
    parser.add_argument(
        "--visible-height",
        type=int,
        default=DEFAULT_VISIBLE_HEIGHT,
        help="Visible OLED height in pixels",
    )
    parser.add_argument(
        "--canvas-width",
        type=int,
        default=DEFAULT_CANVAS_WIDTH,
        help="Full controller canvas width in pixels",
    )
    parser.add_argument(
        "--canvas-height",
        type=int,
        default=DEFAULT_CANVAS_HEIGHT,
        help="Full controller canvas height in pixels",
    )
    parser.add_argument(
        "--offset-x",
        type=int,
        default=DEFAULT_OFFSET_X,
        help="Visible image X offset within the controller canvas",
    )
    parser.add_argument(
        "--offset-y",
        type=int,
        default=DEFAULT_OFFSET_Y,
        help="Visible image Y offset within the controller canvas",
    )
    parser.add_argument(
        "--threshold",
        type=int,
        default=128,
        help="Monochrome threshold in the range 0..255",
    )
    parser.add_argument(
        "--invert",
        action="store_true",
        help="Invert the grayscale image before monochrome conversion",
    )
    parser.add_argument(
        "--alpha-bg",
        choices=("black", "white"),
        default="black",
        help="Background color used to flatten transparent pixels",
    )
    parser.add_argument(
        "--dither",
        action="store_true",
        help="Use Floyd-Steinberg dithering instead of a hard threshold",
    )
    parser.add_argument(
        "--fit",
        choices=("contain", "cover", "stretch"),
        default="contain",
        help="How to resize the source image into the visible OLED area",
    )
    parser.add_argument(
        "--trim-alpha",
        action="store_true",
        help="Crop transparent margins before resizing",
    )
    parser.add_argument(
        "--pixel-size",
        type=int,
        default=8,
        help="Preview pixel size for the OLED emulation image",
    )
    parser.add_argument(
        "--pixel-gap",
        type=int,
        default=1,
        help="Preview pixel gap for the OLED emulation image",
    )
    return parser.parse_args()


def sanitize_identifier(value: str) -> str:
    ident = re.sub(r"\W+", "_", value).strip("_").lower()
    if not ident:
        return "image_asset"
    if ident[0].isdigit():
        ident = f"asset_{ident}"
    return ident


def flatten_to_grayscale(path: pathlib.Path, alpha_bg: str,
                         trim_alpha: bool) -> Image.Image:
    img = Image.open(path).convert("RGBA")
    if trim_alpha:
        bbox = img.getchannel("A").getbbox()
        if bbox is not None:
            img = img.crop(bbox)
    bg_color = (255, 255, 255, 255) if alpha_bg == "white" else (0, 0, 0, 255)
    bg = Image.new("RGBA", img.size, bg_color)
    return Image.alpha_composite(bg, img).convert("L")


def resize_visible(img: Image.Image, target_w: int, target_h: int,
                   fit_mode: str) -> Image.Image:
    if fit_mode == "stretch":
        return img.resize((target_w, target_h), Image.Resampling.LANCZOS)

    src_w, src_h = img.size
    if fit_mode == "contain":
        scale = min(target_w / src_w, target_h / src_h)
    else:
        scale = max(target_w / src_w, target_h / src_h)

    new_w = max(1, round(src_w * scale))
    new_h = max(1, round(src_h * scale))
    resized = img.resize((new_w, new_h), Image.Resampling.LANCZOS)

    if fit_mode == "contain":
        canvas = Image.new("L", (target_w, target_h), color=0)
        pos = ((target_w - new_w) // 2, (target_h - new_h) // 2)
        canvas.paste(resized, pos)
        return canvas

    left = max(0, (new_w - target_w) // 2)
    top = max(0, (new_h - target_h) // 2)
    return resized.crop((left, top, left + target_w, top + target_h))


def to_mono(img: Image.Image, threshold: int, invert: bool,
            dither: bool) -> Image.Image:
    gray = img.convert("L")
    if invert:
        gray = ImageOps.invert(gray)
    if dither:
        return gray.convert("1")
    return gray.point(lambda p: 255 if p >= threshold else 0, mode="1")


def build_canvas(visible_img: Image.Image, canvas_w: int, canvas_h: int,
                 offset_x: int, offset_y: int) -> Image.Image:
    vis_w, vis_h = visible_img.size
    if canvas_h % 8 != 0:
        raise ValueError("canvas height must be divisible by 8 for page packing")
    if offset_x < 0 or offset_y < 0:
        raise ValueError("offsets must be non-negative")
    if offset_x + vis_w > canvas_w or offset_y + vis_h > canvas_h:
        raise ValueError("visible image does not fit within the controller canvas")

    canvas = Image.new("1", (canvas_w, canvas_h), color=0)
    canvas.paste(visible_img, (offset_x, offset_y))
    return canvas


def pack_pages(img: Image.Image) -> list[int]:
    width, height = img.size
    pixels = img.convert("1")
    data: list[int] = []

    for page in range((height + 7) // 8):
        page_y = page * 8
        for x in range(width):
            value = 0
            for bit in range(8):
                if page_y + bit >= height:
                    continue
                if pixels.getpixel((x, page_y + bit)) != 0:
                    value |= 1 << bit
            data.append(value)
    return data


def render_oled_preview(
    mono_img: Image.Image,
    pixel_size: int,
    pixel_gap: int,
    on_color: tuple[int, int, int] = (80, 220, 255),
    off_color: tuple[int, int, int] = (8, 12, 18),
    bg_color: tuple[int, int, int] = (0, 0, 0),
) -> Image.Image:
    width, height = mono_img.size
    out_w = width * pixel_size + (width - 1) * pixel_gap
    out_h = height * pixel_size + (height - 1) * pixel_gap

    out = Image.new("RGB", (out_w, out_h), bg_color)
    draw = ImageDraw.Draw(out)
    mono = mono_img.convert("1")

    for y in range(height):
        for x in range(width):
            px = x * (pixel_size + pixel_gap)
            py = y * (pixel_size + pixel_gap)
            color = on_color if mono.getpixel((x, y)) != 0 else off_color
            draw.rectangle(
                [px, py, px + pixel_size - 1, py + pixel_size - 1],
                fill=color,
            )

    glow_layer = Image.new("RGB", out.size, (0, 0, 0))
    glow_draw = ImageDraw.Draw(glow_layer)
    for y in range(height):
        for x in range(width):
            if mono.getpixel((x, y)) == 0:
                continue
            px = x * (pixel_size + pixel_gap)
            py = y * (pixel_size + pixel_gap)
            glow_draw.rectangle(
                [px, py, px + pixel_size - 1, py + pixel_size - 1],
                fill=on_color,
            )

    glow_layer = glow_layer.filter(
        ImageFilter.GaussianBlur(radius=max(1, pixel_size // 3))
    )
    return Image.blend(out, glow_layer, 0.35)


def format_hex_lines(values: Iterable[int], indent: str = "    ",
                     per_line: int = 12) -> str:
    items = [f"0x{value:02X}" for value in values]
    lines = []
    for start in range(0, len(items), per_line):
        lines.append(indent + ", ".join(items[start:start + per_line]) + ",")
    return "\n".join(lines)


def emit_header(
    output_path: pathlib.Path,
    namespace_name: str,
    source_path: pathlib.Path,
    visible_w: int,
    visible_h: int,
    canvas_w: int,
    canvas_h: int,
    offset_x: int,
    offset_y: int,
    page_data: list[int],
) -> None:
    header = f"""#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// Generated by tools/png_conv.py from {source_path.name}
namespace {namespace_name} {{

inline constexpr std::size_t kVisibleWidth = {visible_w};
inline constexpr std::size_t kVisibleHeight = {visible_h};
inline constexpr std::size_t kCanvasWidth = {canvas_w};
inline constexpr std::size_t kCanvasHeight = {canvas_h};
inline constexpr std::size_t kOffsetX = {offset_x};
inline constexpr std::size_t kOffsetY = {offset_y};
inline constexpr std::size_t kVisiblePageCount = {(visible_h + 7) // 8};
inline constexpr std::array<std::uint8_t, {len(page_data)}> kBitmapData = {{
{format_hex_lines(page_data)}
}};

}}  // namespace {namespace_name}
"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(header)


def default_path(input_path: pathlib.Path, suffix: str,
                 extension: str) -> pathlib.Path:
    return input_path.with_name(f"{input_path.stem}{suffix}{extension}")


def main() -> int:
    args = parse_args()
    if not args.input.is_file():
        raise FileNotFoundError(f"input PNG not found: {args.input}")
    if not 0 <= args.threshold <= 255:
        raise ValueError("threshold must be in the range 0..255")
    if args.visible_width <= 0 or args.visible_height <= 0:
        raise ValueError("visible width and height must be positive")
    if args.canvas_width <= 0 or args.canvas_height <= 0:
        raise ValueError("canvas width and height must be positive")

    mono_out = args.mono_out or default_path(args.input, "_mono", ".png")
    preview_out = args.preview_out or default_path(args.input, "_preview", ".png")
    frame_out = args.frame_out or default_path(args.input, "_frame", ".png")

    gray = flatten_to_grayscale(args.input, args.alpha_bg, args.trim_alpha)
    visible_gray = resize_visible(
        gray, args.visible_width, args.visible_height, args.fit
    )
    visible_mono = to_mono(
        visible_gray, args.threshold, args.invert, args.dither
    )
    canvas_mono = build_canvas(
        visible_mono,
        args.canvas_width,
        args.canvas_height,
        args.offset_x,
        args.offset_y,
    )
    preview = render_oled_preview(
        visible_mono, args.pixel_size, args.pixel_gap
    )
    visible_page_data = pack_pages(visible_mono)

    mono_out.parent.mkdir(parents=True, exist_ok=True)
    preview_out.parent.mkdir(parents=True, exist_ok=True)
    frame_out.parent.mkdir(parents=True, exist_ok=True)
    visible_mono.convert("L").save(mono_out)
    preview.save(preview_out)
    canvas_mono.convert("L").save(frame_out)

    if args.header_out is not None:
        emit_header(
            args.header_out,
            sanitize_identifier(args.name),
            args.input,
            args.visible_width,
            args.visible_height,
            args.canvas_width,
            args.canvas_height,
            args.offset_x,
            args.offset_y,
            visible_page_data,
        )
        print(f"saved header: {args.header_out}")

    print(f"saved mono: {mono_out}")
    print(f"saved preview: {preview_out}")
    print(f"saved frame: {frame_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
