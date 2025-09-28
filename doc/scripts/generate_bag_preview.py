#!/usr/bin/env python3
"""Generate a short preview clip from a rosbag image topic."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image

from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="Path to rosbag directory")
    parser.add_argument(
        "--topic",
        help="Image topic to export (defaults to the first sensor_msgs/msg/Image topic)",
    )
    parser.add_argument(
        "--storage-id",
        default="mcap",
        help="rosbag storage id (default: mcap)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("doc/media/julia_readme.mp4"),
        help="Output video path",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=960,
        help="Target width for downscaling (height keeps aspect ratio)",
    )
    parser.add_argument(
        "--frame-limit",
        type=int,
        default=54,
        help="Maximum number of frames to export",
    )
    parser.add_argument(
        "--frame-stride",
        type=int,
        default=1,
        help="Export every Nth frame (default: 1)",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=18.0,
        help="Frames per second in the resulting clip",
    )
    parser.add_argument(
        "--crf",
        type=int,
        default=23,
        help="FFmpeg CRF value for libx264 (lower is higher quality)",
    )
    return parser.parse_args()


def find_image_topic(reader: SequentialReader, preferred: str | None) -> str:
    topics = reader.get_all_topics_and_types()
    if preferred:
        for info in topics:
            if info.name == preferred:
                return preferred
        raise RuntimeError(f"Topic '{preferred}' not found in bag")

    for info in topics:
        if info.type == "sensor_msgs/msg/Image":
            return info.name
    raise RuntimeError("No sensor_msgs/msg/Image topic found; use --topic explicitly")


def extract_frames(
    bag_path: Path,
    storage_id: str,
    topic_name: str,
    frame_limit: int,
    frame_stride: int,
    target_width: int,
    temp_dir: Path,
) -> tuple[int, Path]:
    storage_options = StorageOptions(uri=str(bag_path), storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topics = reader.get_all_topics_and_types()
    msg_type = get_message(next(info.type for info in topics if info.name == topic_name))

    saved = 0
    total = 0
    frame_idx = 0

    while reader.has_next() and saved < frame_limit:
        topic, data, _timestamp = reader.read_next()
        if topic != topic_name:
            continue

        if frame_idx % frame_stride != 0:
            frame_idx += 1
            continue

        msg = deserialize_message(data, msg_type)
        row_stride = msg.step
        if row_stride % msg.width != 0:
            raise RuntimeError(
                f"Row stride {row_stride} is not divisible by width {msg.width}"
            )
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        arr = raw.reshape(msg.height, row_stride)
        channels = row_stride // msg.width
        arr = arr[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
        if arr.shape[2] == 1:
            arr = np.repeat(arr, 3, axis=2)

        scale = target_width / arr.shape[1]
        target_height = int(round(arr.shape[0] * scale))
        img = Image.fromarray(arr[:, :, :3], mode="RGB")
        img = img.resize((target_width, target_height), Image.BICUBIC)
        img.save(temp_dir / f"frame_{saved:03d}.png")

        saved += 1
        frame_idx += 1
        total += 1

    if saved == 0:
        raise RuntimeError("No frames matched the requested topic and filters")

    return saved, temp_dir / "frame_%03d.png"


def encode_video(image_pattern: Path, fps: float, crf: int, output: Path) -> None:
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg not found in PATH")

    output.parent.mkdir(parents=True, exist_ok=True)

    cmd = [
        "ffmpeg",
        "-y",
        "-framerate",
        str(fps),
        "-i",
        str(image_pattern),
        "-c:v",
        "libx264",
        "-crf",
        str(crf),
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output),
    ]

    subprocess.run(cmd, check=True)


def main() -> int:
    args = parse_args()

    if args.frame_limit <= 0:
        raise SystemExit("--frame-limit must be positive")
    if args.frame_stride <= 0:
        raise SystemExit("--frame-stride must be positive")
    if args.width <= 0:
        raise SystemExit("--width must be positive")

    with tempfile.TemporaryDirectory(prefix="bag_preview_") as tmpdir:
        tmp_path = Path(tmpdir)

        storage_options = StorageOptions(uri=str(args.bag), storage_id=args.storage_id)
        converter_options = ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )
        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        topic_name = find_image_topic(reader, args.topic)

        saved, pattern = extract_frames(
            bag_path=args.bag,
            storage_id=args.storage_id,
            topic_name=topic_name,
            frame_limit=args.frame_limit,
            frame_stride=args.frame_stride,
            target_width=args.width,
            temp_dir=tmp_path,
        )

        encode_video(pattern, args.fps, args.crf, args.output)

    print(f"Saved {saved} frames from '{topic_name}' to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
