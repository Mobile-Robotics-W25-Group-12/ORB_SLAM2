from typing import NamedTuple
import cv2
import numpy as np
from pathlib import Path
import shutil
import argparse

def map_range(x: float, in_start: float, in_end: float, out_start: float, out_end: float) -> float:
    in_span = in_end - in_start
    out_span = out_end - out_start
    scaled = (x - in_start) / in_span
    return out_start + (scaled * out_span)

def augment(kitti_sequences_dir: str, input_sequence_name: str, output_sequence_name: str, dark_frame: int, gamma: float):
    sequences_dir = Path(kitti_sequences_dir)
    original_dir = sequences_dir / input_sequence_name
    original_image_dir = original_dir / "image_0"

    brightness_dir = sequences_dir / output_sequence_name
    brightness_image_dir = brightness_dir / "image_0"
    brightness_image_dir.mkdir(parents=True)

    num_files = len(list(original_image_dir.iterdir()))
    for i in range(num_files):
        img_name = f"{i:06}.png"
        img_path = original_image_dir / img_name
        new_img_path = brightness_image_dir / img_name
        if i >= dark_frame:
            faded_gamma = min(gamma, map_range(i, dark_frame, dark_frame + 15, 1, gamma))
            img = cv2.imread(str(img_path))
            img = ((img / 255) ** faded_gamma * 255).astype(np.uint8)
            cv2.imwrite(str(new_img_path), img)
        else:
            shutil.copy(img_path, new_img_path)

    shutil.copy(original_dir / "times.txt", brightness_dir)
    shutil.copy(original_dir / "calib.txt", brightness_dir)


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter, description=(
        "Augment brightness of a KITTI sequence.\n"
        "Example: python3 brightness_augment.py datasets/kitti/sequences 06 06_brightness 700 -g 2"
    ))
    parser.add_argument("kitti_sequences_dir", type=str, help="Path to KITTI sequences directory.")
    parser.add_argument("input_sequence_name", type=str, help="Name of the input sequence in the sequences directory")
    parser.add_argument("output_sequence_name", type=str, help="Name of the output sequence (to be placed in the sequences directory)")
    parser.add_argument("dark_frame", type=int, help="Frame ID from which to start darkening")
    parser.add_argument("-g", "--gamma", type=float, default=3, help="Parameter for gamma correction. Gamma > 1 darkens")
    
    args = parser.parse_args()

    augment(args.kitti_sequences_dir, args.input_sequence_name, args.output_sequence_name, args.dark_frame, args.gamma)


if __name__ == '__main__':
    main()
