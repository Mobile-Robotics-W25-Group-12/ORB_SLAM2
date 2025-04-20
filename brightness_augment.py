from typing import NamedTuple
import cv2
import numpy as np
from pathlib import Path
import shutil

kitti_sequence = "06"

sequences_dir = Path("datasets/kitti_grey/sequences")
original_dir = sequences_dir / kitti_sequence
original_image_dir = original_dir / "image_0"

# img = cv2.imread('datasets/kitti_grey/sequences/06/image_0/000000.png')
# cv2.imshow('img', ((img / 255)**3 * 255).astype(np.uint8))
# cv2.waitKey(-1)
# exit()

brightness_dir = sequences_dir / f"{kitti_sequence}_brightness_test"
brightness_image_dir = brightness_dir / "image_0"
brightness_image_dir.mkdir(exist_ok=True, parents=True)


class Range(NamedTuple):
    start: int = None
    end: int = None

    def contains(self, val: int) -> bool:
        return (self.start is None or val >= self.start) and (
            self.end is None or val <= self.end
        )

dark_ranges = [Range(701, None)]

gamma = 3

for img_path in original_image_dir.iterdir():
    new_img_path = brightness_image_dir / img_path.name
    if any([range_.contains(int(img_path.stem)) for range_ in dark_ranges]):
        img = cv2.imread(str(img_path))
        img = ((img / 255) ** gamma * 255).astype(np.uint8)
        cv2.imwrite(str(new_img_path), img)
    else:
        shutil.copy(img_path, new_img_path)

shutil.copy(original_dir / "times.txt", brightness_dir)
shutil.copy(original_dir / "calib.txt", brightness_dir)
