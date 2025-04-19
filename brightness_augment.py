import cv2
import numpy as np
from pathlib import Path
import shutil

sequences_dir = Path('datasets/kitti_grey/sequences')
original_dir = sequences_dir / '06'
original_image_dir = original_dir / 'image_0'

# img = cv2.imread('datasets/kitti_grey/sequences/06/image_0/000000.png')
# cv2.imshow('img', ((img / 255)**3 * 255).astype(np.uint8))
# cv2.waitKey(-1)
# exit()

brightness_dir = sequences_dir / '06_brightness'
brightness_image_dir = brightness_dir / 'image_0'
brightness_image_dir.mkdir(exist_ok=True, parents=True)

for img_path in original_image_dir.iterdir():
    new_img_path = brightness_image_dir / img_path.name
    if int(img_path.stem) <= 700:
        shutil.copy(img_path, new_img_path)
    else:
        img = cv2.imread(str(img_path))
        img = ((img / 255)**3 * 255).astype(np.uint8)
        cv2.imwrite(str(new_img_path), img)

shutil.copy(original_dir / 'times.txt', brightness_dir)
shutil.copy(original_dir / 'calib.txt', brightness_dir)
