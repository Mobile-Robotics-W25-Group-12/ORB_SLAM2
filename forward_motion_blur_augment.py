import cv2
import numpy as np
from pathlib import Path
import shutil
import random

sequences_dir = Path('datasets/kitti_grey/sequences')
original_dir = sequences_dir / '06'
original_image_dir = original_dir / 'image_0'

def forward_motion_blur(img, strength=0.07, steps=20):
    h, w = img.shape[:2]
    center = (w / 2, h / 2)
    accum = np.zeros_like(img, dtype=np.float32)

    for i in range(steps):
        alpha = i / (steps - 1) 
        scale = 1.0 + strength * alpha
        M = cv2.getRotationMatrix2D(center, angle=0, scale=scale)

        warped = cv2.warpAffine(img, M, (w, h), flags=cv2.INTER_LINEAR)
        accum += warped.astype(np.float32)

    blurred = (accum / steps).astype(np.uint8)
    return blurred

# img = cv2.imread('datasets/kitti_grey/sequences/06/image_0/000000.png')
# cv2.imshow('img', motion_blur(img))
# cv2.waitKey(-1)
# exit()

brightness_dir = sequences_dir / '06_fwdblur'
brightness_image_dir = brightness_dir / 'image_0'
brightness_image_dir.mkdir(exist_ok=True, parents=True)

for img_path in sorted(original_image_dir.iterdir(),
                       key=lambda p: int(p.stem)):
    new_img_path = brightness_image_dir / img_path.name
    if int(img_path.stem) <= 500:
        shutil.copy(img_path, new_img_path)
    else:
        img = cv2.imread(str(img_path))
        img = forward_motion_blur(img)

        cv2.imwrite(str(new_img_path), img)

shutil.copy(original_dir / 'times.txt', brightness_dir)
shutil.copy(original_dir / 'calib.txt', brightness_dir)
