import cv2
import numpy as np
from pathlib import Path
import shutil
import random

sequences_dir = Path('datasets/kitti_grey/sequences')
original_dir = sequences_dir / '06'
original_image_dir = original_dir / 'image_0'

def motion_blur(image, size, angle):
    kernel = np.zeros((size, size))
    kernel[int((size - 1)/2), :] = np.ones(size)
    
    rot_mat = cv2.getRotationMatrix2D((size / 2 - 0.5, size / 2 - 0.5), angle, 1.0)
    kernel = cv2.warpAffine(kernel, rot_mat, (size, size))

    kernel = kernel / np.sum(kernel)

    blurred = cv2.filter2D(image, -1, kernel)
    return blurred

# img = cv2.imread('datasets/kitti_grey/sequences/06/image_0/000000.png')
# cv2.imshow('img', motion_blur(img))
# cv2.waitKey(-1)
# exit()

brightness_dir = sequences_dir / '06_blur'
brightness_image_dir = brightness_dir / 'image_0'
brightness_image_dir.mkdir(exist_ok=True, parents=True)

# CONSECUTIVE_BLURRED = 8 # 0.8 seconds
BLUR_PROBABILITY = 0.2
blurring = False
blur_count = 0

random.seed(568)

for img_path in sorted(original_image_dir.iterdir(),
                       key=lambda p: int(p.stem)):
    new_img_path = brightness_image_dir / img_path.name
    if int(img_path.stem) <= 500:
        shutil.copy(img_path, new_img_path)
    else:
        img = cv2.imread(str(img_path))
        if not blurring and random.random() < BLUR_PROBABILITY:
            blurring = True
            blur_count = 0
            CONSECUTIVE_BLURRED = random.randint(2, 4)
            
        if blurring:
            img = motion_blur(img, size=20, angle=90)
            blur_count += 1
            if blur_count >= CONSECUTIVE_BLURRED:
                blurring = False
                blur_count = 0

        cv2.imwrite(str(new_img_path), img)

shutil.copy(original_dir / 'times.txt', brightness_dir)
shutil.copy(original_dir / 'calib.txt', brightness_dir)
