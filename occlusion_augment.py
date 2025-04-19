import cv2
import numpy as np
from pathlib import Path
import shutil
import random

sequences_dir = Path('datasets/kitti_grey/sequences')
original_dir = sequences_dir / '06'
original_image_dir = original_dir / 'image_0'

BOX_SIZE_MIN = 20
BOX_SIZE_MAX = 80
NUM_BOXES = 60

random.seed(568)

def gen_occlusions(shape):
    occs = []
    h, w = shape[:2]

    for _ in range(NUM_BOXES):
        for i in range(20):
            rw = random.randint(BOX_SIZE_MIN, BOX_SIZE_MAX)
            rh = random.randint(BOX_SIZE_MIN, BOX_SIZE_MAX)
            x1 = random.randint(0, w - rw)
            y1 = random.randint(0, h - rh)
            rect = (x1, y1, x1 + rw, y1 + rh)

            overlaps = any(
                        not (rect[2] <= r[0] or rect[0] >= r[2] or rect[3] <= r[1] or rect[1] >= r[3])
                        for r in occs
                    )
            
            if overlaps:
                continue

            occs.append(rect)
            break
    
    return occs


def draw_occlusions(img, occs):
    for o in occs:


        k = 103
        roi = img[o[1]:o[3], o[0]:o[2]]
        blurred = cv2.GaussianBlur(roi, (k, k), sigmaX=0)
        img[o[1]:o[3], o[0]:o[2]] = blurred

    return img

img0 = cv2.imread('datasets/kitti_grey/sequences/06/image_0/000000.png')
occs = gen_occlusions(img0.shape)

# cv2.imshow('img', draw_occlusions(img0, occs))
# cv2.waitKey(-1)
# exit()

brightness_dir = sequences_dir / '06_occ'
brightness_image_dir = brightness_dir / 'image_0'
brightness_image_dir.mkdir(exist_ok=True, parents=True)



for img_path in sorted(original_image_dir.iterdir(),
                       key=lambda p: int(p.stem)):
    new_img_path = brightness_image_dir / img_path.name
    if int(img_path.stem) <= 500:
        shutil.copy(img_path, new_img_path)
    else:
        img = cv2.imread(str(img_path))
        img = draw_occlusions(img, occs)

        cv2.imwrite(str(new_img_path), img)

shutil.copy(original_dir / 'times.txt', brightness_dir)
shutil.copy(original_dir / 'calib.txt', brightness_dir)
