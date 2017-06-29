import cv2
import numpy as np


class Region_of_interest:
    def roi_mask(self, img):
        mask = np.zeros(img.shape, dtype=np.uint8)
        roi_corners = np.array([[(0, 450), (1280, 450), (1280, 720), (0, 720)]])
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
        cv2.fillPoly(mask, roi_corners, ignore_mask_color)
        # apply the mask
        return cv2.bitwise_and(img, mask)
