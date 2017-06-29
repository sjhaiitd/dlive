import cv2
import numpy as np


class Polydrawer:
    def draw(self, warp_img, left_fit, right_fit, Minv):
        color_warp = np.zeros_like(warp_img).astype(np.uint8)

        fity = np.linspace(0, warp_img.shape[0] - 1, warp_img.shape[0])
        left_fitx = left_fit[0] * fity ** 2 + left_fit[1] * fity + left_fit[2]
        right_fitx = right_fit[0] * fity ** 2 + right_fit[1] * fity + right_fit[2]

        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, fity]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, fity])))])
        pts = np.hstack((pts_left, pts_right))
        pts = np.array(pts, dtype=np.int32)

        cv2.fillPoly(color_warp, pts, (255, 255, 255))

        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(color_warp, Minv, (warp_img.shape[1], warp_img.shape[0]))
        # Combine the result with the original image
        result = cv2.addWeighted(warp_img, 0.3, newwarp, 0.7, 0)

        return result
