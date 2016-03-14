import numpy as np
import cv2
from matplotlib import pyplot as plt

camera_r = cv2.VideoCapture(0)
camera_l = cv2.VideoCapture(1)

# grab the current frames
(grabbed, frame_r) = camera_r.read()
(grabbed, frame_l) = camera_l.read()

# Convert color images to gray for disparity map
frame_r = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)
frame_l = cv2.cvtColor(frame_l, cv2.COLOR_BGR2GRAY)

print frame_r.shape
print frame_l.shape

rot_frame_r = np.zeros((frame_r.shape[1],frame_r.shape[0]), np.uint8)
rot_frame_l = np.zeros((frame_l.shape[1],frame_l.shape[0]), np.uint8)

print rot_frame_r.shape
print rot_frame_l.shape

cv2.transpose(frame_r, rot_frame_r);
print rot_frame_r.shape
cv2.flip(rot_frame_r, rot_frame_r, 0);
print rot_frame_r.shape

cv2.transpose(frame_l, rot_frame_l);
print rot_frame_l.shape
cv2.flip(rot_frame_l, rot_frame_l, 1);
print rot_frame_l.shape

# rows_r, cols_r = frame_r.shape
# rows_l, cols_l = frame_l.shape

# M_r = cv2.getRotationMatrix2D((cols_r/2,rows_r/2),270,1)
# M_l = cv2.getRotationMatrix2D((cols_l/2,rows_l/2),90,1)

# rot_frame_r = cv2.warpAffine(frame_r,M_r,(cols_r,rows_l))
# rot_frame_l = cv2.warpAffine(frame_l,M_l,(cols_l,rows_l))

cv2.imshow("Right Image", rot_frame_r)
cv2.imshow("Left Image", rot_frame_l)

stereo = cv2.StereoBM(cv2.STEREO_BM_BASIC_PRESET, ndisparities=32, SADWindowSize=15)
disparity = stereo.compute(rot_frame_l, rot_frame_r)

plt.imshow(disparity,'gray')
plt.show()