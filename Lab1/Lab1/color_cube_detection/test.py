from count_cubes import *
import numpy as np
from glob import glob


yellow_lower = np.array([11, 168, 131])
yellow_upper = np.array([179, 255, 255])
green_lower = np.array([17, 0, 0])
green_upper = np.array([179, 255, 255])

datapath = 'data/img01.jpg'
# 'data/img33.jpg'
# 'data/img09.jpg'

# Capture Image
image = cv2.imread(datapath)

# Remove Noise (Image smoothing)
img = cv2.GaussianBlur(image, (11, 11), cv2.BORDER_DEFAULT)
#img = cv2.bilateralFilter(image, 21, 21 * 2, 21 / 2)

# Convert to HSV color space
hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Filter by Color (Color filtering)
mask = cv2.inRange(hsv_image, green_lower, green_upper)
#mask = cv2.inRange(hsv_image, yellow_lower, yellow_upper)
ret, mask = cv2.threshold(mask, 25, 255, cv2.THRESH_BINARY_INV)

masked_image = cv2.bitwise_and(img, img, mask=mask)

# Detect blobs (Blob detection)
# Set up the SimpleBlobdetector with default parameters with specific values.
params = cv2.SimpleBlobDetector_Params()
params.filterByArea = True
params.minArea = 750
params.filterByCircularity = False
params.filterByConvexity = False
params.filterByInertia = False

# builds a blob detector with the given parameters 
detector = cv2.SimpleBlobDetector_create(params)

# use the detector to detect blobs.
keypoints = detector.detect(mask)
blobs = cv2.drawKeypoints(image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
print(len(keypoints))

cv2.imshow("Masked_Image", masked_image)
cv2.imshow("Mask", mask)
cv2.imshow("GaussianBlurred Image", img)
cv2.imshow("Blobs", blobs)
cv2.waitKey(0)
cv2.destroyAllWindows()