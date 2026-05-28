from count_cubes import *
import numpy as np
from glob import glob

def nothing(x):
    pass

# Define the HSV thresholds for yellow and green
yellow_lower = np.array([9, 170, 131])
yellow_upper = np.array([179, 255, 255])
green_lower = np.array([16, 0, 0])
green_upper = np.array([179, 255, 255])

# Load the image
datapath = 'data/img0.jpg'
image = cv2.GaussianBlur(cv2.imread(datapath), (11,11), cv2.BORDER_DEFAULT)
#image = cv2.bilateralFilter(cv2.imread(datapath), 5, 5 * 2, 5 / 2)


# Create windows for image and mask
cv2.namedWindow("Image")
cv2.namedWindow("Mask")

# Create trackbars for HSV values
cv2.createTrackbar('H', 'Mask', 0, 179, nothing)
cv2.createTrackbar('S', 'Mask', 0, 255, nothing)
cv2.createTrackbar('V', 'Mask', 0, 255, nothing)

while True:
    # Convert image to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Get current positions of trackbars
    h = cv2.getTrackbarPos('H', 'Mask')
    s = cv2.getTrackbarPos('S', 'Mask')
    v = cv2.getTrackbarPos('V', 'Mask')

    # Update the lower and upper bounds for the mask
    lower_bound = np.array([h, s, v])
    upper_bound = np.array([179, 255, 255])

    # Create the mask
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

    # Apply the mask to the original image
    masked_image = cv2.bitwise_and(image, image, mask=mask)

    # Display the image and mask
    cv2.imshow("Image", image)
    cv2.imshow("Mask", masked_image)

    # Check for key press
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cv2.destroyAllWindows()