import cv2
import numpy as np

#TODO: Modify these values for yellow color range. Add thresholds for detecting green also.
yellow_lower = np.array([11, 168, 131])
yellow_upper = np.array([179, 255, 255])

# Green Color Range
green_lower = np.array([17, 0, 0])
green_upper = np.array([179, 255, 255])


#TODO: Change this function so that it filters the image based on color using the hsv range for each color.
def filter_image(img, hsv_lower, hsv_upper):

    # Modify mask
    image = cv2.GaussianBlur(img, (11, 11), cv2.BORDER_DEFAULT)
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_image, hsv_lower, hsv_upper)

    return mask

    
#TODO: Change the parameters to make blob detection more accurate. Hint: You might need to set some parameters to specify features such as color, size, and shape. The features have to be selected based on the application. 
def detect_blob(mask):
    
    ret, mask = cv2.threshold(mask, 25, 255, cv2.THRESH_BINARY_INV)

    # Set up the SimpleBlobdetector with default parameters with specific values.
    params = cv2.SimpleBlobDetector_Params()

    #ADD CODE HERE
    params.filterByArea = True
    params.minArea = 750
    params.filterByCircularity = True
    params.minCircularity = 0.28
    params.maxCircularity = 0.9
    params.filterByConvexity = False
    params.filterByInertia = False

    # builds a blob detector with the given parameters 
    detector = cv2.SimpleBlobDetector_create(params)

    # use the detector to detect blobs.
    keypoints = detector.detect(mask)
    print(len(keypoints), end=" ")
    return len(keypoints)

    
def count_cubes(img):
    mask_yellow = filter_image(img, yellow_lower, yellow_upper)
    num_yellow = detect_blob(mask_yellow)

    mask_green = filter_image(img, green_lower, green_upper)
    num_green = detect_blob(mask_green)

    #TODO: Modify to return number of detected cubes for both yellow and green (instead of 0)
    return num_yellow, num_green

def nothing(x):
    pass

def hsv_tool(i):
    # Load the image
    datapath = i
    image = cv2.GaussianBlur(cv2.imread(datapath), (11,11), cv2.BORDER_DEFAULT)
    #image = cv2.bilateralFilter(cv2.imread(datapath), 5, 5 * 2, 5 / 2)


    # Create windows for image and mask
    cv2.namedWindow("Original Image")
    cv2.namedWindow("HSV Mask")

    # Create trackbars for HSV values
    cv2.createTrackbar('H', 'HSV Mask', 0, 179, nothing)
    cv2.createTrackbar('S', 'HSV Mask', 0, 255, nothing)
    cv2.createTrackbar('V', 'HSV Mask', 0, 255, nothing)

    while True:
        # Convert image to HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Get current positions of trackbars
        h = cv2.getTrackbarPos('H', 'HSV Mask')
        s = cv2.getTrackbarPos('S', 'HSV Mask')
        v = cv2.getTrackbarPos('V', 'HSV Mask')

        # Update the lower and upper bounds for the mask
        lower_bound = np.array([h, s, v])
        upper_bound = np.array([179, 255, 255])

        # Create the mask
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

        # Apply the mask to the original image
        masked_image = cv2.bitwise_and(image, image, mask=mask)

        # Display the image and mask
        cv2.imshow("Original Image", image)
        cv2.imshow("HSV Mask", masked_image)

        # Check for key press
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyAllWindows()