import cv2
import numpy as np
import time

def filter_image(img, hsv_lower, hsv_upper):
    img_filt = cv2.GaussianBlur(img, (11, 11), cv2.BORDER_DEFAULT)
    hsv = cv2.cvtColor(img_filt, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    '''cv2.imshow("mask", mask)
    cv2.waitKey(0)'''
    
    return mask

    ###############################################################################
    ### You might need to change the parameter values to get better results
    ###############################################################################
def detect_blob(mask):
    #ret, mask = cv2.threshold(mask, 25, 255, cv2.THRESH_BINARY_INV)
   # Set up the SimpleBlobdetector with default parameters.
    params = cv2.SimpleBlobDetector_Params()
    # Change thresholds
    params.minThreshold = 0
    params.maxThreshold = 256
    #filter by color (on binary)
    params.filterByColor = True
    params.blobColor = 255  # this looks at binary image 0 for looking for dark areas
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 200
    params.maxArea = 20000
    # Filter by Circularity
    params.filterByCircularity = False
    # Filter by Convexity
    params.filterByConvexity = False
    # Filter by Inertia
    params.filterByInertia = False
    detector = cv2.SimpleBlobDetector_create(params)
    # Detect blobs.
    keypoints = detector.detect(mask)
    
    return keypoints

def find_cube(img, hsv_lower, hsv_upper):
    """Find the cube in an image.
        Arguments:
        img -- the image
        hsv_lower -- the h, s, and v lower bounds
        hsv_upper -- the h, s, and v upper bounds
        Returns [x, y, radius] of the target blob, and [0,0,0] or None if no blob is found.
    """
    mask = filter_image(img, hsv_lower, hsv_upper)
    keypoints = detect_blob(mask)

    if len(keypoints) == 0:
        return None
    
    ###############################################################################
    # Todo: Sort the keypoints in a certain way if multiple key points get returned
    ###############################################################################

    # Sort keypoints by radius
    sorted_keypoints = sorted(keypoints, key=lambda keypoint: keypoint.size)

    # Debug code
    '''im_with_keypoints = cv2.drawKeypoints(img, sorted_keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow('image', im_with_keypoints)
    cv2.waitKey(0)'''
    
    return sorted_keypoints[0]

