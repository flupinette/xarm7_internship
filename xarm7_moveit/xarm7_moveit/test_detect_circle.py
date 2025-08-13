#!/usr/bin/env python3
"""
Can the camera recognize circles? This script uses OpenCV to detect circles in a live video stream from the camera.
It uses the Hough Circle Transform method to find circles in the video frames.
It can also be adapted to process a static image if needed.
I do not use ROS here, but it can be integrated into a ROS node if necessary.
The comment at the bottom documents a way to process a static image and another way of trying to detect an object using ellipses.
See the videos folder for a demonstration video.
- ./test_detect_circle.py
"""

import sys
import cv2 as cv
import numpy as np
def main(argv):
    
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)  # Reduce the resolution to 640x480
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv.CAP_PROP_FPS, 15)  # Limit to 15 FPS

    if not cap.isOpened():
        print("Error: Unable to open camera")
        return

    while True:
        ret, src = cap.read()
        if not ret:
            print("Error: Unable to read video stream")
            break
        # Convert to grayscale and blur
        gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
        gray = cv.GaussianBlur(gray, (5,5), 0)

        # edges
        edges = cv.Canny(gray, 50, 150)

        
        # Option A : HoughCircles
        circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, dp=1.2, minDist=50,
                                param1=100, param2=30, minRadius=50, maxRadius=300)
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            # I chose to only keep the biggest circle found : it's what worked the best for me, could be changed depending on the setup
            x,y,r = circles[0]
            # Draw the circle on the frame
            cv.circle(src, (x,y), r, (0,255,0), 4)
            cv.circle(src, (x,y), 2, (0,0,255), 3)
        # Display it
        cv.imshow("Live Circle Detection", src)

        # Quit with 'q'
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()


    """
    default_file = './assets/bowl.PNG'
    filename = argv[0] if len(argv) > 0 else default_file
    # Loads an image
    src = cv.imread(cv.samples.findFile(filename), cv.IMREAD_COLOR)
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_circle.py [image_name -- default ' + default_file + '] \n')
        return -1
    """
    """
    gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    gray = cv.GaussianBlur(gray, (5,5), 0)

    # edges
    edges = cv.Canny(gray, 50, 150)

    
    # Option A : HoughCircles
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, dp=1.2, minDist=50,
                            param1=100, param2=30, minRadius=20, maxRadius=300)
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        # prendre le cercle le plus plausible (par ex. le plus grand)
        x,y,r = circles[0]
        cv.circle(src, (x,y), r, (0,255,0), 4)
        cv.circle(src, (x,y), 2, (0,0,255), 3)
    """
    """
    # Option B : contours + ellipse fit 
    cnts, _ = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    good = [c for c in cnts if cv.contourArea(c) > 500 and len(c) >= 5]
    best = max(good, key=lambda c: cv.arcLength(c, True)) if good else None
    if best is not None:
        ellipse = cv.fitEllipse(best)  # ( (xc,yc), (major,minor), angle )
        (xc,yc),(MA,ma),angle = ellipse
        cv.ellipse(src, ellipse, (255, 0, 0), 2)
    """
    """
    cv.imshow("detected circles", src)
    cv.waitKey(0)
    """
    return 0
if __name__ == "__main__":
    main(sys.argv[1:])
    
