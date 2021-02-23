#!/usr/bin/env python3

# Adapted from https://www.pyimagesearch.com/2018/07/30/opencv-object-tracking/

# import the necessary packages
from cv_bridge.core import CvBridge
from imutils.video import FPS
from sensor_msgs.msg import Image
import imutils
import cv2
import rospy
from cv_bridge import CvBridge

rospy.init_node('target_tracker')

# Trackers = { CSRT, KCF, Boosting, MIL, TLD, MedianFlow, TrackerMOSSE }
tracker = cv2.TrackerMIL_create()

# initialize the bounding box coordinates of the object we are going
# to track
initBB = None

fps = None

bridge = CvBridge()
# loop over frames from the video stream
while True:
    rospy.loginfo('Trying to grab image...')
    # grab the current frame, then handle if we are using a
    # VideoStream or VideoCapture object
    frame = bridge.imgmsg_to_cv2(rospy.wait_for_message('camera', Image))

    if frame is None:
        rospy.logwarn('Frame is null, skipping')
        continue

    # resize the frame (so we can process it faster) and grab the
    # frame dimensions
    frame = imutils.resize(frame, width=500)
    (H, W) = frame.shape[:2]
    # check to see if we are currently tracking an object
    if initBB is not None:
        # grab the new bounding box coordinates of the object
        (success, box) = tracker.update(frame)
        # check to see if the tracking was a success
        if success:
            (x, y, w, h) = [int(v) for v in box]
            cv2.rectangle(frame, (x, y), (x + w, y + h),
                (0, 255, 0), 2)
        # update the FPS counter
        fps.update()
        fps.stop()
        # initialize the set of information we'll be displaying on
        # the frame
        info = [
            ("Success", "Yes" if success else "No"),
            ("FPS", "{:.2f}".format(fps.fps())),
        ]
        # loop over the info tuples and draw them on our frame
        for (i, (k, v)) in enumerate(info):
            text = "{}: {}".format(k, v)
            cv2.putText(frame, text, (10, H - ((i * 20) + 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    # show the output frame
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    # if the 's' key is selected, we are going to "select" a bounding
    # box to track
    if key == ord("s"):
        # select the bounding box of the object we want to track (make
        # sure you press ENTER or SPACE after selecting the ROI)
        initBB = cv2.selectROI("Frame", frame, fromCenter=False,
            showCrosshair=True)
        # start OpenCV object tracker using the supplied bounding box
        # coordinates, then start the FPS throughput estimator as well
        tracker.init(frame, initBB)
        fps = FPS().start()
    # if the `q` key was pressed, break from the loop
    elif key == ord("q"):
        break

# close all windows
cv2.destroyAllWindows()
