#!/usr/bin/env python3

# Adapted from https://www.pyimagesearch.com/2018/07/30/opencv-object-tracking/

import cv2
import rospy
import queue

from sensor_msgs.msg import Image
from alpha_target_tracker.msg import RegionOfInterestWithFullRes
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from alpha_target_tracker.srv import SetRegionOfInterest, SetRegionOfInterestRequest, SetRegionOfInterestResponse

def handle_frame(data: Image, frame_queue: queue.Queue):
    try:
        frame_queue.put_nowait(data)
    except queue.Full:
        rospy.logwarn_throttle_identical(1, 'Frame queue overrun: the image processing is not fast enough to keep up with the framerate')

def handle_set_roi_request(data: SetRegionOfInterestRequest, request_queue: queue.Queue):
    resp = SetRegionOfInterestResponse()
    try:
        request_queue.empty()
        request_queue.put_nowait(data.roi)
        resp.success = True
    except:
        resp.success = False
    return resp

if __name__ == '__main__':
    try:
        rospy.init_node('alpha_target_tracker')

        frame_queue = queue.Queue(2)
        set_roi_request_queue = queue.Queue(1)

        image_sub = rospy.Subscriber('image_raw', Image, handle_frame, frame_queue)
        
        target_roi_pub = rospy.Publisher('target_roi', RegionOfInterestWithFullRes, queue_size=2)
        tracking_status_pub = rospy.Publisher('tracking_status', Bool, queue_size=2)

        set_roi_srv = rospy.Service('set_roi', SetRegionOfInterest, lambda req: handle_set_roi_request(req, set_roi_request_queue))

        is_headless = rospy.get_param('~is_headless', True)
        image_width = rospy.get_param('~image_width', 640)

        tracker = None
        bridge = CvBridge()

        while True:
            try:
                frame = bridge.imgmsg_to_cv2(frame_queue.get(timeout=0.1))
            except queue.Empty:
                if rospy.is_shutdown():
                    break
                continue

            (H, W) = frame.shape[:2]
            H = int(H * image_width / W)
            W = image_width
            frame = cv2.resize(frame, (W, H))

            success = False
            x, y, w, h = 0, 0, 0, 0
            
            if tracker is not None:
                (success, box) = tracker.update(frame)

                if success:
                    (x, y, w, h) = [int(v) for v in box]

                    if not is_headless: 
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            roi = RegionOfInterestWithFullRes()
            roi.roi.x_offset = x
            roi.roi.y_offset = y
            roi.roi.width = w
            roi.roi.height = h
            roi.full_height = H
            roi.full_width = W
            target_roi_pub.publish(roi)

            tracking_status_pub.publish(data=success)

            if not is_headless:
                cv2.imshow("Frame", frame)
                key = cv2.waitKey(1) & 0xFF

                if key == ord("s"):
                    initBB = cv2.selectROI("Frame", frame, fromCenter=False,
                        showCrosshair=True)
                    tracker = cv2.TrackerCSRT_create()
                    tracker.init(frame, initBB)
                elif key == ord("q"):
                    break
            
            try:
                roi_msg = set_roi_request_queue.get_nowait()
                if not roi_msg.enabled:
                    tracker = None
                else:
                    tracker = cv2.TrackerCSRT_create()
                    roi = (roi_msg.x_offset, roi_msg.y_offset, roi_msg.width, roi_msg.height)
                    tracker.init(frame, roi)
                    rospy.loginfo('Target ROI updated: {}'.format(roi))
            except queue.Empty:
                pass
    except rospy.ROSInterruptException:
        pass
