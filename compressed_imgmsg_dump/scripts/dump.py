#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage

output_filename = 'output.raw'

def handle_image(frame: CompressedImage, file):
    file.write(frame.data)

if __name__ == '__main__':
    rospy.init_node('compressed_imgmsg_dump')
    with open(output_filename, 'wb') as f:
        sub = rospy.Subscriber('/image', CompressedImage, handle_image, f)
        rospy.spin()
