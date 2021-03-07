#!/usr/bin/env python3

# Based on https://github.com/mbyzhang/v4l2-stream-python

import queue
import threading
import rospy
from sensor_msgs.msg import Image, CompressedImage
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject, GLib
import logging
import queue



class GstPipeline():
    def __init__(self, input_caps, bitrate=3000, tsmux_alignment=30):
        self.pipeline        = None
        self.mainloop        = None
        self.videosource     = None
        self.videosink       = None
        self.on_pipeline_ready = None
        self.frame_queue     = queue.Queue(30)
        self.input_caps = input_caps
        self.bitrate = bitrate
        self.tsmux_alignment = tsmux_alignment
    
    def pull_frame(self):
        return self.frame_queue.get(True, 31536000)

    def _put_frame(self, data):
        sample = self.videosink.emit("pull-sample")
        if sample is not None:
            self.current_buffer = sample.get_buffer()
            current_data = self.current_buffer.extract_dup(0, self.current_buffer.get_size())
            #logging.info(".")
            try:
                #   logging.info("Frame put into the queue, current size is {}".format(self.frame_queue.qsize()))
                self.frame_queue.put_nowait(current_data)
            except queue.Full:
                logging.warning("Buffer overrun")
        return False
    
    def push_frame(self, buf):
        if self.videosource is not None:
            self.videosource.emit('push-buffer', Gst.Buffer.new_wrapped(buf))
    
    def on_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.EOS:
            self.pipeline.set_state(Gst.State.NULL)
        elif t == Gst.MessageType.ERROR:
            self.pipeline.set_state(Gst.State.NULL)
            err, debug = message.parse_error()
            print("Error: %s" % err, debug)
        
    def gst_thread(self):
        Gst.init(None)

        cmd = "appsrc name=src do-timestamp=1 is-live=1 ! autovideoconvert ! avenc_mpeg1video name=encoder ! mpegvideoparse ! mpegtsmux name=mpegtsmux ! appsink name=sink"
        # cmd = "appsrc name=src do-timestamp=1 is-live=1 ! autovideoconvert ! avenc_mpeg1video name=encoder ! mpegvideoparse ! mpegtsmux name=mpegtsmux ! tsdemux ! mpeg2dec ! xvimagesink"
        logging.info("Starting pipeline {}".format(cmd))
        self.pipeline = Gst.parse_launch(cmd)

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_message)

        self.videosource = self.pipeline.get_by_name("src")
        self.pipeline.get_by_name("src").set_property("caps", Gst.Caps.from_string(self.input_caps))

        self.videosink = self.pipeline.get_by_name("sink")
        self.videosink.set_property("drop", True)
        self.videosink.set_property("max-buffers", 2)
        self.videosink.set_property("emit-signals", True)
        self.videosink.set_property("sync", False)
        self.videosink.connect("new-sample", self._put_frame)

        self.pipeline.get_by_name("encoder").set_property("bitrate", self.bitrate * 1000)
        self.pipeline.get_by_name("mpegtsmux").set_property("alignment", self.tsmux_alignment)
        
        self.pipeline.set_state(Gst.State.PLAYING)
        
        if self.on_pipeline_ready is not None:
            self.on_pipeline_ready()
        
        rospy.loginfo('Pipeline is ready')
        
        self.mainloop = GLib.MainLoop()
        self.mainloop.run()

    def set_playing(self):
        logging.info("Pipeline state is set to PLAYING")
        self.pipeline.set_state(Gst.State.PLAYING)
    
    def set_paused(self):
        logging.info("Pipeline state is set to PAUSED")
        self.pipeline.set_state(Gst.State.PAUSED)


def handle_frame(frame: Image, pipeline: GstPipeline) -> None:
    pipeline.push_frame(frame.data)

def publish_frame(pipeline: GstPipeline, pub: rospy.Publisher):
    while True:
        frame_raw = pipeline.pull_frame()
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = 'mpeg1'
        msg.data = frame_raw
        pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('alpha_video_encoder')
    bitrate = rospy.get_param('~bitrate', 3000)
    tsmux_alignment = rospy.get_param('~tsmux_alignment', 30)
    rospy.loginfo('Waiting for first frame...')
    first_image = rospy.wait_for_message('image_raw', Image)
    if first_image.encoding != 'bgr8':
        raise Exception('Image encoding {} is not supported'.format(first_image.encoding))

    caps = 'video/x-raw,format=BGR,height={},width={},framerate=30/1'.format(first_image.height, first_image.width)
    rospy.loginfo('Determined caps: {}'.format(caps))

    gst_pipeline = GstPipeline(caps, bitrate, tsmux_alignment)
    gst_thread = threading.Thread(target=gst_pipeline.gst_thread)
    gst_thread.daemon = True

    gst_thread.start()

    video_sub = rospy.Subscriber('image_raw', Image, handle_frame, gst_pipeline)
    video_pub = rospy.Publisher('image_mpeg1', CompressedImage, queue_size=30)

    pub_thread = threading.Thread(target=publish_frame, args=(gst_pipeline, video_pub))
    pub_thread.daemon = True
    pub_thread.start()
    rospy.spin()

    # gst_pipeline.set_playing()

