#!/usr/bin/env python3

import rospy
import websockets
import asyncio
import logging
import janus
from rospy.exceptions import ROSInternalException
from sensor_msgs.msg import CompressedImage

class WebSocketBroadcastServer():
    def __init__(self, port=8765, host='0.0.0.0', loop=None):
        self.clients = set()
        self.host = host
        self.port = port
        self.queue = None
        self.on_first_open = None
        self.on_last_closed = None
        if loop is None:
            self.loop = asyncio.get_event_loop()
        else:
            self.loop = loop
    
    def set_queue(self, queue):
        self.queue = queue
    
    def run(self):
        self.loop.create_task(self.broadcast())
        self.loop.run_until_complete(websockets.serve(self.handler, self.host, self.port))

    async def broadcast(self):
        while True:
            message = await self.queue.get()
            await asyncio.gather(
                *[ws.send(message) for ws in self.clients],
                return_exceptions=False,
            )

    async def handler(self, websocket, path):
        if len(self.clients) == 0 and self.on_first_open is not None:
            self.on_first_open()
        self.clients.add(websocket)
        try:
            async for msg in websocket:
                pass
        except websockets.ConnectionClosedError:
            logging.info("Connection closed")
        finally:
            self.clients.remove(websocket)
            if len(self.clients) == 0 and self.on_last_closed is not None:
                self.on_last_closed()

def handle_image(image: CompressedImage, queue: janus.Queue):
    queue.sync_q.put_nowait(image.data)


if __name__ == '__main__':
    try:
        rospy.init_node('alpha_video_ws_publisher')
        server = WebSocketBroadcastServer()
        janus.current_loop = asyncio.get_event_loop
        queue = janus.Queue(30)
        server.set_queue(queue.async_q)
        server.run()
        image_sub = rospy.Subscriber('image_mpeg1', CompressedImage, handle_image, queue)
        asyncio.get_event_loop().run_forever()
    except ROSInternalException:
        rospy.loginfo('Interrupted')
        pass
    


