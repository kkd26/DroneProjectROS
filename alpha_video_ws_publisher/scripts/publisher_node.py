#!/usr/bin/env python3

import rospy
import websockets
import asyncio
import janus
from sensor_msgs.msg import CompressedImage

class WebSocketBroadcastServer():
    def __init__(self, port=8765, host='0.0.0.0', loop=None, queue_size=30):
        self.client_queues = {}
        self.host = host
        self.port = port
        janus.current_loop = asyncio.get_event_loop
        self.queue = janus.Queue(maxsize=queue_size)
        self.on_first_open = None
        self.on_last_closed = None
        self.queue_size = queue_size
        if loop is None:
            self.loop = asyncio.get_event_loop()
        else:
            self.loop = loop
    
    def run(self):
        self.loop.create_task(self._master_broadcast())
        self.loop.run_until_complete(websockets.serve(self.handler, self.host, self.port))
    
    def broadcast(self, message):
        try:
            self.queue.sync_q.put_nowait(message)
        except:
            rospy.logerr_throttle_identical(1.0, 'Master buffer overrun')

    async def _master_broadcast(self):
        try:
            while True:
                message = await self.queue.async_q.get()
                for client_queue in self.client_queues.values():
                    try:
                        client_queue.put_nowait(message)
                    except asyncio.QueueFull:
                        rospy.logwarn_throttle_identical(1.0, 'Client buffer overrun')
        except Exception as e:
            rospy.logerr('Error in master broadcast: {}'.format(e))

    
    async def _client_msg_dummy_consumer(self, websocket):
        try:
            async for _ in websocket:
                pass
        except:
            pass

    async def handler(self, websocket, path):
        if len(self.client_queues) == 0 and self.on_first_open is not None:
            self.on_first_open()
        
        queue = self.client_queues[websocket] = asyncio.Queue(self.queue_size)
        self.loop.create_task(self._client_msg_dummy_consumer(websocket))

        try:
            while True:
                message = await queue.get()
                await websocket.send(message)
        except websockets.ConnectionClosedError:
            rospy.loginfo("Connection closed")
        finally:
            del self.client_queues[websocket]
            if len(self.client_queues) == 0 and self.on_last_closed is not None:
                self.on_last_closed()

def handle_image(image: CompressedImage, server: WebSocketBroadcastServer):
    server.broadcast(image.data)

if __name__ == '__main__':
    try:
        rospy.init_node('alpha_video_ws_publisher', disable_signals=True)
        param_queue_size = rospy.get_param('~queue_size', 30)
        server = WebSocketBroadcastServer(queue_size=param_queue_size)
        server.run()
        image_sub = rospy.Subscriber('image_mpeg1', CompressedImage, handle_image, server)
        asyncio.get_event_loop().run_forever()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.signal_shutdown('user shutdown request')
    


