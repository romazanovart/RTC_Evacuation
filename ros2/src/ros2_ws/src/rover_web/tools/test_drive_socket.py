import asyncio
import threading
import time
import unittest
from aiohttp import ClientSession, WSServerHandshakeError
from rover_web.drive_socket import DriveSocket
from rover_web.drive_sequence import DriveSequence


class FakeGateway:
    def __init__(self):
        self.lock = threading.Lock()
        self.sequence = DriveSequence()
        self.commands = []
        self.owner = None

    def set_web_drive_command(self, payload, stop=False):
        with self.lock:
            if self.sequence.accept(payload['client_id'], payload['sequence']):
                self.owner = payload['client_id']
                self.commands.append((time.monotonic(), 0. if stop else payload.get('linear_x', 0.)))

    def release_web_drive(self, client):
        with self.lock:
            if client == self.owner:
                self.commands.append((time.monotonic(), 0.))
                self.owner = None


class DriveSocketTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.gateway = FakeGateway()
        self.server = DriveSocket(self.gateway, '127.0.0.1', 0, 8765)
        self.client = ClientSession()
        self.url = f'http://127.0.0.1:{self.server.port}/drive?client_id=test'

    async def asyncTearDown(self):
        await self.client.close()
        await asyncio.to_thread(self.server.close)

    def connect(self):
        return self.client.ws_connect(self.url, origin='http://127.0.0.1:8765')

    async def test_continuous_input_without_command_acks(self):
        async with self.connect() as ws:
            for sequence in range(20):
                tick = (await ws.receive_json())['tick']
                await ws.send_json({'tick':tick,'sequence':sequence,'linear_x':.2})
            await asyncio.sleep(.03)
            self.assertEqual(len(self.gateway.commands), 20)
            self.assertTrue(all(value == .2 for _,value in self.gateway.commands))
            gaps = [b[0]-a[0] for a,b in zip(self.gateway.commands,self.gateway.commands[1:])]
            self.assertLess(max(gaps), .2)
        await asyncio.sleep(.05)
        self.assertEqual(self.gateway.commands[-1][1], 0.)

    async def test_stale_and_reordered_commands_do_not_resume_motion(self):
        async with self.connect() as ws:
            old = (await ws.receive_json())['tick']
            await asyncio.sleep(.3)
            await ws.send_json({'tick':old,'sequence':1,'linear_x':.2})
            await asyncio.sleep(.03)
            self.assertEqual(self.gateway.commands, [])
            # STOP is accepted even with stale/missing tick.
            await ws.send_json({'type':'stop','sequence':10})
            current = None
            for _ in range(7): current = (await ws.receive_json())['tick']
            await ws.send_json({'tick':current,'sequence':9,'linear_x':.2})
            await asyncio.sleep(.03)
            self.assertEqual([value for _,value in self.gateway.commands], [0.])

    async def test_unrelated_origin_denied(self):
        with self.assertRaises(WSServerHandshakeError):
            await self.client.ws_connect(self.url, origin='http://unrelated.example:8765')


if __name__ == '__main__': unittest.main()
