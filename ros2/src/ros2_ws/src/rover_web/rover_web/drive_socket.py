"""Dedicated low-latency teleop connection, independent of camera HTTP requests."""
import asyncio
from collections import deque
import contextlib
import json
import threading
import time
from urllib.parse import urlsplit

from aiohttp import web, WSMsgType


class DriveSocket:
    def __init__(self, gateway, host, port, http_port):
        self.gateway = gateway
        self.host, self.port, self.http_port = host, port, http_port
        self.sockets = set()
        self.ready = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self._run, name='drive-websocket', daemon=True)
        self.thread.start()
        if not self.ready.wait(3) or self.error:
            raise RuntimeError(f'Drive socket failed: {self.error}')

    def _run(self):
        try:
            asyncio.run(self._serve())
        except Exception as exc:
            self.error = exc
            self.ready.set()

    async def _serve(self):
        self.loop = asyncio.get_running_loop()
        self.shutdown = asyncio.Event()
        app = web.Application(client_max_size=2048)
        app.router.add_get('/drive', self.handle)
        runner = web.AppRunner(app, shutdown_timeout=.5)
        await runner.setup()
        try:
            site = web.TCPSite(runner, self.host, self.port)
            await site.start()
            self.port = runner.addresses[0][1]
            self.ready.set()
            await self.shutdown.wait()
            await asyncio.gather(*(ws.close() for ws in list(self.sockets)), return_exceptions=True)
        finally:
            await runner.cleanup()

    async def handle(self, request):
        origin = urlsplit(request.headers.get('Origin', ''))
        host = urlsplit('http://' + request.host).hostname
        if (origin.scheme not in {'http', 'https'} or origin.hostname != host
                or origin.port != self.http_port):
            raise web.HTTPForbidden(text='Unexpected control origin')
        client = request.query.get('client_id', '')
        if not 1 <= len(client) <= 80:
            raise web.HTTPBadRequest(text='Missing control session')
        ws = web.WebSocketResponse(max_msg_size=2048, compress=False, heartbeat=2.)
        await ws.prepare(request)
        self.sockets.add(ws)
        # Echoing a recent server ticket prevents replay of old TCP-buffered input.
        tickets = deque(maxlen=8)

        async def clock():
            number = 0
            while not ws.closed:
                number += 1
                tickets.append((number, time.monotonic()))
                await asyncio.wait_for(ws.send_json({'tick': number}), timeout=.3)
                await asyncio.sleep(.05)

        async def run_clock():
            try:
                await clock()
            except (OSError, asyncio.TimeoutError, RuntimeError):
                await ws.close()

        ticker = asyncio.create_task(run_clock())
        try:
            async for message in ws:
                if message.type != WSMsgType.TEXT:
                    break
                try:
                    payload = json.loads(message.data)
                    if not isinstance(payload, dict) or type(payload.get('sequence')) is not int:
                        raise ValueError('Invalid control message')
                    payload['client_id'] = client
                    stop = payload.get('type') == 'stop'
                    if not stop:
                        if type(payload.get('tick')) is not int:
                            continue
                        issued = next((stamp for tick, stamp in tickets if tick == payload.get('tick')), None)
                        if issued is None or time.monotonic() - issued > .25:
                            continue
                    self.gateway.set_web_drive_command(payload, stop=stop)
                except (ValueError, TypeError) as exc:
                    await ws.send_json({'error': str(exc)})
        finally:
            ticker.cancel()
            with contextlib.suppress(asyncio.CancelledError, OSError, RuntimeError):
                await ticker
            self.sockets.discard(ws)
            # Closing an old tab must not stop a newer driver or an autonomous route.
            self.gateway.release_web_drive(client)
        return ws

    def close(self):
        if hasattr(self, 'loop') and not self.loop.is_closed():
            self.loop.call_soon_threadsafe(self.shutdown.set)
            self.thread.join(timeout=2.)
