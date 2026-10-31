"""Run with PYTHONPATH=integration/rover_web; isolated UI probe, NO robot I/O.

Open http://127.0.0.1:18765/ (WebSocket) or /?http=1 (HTTP fallback).
The real drive-page markup and JS handlers feed an in-memory sink only.
"""
import asyncio
import json
from pathlib import Path
import threading
from aiohttp import web
from rover_web.drive_sequence import DriveLease, DriveSequence
from rover_web.drive_socket import DriveSocket


class Sink:
    def __init__(self):
        self.lock = threading.RLock()
        self.sequence = DriveSequence()
        self.lease = DriveLease()
        self.commands = []
        self.owner = None

    def set_web_drive_command(self, payload, stop=False):
        with self.lock:
            if not stop and 'http_ticket' in payload and not self.lease.valid(payload['http_ticket']):
                return {'ok': True, 'ignored': 'expired'}
            if not self.sequence.accept(payload.get('client_id'), payload.get('sequence')):
                return {'ok': True, 'ignored': 'superseded'}
            self.owner = payload.get('client_id')
            self.commands.append({'x': 0 if stop else payload.get('linear_x', 0),
                                  'yaw': 0 if stop else payload.get('angular_z', 0),
                                  'transport': 'HTTP' if 'http_ticket' in payload else 'WS/STOP'})
            self.commands = self.commands[-80:]
            return {'ok': True}

    def release_web_drive(self, client):
        with self.lock:
            if self.owner == client:
                self.commands.append({'x': 0, 'yaw': 0, 'transport': 'close'})
                self.owner = None


async def main():
    root = Path(__file__).resolve().parents[1] / 'web'
    source = (root / 'assets/app.js').read_text()
    sections = [('function updateDriveOutputs()', 'async function issueGlobalStop()'),
                ('function shouldIgnoreDriveKeyEvent(', 'function bindRoutesPage()')]
    handlers = '\n'.join(source[source.index(a):source.index(b)] for a, b in sections)
    markup = (root / 'index.html').read_text()
    start = markup.index('<div class="two-column drive-layout">')
    markup = markup[start:markup.index('</section>', start)]
    page = '''<!doctype html><title>Isolated WASD test — no robot</title>
    <style>.hidden{display:none} .active{background:lightgreen} button{padding:8px} pre{white-space:pre-wrap}</style>
    <h1>Isolated WASD test — no robot</h1><pre id="test-commands"></pre>''' + markup + '''<script>
    const $=s=>document.querySelector(s), $$=s=>[...document.querySelectorAll(s)];
    const formatFloat=(v,n=2)=>Number(v).toFixed(n);
    const state={page:'drive',driveMode:'keyboard',joystick:{active:false,x:0,y:0,pointer:null},
      driveKeys:new Set(),driveSocket:null,driveTick:null,driveTickAt:0,driveTimer:null,
      driveClientId:'probe-'+Date.now(),driveSequence:0,driveInputEpoch:0,driveHttpPending:false,
      config:{drive_websocket_port:location.search.includes('http=1')?18767:18766}};
    async function api(path,options={}) {
      const response=await fetch(path,{...options,headers:{'Content-Type':'application/json'}});
      if(!response.ok) throw Error(response.status);
      return response.json();
    }
    ''' + handlers + '''
    bindDrivePage();
    setInterval(async()=>{const r=await api('/test/commands');
      $('#test-commands').textContent=JSON.stringify(r);},100);
    </script>'''
    sink = Sink()
    drive = DriveSocket(sink, '127.0.0.1', 18766, 18765)
    app = web.Application()

    async def index(request):
        return web.Response(text=page, content_type='text/html')

    async def ticket(request):
        return web.json_response({'ticket': sink.lease.issue()})

    async def command(request):
        return web.json_response(sink.set_web_drive_command(await request.json(), stop=request.path.endswith('/stop')))

    async def commands(request):
        with sink.lock:
            return web.Response(text=json.dumps(sink.commands), content_type='application/json')

    app.router.add_get('/', index)
    app.router.add_get('/api/drive/ticket', ticket)
    app.router.add_post('/api/drive/command', command)
    app.router.add_post('/api/drive/stop', command)
    app.router.add_get('/test/commands', commands)
    runner = web.AppRunner(app)
    await runner.setup()
    await web.TCPSite(runner, '127.0.0.1', 18765).start()
    print('Isolated probe: http://127.0.0.1:18765/ — no robot output', flush=True)
    try:
        await asyncio.Event().wait()
    finally:
        drive.close()
        await runner.cleanup()


if __name__ == '__main__':
    asyncio.run(main())
