"""Laptop field console with a deterministic mission scenario."""
import json
from map_code import encode_map
import hmac
import os
import random
import re
import secrets
import sys
import threading
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs
from urllib.error import HTTPError
from urllib.request import Request, urlopen

REPOSITORY = Path(__file__).resolve().parents[1]
ROVER_PACKAGE = REPOSITORY / 'ros2/src/ros2_ws/src/rover_web'
ROOT = ROVER_PACKAGE / 'web'
sys.path.insert(0, str(ROVER_PACKAGE))
from rover_web.lan_peer import LANPeer
ROBOT = LANPeer(os.environ.get('FROB_ROBOT_MAC', '00:00:00:00:00:00'), 8765, 'rover_web')
PASSWORD = os.environ.get('FROB_FIELD_PASSWORD', '')
SESSION_TOKEN = secrets.token_urlsafe(32)
OBJECTS = {
    'accident': (('авари', 'дтп', 'аварийн', 'столкнов', 'катастроф',
                  'происшеств', 'автокатастроф', 'дорожн происшеств'), 'авария'),
    'tanker': (('бензовоз', 'цистерн', 'топливоз', 'автоцистерн',
                'нефтевоз', 'топливн машин', 'машин с топлив'), 'бензовоз'),
    'tree': (('дерев', 'ствол', 'берез', 'дуб', 'сосн', 'елк', 'елов',
              'пихт', 'лиственниц', 'топол', 'клен', 'осин', 'рябин',
              'ива', 'верб', 'лип', 'каштан', 'акаци', 'ясен', 'вяз',
              'кедр', 'пальм', 'кустарник', 'яблон', 'груш', 'вишн',
              'черешн', 'слив', 'орех', 'секвой', 'баобаб', 'кипарис',
              'можжевельник'), 'дерево'),
    'bus_stop': (('остановк', 'павильон', 'остановочн', 'посадочн площадк',
                  'автобусн платформ', 'автобусн пункт'), 'остановка'),
    'rubble': (('облом', 'завал', 'развалин', 'руин', 'обрушен',
                'разрушен', 'строительн мусор', 'бит кирпич'), 'обломки'),
}

PLACE_NAMES = {
    'blue_area': 'синий/голубой дом',
    'yellow_area': 'жёлтый дом',
    'bridge': 'мост',
}
ACKNOWLEDGEMENTS = [
    'Задачу принял. Подготовил маршрут.',
    'Принято. Маршрут готов к проверке.',
    'Задание понятно. Можно проверить маршрут и начать.',
    'Команду принял. Путь до цели построен.',
    'Цель и ориентир определены. Маршрут подготовлен.',
    'Понял задачу. План движения готов.',
]
_ack_lock = threading.Lock()
_ack_pool = []
_last_ack = None

def robot_request(path, payload=None, timeout=10):
    endpoint = ROBOT.resolve()
    request = Request(endpoint+path, data=None if payload is None else json.dumps(payload).encode(),
                      headers={'Content-Type':'application/json'})
    try:
        return urlopen(request, timeout=timeout)
    except HTTPError:
        raise
    except OSError:
        ROBOT.invalidate()
        # Never replay a possibly accepted movement request after a timeout.
        raise

def intent(text):
    objects, places, _ = analyze_intent(text)
    if len(objects) != 1 or len(places) != 1:
        raise ValueError('Укажи один объект и один ориентир: синий/голубой дом, жёлтый дом или мост.')
    return objects[0], places[0]

def analyze_intent(text):
    text = re.sub(r'[^а-яa-z0-9]+', ' ', text.lower().replace('ё', 'е')).strip()
    objects = [key for key, (stems, _) in OBJECTS.items()
               if any(stem in text for stem in stems)]
    building = bool(re.search(r'\b(?:дом\w*|здани\w*|строени\w*|постройк\w*|корпус\w*|сооружени\w*)\b', text))
    blue = bool(re.search(r'\b(?:син\w*|голуб\w*|лазур\w*|небесн\w*|бирюз\w*)\b', text))
    yellow = bool(re.search(r'\b(?:желт\w*|золотист\w*|лимонн\w*|янтарн\w*|песочн\w*)\b', text))
    places = []
    if building and blue:
        places.append('blue_area')
    if building and yellow:
        places.append('yellow_area')
    if re.search(r'\b(?:мост\w*|переправ\w*|виадук\w*|путепровод\w*|эстакад\w*)\b', text):
        places.append('bridge')
    keywords = [OBJECTS[key][1] for key in objects]
    if 'blue_area' in places:
        keywords.append('синий дом')
        for pattern, label in (('голуб', 'голубой дом'), ('лазур', 'лазурный дом'),
                               ('небесн', 'небесно-голубой дом'), ('бирюз', 'бирюзовый дом')):
            if pattern in text:
                keywords.append(label)
    if 'yellow_area' in places:
        keywords.append('жёлтый дом')
        for pattern, label in (('золотист', 'золотистый дом'), ('лимонн', 'лимонный дом'),
                               ('янтарн', 'янтарный дом'), ('песочн', 'песочный дом')):
            if pattern in text:
                keywords.append(label)
    if 'bridge' in places:
        keywords.append('мост')
        for pattern, label in (('переправ', 'переправа'), ('виадук', 'виадук'),
                               ('путепровод', 'путепровод'), ('эстакад', 'эстакада')):
            if pattern in text:
                keywords.append(label)
    return objects, places, keywords

def acknowledgement():
    # A shuffled bag gives variety without repeating the same reply twice.
    global _last_ack
    with _ack_lock:
        if not _ack_pool:
            _ack_pool.extend(ACKNOWLEDGEMENTS)
            random.shuffle(_ack_pool)
            if _last_ack is not None and _ack_pool[-1] == _last_ack:
                _ack_pool[0], _ack_pool[-1] = _ack_pool[-1], _ack_pool[0]
        _last_ack = _ack_pool.pop()
        return _last_ack

class Handler(BaseHTTPRequestHandler):
    disable_nagle_algorithm = True
    def authenticated(self):
        if not PASSWORD:
            return True
        cookie = SimpleCookie(self.headers.get('Cookie', ''))
        value = cookie.get('frob_field_session')
        return bool(value and hmac.compare_digest(value.value, SESSION_TOKEN))

    def login_page(self, error=False):
        notice = '<p class="error">Неверный пароль</p>' if error else '<p>Введите пароль для доступа к роботу</p>'
        body = f'''<!doctype html><html lang="ru"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Вход · FROB</title>
<style>body{{margin:0;min-height:100vh;display:grid;place-items:center;background:#edf6fa;font:16px system-ui;color:#19384a}}form{{background:#fff;padding:32px;border-radius:18px;box-shadow:0 12px 40px #17384a18;width:min(320px,75vw)}}input,button{{box-sizing:border-box;width:100%;padding:14px;font:inherit;border-radius:10px;margin-top:12px;border:1px solid #b9d1dd}}button{{background:#10afe1;color:#fff;cursor:pointer}}p{{color:#607783}}.error{{color:#c33}}</style>
<form method="post" action="/login"><h1>FROB</h1>{notice}<input type="password" name="password" placeholder="Пароль" autocomplete="current-password" required autofocus><button type="submit">Войти</button></form></html>'''.encode()
        self.send(body, 401 if error else 200, 'text/html; charset=utf-8')

    def login(self):
        length = int(self.headers.get('Content-Length', 0))
        if not 0 < length <= 1024:
            return self.login_page(True)
        supplied = parse_qs(self.rfile.read(length).decode(errors='replace')).get('password', [''])[0]
        if not hmac.compare_digest(supplied, PASSWORD):
            return self.login_page(True)
        self.send_response(303)
        self.send_header('Location', '/')
        self.send_header('Set-Cookie', f'frob_field_session={SESSION_TOKEN}; Path=/; HttpOnly; SameSite=Strict')
        self.send_header('Content-Length', '0')
        self.end_headers()

    def send(self, body, code=200, content_type='application/json; charset=utf-8'):
        if not isinstance(body, bytes):
            body = json.dumps(body, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def dispatch(self, payload=None):
        path = self.path.split('?')[0]
        if path == '/api/health' and payload is None:
            return self.send({'ok':True, 'service':'rtc_field_console'})
        if path == '/api/robot-address' and payload is None:
            return self.send({'ok':True, 'url':ROBOT.resolve()})
        if path in ('/', '/assets/field.js', '/assets/evac-map.js'):
            if payload is not None:
                raise ValueError('Этот адрес только для чтения')
            file = ROOT / ('field.html' if path == '/' else path.lstrip('/'))
            return self.send(file.read_bytes(), content_type='text/html; charset=utf-8' if path == '/' else 'text/javascript; charset=utf-8')
        if path == '/api/map-code' and payload is None:
            with robot_request('/api/evacuation/status') as response:
                state = json.load(response)
            return self.send({'code': encode_map(state['map'])})
        if path == '/api/mission/prepare' and payload is not None:
            objects, places, keywords = analyze_intent(str(payload.get('text', ''))[:2000])
            exact = len(objects) == 1 and len(places) == 1
            if exact:
                request = {'target':places[0], 'mission_object':objects[0]}
            elif len(places) == 1:
                request = {'target':places[0]}
            else:
                request = {'target':'person'}
            try:
                with robot_request('/api/evacuation/plan', request) as response:
                    plan = json.load(response)
            except HTTPError:
                if not exact:
                    raise
                with robot_request('/api/evacuation/plan', {'target':'person'}) as response:
                    plan = json.load(response)
            return self.send({'plan':plan, 'reply':acknowledgement(), 'source':'scenario',
                              'keywords':keywords})
        if path == '/api/route/preview' and payload is not None:
            with robot_request('/api/evacuation/plan', {'target':'person'}) as response:
                return self.send(response.read(), content_type=response.headers.get('Content-Type','application/json'))
        allowed = ('/api/evacuation/status','/api/evacuation/config','/api/evacuation/start','/api/evacuation/stop','/api/camera/frame')
        if path not in allowed:
            return self.send({'error':'Not found'},404)
        if (payload is None) != (path in ('/api/evacuation/status','/api/camera/frame')):
            raise ValueError('Неверный метод')
        with robot_request(self.path, payload, timeout=15) as response:
            self.send(response.read(), content_type=response.headers.get('Content-Type','application/json'))

    def handle_request(self, post=False):
        try:
            path = self.path.split('?')[0]
            if path == '/login' and not post:
                return self.login_page()
            if path != '/api/health' and not self.authenticated():
                if path.startswith('/api/'):
                    return self.send({'error':'Требуется вход'}, 401)
                return self.login_page()
            payload = None
            if post:
                origin = self.headers.get('Origin')
                if origin and origin != 'http://'+self.headers.get('Host',''):
                    return self.send({'error':'Cross-origin request denied'},403)
                length = int(self.headers.get('Content-Length',0))
                if not 0 < length <= 64000:
                    raise ValueError('Недопустимый размер запроса')
                payload = json.loads(self.rfile.read(length))
            self.dispatch(payload)
        except HTTPError as exc:
            self.send(exc.read(),exc.code)
        except (OSError, ValueError) as exc:
            self.send({'error':str(exc)},503 if isinstance(exc,OSError) else 400)

    def do_GET(self): self.handle_request()
    def do_POST(self):
        if self.path.split('?')[0] == '/login':
            return self.login()
        self.handle_request(True)

if __name__ == '__main__':
    ThreadingHTTPServer(('0.0.0.0',1111),Handler).serve_forever()
