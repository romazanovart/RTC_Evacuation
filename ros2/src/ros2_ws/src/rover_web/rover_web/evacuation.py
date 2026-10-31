import copy
import heapq
import json
import math
import secrets
import threading
import time
from pathlib import Path


DIRECTIONS = [(1, 0), (0, 1), (-1, 0), (0, -1)]
OBJECT_CELL_KINDS = {'road', 'bridge', 'beyond'}
NAMES = {'road': 'Дорога', 'rubble': 'Обломки жёлтого здания',
         'yellow': 'Жёлтое здание', 'blue': 'Синее здание',
         'river': 'Речка', 'bridge': 'Мост — въезд запрещён',
         'beyond': 'Дорога за мостом — въезд запрещён'}


def default_map():
    cells = []
    special = {(2, 1): 'yellow', (3, 1): 'yellow',
               (1, 3): 'blue', (3, 3): 'river', (4, 3): 'bridge',
               (3, 4): 'bridge', (4, 4): 'beyond'}
    for y in range(5):
        for x in range(5):
            kind = special.get((x, y), 'road')
            cells.append({'x': x, 'y': y, 'kind': kind, 'description': 'Парковка' if (x, y) == (1, 1) else NAMES[kind]})
    return {'cells': cells, 'markers': {}, 'person': None, 'cell_size': .8,
            'blue_neighborhood': [[0, 2], [1, 2], [2, 2], [0, 3], [2, 3], [0, 4], [1, 4], [2, 4]],
            'yellow_neighborhood': [[1, 0], [2, 0], [3, 0], [4, 0], [1, 1], [4, 1], [1, 2], [2, 2], [3, 2], [4, 2]]}


def cell(value):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError('Нужны координаты [x, y]')
    if any(type(v) is not int or not 0 <= v < 5 for v in value):
        raise ValueError('Координаты клетки должны быть целыми от 0 до 4')
    return tuple(value)


def heading(value):
    if type(value) is not int or not 0 <= value < 4:
        raise ValueError('Некорректное направление')
    return value


def validate_map(value):
    result = default_map()
    descriptions = value.get('cells', [])
    for item in descriptions:
        x, y = cell([item.get('x'), item.get('y')])
        result['cells'][y * 5 + x]['description'] = str(item.get('description', ''))[:300]
    result['cells'][6]['description'] = 'Парковка'
    occupied = set()
    for name, position in value.get('markers', {}).items():
        if name not in ('accident', 'tanker', 'tree', 'bus_stop', 'rubble'):
            raise ValueError('Неизвестный маркер')
        point = cell(position)
        if result['cells'][point[1]*5+point[0]]['kind'] not in OBJECT_CELL_KINDS:
            raise ValueError('Объекты можно размещать на дороге, мосту или за мостом')
        if point in occupied:
            raise ValueError('Маркеры должны стоять в разных клетках')
        occupied.add(point)
        result['markers'][name] = list(point)
    person = value.get('person')
    if person is not None:
        position = cell(person.get('cell'))
        if result['cells'][position[1]*5+position[0]]['kind'] not in OBJECT_CELL_KINDS:
            raise ValueError('QR можно размещать на дороге, мосту или за мостом')
        result['person'] = {'cell': list(position), 'facing': heading(person.get('facing'))}
    return result


def walkable(model):
    allowed = {(c['x'], c['y']) for c in model['cells'] if c['kind'] == 'road'}
    allowed -= {tuple(v) for v in model['markers'].values()}
    if model['person']:
        allowed.discard(tuple(model['person']['cell']))
    return allowed


def safe_approaches(position, allowed):
    goals = set()
    for side, (dx, dy) in enumerate(DIRECTIONS):
        approach = (position[0] + dx, position[1] + dy)
        if approach in allowed:
            goals.add((*approach, (side + 2) % 4))
    if not goals and tuple(position) == (4, 4):
        if (2, 4) in allowed:
            goals.add((2, 4, 0))
        if (4, 2) in allowed:
            goals.add((4, 2, 1))
    return goals


def plan_route(model, pose, target='person'):
    labels = {'accident': 'аварии', 'tanker': 'бензовозу',
              'tree': 'дереву', 'bus_stop': 'остановке', 'bridge': 'мосту', 'person': 'человеку',
              'rubble': 'обломкам жёлтого здания', 'collection': 'зоне сбора',
              'blue_area': 'окрестностям синего дома', 'yellow_area': 'окрестностям жёлтого дома'}
    if target not in labels:
        raise ValueError('Неизвестная цель')
    allowed = walkable(model)
    start = (*cell(pose['cell']), heading(pose['heading']))
    if target in ('blue_area', 'yellow_area'):
        area = model['blue_neighborhood' if target == 'blue_area' else 'yellow_neighborhood']
        goals = {(p[0], p[1], d) for p in area for d in range(4)}
    elif target == 'collection':
        goals = {(0, 0, 1)}
    elif target == 'person':
        person = model['person']
        if not person:
            raise ValueError('Человек не отмечен на карте')
        dx, dy = DIRECTIONS[person['facing']]
        goals = {(person['cell'][0] + dx, person['cell'][1] + dy,
                  (person['facing'] + 2) % 4)}
        person_kind = model['cells'][person['cell'][1]*5+person['cell'][0]]['kind']
        if next(iter(goals))[:2] not in allowed and person_kind in ('bridge', 'beyond'):
            if tuple(person['cell']) == (4, 4) and person['facing'] == 2 and (2, 4) in allowed:
                goals = {(2, 4, 0)}
            elif tuple(person['cell']) == (4, 4) and person['facing'] == 3 and (4, 2) in allowed:
                goals = {(4, 2, 1)}
            else:
                goals = safe_approaches(person['cell'], allowed)
    elif target in ('rubble', 'bridge'):
        position = model['markers'].get('rubble') if target == 'rubble' else [4, 3]
        if position is None:
            raise ValueError('Обломки не отмечены на карте')
        person = model.get('person')
        if person and person.get('cell') == position:
            dx, dy = DIRECTIONS[person['facing']]
            goals = {(position[0] + dx, position[1] + dy, (person['facing'] + 2) % 4)}
        else:
            goals = safe_approaches(position, allowed)
    else:
        position = model['markers'].get(target)
        if position is None:
            raise ValueError(f'Цель «{labels[target]}» не отмечена на карте')
        goals = safe_approaches(position, allowed)
    if start[:2] not in allowed:
        raise ValueError('Робот стоит в запрещённой клетке')
    goals = {goal for goal in goals if goal[:2] in allowed}
    if not goals:
        raise ValueError('Рядом с целью нет доступной дорожной клетки')
    queue = [((0, 0), start)]
    distance = {start: (0, 0)}
    previous = {}
    goal = None
    best_total = (float("inf"), float("inf"))
    while queue:
        cost, state = heapq.heappop(queue)
        if cost != distance[state]:
            continue
        if state in goals:
            if target in ('blue_area', 'yellow_area') and model.get('person'):
                try:
                    tail = plan_route(model, {'cell': list(state[:2]), 'heading': state[2]}, 'person')
                except ValueError:
                    continue
                turns = sum(step['action'] != 'forward' for step in tail['steps'])
                moves = sum(step['action'] == 'forward' for step in tail['steps'])
                total = (cost[0] + turns, cost[1] + moves)
                if total < best_total:
                    goal, best_total = state, total
            else:
                goal = state
                break
        x, y, direction = state
        dx, dy = DIRECTIONS[direction]
        successors = [((x, y, (direction + 1) % 4), 'right', (1, 0)),
                      ((x, y, (direction - 1) % 4), 'left', (1, 0))]
        if (x + dx, y + dy) in allowed:
            successors.append(((x + dx, y + dy, direction), 'forward', (0, 1)))
        for destination, action, weight in successors:
            candidate = (cost[0] + weight[0], cost[1] + weight[1])
            if candidate < distance.get(destination, (float('inf'), float('inf'))):
                distance[destination] = candidate
                previous[destination] = (state, action)
                heapq.heappush(queue, (candidate, destination))
    if goal is None:
        raise ValueError(f'Безопасного маршрута к {labels[target]} нет')
    actions = []
    cursor = goal
    while cursor != start:
        source, action = previous[cursor]
        if action == 'forward':
            request = {'kind': 'move', 'forward': .8, 'left': 0, 'speed': .30,
                       'approach_speed': .1, 'position_tolerance': .025, 'correction_speed': .1}
            text = f'Еду вперёд на 0,8 м до клетки ({cursor[0]}, {cursor[1]}).'
        else:
            request = {'kind': 'turn', 'degrees': 90 if action == 'left' else -90,
                       'speed': .45, 'tolerance_deg': 3.0}
            text = 'Поворот на 90° ' + ('налево.' if action == 'left' else 'направо.')
        actions.append({'action': action, 'text': text, 'request': request,
                        'pose': {'cell': list(cursor[:2]), 'heading': cursor[2]}})
        cursor = source
    actions.reverse()
    path = [list(start[:2])] + [a['pose']['cell'] for a in actions if a['action'] == 'forward']
    return {'id': secrets.token_hex(12), 'path': path, 'steps': actions,
            'target': target, 'target_label': labels[target],
            'start': copy.deepcopy(pose), 'goal': list(goal[:2]), 'map': copy.deepcopy(model)}


class Evacuation:
    START_POSE = {'cell': [0, 0], 'heading': 1}

    def __init__(self, gateway):
        self.gateway = gateway
        self.lock = threading.RLock()
        self.path = Path.home() / '.local/share/frob/evacuation.json'
        self.model = default_map()
        self.pose = {'cell': [0, 0], 'heading': 1}
        self.pose_valid = False
        if self.path.exists():
            saved = json.loads(self.path.read_text())
            self.model = validate_map(saved['map'])
            self.pose = saved['pose']
            self.pose_valid = (bool(saved.get('pose_valid', True))
                               and not saved.get('motion_in_progress', False)
                               and tuple(self.pose['cell']) in walkable(self.model))
        self.initial_model = validate_map(saved.get('initial_map', self.model) if self.path.exists() else self.model)
        self.plan = None
        self.running = False
        self.resetting = False
        self.cancel = threading.Event()
        self.events = []
        self.discovered = set()
        self.vision_confirmed = set()
        self.last_phrase = {}
        self.sequence = 0
        self.epoch = secrets.token_hex(8)

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps({'map': self.model, 'pose': self.pose, 'initial_map': self.initial_model,
                                         'pose_valid': self.pose_valid,
                                         'motion_in_progress': self.running}, ensure_ascii=False))
        temporary.replace(self.path)

    def event(self, text, **extra):
        with self.lock:
            self.sequence += 1
            self.events.append({'id': self.sequence, 'text': text, 'timestamp': int(time.time() * 1000), **extra})
            self.events = self.events[-250:]

    def phrase(self, key, variants, **values):
        """Choose a synonymous scripted phrase, avoiding immediate repeats."""
        with self.lock:
            phrase_state = getattr(self, 'last_phrase', None)
            if phrase_state is None:
                phrase_state = self.last_phrase = {}
            previous = phrase_state.get(key)
            choices = [value for value in variants if value != previous] or list(variants)
            template = secrets.choice(choices)
            phrase_state[key] = template
        return template.format(**values)

    def expected_camera_object(self):
        """Return the map marker directly in front of the robot, if any."""
        with self.lock:
            x, y = self.pose['cell']
            dx, dy = DIRECTIONS[self.pose['heading']]
            return next((name for name, point in self.model['markers'].items()
                         if point == [x + dx, y + dy]), None)

    def confirm_camera_object(self, name, label, confidence, image_data, perception_id):
        """Publish one camera proof only when map geometry and model agree."""
        with self.lock:
            if (not self.running or name in self.vision_confirmed
                    or self.expected_camera_object() != name):
                return False
            self.vision_confirmed.add(name)
            self.discovered.add(name)
            # Sequence matters in chat: image-only event first, conclusion second.
            self.event('', imageData=image_data,
                       imageAlt=f'Реальный кадр: {label}', perceptionId=perception_id)
            self.event(self.phrase('object_found', (
                'Объект найден: {label} ({confidence}%).',
                'Объект найден: {label}. Уверенность {confidence}%.',
                'Камера подтвердила объект: {label} ({confidence}%).',
                'Распознан объект «{label}». Точность {confidence}%.',
            ), label=label, confidence=round(float(confidence) * 100)),
                       perceptionId=perception_id)
            return True

    def discover(self):
        with self.lock:
            name = self.expected_camera_object()
            if name is None or name in self.discovered:
                return set()
            self.discovered.add(name)
            labels = {
                'accident': 'аварию',
                'tanker': 'бензовоз',
                'tree': 'дерево',
                'bus_stop': 'остановку',
                'rubble': 'обломки',
            }
            visible = copy.deepcopy(self.model)
            visible['markers'] = {
                key: value for key, value in visible['markers'].items()
                if key in self.discovered
            }
            if 'person' not in self.discovered:
                visible['person'] = None
            self.event(
                f'Обнаружил {labels[name]} перед собой.',
                map=visible,
                pose=copy.deepcopy(self.pose),
                path=[copy.deepcopy(self.pose['cell'])],
            )
            return {name}

    def status(self):
        with self.lock:
            return copy.deepcopy({'map': self.model, 'pose': self.pose, 'pose_valid': self.pose_valid,
                                  'running': self.running, 'plan': self.plan,
                                  'events': self.events, 'epoch': self.epoch})

    def reset(self):
        with self.lock:
            if self.resetting:
                raise ValueError('Сброс уже выполняется')
            self.resetting = True
            self.cancel.set()
        try:
            self.gateway.stop_motion()
            deadline = time.monotonic() + 5
            while self.running and time.monotonic() < deadline:
                time.sleep(.05)
            with self.lock:
                if self.running:
                    raise RuntimeError('Движение ещё останавливается. Повтори сброс.')
                self.model = default_map()
                self.initial_model = copy.deepcopy(self.model)
                self.pose = {'cell': [0, 0], 'heading': 1}
                self.pose_valid = False
                self.plan = None
                self.discovered.clear()
                self.vision_confirmed.clear()
                self.events.clear()
                self.sequence = 0
                self.epoch = secrets.token_hex(8)
                self.save()
            return self.status()
        finally:
            with self.lock:
                self.resetting = False

    def new_dialog(self):
        with self.lock:
            if self.running or self.resetting:
                raise ValueError('Сначала останови текущее задание.')
            self.plan = None
            self.discovered.clear()
            self.events.clear()
            self.sequence = 0
            self.epoch = secrets.token_hex(8)
            self._arm_saved_map_locked()
            self.save()
        return self.status()

    def _arm_saved_map_locked(self):
        """Reuse the committed field map from the permanent start pose."""
        point = tuple(self.START_POSE['cell'])
        if point not in walkable(self.model):
            raise ValueError('Стартовая клетка (0, 0) должна оставаться свободной')
        self.pose = copy.deepcopy(self.START_POSE)
        self.pose_valid = True
        self.discovered.clear()
        self.vision_confirmed.clear()

    def configure(self, payload):
        with self.lock:
            if self.running:
                raise ValueError('Сначала останови выполнение маршрута')
            if 'map' in payload:
                model = validate_map(payload['map'])
                if tuple(self.START_POSE['cell']) not in walkable(model):
                    raise ValueError('Стартовая клетка (0, 0) должна оставаться свободной')
                self.model = model
                self.initial_model = copy.deepcopy(model)
                self._arm_saved_map_locked()
            if 'pose' in payload:
                self._arm_saved_map_locked()
            self.plan = None
            self.save()
        return self.status()

    def prepare(self, target='person', mission_object=None):
        with self.lock:
            if self.running:
                raise ValueError('Маршрут уже выполняется')
            # The field and start direction are fixed. Planning a new mission
            # reuses the last committed map even after a previous aborted run.
            self._arm_saved_map_locked()
            self.plan = None
            self.plan = plan_route(self.model, self.pose, target)
            if mission_object is not None:
                if mission_object not in self.model['markers']:
                    raise ValueError('Целевой объект не размещён на карте')
                if not self.model.get('person') or self.model['person']['cell'] != self.model['markers'][mission_object]:
                    raise ValueError('Размести QR в клетке целевого объекта')
                end_pose = self.plan['steps'][-1]['pose'] if self.plan['steps'] else self.pose
                object_plan = plan_route(self.model, end_pose, mission_object)
                end_pose = object_plan['steps'][-1]['pose'] if object_plan['steps'] else end_pose
                plan_route(self.model, end_pose, 'person')
                self.plan['mission_object'] = mission_object
            if target in ('blue_area', 'yellow_area'):
                end_pose = self.plan['steps'][-1]['pose'] if self.plan['steps'] else self.pose
                self.plan['next_plan'] = plan_route(self.model, end_pose, 'person')
            self.plan['discovered'] = sorted(self.discovered)
            return copy.deepcopy(self.plan)

    def start(self, plan_id):
        with self.lock:
            if self.resetting or self.running or not self.plan or self.plan['id'] != plan_id or not self.pose_valid:
                raise ValueError('Маршрут устарел или уже выполняется. Построй новый.')
            nav = self.gateway.navigation_status()
            if self.gateway.motion_status_payload()['running'] or nav.get('goal_status') in ('active', 'sending', 'canceling'):
                raise ValueError('Сначала останови текущую команду или Nav2')
            # Validate sensors before reporting that the route has started.
            # This also selects fresh encoder odometry as a global fallback
            # when the IMU is temporarily unavailable.
            first_kind = self.plan['steps'][0]['request']['kind'] if self.plan['steps'] else 'move'
            self.gateway.motion_feedback_selection(first_kind)
            self.running = True
            self.cancel.clear()
            self.save()
            plan = copy.deepcopy(self.plan)
            threading.Thread(target=self._run, args=(plan,), daemon=True, name='evacuation-route').start()
        return {'ok': True}

    def start_qr(self):
        with self.lock:
            if self.resetting or self.running:
                raise ValueError('Уже выполняется движение или сброс')
            nav = self.gateway.navigation_status()
            if self.gateway.motion_status_payload()['running'] or nav.get('goal_status') in ('active', 'sending', 'canceling'):
                raise ValueError('Сначала останови текущую команду или Nav2')
            self.running = True
            self.cancel.clear()
            self.save()
            threading.Thread(target=self._run_qr, daemon=True, name='evacuation-route').start()
        return {'ok': True}

    def _run_qr(self):
        try:
            self._wait_for_qr()
            self.event('QR считан. Робот остановлен.')
        except Exception as exc:
            self.event(str(exc))
        finally:
            self.gateway.set_drive_command(0, 0, 0)
            with self.lock:
                self.pose_valid = False
                self.running = False
                self.save()

    def _execute_steps(self, steps):
        previous_kind = None
        for step in steps:
            current_kind = step['request']['kind']
            if previous_kind is not None:
                self.gateway.set_drive_command(0, 0, 0)
                if self.cancel.wait(1.0):
                    raise RuntimeError('Маршрут остановлен')
            with self.lock:
                if self.cancel.is_set():
                    raise RuntimeError('Маршрут остановлен')
                if current_kind == 'move':
                    visible = copy.deepcopy(self.model)
                    visible['markers'] = {key: value for key, value in visible['markers'].items()
                                          if key in self.discovered}
                    if 'person' not in self.discovered:
                        visible['person'] = None
                    self.event(step['text'], map=visible, pose=copy.deepcopy(self.pose),
                               path=[copy.deepcopy(self.pose['cell']), copy.deepcopy(step['pose']['cell'])])
                else:
                    self.event(step['text'])
                with self.gateway._lock:
                    self.gateway.start_motion(step['request'])
                    process = self.gateway._motion_process
            deadline = time.monotonic() + (30 if current_kind == 'move' else 20)
            while process.poll() is None:
                if self.cancel.wait(.1) or time.monotonic() > deadline:
                    self.gateway.stop_motion()
                    raise RuntimeError('Движение остановлено: шаг не завершён. Проверь положение робота перед повторным запуском.')
            if process.returncode != 0 or self.cancel.is_set():
                raise RuntimeError('Шаг не подтверждён одометрией. Маршрут остановлен, проверь положение робота.')
            self.gateway.set_drive_command(0, 0, 0)
            previous_kind = current_kind
            with self.lock:
                self.pose = copy.deepcopy(step['pose'])
                self.save()
            x, y = self.pose['cell']
            dx, dy = DIRECTIONS[self.pose['heading']]
            if any(point == [x+dx, y+dy] and name not in self.discovered for name, point in self.model['markers'].items()):
                if self.cancel.wait(4.5):
                    raise RuntimeError('Осмотр остановлен')
                self.discover()

    def _fresh_pose(self):
        with self.gateway._lock:
            stamp = self.gateway._topic_state['odom'].received_monotonic
            pose = self.gateway._current_odometry_pose()
        if pose is None or time.monotonic() - stamp > .7:
            raise RuntimeError('Нет свежей одометрии. Робот остановлен.')
        return pose

    def _return_to_cell(self, trail):
        deadline = time.monotonic() + 60
        targets = list(reversed(trail))
        try:
            for target in targets:
                while True:
                    if self.cancel.wait(.05) or time.monotonic() > deadline:
                        raise RuntimeError('Возврат остановлен или истёк лимит времени.')
                    pose = self._fresh_pose()
                    dx, dy = target['x'] - pose['x'], target['y'] - pose['y']
                    distance = math.hypot(dx, dy)
                    if distance < .035:
                        break
                    desired = math.atan2(dy, dx) + math.pi
                    error = math.atan2(math.sin(desired - pose['yaw']), math.cos(desired - pose['yaw']))
                    angular = max(-.4, min(.4, 1.2 * error))
                    speed = -min(.12, max(.04, distance * .8)) if abs(error) < .25 else 0.
                    self.gateway.set_drive_command(speed, 0, angular)
            while True:
                if self.cancel.wait(.05) or time.monotonic() > deadline:
                    raise RuntimeError('Возврат остановлен или истёк лимит времени.')
                pose = self._fresh_pose()
                error = math.atan2(math.sin(trail[0]['yaw'] - pose['yaw']), math.cos(trail[0]['yaw'] - pose['yaw']))
                if abs(error) < math.radians(2):
                    break
                self.gateway.set_drive_command(0, 0, max(-.35, min(.35, error)))
        finally:
            self.gateway.set_drive_command(0, 0, 0)

    def _restore_qr_heading(self, initial_yaw):
        self.event('QR считан. Возвращаю исходное направление корпуса.')
        try:
            if self.cancel.is_set():
                raise RuntimeError('Выравнивание остановлено.')
            pose = self._fresh_pose()
            error = math.atan2(math.sin(initial_yaw - pose['yaw']), math.cos(initial_yaw - pose['yaw']))
            if abs(error) > math.radians(3):
                self.gateway.start_motion({'kind': 'turn', 'degrees': math.degrees(error),
                                           'speed': .45, 'tolerance_deg': 3.0})
                process = self.gateway._motion_process
                deadline = time.monotonic() + 30
                while process.poll() is None:
                    if self.cancel.wait(.05) or time.monotonic() >= deadline:
                        self.gateway.stop_motion()
                        raise RuntimeError('Выравнивание остановлено: превышено время или нажата остановка.')
                if process.returncode != 0:
                    raise RuntimeError('Поворот после QR не завершён. Проверь приводы и обратную связь.')
            if self.cancel.is_set():
                raise RuntimeError('Выравнивание остановлено.')
            pose = self._fresh_pose()
            remaining = math.atan2(math.sin(initial_yaw - pose['yaw']), math.cos(initial_yaw - pose['yaw']))
            if abs(remaining) > math.radians(3):
                raise RuntimeError('Направление после QR не подтверждено: ошибка %.1f°.' % math.degrees(remaining))
            self.event('Исходное направление восстановлено. Робот остановлен.')
        finally:
            self.gateway.set_drive_command(0, 0, 0)

    def _wait_for_qr(self, trail=None):
        from .stand_motion import command as stand_command, SmoothCommand
        smoother = SmoothCommand()
        baseline = self.gateway.qr_status()
        baseline_sequence = baseline.get('decoded_sequence', baseline.get('candidate_sequence', 0))
        self.event(self.phrase('qr_search', (
            'Ищу QR-код. Приближаюсь с плавным подруливанием к цели.',
            'Начинаю поиск QR-кода и плавно двигаюсь к нему.',
            'Перехожу к поиску QR-кода. Курс корректирую плавно.',
        )))
        search_started = time.monotonic()
        last_seen = 0.0
        last_tick = time.monotonic()
        approached = 0.0
        last_pose = self._fresh_pose()
        initial_yaw = last_pose['yaw']
        search_yaw = initial_yaw
        sweep_direction = 1.0
        sweep_progress_yaw = initial_yaw
        sweep_progress_time = time.monotonic()
        traveled = 0.0
        last_target_offset = None
        state = ''
        while True:
            if self.cancel.wait(.08):
                self.gateway.set_drive_command(0, 0, 0)
                raise RuntimeError('Сценарий остановлен')
            remaining_to_person = None
            if trail is not None:
                pose = self._fresh_pose()
                origin = trail[0]
                progress = ((pose['x'] - origin['x']) * math.cos(origin['yaw'])
                            + (pose['y'] - origin['y']) * math.sin(origin['yaw']))
                remaining_to_person = .60 - progress
                if progress >= .40 and 'person' not in self.discovered and self.model.get('person'):
                    self.discovered.add('person')
                    visible = copy.deepcopy(self.model)
                    visible['markers'] = {k: v for k, v in visible['markers'].items() if k in self.discovered}
                    self.event('Достиг зоны человека. Считываю QR-код.', map=visible,
                               pose=copy.deepcopy(self.pose), path=self.plan['path'] if self.plan else [])
                if math.hypot(pose['x'] - trail[-1]['x'], pose['y'] - trail[-1]['y']) >= .08:
                    trail.append(pose)
            status = self.gateway.qr_status()
            pose_now = self._fresh_pose()
            traveled += math.hypot(pose_now['x']-last_pose['x'], pose_now['y']-last_pose['y'])
            last_pose = pose_now
            camera = self.gateway.camera_status('/camera/qr_annotated', 'sensor_msgs/msg/Image')
            if not camera.get('frame_ready') or camera.get('age_sec') is None or camera['age_sec'] > .7:
                self.gateway.set_drive_command(0, 0, 0)
                smoother = SmoothCommand()
                continue
            has_decode_channel = 'decoded_sequence' in status
            decoded = status.get('decoded' if has_decode_channel else 'candidate') or {}
            text = str(decoded.get('text') or '').strip()
            detection_age = status.get('decoded_age_sec' if has_decode_channel else 'candidate_age_sec')
            sequence = status.get('decoded_sequence' if has_decode_channel else 'candidate_sequence', 0)
            if (text and not decoded.get('candidate', False)
                    and detection_age is not None and detection_age < .7
                    and sequence > baseline_sequence):
                self.gateway.set_drive_command(0, 0, 0)
                print(f'QR: {text}', flush=True)
                self.event(f'QR считан: {text}', imageData=decoded.get('imageData', ''), imageAlt='Кадр считывания QR-кода')
                self._restore_qr_heading(initial_yaw)
                return text
            candidate = status.get('candidate') or {}
            candidate_age = status.get('candidate_age_sec')
            fresh = candidate and candidate_age is not None and candidate_age <= .7
            if fresh:
                tick = time.monotonic()
                elapsed = min(.25, tick - last_tick)
                last_tick = tick
                last_seen = time.monotonic()
                search_yaw = pose_now['yaw']
                offset = float((candidate.get('offset_normalized') or [0])[0])
                last_target_offset = offset
                linear, angular = stand_command(candidate, candidate_age)
                if traveled >= 2.0:
                    linear = 0.
                if remaining_to_person is not None:
                    if remaining_to_person <= .025:
                        linear = 0.0
                    else:
                        linear = min(linear, max(.03, remaining_to_person * .6))
                approached += linear * elapsed
                linear, angular = smoother.step(linear, angular, elapsed)
                if traveled >= 2.0 or (remaining_to_person is not None and remaining_to_person <= .025):
                    linear = smoother.linear = 0.
                self.gateway.set_drive_command(linear, 0, angular)
                next_state = 'approach' if linear else 'align'
                if next_state != state:
                    self.event('QR найден. Приближаюсь для расшифровки.' if linear else
                               ('Достигнут предел проезда 2 м. Чтение продолжается на месте.' if traveled >= 2.0 else
                                'Подставка найдена, QR пока не расшифрован. Читаю на месте.'))
                    state = next_state
            else:
                tick = time.monotonic()
                elapsed = min(.25, tick - last_tick)
                last_tick = tick
                missing_for = tick - (last_seen or search_started)
                linear = 0.0
                if last_seen and missing_for < .5 and last_target_offset is not None:
                    camera = self.gateway.camera_status('/camera/qr_annotated', 'sensor_msgs/msg/Image')
                    if camera.get('frame_ready') and camera.get('age_sec') is not None and camera['age_sec'] < .4:
                        linear = .04
                if remaining_to_person is not None and remaining_to_person <= .025:
                    linear = 0.0
                approached += linear * elapsed
                # Search by a smooth bounded heading sweep, never drive blindly.
                if missing_for < .5:
                    sweep_progress_time = tick
                    sweep_progress_yaw = pose_now['yaw']
                desired = search_yaw + sweep_direction * .55
                error = math.atan2(math.sin(desired-pose_now['yaw']), math.cos(desired-pose_now['yaw']))
                if abs(error) < .07:
                    sweep_direction *= -1
                    desired = search_yaw + sweep_direction * .55
                    error = math.atan2(math.sin(desired-pose_now['yaw']), math.cos(desired-pose_now['yaw']))
                angular = math.copysign(min(.35, max(.24, abs(error))), error)
                if tick - sweep_progress_time >= 3.0:
                    progress = abs(math.atan2(math.sin(pose_now['yaw']-sweep_progress_yaw), math.cos(pose_now['yaw']-sweep_progress_yaw)))
                    if progress < math.radians(1):
                        self.gateway.set_drive_command(0, 0, 0)
                        raise RuntimeError('Поиск QR остановлен: команда поворота не подтверждена датчиками.')
                    sweep_progress_time = tick
                    sweep_progress_yaw = pose_now['yaw']
                linear, angular = smoother.step(linear, angular, elapsed)
                if traveled >= 2.0 or missing_for >= .5 or (remaining_to_person is not None and remaining_to_person <= .025):
                    linear = smoother.linear = 0.
                self.gateway.set_drive_command(linear, 0, angular)
                if state != 'search':
                    self.event('Ищу подставку плавным поворотом.' if not last_seen else
                               ('Краткая потеря цели: медленно продолжаю до 0,5 с.' if linear else
                                'QR потерян. Продолжаю поиск плавным поворотом.'))
                    state = 'search'

    def _run(self, plan):
        try:
            self.gateway.set_drive_command(0, 0, 0)
            if self.cancel.wait(2.0):
                raise RuntimeError('Маршрут остановлен')
            self.event(self.phrase('route_start', (
                'Начинаю движение к {target} по маршруту.',
                'Выдвигаюсь к {target} по построенному пути.',
                'Начинаю проезд к {target}.',
            ), target=plan['target_label']))
            self._execute_steps(plan['steps'])
            if plan.get('mission_object'):
                mission_object = plan['mission_object']
                self.event(self.phrase('explore', (
                    'Начинаю исследовать окрестности. Продвигаюсь по одной клетке.',
                    'Приступаю к осмотру окрестностей. Двигаюсь пошагово, по одной клетке.',
                    'Начинаю обследование зоны. Проверяю маршрут по клеткам.',
                )))
                plan = plan_route(self.model, self.pose, mission_object)
                with self.lock:
                    self.plan = plan
                self.event('Начинаю движение по окрестностям, по одной клетке.')
                self._execute_steps(plan['steps'])
                target_name = {'accident':'авария', 'tanker':'бензовоз', 'tree':'дерево', 'bus_stop':'остановка', 'rubble':'обломки'}[mission_object]
                with self.lock:
                    camera_confirmed = mission_object in self.vision_confirmed
                    self.discovered.add(mission_object)
                if not camera_confirmed:
                    # Map fallback: report the expected object without attaching
                    # a camera frame and continue the mission immediately.
                    self.event(self.phrase('object_found_map', (
                        'Объект найден: {target}.',
                        'Обнаружен объект: {target}.',
                        'Нашёл заданный объект — {target}.',
                        'Объект «{target}» найден.',
                    ), target=target_name))
                self.event(self.phrase('target_confirmed', (
                    'Это наш целевой объект — {target}. Начинаю искать пострадавшего.',
                    'Целевой объект подтверждён: {target}. Перехожу к поиску пострадавшего.',
                    'Нужный объект найден — {target}. Начинаю поиски человека.',
                    'Опасность обнаружена: {target}. Ищу пострадавшего.',
                ), target=target_name))
                plan = plan_route(self.model, self.pose, 'person')
                with self.lock:
                    self.plan = plan
                self.event('Подъезжаю к лицевой стороне QR по одной клетке.')
                self._execute_steps(plan['steps'])
            elif plan['target'] in ('blue_area', 'yellow_area'):
                self.event('Достиг окрестностей дома. Продолжаю движение к зоне поиска.')
                plan = plan_route(self.model, self.pose, 'person')
                with self.lock:
                    self.plan = plan
                visible = copy.deepcopy(self.model)
                visible['markers'] = {k: v for k, v in visible['markers'].items() if k in self.discovered}
                visible['person'] = None
                self.event('Начинаю движение по зоне поиска, по одной клетке.',
                           map=visible, pose=copy.deepcopy(self.pose))
                self._execute_steps(plan['steps'])
            if plan['target'] in ('rubble', 'person'):
                trail = [self._fresh_pose()]
                self.discover()
                qr_text = self._wait_for_qr(trail)
                if not qr_text or not qr_text.strip() or self.cancel.is_set():
                    raise RuntimeError('QR не считан. Возврат не запущен.')
                self.gateway.set_drive_command(0, 0, 0)
                if self.cancel.wait(2.0):
                    raise RuntimeError('Маршрут остановлен')
                return_plan = plan_route(self.model, self.pose, 'collection')
                visible = copy.deepcopy(self.model)
                visible['markers'] = {k: v for k, v in visible['markers'].items() if k in self.discovered}
                if 'person' not in self.discovered:
                    visible['person'] = None
                current = self._fresh_pose()
                origin = trail[0]
                dx, dy = current['x'] - origin['x'], current['y'] - origin['y']
                forward = dx * math.cos(origin['yaw']) + dy * math.sin(origin['yaw'])
                left = -dx * math.sin(origin['yaw']) + dy * math.cos(origin['yaw'])
                fx, fy = DIRECTIONS[self.pose['heading']]
                lx, ly = DIRECTIONS[(self.pose['heading'] - 1) % 4]
                current_cell = [self.pose['cell'][0] + (forward * fx + left * lx) / .8,
                                self.pose['cell'][1] + (forward * fy + left * ly) / .8]
                self.event('Начал движение в зону старта.', map=visible,
                           pose={'cell': current_cell, 'heading': self.pose['heading']},
                           path=[current_cell] + return_plan['path'])
                self._return_to_cell(trail)
                with self.lock:
                    self.plan = return_plan
                self._execute_steps(return_plan['steps'])
                self.event('Вернулся в зону старта. Робот остановлен.')
            else:
                self.event(f'Доехал до {plan["target_label"]}. Маршрут завершён.')
        except Exception as exc:
            with self.lock:
                self.pose_valid = False
            self.event(str(exc))
            self.gateway.stop_motion()
        finally:
            self.gateway.set_drive_command(0, 0, 0)
            with self.lock:
                self.running = False
                self.save()

    def stop(self):
        self.cancel.set()
        if self.running:
            self.gateway.stop_motion()
        return {'ok': True}
