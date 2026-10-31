'use strict';

const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => [...document.querySelectorAll(selector)];

const STORAGE_KEYS = {
  page: 'rover_web.page',
  compact: 'rover_web.compact',
  vizScale: 'rover_web.viz_scale',
  vizFollow: 'rover_web.viz_follow',
  vizMapVisible: 'rover_web.viz_map_visible',
  vizMapSelected: 'rover_web.viz_map_selected',
  sessionId: 'rover_web.session_id',
  rosTab: 'rover_web.ros_tab',
  movementPage: 'rover_web.movement_page',
  peripheralsPage: 'rover_web.peripherals_page',
  servicePage: 'rover_web.service_page',
  hackathonFile: 'rover_web.hackathon_file',
  ledStaticPresets: 'rover_web.led_static_presets',
  servoEnabled: 'rover_web.servo_enabled',
  cameraMask: 'rover_web.camera_mask',
  llmMessages: 'rover_web.llm_messages',
  llmMission: 'rover_web.llm_mission',
  routeNavPoints: 'rover_web.route_nav_points',
};

const LLM_PHRASES = {
  startQr: [
    'Начал движение к зоне считывания QR-кода.',
    'Направляюсь к точке, где нужно считать QR-код.',
    'Движение к QR-маркеру начато.',
    'Приступил к поиску и считыванию QR-кода.',
  ],
  qrRead: [
    'QR-код успешно считан.',
    'Код распознан, данные QR-кода получены.',
    'Считывание QR-кода завершено успешно.',
    'QR-маркер найден и обработан.',
  ],
  taskStart: [
    'Начал выполнять полученное задание.',
    'Данные приняты. Приступаю к выполнению миссии.',
    'Задание получено, выполнение начато.',
    'Перехожу к основному этапу задания.',
  ],
  moveObject: [
    'Начал движение к целевому объекту.',
    'Двигаюсь к объекту, указанному в задании.',
    'Маршрут к целевому объекту построен, начинаю движение.',
    'Направляюсь к выбранной цели.',
  ],
  continueMove: [
    'Продолжаю движение по маршруту.',
    'Маршрут свободен, движение продолжено.',
    'Продолжаю следовать к цели.',
    'Движение продолжается, отклонений от маршрута нет.',
  ],
};

const PAGE_GROUPS = {
  overview: 'home',
  ros: 'ros',
  drive: 'movement',
  routes: 'movement',
  visualization: 'movement',
  camera: 'peripherals',
  lidar: 'peripherals',
  lights: 'peripherals',
  audio: 'peripherals',
  octoliner: 'peripherals',
  actuators: 'peripherals',
  hackathon: 'hackathon',
  llm: 'llm',
  terminal: 'terminal',
  diagnostics: 'service',
  settings: 'service',
};

const GROUP_DEFAULT_PAGES = {
  home: 'overview',
  ros: 'ros',
  movement: 'drive',
  peripherals: 'camera',
  hackathon: 'hackathon',
  llm: 'llm',
  terminal: 'terminal',
  service: 'diagnostics',
};

const GROUP_PAGE_STORAGE = {
  movement: STORAGE_KEYS.movementPage,
  peripherals: STORAGE_KEYS.peripheralsPage,
  service: STORAGE_KEYS.servicePage,
};

const ROUTE_STEP_LABELS = {
  move: 'Проехать расстояние',
  move_polar: 'По направлению',
  turn: 'Поворот',
  pause: 'Пауза',
  return_to_start: 'Вернуться в начало',
};

const ROUTE_STEP_FIELDS = {
  move: [
    { key: 'forward', label: 'Расстояние, м', type: 'number', required: true, step: '0.01', defaultValue: 0.50 },
    { key: 'linear_speed', label: 'Скорость, м/с', type: 'number', required: true, step: '0.01', defaultValue: 0.18 },
  ],
  move_polar: [
    { key: 'distance', label: 'Дистанция, м', type: 'number', required: true, step: '0.01', defaultValue: 0.50 },
    { key: 'direction_deg', label: 'Направление, °', type: 'number', required: true, step: '1', defaultValue: 0 },
    { key: 'linear_speed', label: 'Скорость, м/с', type: 'number', step: '0.01', placeholder: 'по умолчанию' },
    { key: 'approach_speed', label: 'Скорость подхода, м/с', type: 'number', step: '0.01', placeholder: 'по умолчанию' },
    { key: 'position_tolerance', label: 'Допуск, м', type: 'number', step: '0.01', placeholder: 'по умолчанию' },
    { key: 'maximum_time', label: 'Лимит, с', type: 'number', step: '1', placeholder: 'по умолчанию' },
  ],
  turn: [
    { key: 'degrees', label: 'Угол, °', type: 'number', required: true, step: '1', defaultValue: 90 },
    { key: 'angular_speed', label: 'Скорость, рад/с', type: 'number', required: true, step: '0.01', defaultValue: 0.32 },
  ],
  pause: [
    { key: 'seconds', label: 'Пауза, с', type: 'number', required: true, step: '0.1', defaultValue: 1.0 },
  ],
  return_to_start: [
    { key: 'restore_heading', label: 'Восстановить курс', type: 'checkbox', defaultValue: true },
    { key: 'linear_speed', label: 'Скорость, м/с', type: 'number', step: '0.01', placeholder: 'по умолчанию' },
    { key: 'approach_speed', label: 'Скорость подхода, м/с', type: 'number', step: '0.01', placeholder: 'по умолчанию' },
    { key: 'position_tolerance', label: 'Допуск, м', type: 'number', step: '0.01', placeholder: 'по умолчанию' },
    { key: 'maximum_time', label: 'Лимит, с', type: 'number', step: '1', placeholder: 'по умолчанию' },
  ],
};

const LASER_SCAN_TYPE = 'sensor_msgs/msg/LaserScan';
const LED_STRIP_STATE_TYPE = 'rover_interfaces/msg/LedStripState';
const OCTOLINER_READING_TYPE = 'rover_interfaces/msg/OctolinerReading';

const state = {
  sessionId: ensureSessionId(),
  page: new URLSearchParams(location.search).has('shell') ? 'terminal' : (localStorage.getItem(STORAGE_KEYS.page) || 'overview'),
  system: null,
  status: null,
  identity: null,
  config: null,
  activity: [],
  rosGraph: null,
  rosTab: localStorage.getItem(STORAGE_KEYS.rosTab) || 'topics',
  selectedNode: null,
  selectedNodeInfo: null,
  selectedTopic: null,
  selectedTopicType: null,
  selectedTopicInfo: null,
  selectedService: null,
  selectedServiceType: null,
  selectedServiceInfo: null,
  selectedCameraTopic: '/camera/qr_annotated',
  selectedCameraType: 'sensor_msgs/msg/Image',
  cameraTimer: null,
  cameraMaskTimer: null,
  cameraMaskBusy: false,
  terminalTimer: null,
  terminalSession: (() => {
    const id = new URLSearchParams(location.search).get('shell') || sessionStorage.getItem('frob-shell') || 'terminal-1';
    return /^terminal-[1-4]$/.test(id) ? id : 'terminal-1';
  })(),
  terminalCursor: 0,
  terminalStarted: false,
  terminalPolling: false,
  cameraUrl: null,
  cameraMask: '',
  cameraSettings: null,
  cameraVisionSettings: null,
  signDetection: null,
  cameraSettingsLastRefresh: 0,
  voiceStatus: null,
  selectedLidarTopic: null,
  selectedLidarType: null,
  lidarTimer: null,
  lidarData: null,
  lidarSettings: null,
  selectedLedStripTopic: null,
  selectedLedStripType: null,
  ledStripTimer: null,
  ledStripData: null,
  ledStripSettings: null,
  ledStripControlInitialized: false,
  ledStripControlDirty: false,
  ledStripPixels: [],
  ledStripPixelsDirty: false,
  ledStripManualInitialized: false,
  ledStripStaticPresets: loadLedStripStaticPresets(),
  selectedOctolinerTopic: null,
  selectedOctolinerType: null,
  octolinerTimer: null,
  octolinerData: null,
  octolinerSettings: null,
  driveMode: localStorage.getItem('frob-drive-mode') === 'joystick' ? 'joystick' : 'keyboard',
  joystick: { active: false, x: 0, y: 0, pointer: null },
  driveKeys: new Set(),
  driveTimer: null,
  driveClientId: `${Date.now()}-${Math.random().toString(36).slice(2)}`,
  driveSequence: 0,
  driveSocket: null,
  driveTick: null,
  driveTickAt: 0,
  driveHttpPending: false,
  driveHttpTicket: null,
  driveInputEpoch: 0,
  route: {
    plans: [],
    selectedName: '',
    draft: null,
    navPoints: (() => {
      try {
        const value = JSON.parse(localStorage.getItem(STORAGE_KEYS.routeNavPoints) || '[]');
        return Array.isArray(value) ? value : [];
      } catch (_error) {
        return [];
      }
    })(),
    navRunning: false,
    navStopRequested: false,
    navImage: null,
    navImagePath: '',
    navView: null,
  },
  hackathonFiles: [],
  selectedHackathonFile: localStorage.getItem(STORAGE_KEYS.hackathonFile) || '',
  viz: {
    trail: [],
    scale: Math.max(180, Number(localStorage.getItem(STORAGE_KEYS.vizScale) || '180')),
    follow: localStorage.getItem(STORAGE_KEYS.vizFollow) !== 'false',
    mapVisible: localStorage.getItem(STORAGE_KEYS.vizMapVisible) !== 'false',
    maps: [],
    mapsRoot: '',
    selectedMap: localStorage.getItem(STORAGE_KEYS.vizMapSelected) || '',
    mapImage: null,
    mapImagePath: '',
    mapImageLoading: false,
    liveMap: null,
    view: null,
    navigation: null,
    pickMode: null,
    pickStart: null,
    pickedStartPose: null,
    pickedGoalPose: null,
  },
  apiHealthy: false,
  rosHealthy: false,
  servoEnabled: localStorage.getItem(STORAGE_KEYS.servoEnabled) !== 'false',
  llmMessages: (() => {
    try {
      const value = JSON.parse(localStorage.getItem(STORAGE_KEYS.llmMessages) || '[]');
      return Array.isArray(value) ? value.filter((message) => !message.liveCamera) : [];
    } catch (_error) {
      return [];
    }
  })(),
  llmMission: (() => {
    try {
      const value = JSON.parse(localStorage.getItem(STORAGE_KEYS.llmMission) || 'null');
      if (!['moving', 'waiting_qr'].includes(value?.stage)
          || Date.now() - Number(value.startedAt || 0) > 15 * 60 * 1000) {
        return null;
      }
      return value;
    } catch (_error) {
      return null;
    }
  })(),
  llmQrBusy: false,
  llmLastPhrase: {},
  llmRouteMap: null,
  llmRouteSignature: '',
};

function ensureSessionId() {
  const existing = localStorage.getItem(STORAGE_KEYS.sessionId);
  if (existing) return existing;
  const generated = typeof crypto?.randomUUID === 'function'
    ? crypto.randomUUID()
    : `session-${Date.now()}-${Math.random().toString(16).slice(2)}`;
  localStorage.setItem(STORAGE_KEYS.sessionId, generated);
  return generated;
}

function formatBytes(bytes) {
  if (!Number.isFinite(bytes)) return '—';
  const units = ['B', 'KB', 'MB', 'GB', 'TB'];
  let value = Number(bytes);
  let index = 0;
  while (value >= 1024 && index < units.length - 1) {
    value /= 1024;
    index += 1;
  }
  return `${value.toFixed(value >= 10 || index === 0 ? 0 : 1)} ${units[index]}`;
}

function formatGigabytes(bytes) {
  if (!Number.isFinite(bytes)) return '—';
  return (Number(bytes) / (1024 ** 3)).toFixed(1);
}

function formatPercent(value, digits = 0) {
  return Number.isFinite(Number(value)) ? `${Number(value).toFixed(digits)}%` : '—';
}

function formatSeconds(total) {
  if (!Number.isFinite(total)) return '—';
  const seconds = Math.max(0, Math.round(Number(total)));
  const days = Math.floor(seconds / 86400);
  const hours = Math.floor((seconds % 86400) / 3600);
  const minutes = Math.floor((seconds % 3600) / 60);
  const remain = seconds % 60;
  if (days > 0) return `${days}d ${hours}h ${minutes}m`;
  if (hours > 0) return `${hours}h ${minutes}m ${remain}s`;
  if (minutes > 0) return `${minutes}m ${remain}s`;
  return `${remain}s`;
}

function formatFloat(value, digits = 2) {
  return Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : '—';
}

function escapeHtml(text) {
  return String(text ?? '').replace(/[&<>"']/g, (char) => ({
    '&': '&amp;',
    '<': '&lt;',
    '>': '&gt;',
    '"': '&quot;',
    "'": '&#39;',
  }[char] || char));
}

function hackathonFileUrl(path) {
  return `/hackathon-files/${String(path || '')
    .split('/')
    .filter(Boolean)
    .map((segment) => encodeURIComponent(segment))
    .join('/')}`;
}

function hackathonKindLabel(kind) {
  return {
    markdown: 'Markdown',
    html: 'HTML',
    pdf: 'PDF',
  }[String(kind || '').toLowerCase()] || 'Файл';
}

function renderMarkdownInline(text) {
  let html = escapeHtml(text);
  html = html.replace(
    /\[([^\]]+)\]\(([^)\s]+)\)/g,
    (_match, label, url) => {
      const safeUrl = /^(https?:\/\/|mailto:|\/|#)/i.test(url) ? escapeHtml(url) : '#';
      return `<a href="${safeUrl}" target="_blank" rel="noreferrer">${label}</a>`;
    },
  );
  html = html.replace(/`([^`]+)`/g, '<code>$1</code>');
  html = html.replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>');
  html = html.replace(/\*([^*]+)\*/g, '<em>$1</em>');
  return html;
}

function renderMarkdownDocument(text) {
  const lines = String(text || '').replace(/\r\n?/g, '\n').split('\n');
  const parts = [];
  let paragraph = [];
  let listType = null;
  let listItems = [];
  let inCode = false;
  let codeLines = [];

  const flushParagraph = () => {
    if (!paragraph.length) return;
    parts.push(`<p>${renderMarkdownInline(paragraph.join(' '))}</p>`);
    paragraph = [];
  };

  const flushList = () => {
    if (!listType || !listItems.length) return;
    const items = listItems
      .map((item) => `<li>${renderMarkdownInline(item)}</li>`)
      .join('');
    parts.push(`<${listType}>${items}</${listType}>`);
    listType = null;
    listItems = [];
  };

  const flushCode = () => {
    parts.push(`<pre><code>${escapeHtml(codeLines.join('\n'))}</code></pre>`);
    codeLines = [];
  };

  for (const rawLine of lines) {
    const trimmed = rawLine.trim();
    if (trimmed.startsWith('```')) {
      flushParagraph();
      flushList();
      if (inCode) {
        flushCode();
      }
      inCode = !inCode;
      continue;
    }

    if (inCode) {
      codeLines.push(rawLine);
      continue;
    }

    if (!trimmed) {
      flushParagraph();
      flushList();
      continue;
    }

    const heading = trimmed.match(/^(#{1,3})\s+(.*)$/);
    if (heading) {
      flushParagraph();
      flushList();
      const level = heading[1].length;
      parts.push(`<h${level}>${renderMarkdownInline(heading[2])}</h${level}>`);
      continue;
    }

    const unordered = trimmed.match(/^[-*]\s+(.*)$/);
    if (unordered) {
      flushParagraph();
      if (listType && listType !== 'ul') flushList();
      listType = 'ul';
      listItems.push(unordered[1]);
      continue;
    }

    const ordered = trimmed.match(/^\d+\.\s+(.*)$/);
    if (ordered) {
      flushParagraph();
      if (listType && listType !== 'ol') flushList();
      listType = 'ol';
      listItems.push(ordered[1]);
      continue;
    }

    const quote = trimmed.match(/^>\s?(.*)$/);
    if (quote) {
      flushParagraph();
      flushList();
      parts.push(`<blockquote>${renderMarkdownInline(quote[1])}</blockquote>`);
      continue;
    }

    paragraph.push(trimmed);
  }

  if (inCode) {
    flushCode();
  }
  flushParagraph();
  flushList();
  return parts.join('\n') || '<p>Файл пустой.</p>';
}

function formatAngleRad(rad) {
  return Number.isFinite(Number(rad)) ? `${(Number(rad) * 180 / Math.PI).toFixed(1)}°` : '—';
}

function formatAge(seconds) {
  if (!Number.isFinite(Number(seconds))) return 'нет данных';
  const value = Number(seconds);
  if (value < 1) return `${value.toFixed(2)} с`;
  if (value < 10) return `${value.toFixed(1)} с`;
  return `${Math.round(value)} с`;
}

function pretty(value) {
  return JSON.stringify(value ?? {}, null, 2);
}

function deepClone(value) {
  return JSON.parse(JSON.stringify(value));
}

function levelTone(level) {
  if (level >= 2) return 'error';
  if (level === 1) return 'warn';
  return 'ok';
}

function levelLabel(level) {
  if (level >= 2) return 'Ошибка';
  if (level === 1) return 'Предупреждение';
  return 'Норма';
}

function topicAgeTone(seconds, warn = 1.5, error = 4.0) {
  if (!Number.isFinite(Number(seconds))) return 'error';
  const value = Number(seconds);
  if (value >= error) return 'error';
  if (value >= warn) return 'warn';
  return 'ok';
}

function setToneClass(element, tone, text) {
  if (!element) return;
  element.classList.remove('ok', 'warn', 'error', 'unknown');
  element.classList.add(tone || 'unknown');
  if (typeof text === 'string') {
    element.textContent = text;
  }
}

function showToast(message, tone = 'ok') {
  const container = $('#toast-container');
  const toast = document.createElement('div');
  toast.className = `toast ${tone}`;
  toast.textContent = message;
  container.append(toast);
  window.setTimeout(() => {
    toast.remove();
  }, 3400);
}

function safeArray(value) {
  return Array.isArray(value) ? value : [];
}

function loadLedStripStaticPresets() {
  try {
    const raw = localStorage.getItem(STORAGE_KEYS.ledStaticPresets);
    const parsed = raw ? JSON.parse(raw) : [];
    return safeArray(parsed)
      .filter((item) => item && typeof item.name === 'string' && Array.isArray(item.colors))
      .map((item) => ({
        name: item.name.trim(),
        colors: safeArray(item.colors).map((color) => normalizeHexColor(color, '#000000')),
      }))
      .filter((item) => item.name);
  } catch (error) {
    return [];
  }
}

function saveLedStripStaticPresets() {
  localStorage.setItem(
    STORAGE_KEYS.ledStaticPresets,
    JSON.stringify(state.ledStripStaticPresets),
  );
}

function packedColorToHex(value) {
  const { red, green, blue } = parsePackedColor(value);
  return '#'
    + red.toString(16).padStart(2, '0')
    + green.toString(16).padStart(2, '0')
    + blue.toString(16).padStart(2, '0');
}

function parseHexColorToRgb(value) {
  const text = normalizeHexColor(value, '#000000').slice(1);
  return {
    r: Number.parseInt(text.slice(0, 2), 16),
    g: Number.parseInt(text.slice(2, 4), 16),
    b: Number.parseInt(text.slice(4, 6), 16),
  };
}

function currentLedStripCount() {
  const settingsCount = Number(state.ledStripSettings?.parameters?.led_count);
  if (Number.isFinite(settingsCount) && settingsCount > 0) {
    return settingsCount;
  }
  const statusCount = Number(state.ledStripData?.led_count);
  if (Number.isFinite(statusCount) && statusCount > 0) {
    return statusCount;
  }
  return Number($('#led-strip-setting-count')?.value || '16') || 16;
}

function ensureLedStripPixels(count = currentLedStripCount(), seedColors = null) {
  const safeCount = Math.max(1, Math.min(1024, Number(count) || 16));
  const current = safeArray(seedColors ?? state.ledStripPixels).map((color) => (
    normalizeHexColor(color, '#000000')
  ));
  while (current.length < safeCount) {
    current.push('#000000');
  }
  return current.slice(0, safeCount);
}

function currentPageTitle(page) {
  return {
    overview: 'Главная',
    ros: 'ROS State',
    camera: 'Камера',
    drive: 'Управление',
    routes: 'Маршруты',
    visualization: 'Визуализация',
    lidar: 'Лидар',
    lights: 'Свет',
    audio: 'Распознавание голоса',
    octoliner: 'Octoliner',
    actuators: 'Приводы',
    hackathon: 'Хакатон',
    llm: 'LLM',
    terminal: 'Терминал',
    diagnostics: 'Диагностика',
    settings: 'Настройки',
  }[page] || 'Rover';
}

function groupForPage(page) {
  return PAGE_GROUPS[page] || 'home';
}

function resolveGroupPage(group, fallback = null) {
  const storageKey = GROUP_PAGE_STORAGE[group];
  const stored = storageKey ? localStorage.getItem(storageKey) : null;
  if (stored && groupForPage(stored) === group && isPageAvailable(stored)) {
    return stored;
  }
  const preferred = fallback || GROUP_DEFAULT_PAGES[group] || 'overview';
  return isPageAvailable(preferred) ? preferred : 'overview';
}

function rememberGroupPage(page) {
  if (!isPageAvailable(page)) return;
  const group = groupForPage(page);
  const storageKey = GROUP_PAGE_STORAGE[group];
  if (storageKey) {
    localStorage.setItem(storageKey, page);
  }
}

async function api(path, options = {}) {
  const response = await fetch(path, {
    cache: 'no-store',
    headers: {
      'Content-Type': 'application/json',
      ...(options.headers || {}),
    },
    ...options,
  });
  const contentType = response.headers.get('content-type') || '';
  if (response.status === 401) {
    window.location.reload();
    throw new Error('Сессия истекла. Введите пароль снова.');
  }
  const payload = contentType.includes('application/json')
    ? await response.json().catch(() => ({}))
    : await response.text();
  if (!response.ok) {
    const message = typeof payload === 'string'
      ? payload
      : (payload.error || payload.message || response.statusText);
    throw new Error(message || `HTTP ${response.status}`);
  }
  return payload;
}

async function recordActivity(message, details = {}, source = 'web') {
  try {
    await api('/api/activity', {
      method: 'POST',
      body: JSON.stringify({ source, message, details }),
    });
  } catch (error) {
    // Best effort only.
  }
}

function updateCompactMode(enabled) {
  document.body.classList.toggle('compact', enabled);
  $('#compact-mode').checked = enabled;
  localStorage.setItem(STORAGE_KEYS.compact, enabled ? 'true' : 'false');
}

function bindCompactMode() {
  const enabled = localStorage.getItem(STORAGE_KEYS.compact) === 'true';
  updateCompactMode(enabled);
  $('#compact-mode').addEventListener('change', (event) => {
    updateCompactMode(event.target.checked);
  });
}

function isPageAvailable(page) {
  return page !== 'actuators' || state.servoEnabled;
}

function updateServoUsage(enabled, options = {}) {
  const { persist = true, redirect = true } = options;
  state.servoEnabled = Boolean(enabled);
  if (persist) {
    localStorage.setItem(STORAGE_KEYS.servoEnabled, state.servoEnabled ? 'true' : 'false');
  }

  const toggle = $('#servo-enabled');
  if (toggle) {
    toggle.checked = state.servoEnabled;
  }

  $$('.section-tab[data-page="actuators"]').forEach((button) => {
    button.classList.toggle('hidden', !state.servoEnabled);
    button.disabled = !state.servoEnabled;
  });

  if (!state.servoEnabled) {
    if (localStorage.getItem(STORAGE_KEYS.peripheralsPage) === 'actuators') {
      localStorage.setItem(STORAGE_KEYS.peripheralsPage, 'camera');
    }
    if (redirect && state.page === 'actuators') {
      setPage('camera');
    }
  }
}

function applyFeatureDefaultsFromConfig(config) {
  const webConfig = config?.web || {};
  const hasStoredServoPreference = localStorage.getItem(STORAGE_KEYS.servoEnabled) !== null;
  if (
    !hasStoredServoPreference
    && Object.prototype.hasOwnProperty.call(webConfig, 'servo_enabled')
  ) {
    updateServoUsage(webConfig.servo_enabled !== false, {
      persist: false,
      redirect: false,
    });
  }
}

function bindServoUsage() {
  updateServoUsage(state.servoEnabled, { persist: false, redirect: false });
  $('#servo-enabled').addEventListener('change', (event) => {
    updateServoUsage(event.target.checked);
  });
}

function closeSidebar() {
  $('#sidebar').classList.remove('open');
}

function setPage(page) {
  if (state.joystick.active && page !== 'drive') stopDrive();
  if (!isPageAvailable(page)) {
    page = resolveGroupPage(groupForPage(page), 'camera');
  }
  state.page = page;
  rememberGroupPage(page);
  localStorage.setItem(STORAGE_KEYS.page, page);
  document.title = `${currentPageTitle(page)} · FROB Robot`;
  $('#page-title').textContent = currentPageTitle(page);

  $$('.nav-item').forEach((button) => {
    const navGroup = button.dataset.navGroup || groupForPage(button.dataset.page);
    button.classList.toggle('active', navGroup === groupForPage(page));
  });
  $$('.page').forEach((section) => {
    section.classList.toggle('active', section.id === `page-${page}`);
  });
  $$('.section-tab[data-page]').forEach((button) => {
    button.classList.toggle('active', button.dataset.page === page);
  });

  if (page !== 'camera') {
    stopCameraLoop();
  } else if (state.selectedCameraTopic && state.selectedCameraType) {
    refreshCameraSettings();
    refreshCameraVisionSettings();
    connectCamera();
  } else {
    refreshCameraSettings();
    refreshCameraVisionSettings();
  }

  if (page !== 'lidar') {
    stopLidarLoop();
  } else if (state.selectedLidarTopic && state.selectedLidarType) {
    refreshLidarSettings();
    connectLidar();
  } else {
    refreshLidarSettings();
    renderLidarTopics();
  }

  if (page === 'hackathon') {
    refreshHackathonFiles().then(() => {
      if (state.selectedHackathonFile) {
        openSelectedHackathonFile();
      }
    });
  }

  if (page === 'visualization') {
    requestAnimationFrame(() => fitVisualizationMap());
  }

  if (page !== 'lights') {
    stopLedStripLoop();
  } else if (state.selectedLedStripTopic && state.selectedLedStripType) {
    refreshLedStripSettings();
    renderLedStripStaticPresets();
    renderLedStripPixelGrid();
    connectLedStrip();
  } else {
    refreshLedStripSettings();
    renderLedStripStaticPresets();
    renderLedStripPixelGrid();
    renderLedStripTopics();
  }

  if (page !== 'octoliner') {
    stopOctolinerLoop();
  } else if (state.selectedOctolinerTopic && state.selectedOctolinerType) {
    refreshOctolinerSettings();
    connectOctoliner();
  } else {
    refreshOctolinerSettings();
    renderOctolinerTopics();
  }

  if (page === 'terminal') {
    startTerminalSession();
  } else {
    stopTerminalPolling();
  }

  if (page === 'settings') {
    refreshIdentityAndConfig();
    refreshCameraSettings();
    refreshLidarSettings();
    refreshLedStripSettings();
    refreshOctolinerSettings();
  }

  closeSidebar();
  sendHeartbeat();
}

function updateHealthIndicators() {
  setToneClass($('#api-dot'), state.apiHealthy ? 'ok' : 'error');
  setToneClass($('#ros-dot'), state.rosHealthy ? 'ok' : 'warn');
}

function applyIdentity() {
  const identity = state.identity || state.status?.identity || {};
  const system = state.system || state.status?.system || {};
  $('#robot-name').textContent = identity.robot_id || identity.hostname || system.hostname || 'rover';
  const addresses = safeArray(identity.ip_addresses || system.ip_addresses);
  const addressText = addresses.length ? addresses.join(', ') : `${location.hostname}:${location.port || '80'}`;
  $('#robot-address').textContent = addressText;
}

function renderDetailList(element, rows) {
  element.innerHTML = '';
  rows.forEach((row) => {
    const dt = document.createElement('dt');
    dt.textContent = row.label;
    const dd = document.createElement('dd');
    dd.textContent = row.value;
    element.append(dt, dd);
  });
}

function createStatusRow(title, subtitle, tone, valueText) {
  const row = document.createElement('div');
  row.className = 'status-row';

  const info = document.createElement('div');
  const strong = document.createElement('strong');
  strong.textContent = title;
  const small = document.createElement('small');
  small.textContent = subtitle;
  info.append(strong, small);

  const value = document.createElement('span');
  value.className = `status-value ${tone}`;
  value.textContent = valueText;
  row.append(info, value);
  return row;
}

function renderOverview() {
  const status = state.status;
  const system = state.system || status?.system;
  if (!system) return;

  const pose = status?.odom;
  const speed = pose ? Math.hypot(Number(pose.vx || 0), Number(pose.vy || 0)) : null;
  const memoryTotal = Number(system.memory_total_bytes);
  const memoryAvailable = Number(system.memory_available_bytes);
  const memoryUsedPercent = Number.isFinite(memoryTotal) && memoryTotal > 0 && Number.isFinite(memoryAvailable)
    ? (100 * (memoryTotal - memoryAvailable) / memoryTotal)
    : null;

  $('#metric-x').textContent = pose ? formatFloat(pose.x, 2) : '—';
  $('#metric-y').textContent = pose ? formatFloat(pose.y, 2) : '—';
  $('#metric-yaw').textContent = pose ? formatFloat(pose.yaw * 180 / Math.PI, 1) : '—';
  $('#metric-speed').textContent = Number.isFinite(speed) ? formatFloat(speed, 2) : '—';
  $('#metric-cpu').textContent = Number.isFinite(system.cpu_percent) ? formatFloat(system.cpu_percent, 0) : '—';
  $('#metric-temp').textContent = Number.isFinite(system.temperature_c) ? formatFloat(system.temperature_c, 1) : '—';
  $('#metric-ram').textContent = Number.isFinite(memoryUsedPercent) ? formatFloat(memoryUsedPercent, 0) : '—';
  $('#metric-disk').textContent = formatGigabytes(system.disk_free_bytes);

  const topicContainer = $('#topic-health');
  topicContainer.innerHTML = '';
  const topicState = status?.topics || {};
  [
    ['odom', 'Фильтрованная одометрия', topicState.odom],
    ['wheel_odometry', 'Колёсная одометрия', topicState.wheel_odometry],
    ['imu', 'IMU', topicState.imu],
    ['diagnostics', 'Диагностика', topicState.diagnostics],
  ].forEach(([key, label, item]) => {
    const age = item?.age_sec;
    const count = item?.message_count ?? 0;
    const tone = topicAgeTone(age);
    topicContainer.append(
      createStatusRow(
        label,
        `${key} · сообщений: ${count}`,
        tone,
        Number.isFinite(age) ? formatAge(age) : 'нет',
      ),
    );
  });

  const diagnosticSummary = $('#diagnostic-summary');
  diagnosticSummary.innerHTML = '';
  const diagnostics = safeArray(status?.diagnostics?.items);
  if (!diagnostics.length) {
    diagnosticSummary.append(createStatusRow('Диагностика', 'Сообщений пока нет', 'warn', 'ожидание'));
  } else {
    diagnostics.slice(0, 6).forEach((item) => {
      diagnosticSummary.append(
        createStatusRow(
          item.name || 'diagnostic',
          item.message || '—',
          levelTone(item.level),
          levelLabel(item.level),
        ),
      );
    });
  }

  renderDetailList($('#overview-system-details'), [
    { label: 'Hostname', value: system.hostname || '—' },
    { label: 'IP', value: safeArray(system.ip_addresses).join(', ') || '—' },
    { label: 'Uptime', value: formatSeconds(system.uptime_sec) },
    { label: 'Load 1m', value: formatFloat(system.load_average?.one_min, 2) },
    { label: 'Throttling', value: system.throttled || '—' },
    { label: 'Командный топик', value: system.command_topic || state.config?.command_topic || '—' },
    { label: 'Подключённых клиентов', value: String(status?.connected_clients ?? 0) },
    { label: 'Маршрут', value: status?.motion?.running ? 'выполняется' : 'не активен' },
  ]);

  $('#overview-devices').textContent = pretty(system.devices || status?.device_discovery || {});
}

function setSystemError(message) {
  $('#overview-system-details').innerHTML = '';
  $('#overview-devices').textContent = message;
}

async function refreshSystem() {
  try {
    const payload = await api('/api/system');
    state.system = payload;
    state.apiHealthy = true;
    applyIdentity();
    renderOverview();
    updateHealthIndicators();
  } catch (error) {
    state.apiHealthy = false;
    setSystemError(String(error.message || error));
    updateHealthIndicators();
  }
}

function appendUniqueTrailPoint(pose) {
  if (!pose) return;
  const trail = state.viz.trail;
  const last = trail[trail.length - 1];
  if (!last) {
    trail.push({ x: pose.x, y: pose.y, yaw: pose.yaw });
    return;
  }
  const distance = Math.hypot(pose.x - last.x, pose.y - last.y);
  const yawDelta = Math.abs(pose.yaw - last.yaw);
  if (distance > 0.01 || yawDelta > 0.04) {
    trail.push({ x: pose.x, y: pose.y, yaw: pose.yaw });
    if (trail.length > 1500) {
      trail.splice(0, trail.length - 1500);
    }
  }
}

function renderDiagnostics() {
  const status = state.status;
  if (!status) return;

  const diagnostics = safeArray(status.diagnostics?.items);
  const topicState = status.topics || {};
  const motorTone = topicAgeTone(topicState.wheel_odometry?.age_sec, 1.2, 3.0);
  const imuTone = topicAgeTone(topicState.imu?.age_sec, 1.2, 3.0);
  const odomTone = topicAgeTone(topicState.odom?.age_sec, 1.2, 3.0);
  const diagTopicTone = diagnostics.length
    ? levelTone(status.diagnostics?.highest_level ?? 0)
    : topicAgeTone(topicState.diagnostics?.age_sec, 2.0, 5.0);

  setToneClass($('#diagnostic-motor-status'), motorTone, motorTone === 'ok' ? 'OK' : motorTone === 'warn' ? 'STALE' : 'ERROR');
  $('#diagnostic-motor-detail').textContent = `/odom: ${formatAge(topicState.wheel_odometry?.age_sec)}`;

  setToneClass($('#diagnostic-imu-status'), imuTone, imuTone === 'ok' ? 'OK' : imuTone === 'warn' ? 'STALE' : 'ERROR');
  $('#diagnostic-imu-detail').textContent = `/imu/corrected: ${formatAge(topicState.imu?.age_sec)}`;

  setToneClass($('#diagnostic-odom-status'), odomTone, odomTone === 'ok' ? 'OK' : odomTone === 'warn' ? 'STALE' : 'ERROR');
  $('#diagnostic-odom-detail').textContent = `/odometry/filtered: ${formatAge(topicState.odom?.age_sec)}`;

  setToneClass($('#diagnostic-topic-status'), diagTopicTone, diagnostics.length ? levelLabel(status.diagnostics?.highest_level ?? 0) : 'WAIT');
  $('#diagnostic-topic-detail').textContent = diagnostics.length
    ? `${diagnostics.length} сообщений`
    : `/diagnostics: ${formatAge(topicState.diagnostics?.age_sec)}`;

  const detailList = $('#diagnostic-details');
  detailList.innerHTML = '';
  if (!diagnostics.length) {
    const empty = document.createElement('div');
    empty.className = 'status-note';
    empty.textContent = 'Сообщения /diagnostics пока не поступали.';
    detailList.append(empty);
  } else {
    diagnostics.forEach((item) => {
      const card = document.createElement('article');
      card.className = 'diagnostic-item';
      const header = document.createElement('header');
      const title = document.createElement('strong');
      title.textContent = item.name || 'diagnostic';
      const badge = document.createElement('span');
      badge.className = `status-value ${levelTone(item.level)}`;
      badge.textContent = levelLabel(item.level);
      header.append(title, badge);

      const text = document.createElement('p');
      text.textContent = item.message || '—';

      const values = document.createElement('dl');
      Object.entries(item.values || {}).slice(0, 12).forEach(([key, value]) => {
        const dt = document.createElement('dt');
        dt.textContent = key;
        const dd = document.createElement('dd');
        dd.textContent = String(value);
        values.append(dt, dd);
      });

      card.append(header, text);
      if (values.children.length) {
        card.append(values);
      }
      detailList.append(card);
    });
  }
}

function renderActivity() {
  const container = $('#activity-list');
  container.innerHTML = '';
  if (!state.activity.length) {
    const empty = document.createElement('div');
    empty.className = 'status-note';
    empty.textContent = 'Журнал пока пуст.';
    container.append(empty);
    return;
  }

  state.activity.forEach((item) => {
    const card = document.createElement('article');
    card.className = 'activity-item';
    const header = document.createElement('header');
    const title = document.createElement('strong');
    title.textContent = `${item.source || 'web'} · ${new Date((item.timestamp || 0) * 1000).toLocaleTimeString()}`;
    const message = document.createElement('span');
    message.className = 'status-value';
    message.textContent = item.message || 'Activity';
    header.append(title, message);

    const body = document.createElement('p');
    body.textContent = item.message || '—';
    card.append(header, body);

    if (item.details && Object.keys(item.details).length) {
      const details = document.createElement('pre');
      details.className = 'message-box';
      details.textContent = pretty(item.details);
      card.append(details);
    }
    container.append(card);
  });
}

async function refreshActivity() {
  try {
    const payload = await api('/api/activity?limit=120');
    state.activity = safeArray(payload.items);
    renderActivity();
  } catch (error) {
    // Keep previous activity in place.
  }
}

function renderMotionStatus(motion) {
  const lines = [];
  if (!motion || !motion.running) {
    lines.push('Исполнитель не запущен.');
  } else {
    lines.push(`PID: ${motion.pid}`);
    lines.push(`Старт: ${motion.started_at ? new Date(motion.started_at * 1000).toLocaleString() : '—'}`);
    lines.push(`Команда: ${(motion.command || []).join(' ') || '—'}`);
    lines.push('');
  }
  if (motion?.return_code != null) {
    lines.push(`Последний код завершения: ${motion.return_code}`);
    lines.push('');
  }
  const logLines = safeArray(motion?.log);
  if (logLines.length) {
    lines.push(...logLines);
  }
  $('#motion-log').textContent = lines.join('\n');
  $('#route-stop').disabled = !motion?.running;
  $('#plan-run').disabled = Boolean(motion?.running);

  const warning = $('#drive-warning');
  if (motion?.running) {
    warning.textContent = 'Маршрут сейчас выполняется. Ручное управление может конфликтовать с исполнителем.';
    warning.classList.remove('hidden');
  } else {
    warning.classList.add('hidden');
    warning.textContent = '';
  }
}

function renderSettings() {
  if (state.identity) {
    $('#identity-json').textContent = pretty(state.identity);
  }
  if (state.config) {
    $('#config-json').textContent = pretty(state.config);
    $('#drive-topic-label').textContent = state.config.command_topic || '/cmd_vel';
  }
}

function rosboardUrl() {
  const web = state.config?.web || {};
  if (web.rosboard_enabled === false) return '';
  const port = Number(web.rosboard_port || 8888) || 8888;
  const protocol = location.protocol === 'https:' ? 'http:' : (location.protocol || 'http:');
  return `${protocol}//${location.hostname}:${port}/`;
}

function updateOverviewRosboardLink() {
  const button = $('#overview-open-rosboard');
  const note = $('#overview-rosboard-url');
  if (!button || !note) return;
  const url = rosboardUrl();
  const enabled = Boolean(url);
  button.disabled = !enabled;
  note.textContent = enabled ? `Rosboard: ${url}` : 'Rosboard: выключен';
}

function updateExternalToolButtons() {
  const terminalButton = $('#overview-open-terminal');
  if (terminalButton) {
    terminalButton.disabled = false;
    terminalButton.title = 'Открыть встроенный терминал робота';
  }
}

function configureTerminal() {
  const terminalButton = $('#nav-terminal');
  terminalButton.disabled = false;
  updateExternalToolButtons();
}

function terminalUrl() {
  const web = state.config?.web || {};
  if (!web.terminal_enabled) return '';
  if (web.terminal_url) return web.terminal_url;
  const port = web.terminal_port || 7681;
  const path = web.terminal_path || '/terminal/';
  return `${location.protocol}//${location.hostname}:${port}${path}`;
}

function refreshTerminalFrame(force = false) {
  if (state.page === 'terminal') startTerminalSession(Boolean(force));
}

function cleanTerminalText(value) {
  return String(value || '')
    .replace(/\x1b\][^\x07]*(?:\x07|\x1b\\)/g, '')
    .replace(/\x1b\[[0-?]*[ -/]*[@-~]/g, '')
    .replace(/\r/g, '');
}

function stopTerminalPolling() {
  if (state.terminalTimer) window.clearTimeout(state.terminalTimer);
  state.terminalTimer = null;
}

async function pollTerminalOutput() {
  if (state.page !== 'terminal' || state.terminalPolling) return;
  state.terminalPolling = true;
  try {
    const sessionId = state.terminalSession;
    const result = await api(`/api/terminal/session/output?cursor=${state.terminalCursor}&session_id=${sessionId}`);
    if (sessionId !== state.terminalSession) return;
    const output = $('#terminal-command-output');
    if (result.reset) robotTerminal.reset();
    if (result.output) robotTerminal.write(result.output);
    state.terminalCursor = Number(result.cursor || 0);
    output.scrollTop = output.scrollHeight;
    $('#terminal-title').textContent = result.running ? 'Терминал робота · подключён' : 'Терминал робота · сессия завершена';
  } catch (error) {
    $('#terminal-title').textContent = `Терминал робота · ${String(error.message || error)}`;
  } finally {
    state.terminalPolling = false;
    if (state.page === 'terminal') {
      state.terminalTimer = window.setTimeout(pollTerminalOutput, 300);
    }
  }
}

async function startTerminalSession(restart = false) {
  ensureRobotTerminal();
  stopTerminalPolling();
  try {
    const sessionId = state.terminalSession;
    const result = await api('/api/terminal/session/start', {
      method: 'POST',
      body: JSON.stringify({ restart, session_id: sessionId }),
    });
    if (sessionId !== state.terminalSession) return;
    if (restart || !state.terminalStarted) {
      state.terminalCursor = Number(result.cursor || 0);
      robotTerminal.reset();
    }
    state.terminalStarted = true;
    resizeRobotTerminal();
    robotTerminal.focus();
    $('#terminal-title').textContent = 'Терминал робота · подключён';
    await pollTerminalOutput();
  } catch (error) {
    $('#terminal-title').textContent = `Терминал робота · ${String(error.message || error)}`;
    showToast(String(error.message || error), 'error');
  }
}

async function refreshIdentityAndConfig() {
  try {
    const [identity, config] = await Promise.all([
      api('/api/identity'),
      api('/api/config'),
    ]);
    state.identity = identity;
    state.config = config;
    applyFeatureDefaultsFromConfig(config);
    applyIdentity();
    renderSettings();
    updateOverviewRosboardLink();
    configureTerminal();
    updateExternalToolButtons();
    applyRouteDefaults(config);
    refreshDriveConfigFromConfig(config);
  } catch (error) {
    state.apiHealthy = false;
    updateHealthIndicators();
  }
}

async function refreshStatus() {
  if (state.statusBusy) return;
  state.statusBusy = true;
  try {
    const payload = await api('/api/status', {signal: AbortSignal.timeout(3000)});
    state.status = payload;
    state.apiHealthy = true;
    state.rosHealthy = Boolean(payload.ok);
    if (!state.system) {
      state.system = payload.system;
    }
    if (!state.identity) {
      state.identity = payload.identity;
    }
    applyIdentity();
    appendUniqueTrailPoint(payload.odom);
    renderOverview();
    renderDiagnostics();
    renderVisualization();
    renderMotionStatus(payload.motion);
    updateHealthIndicators();
  } catch (error) {
    state.apiHealthy = false;
    state.rosHealthy = false;
    updateHealthIndicators();
  } finally {
    state.statusBusy = false;
  }
}

function renderList(element, items, selectedKey, factory) {
  element.innerHTML = '';
  if (!items.length) {
    const empty = document.createElement('div');
    empty.className = 'status-note';
    empty.textContent = 'Ничего не найдено.';
    element.append(empty);
    return;
  }
  items.forEach((item) => {
    const node = factory(item);
    if (selectedKey && node.dataset.key === selectedKey) {
      node.classList.add('active');
    }
    element.append(node);
  });
}

function setRosTab(tab) {
  const next = ['topics', 'services', 'nodes'].includes(tab) ? tab : 'topics';
  state.rosTab = next;
  localStorage.setItem(STORAGE_KEYS.rosTab, next);
  $$('.section-tab[data-ros-tab]').forEach((button) => {
    button.classList.toggle('active', button.dataset.rosTab === next);
  });
  $$('.ros-section').forEach((section) => {
    section.classList.toggle('active', section.id === `ros-section-${next}`);
  });
}

function filterByText(items, text, mapper) {
  const needle = text.trim().toLowerCase();
  if (!needle) return items;
  return items.filter((item) => mapper(item).toLowerCase().includes(needle));
}

async function refreshRosGraph() {
  try {
    const payload = await api('/api/ros/graph');
    state.rosGraph = payload;
    state.apiHealthy = true;
    state.rosHealthy = true;
    $('#ros-nodes-count').textContent = payload.nodes.length;
    $('#ros-topics-count').textContent = payload.topics.length;
    $('#ros-services-count').textContent = payload.services.length;
    $('#ros-nodes-tab-count').textContent = payload.nodes.length;
    $('#ros-topics-tab-count').textContent = payload.topics.length;
    $('#ros-services-tab-count').textContent = payload.services.length;
    $('#ros-image-count').textContent = payload.image_topics.length;
    renderTopics();
    renderServices();
    renderNodes();
    if (state.selectedNode) {
      selectNode(state.selectedNode);
    } else {
      renderNodeDetails();
    }
    renderCameraTopics();
    renderLidarTopics();
    renderOctolinerTopics();
    updateHealthIndicators();
  } catch (error) {
    state.rosHealthy = false;
    updateHealthIndicators();
  }
}

function renderNodes() {
  const items = filterByText(
    safeArray(state.rosGraph?.nodes),
    $('#nodes-filter').value,
    (item) => `${item.full_name || ''} ${item.namespace || ''}`,
  );
  renderList($('#nodes-list'), items, state.selectedNode, (item) => {
    const button = document.createElement('button');
    button.type = 'button';
    button.className = 'list-item';
    button.dataset.key = item.full_name || '';
    button.innerHTML = `<strong>${item.full_name || item.name || 'node'}</strong>`;
    button.addEventListener('click', () => selectNode(item.full_name));
    return button;
  });
}

function renderNodeDetails(info) {
  const node = info || state.selectedNodeInfo;
  if (!node) {
    $('#node-detail-title').textContent = 'Выбери ноду';
    renderDetailList($('#node-meta'), []);
    $('#node-detail-json').textContent = 'Нет данных.';
    return;
  }
  $('#node-detail-title').textContent = node.full_name || node.name || 'Нода';
  renderDetailList($('#node-meta'), [
    { label: 'Имя', value: node.name || '—' },
    { label: 'Namespace', value: node.namespace || '/' },
    { label: 'Полный путь', value: node.full_name || '—' },
  ]);
  $('#node-detail-json').textContent = pretty(node);
}

function selectNode(fullName) {
  const info = safeArray(state.rosGraph?.nodes).find((item) => item.full_name === fullName) || null;
  state.selectedNode = fullName || null;
  state.selectedNodeInfo = info;
  renderNodes();
  renderNodeDetails(info);
}

function renderTopics() {
  const items = filterByText(
    safeArray(state.rosGraph?.topics),
    $('#topics-filter').value,
    (item) => `${item.name || ''} ${(item.types || []).join(' ')}`,
  );
  renderList($('#topics-list'), items, state.selectedTopic, (item) => {
    const button = document.createElement('button');
    button.type = 'button';
    button.className = 'list-item';
    button.dataset.key = item.name || '';
    const type = safeArray(item.types)[0] || 'unknown';
    const tags = [`pub ${item.publishers ?? 0}`, `sub ${item.subscribers ?? 0}`];
    if (item.is_image) tags.push('image');
    button.innerHTML = `<strong>${item.name || 'topic'}</strong><small>${type}</small><small>${tags.join(' · ')}</small>`;
    button.addEventListener('click', () => selectTopic(item.name, type));
    return button;
  });
}

function renderServices() {
  const items = filterByText(
    safeArray(state.rosGraph?.services),
    $('#services-filter').value,
    (item) => `${item.name || ''} ${(item.types || []).join(' ')}`,
  );
  renderList($('#services-list'), items, state.selectedService, (item) => {
    const button = document.createElement('button');
    button.type = 'button';
    button.className = 'list-item';
    button.dataset.key = item.name || '';
    const type = safeArray(item.types)[0] || 'unknown';
    button.innerHTML = `<strong>${item.name || 'service'}</strong><small>${type}</small>`;
    button.addEventListener('click', () => selectService(item.name, type));
    return button;
  });
}

async function selectTopic(name, type) {
  state.selectedTopic = name;
  state.selectedTopicType = type;
  $('#topic-detail-title').textContent = name || 'Выбери топик';
  $('#topic-action-status').textContent = 'Загрузка топика...';
  renderTopics();
  try {
    const info = await api(`/api/ros/topic?name=${encodeURIComponent(name)}&type=${encodeURIComponent(type)}`);
    state.selectedTopicInfo = info;
    renderDetailList($('#topic-meta'), [
      { label: 'Type', value: info.type || '—' },
      { label: 'Kind', value: info.kind || '—' },
      { label: 'Publishers', value: String(info.publishers ?? '—') },
      { label: 'Subscribers', value: String(info.subscribers ?? '—') },
      { label: 'Messages', value: String(info.message_count ?? '—') },
      { label: 'Age', value: info.age_sec == null ? '—' : formatAge(info.age_sec) },
    ]);
    if (info.kind === 'image') {
      $('#topic-payload').value = '';
      $('#topic-latest-message').textContent = pretty({
        note: 'Это image topic. Для просмотра кадра открой страницу "Камера".',
        frame_url: info.frame_url,
        width: info.width,
        height: info.height,
        encoding: info.encoding,
        last_error: info.last_error,
      });
      $('#topic-action-status').textContent = 'Image topic найден.';
    } else {
      $('#topic-payload').value = pretty(info.template || {});
      $('#topic-latest-message').textContent = pretty(info.latest_message || {});
      $('#topic-action-status').textContent = 'Можно публиковать JSON в этот топик.';
    }
  } catch (error) {
    $('#topic-action-status').textContent = String(error.message || error);
    $('#topic-latest-message').textContent = 'Ошибка чтения топика.';
  }
}

async function refreshSelectedTopic() {
  if (state.selectedTopic && state.selectedTopicType) {
    await selectTopic(state.selectedTopic, state.selectedTopicType);
  }
}

function restoreTopicTemplate() {
  if (state.selectedTopicInfo?.template) {
    $('#topic-payload').value = pretty(state.selectedTopicInfo.template);
  }
}

async function publishSelectedTopic() {
  if (!state.selectedTopic || !state.selectedTopicType) return;
  try {
    const message = JSON.parse($('#topic-payload').value || '{}');
    await api('/api/ros/topic/publish', {
      method: 'POST',
      body: JSON.stringify({
        topic: state.selectedTopic,
        type: state.selectedTopicType,
        message,
      }),
    });
    $('#topic-action-status').textContent = 'Сообщение опубликовано.';
    showToast(`Опубликовано в ${state.selectedTopic}`);
    await recordActivity('Опубликовано сообщение в ROS topic', { topic: state.selectedTopic, type: state.selectedTopicType }, 'ros');
    await refreshSelectedTopic();
  } catch (error) {
    $('#topic-action-status').textContent = String(error.message || error);
    showToast(String(error.message || error), 'error');
  }
}

async function selectService(name, type) {
  state.selectedService = name;
  state.selectedServiceType = type;
  $('#service-detail-title').textContent = name || 'Выбери сервис';
  $('#service-action-status').textContent = 'Загрузка сервиса...';
  renderServices();
  try {
    const info = await api(`/api/ros/service?name=${encodeURIComponent(name)}&type=${encodeURIComponent(type)}`);
    state.selectedServiceInfo = info;
    renderDetailList($('#service-meta'), [
      { label: 'Type', value: info.type || '—' },
      { label: 'Ready', value: info.ready ? 'yes' : 'no' },
      { label: 'Available types', value: safeArray(info.available_types).join(', ') || '—' },
    ]);
    $('#service-request').value = pretty(info.request_template || {});
    $('#service-response').textContent = 'Ожидание вызова.';
    $('#service-action-status').textContent = 'Можно вызвать сервис.';
  } catch (error) {
    $('#service-action-status').textContent = String(error.message || error);
    $('#service-response').textContent = 'Ошибка чтения сервиса.';
  }
}

async function refreshSelectedService() {
  if (state.selectedService && state.selectedServiceType) {
    await selectService(state.selectedService, state.selectedServiceType);
  }
}

function restoreServiceTemplate() {
  if (state.selectedServiceInfo?.request_template) {
    $('#service-request').value = pretty(state.selectedServiceInfo.request_template);
  }
}

async function callSelectedService() {
  if (!state.selectedService || !state.selectedServiceType) return;
  try {
    const request = JSON.parse($('#service-request').value || '{}');
    const response = await api('/api/ros/service/call', {
      method: 'POST',
      body: JSON.stringify({
        service: state.selectedService,
        type: state.selectedServiceType,
        request,
      }),
    });
    $('#service-response').textContent = pretty(response.response || response);
    $('#service-action-status').textContent = `Сервис вызван за ${formatFloat(response.duration_sec, 3)} s`;
    showToast(`Сервис ${state.selectedService} вызван`);
    await recordActivity('Вызван ROS service', { service: state.selectedService, type: state.selectedServiceType }, 'ros');
  } catch (error) {
    $('#service-action-status').textContent = String(error.message || error);
    showToast(String(error.message || error), 'error');
  }
}

function renderCameraTopics() {
  const select = $('#camera-topic-select');
  const topics = safeArray(state.rosGraph?.image_topics).filter((item) => item.name === '/camera/qr_annotated').sort((left, right) => {
    const leftType = safeArray(left.types)[0] || '';
    const rightType = safeArray(right.types)[0] || '';
    const leftCompressed = leftType.includes('CompressedImage') || left.name?.endsWith('/compressed');
    const rightCompressed = rightType.includes('CompressedImage') || right.name?.endsWith('/compressed');
    if (leftCompressed !== rightCompressed) {
      return leftCompressed ? -1 : 1;
    }
    return String(left.name || '').localeCompare(String(right.name || ''));
  });
  select.innerHTML = '';
  if (!topics.length) {
    const option = document.createElement('option');
    option.value = '';
    option.textContent = 'Нет image topics';
    select.append(option);
    state.selectedCameraTopic = null;
    state.selectedCameraType = null;
    stopCameraLoop();
    $('#camera-empty').classList.remove('hidden');
    return;
  }

  if (!topics.some((item) => item.name === state.selectedCameraTopic)) {
    const preferred = topics.find((item) => {
      const type = safeArray(item.types)[0] || '';
      return type.includes('CompressedImage') || item.name?.endsWith('/compressed');
    }) || topics[0];
    state.selectedCameraTopic = preferred.name;
    state.selectedCameraType = safeArray(preferred.types)[0] || '';
  }

  topics.forEach((item) => {
    const option = document.createElement('option');
    option.value = item.name;
    option.dataset.type = safeArray(item.types)[0] || '';
    const labels = {
      '/camera/image_raw': 'Обычная камера',
      '/camera/qr_annotated': 'QR + нейросеть',
      '/sign_detector/image': 'Дорожные знаки',
      '/sign_detector/image/compressed': 'Дорожные знаки (JPEG)',
    };
    option.textContent = `${labels[item.name] || item.name} — ${item.name}`;
    option.selected = item.name === state.selectedCameraTopic;
    select.append(option);
  });
}

function summarizeCameraCapabilities(capabilities) {
  if (!capabilities?.available) {
    return pretty({
      device: capabilities?.device || state.cameraSettings?.parameters?.device || '/dev/video0',
      error: capabilities?.error || 'v4l2-ctl недоступен',
    });
  }

  const rows = [];
  safeArray(capabilities.formats).forEach((format) => {
    rows.push(`${format.pixel_format} · ${format.description}`);
    safeArray(format.modes).forEach((mode) => {
      const fpsValues = safeArray(mode.fps);
      const fpsText = fpsValues.length
        ? fpsValues.map((value) => Number(value).toFixed(value >= 10 ? 0 : 1)).join(', ')
        : 'unknown';
      rows.push(`  ${mode.width}x${mode.height} @ ${fpsText} fps`);
    });
  });
  return rows.length ? rows.join('\n') : 'Форматы не обнаружены.';
}

function setCameraSettingsForm(parameters = {}) {
  $('#camera-setting-device').value = parameters.device || '/dev/video0';
  $('#camera-setting-frame-id').value = parameters.frame_id || 'camera_optical_frame';
  $('#camera-setting-image-topic').value = parameters.image_topic || '/image_raw';
  $('#camera-setting-compressed-topic').value = parameters.compressed_image_topic || '/image_raw/compressed';
  $('#camera-setting-width').value = String(parameters.width ?? 1280);
  $('#camera-setting-height').value = String(parameters.height ?? 720);
  $('#camera-setting-fps').value = String(parameters.fps ?? 30);
  $('#camera-setting-jpeg-quality').value = String(parameters.jpeg_quality ?? 85);
  $('#camera-setting-use-mjpeg').checked = Boolean(parameters.use_mjpeg ?? true);
  $('#camera-setting-publish-raw').checked = Boolean(parameters.publish_raw ?? true);
  $('#camera-setting-publish-compressed').checked = Boolean(parameters.publish_compressed ?? true);
}

function renderCameraVisionModels(models = [], selectedModelName = '') {
  const select = $('#camera-vision-model');
  select.innerHTML = '';
  select.disabled = true;
  if (!models.length) {
    const option = document.createElement('option');
    option.value = '';
    option.textContent = 'Нет manifest-файлов моделей';
    select.append(option);
    return;
  }

  models.forEach((model) => {
    const option = document.createElement('option');
    option.value = model.id || '';
    option.textContent = model.valid
      ? `${model.name || model.id} (${model.format})`
      : `${model.name || model.id} — ${model.error || 'invalid'}`;
    option.disabled = !model.valid;
    option.selected = option.value === selectedModelName;
    select.append(option);
  });

  if (!select.value) {
    const firstValid = models.find((item) => item.valid);
    if (firstValid) {
      select.value = firstValid.id;
    }
  }
  select.title = 'Сейчас используется одна фиксированная модель';
}

function setCameraVisionPanelExpanded(enabled) {
  $('#camera-vision-body').classList.toggle('hidden', !enabled);
  $('#camera-vision-status').classList.toggle('hidden', !enabled);
  $('#camera-vision-details').classList.toggle('hidden', !enabled);
}

function setCameraVisionSettingsForm(payload = {}) {
  const parameters = payload.parameters || {};
  const enabled = Boolean(parameters.enabled ?? false);
  $('#camera-vision-enabled').checked = enabled;
  setCameraVisionPanelExpanded(enabled);
  renderCameraVisionModels(safeArray(payload.models), String(parameters.model_name || ''));
  $('#camera-vision-input-topic').value = parameters.input_topic || '/image_raw';
  $('#camera-vision-output-topic').value = parameters.processed_image_topic || '/image_processed';
  $('#camera-vision-output-compressed-topic').value = parameters.processed_compressed_image_topic || '/image_processed/compressed';
  $('#camera-vision-detections-topic').value = parameters.detections_topic || '/detections';
  $('#camera-vision-fps').value = String(parameters.max_processing_fps ?? 10.0);
  $('#camera-vision-confidence').value = String(parameters.confidence_threshold ?? 0.30);
  $('#camera-vision-nms').value = String(parameters.nms_threshold ?? 0.45);
  $('#camera-vision-publish-raw').checked = Boolean(parameters.publish_raw ?? true);
  $('#camera-vision-publish-compressed').checked = Boolean(parameters.publish_compressed ?? true);
  $('#camera-vision-publish-detections').checked = Boolean(parameters.publish_detections ?? true);
  $('#camera-vision-annotate-labels').checked = Boolean(parameters.annotate_labels ?? true);
  $('#camera-vision-annotate-confidence').checked = Boolean(parameters.annotate_confidence ?? true);
}

function summarizeCameraVisionDetails(payload = {}) {
  const selected = payload.selected_model || null;
  return pretty({
    node_name: payload.node_name || '/camera_detector_node',
    models_directory: payload.models_directory || 'models',
    detections_topic: payload.parameters?.detections_topic || '/detections',
    selected_model: selected ? {
      id: selected.id,
      name: selected.name,
      valid: selected.valid,
      model_path: selected.model_path,
      input_size: selected.input_size,
      description: selected.description,
      error: selected.error,
    } : null,
    models_count: safeArray(payload.models).length,
    available_models: safeArray(payload.models).map((model) => ({
      id: model.id,
      valid: model.valid,
      format: model.format,
      model_path: model.model_path,
      error: model.error,
    })),
    notes: payload.notes || {},
  });
}

async function refreshCameraVisionSettings() {
  try {
    const payload = await api('/api/vision/settings');
    state.cameraVisionSettings = payload;
    setCameraVisionSettingsForm(payload);
    $('#camera-vision-details').textContent = summarizeCameraVisionDetails(payload);
    const selected = payload.selected_model;
    if (payload.parameters?.enabled) {
      $('#camera-vision-status').textContent = selected?.valid
        ? `Выбрана модель ${selected.name || selected.id}.`
        : 'Обработка включена, но модель ещё не готова.';
    } else {
      $('#camera-vision-status').textContent = 'Обработка нейросетью выключена.';
    }
    return payload;
  } catch (error) {
    $('#camera-vision-status').textContent = String(error.message || error);
    $('#camera-vision-details').textContent = 'Не удалось получить параметры обработки.';
    return null;
  }
}

async function toggleCameraVisionEnabled() {
  const enabled = $('#camera-vision-enabled').checked;
  setCameraVisionPanelExpanded(enabled);
  try {
    $('#camera-vision-status').textContent = enabled
      ? 'Включение обработки нейросетью...'
      : 'Выключение обработки нейросетью...';
    const payload = await api('/api/vision/settings', {
      method: 'POST',
      body: JSON.stringify(cameraVisionSettingsPayloadFromForm()),
    });
    state.cameraVisionSettings = payload;
    setCameraVisionSettingsForm(payload);
    $('#camera-vision-details').textContent = summarizeCameraVisionDetails(payload);
    $('#camera-vision-status').textContent = enabled
      ? 'Обработка нейросетью включена.'
      : 'Обработка нейросетью выключена.';
    await Promise.all([
      refreshRosGraph(),
      refreshCameraVisionSettings(),
    ]);
    renderCameraTopics();
  } catch (error) {
    $('#camera-vision-status').textContent = String(error.message || error);
    showToast(String(error.message || error), 'error');
    await refreshCameraVisionSettings();
  }
}

function cameraVisionSettingsPayloadFromForm() {
  return {
    enabled: $('#camera-vision-enabled').checked,
    model_name: $('#camera-vision-model').value.trim(),
    input_topic: $('#camera-vision-input-topic').value.trim(),
    processed_image_topic: $('#camera-vision-output-topic').value.trim(),
    processed_compressed_image_topic: $('#camera-vision-output-compressed-topic').value.trim(),
    detections_topic: $('#camera-vision-detections-topic').value.trim(),
    publish_raw: $('#camera-vision-publish-raw').checked,
    publish_compressed: $('#camera-vision-publish-compressed').checked,
    publish_detections: $('#camera-vision-publish-detections').checked,
    confidence_threshold: Number($('#camera-vision-confidence').value || '0.25'),
    nms_threshold: Number($('#camera-vision-nms').value || '0.45'),
    max_processing_fps: Number($('#camera-vision-fps').value || '8'),
    annotate_labels: $('#camera-vision-annotate-labels').checked,
    annotate_confidence: $('#camera-vision-annotate-confidence').checked,
  };
}

async function applyCameraVisionSettings() {
  try {
    $('#camera-vision-status').textContent = 'Применение параметров обработки...';
    const payload = await api('/api/vision/settings', {
      method: 'POST',
      body: JSON.stringify(cameraVisionSettingsPayloadFromForm()),
    });
    state.cameraVisionSettings = payload;
    setCameraVisionSettingsForm(payload);
    $('#camera-vision-details').textContent = summarizeCameraVisionDetails(payload);
    $('#camera-vision-status').textContent = 'Параметры обработки применены.';
    showToast('Настройки обработки камеры применены');
    await Promise.all([
      refreshRosGraph(),
      refreshCameraVisionSettings(),
    ]);
    renderCameraTopics();
  } catch (error) {
    $('#camera-vision-status').textContent = String(error.message || error);
    showToast(String(error.message || error), 'error');
  }
}

async function openProcessedCameraStream() {
  const parameters = state.cameraVisionSettings?.parameters || cameraVisionSettingsPayloadFromForm();
  const targetTopic = parameters.publish_compressed
    ? parameters.processed_compressed_image_topic
    : parameters.processed_image_topic;
  if (!targetTopic) {
    showToast('Не задан topic для обработанного потока', 'error');
    return;
  }
  await refreshRosGraph();
  const topic = safeArray(state.rosGraph?.image_topics).find((item) => item.name === targetTopic);
  if (!topic) {
    showToast('Обработанный поток пока не появился в ROS graph', 'error');
    $('#camera-vision-status').textContent = 'Обработанный image topic пока недоступен.';
    renderCameraTopics();
    return;
  }
  state.selectedCameraTopic = topic.name;
  state.selectedCameraType = safeArray(topic.types)[0] || null;
  renderCameraTopics();
  $('#camera-topic-select').value = topic.name;
  $('#camera-vision-status').textContent = `Открыт обработанный поток: ${topic.name}`;
  await connectCamera();
}

function voiceStatusText(payload = {}) {
  if (!payload.enabled) {
    return 'Обработка голоса выключена.';
  }
  const parts = [];
  if (payload.managed_running) {
    parts.push(`Обработка включена${payload.pid ? `, PID ${payload.pid}` : ''}.`);
  } else if (payload.external_running) {
    parts.push('Нода распознавания запущена вне веб-интерфейса.');
  } else if (payload.node_visible) {
    parts.push('Нода распознавания видна в ROS graph.');
  } else {
    parts.push('Запуск обработки голоса...');
  }
  if (payload.latest_status) {
    parts.push(payload.latest_status);
  }
  return parts.join(' ');
}

function renderVoiceHistory(history = [], enabled = false) {
  const container = $('#voice-history');
  container.innerHTML = '';
  const items = safeArray(history).slice().reverse();
  if (!enabled) {
    const empty = document.createElement('div');
    empty.className = 'notice';
    empty.textContent = 'Распознавание выключено.';
    container.append(empty);
    return;
  }
  if (!items.length) {
    const empty = document.createElement('div');
    empty.className = 'status-note';
    empty.textContent = 'Текст пока не распознан.';
    container.append(empty);
    return;
  }
  items.forEach((item) => {
    const row = document.createElement('article');
    row.className = 'voice-history-item';
    const timeText = item.timestamp
      ? new Date(item.timestamp * 1000).toLocaleTimeString()
      : '—';
    row.innerHTML = `<strong>${escapeHtml(timeText)}</strong><span>${escapeHtml(item.text || '—')}</span>`;
    container.append(row);
  });
}

function renderVoiceStatus(payload = {}) {
  const enabled = Boolean(payload.enabled);
  $('#voice-enabled').checked = enabled;
  $('#voice-body').classList.toggle('hidden', !enabled);
  $('#voice-status').textContent = voiceStatusText(payload);
  renderDetailList($('#voice-meta'), [
    { label: 'Нода', value: payload.node_name || '/waveshare_audio_node' },
    { label: 'Текстовый топик', value: payload.text_topic || '/voice/text' },
    { label: 'Статус топик', value: payload.status_topic || '/waveshare_audio/status' },
    { label: 'Конфиг', value: payload.config_file || 'по умолчанию' },
    { label: 'Режим', value: payload.external_running ? 'external' : (payload.managed_running ? 'web-managed' : 'off') },
  ]);
  $('#voice-latest-text').textContent = payload.latest_text || 'Текст пока не получен.';
  renderVoiceHistory(payload.history, enabled);
  const logLines = safeArray(payload.log);
  $('#voice-log').textContent = logLines.length
    ? logLines.slice(-40).join('\n')
    : 'Лог распознавания пуст.';
}

async function refreshVoiceStatus() {
  try {
    const payload = await api('/api/voice/status');
    state.voiceStatus = payload;
    renderVoiceStatus(payload);
    return payload;
  } catch (error) {
    $('#voice-status').textContent = String(error.message || error);
    $('#voice-log').textContent = 'Не удалось получить статус распознавания.';
    return null;
  }
}

async function toggleVoiceEnabled() {
  const enabled = $('#voice-enabled').checked;
  $('#voice-body').classList.toggle('hidden', !enabled);
  $('#voice-status').textContent = enabled
    ? 'Запуск обработки голоса...'
    : 'Остановка обработки голоса...';
  try {
    const payload = await api('/api/voice/control', {
      method: 'POST',
      body: JSON.stringify({ enabled }),
    });
    state.voiceStatus = payload;
    renderVoiceStatus(payload);
    showToast(enabled ? 'Распознавание голоса включено' : 'Распознавание голоса выключено');
    await refreshRosGraph();
  } catch (error) {
    $('#voice-status').textContent = String(error.message || error);
    showToast(String(error.message || error), 'error');
    await refreshVoiceStatus();
  }
}

async function refreshCameraSettings() {
  try {
    const payload = await api('/api/camera/settings');
    state.cameraSettings = payload;
    state.cameraSettingsLastRefresh = Date.now();
    setCameraSettingsForm(payload.parameters || {});
    $('#camera-capabilities').textContent = summarizeCameraCapabilities(payload.capabilities);
    $('#camera-settings-status').textContent = `Параметры получены из ${payload.node_name || '/usb_camera_node'}.`;
  } catch (error) {
    $('#camera-settings-status').textContent = String(error.message || error);
    $('#camera-capabilities').textContent = 'Не удалось получить параметры камеры.';
  }
}

function cameraSettingsPayloadFromForm() {
  return {
    device: $('#camera-setting-device').value.trim(),
    frame_id: $('#camera-setting-frame-id').value.trim(),
    image_topic: $('#camera-setting-image-topic').value.trim(),
    compressed_image_topic: $('#camera-setting-compressed-topic').value.trim(),
    width: Number($('#camera-setting-width').value || '1280'),
    height: Number($('#camera-setting-height').value || '720'),
    fps: Number($('#camera-setting-fps').value || '30'),
    jpeg_quality: Number($('#camera-setting-jpeg-quality').value || '85'),
    use_mjpeg: $('#camera-setting-use-mjpeg').checked,
    publish_raw: $('#camera-setting-publish-raw').checked,
    publish_compressed: $('#camera-setting-publish-compressed').checked,
  };
}

async function applyCameraSettings() {
  try {
    $('#camera-settings-status').textContent = 'Применение параметров камеры...';
    const payload = await api('/api/camera/settings', {
      method: 'POST',
      body: JSON.stringify(cameraSettingsPayloadFromForm()),
    });
    state.cameraSettings = payload;
    setCameraSettingsForm(payload.parameters || {});
    $('#camera-capabilities').textContent = summarizeCameraCapabilities(payload.capabilities);
    $('#camera-settings-status').textContent = 'Параметры камеры применены.';
    showToast('Настройки камеры применены');
    await Promise.all([refreshRosGraph(), refreshCameraSettings()]);
    await connectCamera();
  } catch (error) {
    $('#camera-settings-status').textContent = String(error.message || error);
    showToast(String(error.message || error), 'error');
  }
}

function applyCameraPreset(width, height, fps) {
  $('#camera-setting-width').value = String(width);
  $('#camera-setting-height').value = String(height);
  $('#camera-setting-fps').value = String(fps);
}

async function refreshCameraStatus() {
  if (state.cameraStatusBusy || !state.selectedCameraTopic || !state.selectedCameraType) return;
  state.cameraStatusBusy = true;
  try {
    const info = await api(`/api/camera/status?topic=${encodeURIComponent(state.selectedCameraTopic)}&type=${encodeURIComponent(state.selectedCameraType)}`, {signal: AbortSignal.timeout(2000)});
    const note = $('#camera-neural-status');
    if (note) {
      const neural = info.neural;
      note.textContent = !$('#camera-neural-enabled')?.checked
        ? 'Нейросеть: рамки выключены'
        : neural?.active
          ? `Нейросеть · ноутбук · ${Math.round(neural.inference_ms)} мс · ${neural.detections.map(item => `${item.label} ${Math.round(item.confidence * 100)}%`).join(', ') || 'объекты не обнаружены'}`
          : 'Нейросеть: нет свежего результата · показывается камера с QR';
    }
    return info;
  } catch (error) {
    return null;
  } finally {
    state.cameraStatusBusy = false;
  }
}

function startCameraLoop() {
  stopCameraLoop();
  if (!state.selectedCameraTopic || !state.selectedCameraType) {
    return;
  }
  const frame = $('#camera-frame');
  const canvas = $('#camera-mask-canvas');
  if (state.cameraMask) {
    frame.src = '';
    frame.classList.add('hidden');
    canvas.classList.remove('hidden');
    refreshCameraColorMask();
    state.cameraMaskTimer = window.setInterval(refreshCameraColorMask, 400);
  } else {
    canvas.classList.add('hidden');
    frame.classList.remove('hidden');
    state.cameraPreviewGeneration = (state.cameraPreviewGeneration || 0) + 1;
    state.cameraPreviewProgress = Date.now();
    refreshCameraPreview(state.cameraPreviewGeneration);
  }
  $('#camera-empty').classList.add('hidden');
  state.cameraTimer = window.setInterval(() => {
    if (!state.cameraMask && Date.now() - (state.cameraPreviewProgress || 0) > 15000) {
      state.cameraPreviewController?.abort();
      state.cameraPreviewGeneration += 1;
      state.cameraPreviewProgress = Date.now();
      refreshCameraPreview(state.cameraPreviewGeneration);
    }
    refreshCameraStatus();
    refreshQrStatus();
    refreshPerceptionHistory();
  }, 500);
}

let perceptionHistoryBusy = false;
let perceptionHistoryChecked = 0;
let perceptionHistoryRevision = null;
async function refreshPerceptionHistory() {
  const container = $('#perception-history');
  if (!container || perceptionHistoryBusy || Date.now() - perceptionHistoryChecked < 2000) return;
  perceptionHistoryBusy = true;
  perceptionHistoryChecked = Date.now();
  try {
    const data = await api('/api/perception/history', {signal: AbortSignal.timeout(2000)});
    $('#perception-save-status').textContent = data.save_error ? 'Не удалось сохранить снимки на роботе' : '';
    if (perceptionHistoryRevision === data.revision) return;
    perceptionHistoryRevision = data.revision;
    container.replaceChildren();
    if (!data.items.length) container.textContent = 'Пока нет снимков.';
    for (const record of data.items) {
      const figure = document.createElement('figure');
      const link = document.createElement('a');
      link.href = record.image_url;
      link.download = `${record.kind}-${record.id}.jpg`;
      const image = document.createElement('img');
      image.src = record.image_url;
      image.alt = record.label;
      link.append(image);
      const caption = document.createElement('figcaption');
      caption.textContent = record.kind === 'qr' ? `QR: ${record.text}`
        : `${record.label} · ${Math.round(record.confidence * 100)}%`;
      const time = document.createElement('small');
      time.textContent = new Date(record.timestamp * 1000).toLocaleString('ru-RU');
      caption.append(time);
      figure.append(link, caption);
      container.append(figure);
    }
  } catch (_error) {
    $('#perception-save-status').textContent = 'Снимки временно недоступны';
  } finally {
    perceptionHistoryBusy = false;
  }
}

async function refreshCameraPreview(generation) {
  if (generation !== state.cameraPreviewGeneration || !state.selectedCameraTopic) return;
  const controller = new AbortController();
  state.cameraPreviewController = controller;
  const timeout = window.setTimeout(() => controller.abort(), 12000);
  let nextUrl = null;
  try {
    const neural = $('#camera-neural-enabled')?.checked ? '&neural=1' : '';
    const response = await fetch(`/api/camera/frame?preview=1${neural}&topic=${encodeURIComponent(state.selectedCameraTopic)}&type=${encodeURIComponent(state.selectedCameraType)}&t=${Date.now()}`, {
      cache: 'no-store', signal: controller.signal,
    });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const blob = await response.blob();
    nextUrl = URL.createObjectURL(blob);
    const decoded = new Image();
    decoded.src = nextUrl;
    await Promise.race([
      decoded.decode(),
      new Promise((_, reject) => controller.signal.addEventListener('abort', () => reject(new Error('Frame timeout')), { once: true })),
    ]);
    if (generation !== state.cameraPreviewGeneration) return;
    const previous = state.cameraPreviewBlob;
    state.cameraPreviewBlob = nextUrl;
    nextUrl = null;
    $('#camera-frame').src = state.cameraPreviewBlob;
    if (previous) URL.revokeObjectURL(previous);
    $('#camera-empty').classList.add('hidden');
  } catch (error) {
    if (generation !== state.cameraPreviewGeneration) return;
    $('#camera-empty').textContent = `Загрузка кадра прервана (${error.message || error}). Переподключаюсь…`;
    $('#camera-empty').classList.remove('hidden');
  } finally {
    window.clearTimeout(timeout);
    if (nextUrl) URL.revokeObjectURL(nextUrl);
    if (generation === state.cameraPreviewGeneration) state.cameraPreviewProgress = Date.now();
  }
  if (generation === state.cameraPreviewGeneration) {
    window.setTimeout(() => refreshCameraPreview(generation), 200);
  }
}

async function refreshCameraColorMask() {
  if (state.cameraMaskBusy || !state.cameraMask || !state.selectedCameraTopic || !state.selectedCameraType) return;
  state.cameraMaskBusy = true;
  try {
    const url = `/api/camera/frame?topic=${encodeURIComponent(state.selectedCameraTopic)}&type=${encodeURIComponent(state.selectedCameraType)}&t=${Date.now()}`;
    const response = await fetch(url, { cache: 'no-store' });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const bitmap = await createImageBitmap(await response.blob());
    const canvas = $('#camera-mask-canvas');
    canvas.width = bitmap.width;
    canvas.height = bitmap.height;
    const ctx = canvas.getContext('2d', { willReadFrequently: true });
    ctx.drawImage(bitmap, 0, 0);
    bitmap.close();
    const image = ctx.getImageData(0, 0, canvas.width, canvas.height);
    const data = image.data;
    const selected = state.cameraMask;
    for (let index = 0; index < data.length; index += 4) {
      const red = data[index];
      const green = data[index + 1];
      const blue = data[index + 2];
      const keep = selected === 'red'
        ? red > 70 && red > green * 1.25 && red > blue * 1.25
        : selected === 'green'
          ? green > 60 && green > red * 1.15 && green > blue * 1.15
          : blue > 60 && blue > red * 1.2 && blue > green * 1.1;
      if (!keep) {
        data[index] = 0;
        data[index + 1] = 0;
        data[index + 2] = 0;
      }
    }
    ctx.putImageData(image, 0, 0);
    $('#camera-empty').classList.add('hidden');
  } catch (_error) {
    $('#camera-empty').classList.remove('hidden');
  } finally {
    state.cameraMaskBusy = false;
  }
}

async function refreshQrStatus() {
  const note = $('#camera-qr-status');
  if (!note || state.qrStatusBusy) return;
  state.qrStatusBusy = true;
  try {
    const payload = await api('/api/qr/status', {signal: AbortSignal.timeout(2000)});
    const run = await api('/api/evacuation/status', {signal: AbortSignal.timeout(2000)});
    $('#camera-qr-follow').disabled = Boolean(run.running);
    if (run.events?.length) $('#camera-qr-follow-status').textContent = run.events[run.events.length - 1].text;
    note.textContent = payload.text ? `QR: ${payload.text}` : 'QR: ничего не распознано';
    const terminal = $('#camera-qr-terminal');
    if (terminal) {
      const history = safeArray(payload.history);
      terminal.textContent = history.length
        ? history.map((item) => `[${item.timestamp || '--:--:--'}] ${item.text}`).join('\n')
        : 'Ожидание QR-кода…';
      terminal.scrollTop = terminal.scrollHeight;
    }
  } catch (_error) {
    note.textContent = 'QR: детектор недоступен';
  } finally {
    state.qrStatusBusy = false;
  }
}

function renderSignDetectionStatus() {
  const status = state.signDetection;
  const running = Boolean(status?.running);
  $('#sign-detection-start').disabled = running;
  $('#sign-detection-stop').disabled = !running;
  $('#sign-detection-status').textContent = running
    ? 'Модель знаков работает. Обнови источники и выбери «Дорожные знаки».'
    : 'Модель знаков выключена.';
}

async function refreshSignDetectionStatus() {
  if (!$('#sign-detection-status')) return;
  try {
    state.signDetection = await api('/api/sign-detection/status');
    renderSignDetectionStatus();
  } catch (error) {
    $('#sign-detection-status').textContent = String(error.message || error);
  }
}

function stopCameraLoop() {
  state.cameraPreviewController?.abort();
  state.cameraPreviewGeneration = (state.cameraPreviewGeneration || 0) + 1;
  if (state.cameraPreviewBlob) {
    URL.revokeObjectURL(state.cameraPreviewBlob);
    state.cameraPreviewBlob = null;
  }
  if (state.cameraTimer) {
    window.clearInterval(state.cameraTimer);
    state.cameraTimer = null;
  }
  if (state.cameraMaskTimer) {
    window.clearInterval(state.cameraMaskTimer);
    state.cameraMaskTimer = null;
  }
  state.cameraMaskBusy = false;
  $('#camera-frame').src = '';
  $('#camera-mask-canvas').classList.add('hidden');
  state.cameraUrl = null;
}

async function connectCamera() {
  const select = $('#camera-topic-select');
  state.selectedCameraTopic = '/camera/qr_annotated';
  state.selectedCameraType = 'sensor_msgs/msg/Image';
  stopCameraLoop();
  if (!state.selectedCameraTopic || !state.selectedCameraType) {
    showToast('Нет выбранного источника', 'error');
    return;
  }
  await recordActivity('Выбран источник камеры', { topic: state.selectedCameraTopic, type: state.selectedCameraType }, 'camera');
  const info = await refreshCameraStatus();
  if (!info) {
    $('#camera-empty').classList.remove('hidden');
    return;
  }
  if (!info?.frame_ready && !info?.last_error) {
    $('#camera-empty').classList.remove('hidden');
  }
  startCameraLoop();
}

function renderLidarTopics() {
  const select = $('#lidar-topic-select');
  const topics = safeArray(state.rosGraph?.topics)
    .filter((item) => safeArray(item.types).includes(LASER_SCAN_TYPE))
    .sort((left, right) => String(left.name || '').localeCompare(String(right.name || '')));
  select.innerHTML = '';
  if (!topics.length) {
    const option = document.createElement('option');
    option.value = '';
    option.textContent = 'Нет LaserScan topics';
    select.append(option);
    state.selectedLidarTopic = null;
    state.selectedLidarType = null;
    $('#lidar-status').textContent = 'Лидар не найден.';
    return;
  }

  if (!topics.some((item) => item.name === state.selectedLidarTopic)) {
    const preferredTopic = topics.find((item) => item.name === '/scan/filtered') || topics[0];
    state.selectedLidarTopic = preferredTopic.name;
    state.selectedLidarType = LASER_SCAN_TYPE;
  }

  topics.forEach((item) => {
    const option = document.createElement('option');
    option.value = item.name;
    option.dataset.type = LASER_SCAN_TYPE;
    option.textContent = `${item.name} (${LASER_SCAN_TYPE})`;
    option.selected = item.name === state.selectedLidarTopic;
    select.append(option);
  });
}

function drawLidarVisualization(data = state.lidarData) {
  const canvas = $('#lidar-canvas');
  const ctx = setupCanvas(canvas);
  if (!ctx) return;

  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = '#f9fdff';
  ctx.fillRect(0, 0, width, height);

  const hasData = data && safeArray(data.points).length;
  $('#lidar-empty').classList.toggle('hidden', Boolean(hasData));
  if (!hasData) return;

  const points = safeArray(data.points);
  const distances = points
    .map((point) => Math.hypot(Number(point[0] || 0), Number(point[1] || 0)))
    .filter((distance) => Number.isFinite(distance) && distance > 0)
    .sort((left, right) => left - right);
  const percentileIndex = Math.max(0, Math.floor((distances.length - 1) * 0.95));
  const maxRadius = Math.max(0.75, Math.min(8.0, (distances[percentileIndex] || 1.0) * 1.15));
  const centerX = width / 2;
  const centerY = height / 2;
  const scale = (Math.min(width, height) * 0.42) / maxRadius;

  ctx.strokeStyle = '#e0edf3';
  ctx.lineWidth = 1;
  [0.25, 0.5, 0.75, 1.0].forEach((ratio) => {
    ctx.beginPath();
    ctx.arc(centerX, centerY, maxRadius * scale * ratio, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillStyle = '#6b8794';
    ctx.font = '12px sans-serif';
    ctx.fillText(`${(maxRadius * ratio).toFixed(1)} м`, centerX + 6, centerY - maxRadius * scale * ratio + 14);
  });

  ctx.strokeStyle = '#9fd7ec';
  ctx.beginPath();
  ctx.moveTo(centerX, 0);
  ctx.lineTo(centerX, height);
  ctx.moveTo(0, centerY);
  ctx.lineTo(width, centerY);
  ctx.stroke();

  ctx.fillStyle = '#16b8f3';
  points.forEach((point) => {
    const x = Number(point[0] || 0);
    const y = Number(point[1] || 0);
    if (Math.hypot(x, y) > maxRadius) return;
    const screenX = centerX - y * scale;
    const screenY = centerY - x * scale;
    ctx.fillRect(screenX, screenY, 2, 2);
  });

  drawRoverArrow(ctx, { x: centerX, y: centerY }, 0, '#075f89', 16);
}

async function refreshLidarStatus() {
  if (!state.selectedLidarTopic || !state.selectedLidarType) return null;
  try {
    const info = await api(`/api/lidar/status?topic=${encodeURIComponent(state.selectedLidarTopic)}&type=${encodeURIComponent(state.selectedLidarType)}`);
    state.lidarData = info;
    renderDetailList($('#lidar-meta'), [
      { label: 'Topic', value: info.topic || '—' },
      { label: 'Type', value: info.type || '—' },
      { label: 'Frame', value: info.frame_id || '—' },
      { label: 'Messages', value: String(info.message_count ?? '—') },
      { label: 'Valid points', value: String(info.valid_points ?? 0) },
      { label: 'Age', value: info.age_sec == null ? '—' : formatAge(info.age_sec) },
    ]);
    $('#lidar-status').textContent = info.last_error
      ? `Ошибка: ${info.last_error}`
      : (info.frame_ready ? 'Скан поступает.' : 'Ожидание первого скана.');
    $('#lidar-points').textContent = `Точек: ${info.valid_points ?? 0}/${info.total_ranges ?? 0}`;
    $('#lidar-range').textContent = `Диапазон: ${formatFloat(info.range_min, 2)}..${formatFloat(info.range_max, 2)} м`;
    $('#lidar-age').textContent = `Возраст: ${info.age_sec == null ? '—' : formatAge(info.age_sec)}`;
    $('#lidar-frame').textContent = `Frame: ${info.frame_id || '—'}`;
    drawLidarVisualization(info);
    return info;
  } catch (error) {
    $('#lidar-status').textContent = String(error.message || error);
    return null;
  }
}

function startLidarLoop() {
  stopLidarLoop();
  state.lidarTimer = window.setInterval(() => {
    refreshLidarStatus();
  }, 250);
}

function stopLidarLoop() {
  if (state.lidarTimer) {
    window.clearInterval(state.lidarTimer);
    state.lidarTimer = null;
  }
}

async function connectLidar() {
  const select = $('#lidar-topic-select');
  state.selectedLidarTopic = select.value || null;
  state.selectedLidarType = select.selectedOptions[0]?.dataset.type || null;
  stopLidarLoop();
  if (!state.selectedLidarTopic || !state.selectedLidarType) {
    $('#lidar-status').textContent = 'Нет выбранного источника.';
    return;
  }
  $('#lidar-status').textContent = 'Подключение...';
  const info = await refreshLidarStatus();
  if (!info) return;
  startLidarLoop();
}

function setLidarSettingsForm(parameters = {}) {
  $('#lidar-setting-channel-type').value = parameters.channel_type || 'serial';
  $('#lidar-setting-serial-port').value = parameters.serial_port || '/tmp/rover_devices/lidar';
  $('#lidar-setting-serial-baudrate').value = String(parameters.serial_baudrate ?? 460800);
  $('#lidar-setting-frame-id').value = parameters.frame_id || 'lidar_link';
  $('#lidar-setting-scan-mode').value = parameters.scan_mode || 'Standard';
  $('#lidar-setting-scan-frequency').value = String(parameters.scan_frequency ?? 10.0);
  $('#lidar-setting-inverted').checked = Boolean(parameters.inverted ?? false);
  $('#lidar-setting-angle-compensate').checked = Boolean(parameters.angle_compensate ?? true);
}

function lidarSettingsPayloadFromForm() {
  return {
    channel_type: $('#lidar-setting-channel-type').value.trim() || 'serial',
    serial_port: $('#lidar-setting-serial-port').value.trim() || '/tmp/rover_devices/lidar',
    serial_baudrate: Number($('#lidar-setting-serial-baudrate').value || '460800'),
    frame_id: $('#lidar-setting-frame-id').value.trim() || 'lidar_link',
    scan_mode: $('#lidar-setting-scan-mode').value.trim() || 'Standard',
    scan_frequency: Number($('#lidar-setting-scan-frequency').value || '10'),
    inverted: $('#lidar-setting-inverted').checked,
    angle_compensate: $('#lidar-setting-angle-compensate').checked,
  };
}

function summarizeLidarSettings(payload) {
  return pretty({
    node_name: payload?.node_name || '/sllidar_node',
    runtime_parameters: payload?.runtime_parameters || [],
    parameters: payload?.parameters || {},
    notes: payload?.notes || {},
  });
}

async function refreshLidarSettings() {
  try {
    const payload = await api('/api/lidar/settings');
    state.lidarSettings = payload;
    setLidarSettingsForm(payload.parameters || {});
    $('#lidar-settings-details').textContent = summarizeLidarSettings(payload);
    $('#lidar-settings-status').textContent = `Параметры получены из ${payload.node_name || '/sllidar_node'}.`;
    return payload;
  } catch (error) {
    $('#lidar-settings-status').textContent = String(error.message || error);
    $('#lidar-settings-details').textContent = 'Не удалось получить параметры лидара.';
    return null;
  }
}

async function applyLidarSettings() {
  try {
    $('#lidar-settings-status').textContent = 'Применение параметров лидара...';
    const payload = await api('/api/lidar/settings', {
      method: 'POST',
      body: JSON.stringify(lidarSettingsPayloadFromForm()),
    });
    state.lidarSettings = payload;
    setLidarSettingsForm(payload.parameters || {});
    $('#lidar-settings-details').textContent = summarizeLidarSettings(payload);
    $('#lidar-settings-status').textContent = 'Параметры лидара применены.';
    showToast('Настройки лидара применены');
    await Promise.all([refreshLidarSettings(), refreshLidarStatus()]);
  } catch (error) {
    $('#lidar-settings-status').textContent = String(error.message || error);
    showToast(String(error.message || error), 'error');
  }
}

function normalizeHexColor(value, fallback = '#000000') {
  const raw = String(value || fallback).trim();
  return /^#[0-9a-f]{6}$/i.test(raw) ? raw.toUpperCase() : fallback;
}

function parsePackedColor(value) {
  const numeric = Number(value) >>> 0;
  return {
    red: (numeric >> 16) & 0xFF,
    green: (numeric >> 8) & 0xFF,
    blue: numeric & 0xFF,
  };
}

function updateLedStripBrightnessOutput() {
  $('#led-strip-brightness-output').textContent = formatFloat(
    $('#led-strip-brightness').value,
    2,
  );
}

function updateLedStripSpeedOutput() {
  $('#led-strip-speed-output').textContent = formatFloat(
    $('#led-strip-speed').value,
    2,
  );
}

function setLedStripSettingsForm(parameters = {}) {
  $('#led-strip-setting-transport').value = parameters.led_transport || 'auto';
  $('#led-strip-setting-spi-bus').value = String(parameters.spi_bus ?? 1);
  $('#led-strip-setting-spi-device').value = String(parameters.spi_device ?? 0);
  $('#led-strip-setting-count').value = String(parameters.led_count ?? 16);
  $('#led-strip-setting-frame-id').value = parameters.frame_id || 'led_strip';
  $('#led-strip-setting-animation-rate').value = String(parameters.animation_rate_hz ?? 30);
  $('#led-strip-setting-publish-rate').value = String(parameters.state_publish_hz ?? 5);
}

function setLedStripControlForm(parameters = {}) {
  $('#led-strip-enabled').checked = Boolean(parameters.enabled ?? false);
  $('#led-strip-effect').value = parameters.effect || 'fill';
  $('#led-strip-brightness').value = String(parameters.brightness ?? 0.35);
  $('#led-strip-speed').value = String(parameters.effect_speed_hz ?? 1.0);
  $('#led-strip-primary-color').value = normalizeHexColor(
    parameters.primary_color,
    '#16B8F3',
  );
  $('#led-strip-secondary-color').value = normalizeHexColor(
    parameters.secondary_color,
    '#FFFFFF',
  );
  updateLedStripBrightnessOutput();
  updateLedStripSpeedOutput();
}

function ledStripSettingsPayloadFromForm() {
  return {
    led_transport: $('#led-strip-setting-transport').value.trim().toLowerCase() || 'auto',
    spi_bus: Number($('#led-strip-setting-spi-bus').value || '1'),
    spi_device: Number($('#led-strip-setting-spi-device').value || '0'),
    led_count: Number($('#led-strip-setting-count').value || '16'),
    frame_id: $('#led-strip-setting-frame-id').value.trim(),
    animation_rate_hz: Number($('#led-strip-setting-animation-rate').value || '30'),
    state_publish_hz: Number($('#led-strip-setting-publish-rate').value || '5'),
  };
}

function ledStripCommandPayloadFromForm(enabledOverride = null) {
  return {
    enabled: enabledOverride == null
      ? $('#led-strip-enabled').checked
      : Boolean(enabledOverride),
    effect: $('#led-strip-effect').value,
    brightness: Number($('#led-strip-brightness').value || '0.35'),
    effect_speed_hz: Number($('#led-strip-speed').value || '1.0'),
    primary_color: normalizeHexColor($('#led-strip-primary-color').value, '#16B8F3'),
    secondary_color: normalizeHexColor($('#led-strip-secondary-color').value, '#FFFFFF'),
  };
}

function summarizeLedStripSettings(payload) {
  const parameters = payload?.parameters || {};
  return pretty({
    node_name: payload?.node_name || '/led_strip_node',
    runtime_parameters: payload?.runtime_parameters || [],
    effects: payload?.effects || [],
    parameters,
    notes: payload?.notes || {},
  });
}

async function refreshLedStripSettings() {
  try {
    const payload = await api('/api/led_strip/settings');
    state.ledStripSettings = payload;
    setLedStripSettingsForm(payload.parameters || {});
    if (!state.ledStripControlInitialized) {
      setLedStripControlForm(payload.parameters || {});
      state.ledStripControlInitialized = true;
    }
    if (!state.ledStripManualInitialized) {
      syncLedStripPixelsFromFrame([], { force: true });
    } else {
      state.ledStripPixels = ensureLedStripPixels(
        Number(payload?.parameters?.led_count ?? currentLedStripCount()),
      );
      renderLedStripPixelGrid();
    }
    $('#led-strip-settings-details').textContent = summarizeLedStripSettings(payload);
    $('#led-strip-settings-status').textContent = `Параметры получены из ${payload.node_name || '/led_strip_node'}.`;
    return payload;
  } catch (error) {
    $('#led-strip-settings-status').textContent = String(error.message || error);
    $('#led-strip-settings-details').textContent = 'Не удалось получить параметры LED strip.';
    return null;
  }
}

async function applyLedStripSettings() {
  try {
    $('#led-strip-settings-status').textContent = 'Применение параметров ленты...';
    const payload = await api('/api/led_strip/settings', {
      method: 'POST',
      body: JSON.stringify(ledStripSettingsPayloadFromForm()),
    });
    state.ledStripSettings = payload;
    setLedStripSettingsForm(payload.parameters || {});
    $('#led-strip-settings-details').textContent = summarizeLedStripSettings(payload);
    $('#led-strip-settings-status').textContent = 'Параметры LED strip применены.';
    state.ledStripPixels = ensureLedStripPixels(
      Number(payload?.parameters?.led_count ?? currentLedStripCount()),
    );
    renderLedStripPixelGrid();
    showToast('Настройки LED strip применены');
    await Promise.all([refreshLedStripSettings(), refreshLedStripStatus()]);
  } catch (error) {
    $('#led-strip-settings-status').textContent = String(error.message || error);
    showToast(String(error.message || error), 'error');
  }
}

function renderLedStripTopics() {
  const select = $('#led-strip-topic-select');
  const topics = safeArray(state.rosGraph?.topics)
    .filter((item) => safeArray(item.types).includes(LED_STRIP_STATE_TYPE))
    .sort((left, right) => String(left.name || '').localeCompare(String(right.name || '')));
  select.innerHTML = '';
  if (!topics.length) {
    const option = document.createElement('option');
    option.value = '';
    option.textContent = 'Нет LedStripState topics';
    select.append(option);
    state.selectedLedStripTopic = null;
    state.selectedLedStripType = null;
    $('#led-strip-status').textContent = 'LED strip не найден.';
    return;
  }

  if (!topics.some((item) => item.name === state.selectedLedStripTopic)) {
    state.selectedLedStripTopic = topics[0].name;
    state.selectedLedStripType = LED_STRIP_STATE_TYPE;
  }

  topics.forEach((item) => {
    const option = document.createElement('option');
    option.value = item.name;
    option.dataset.type = LED_STRIP_STATE_TYPE;
    option.textContent = `${item.name} (${LED_STRIP_STATE_TYPE})`;
    option.selected = item.name === state.selectedLedStripTopic;
    select.append(option);
  });
}

function drawLedStripVisualization(data = state.ledStripData) {
  const canvas = $('#led-strip-canvas');
  const ctx = setupCanvas(canvas);
  if (!ctx) return;

  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = '#f9fdff';
  ctx.fillRect(0, 0, width, height);

  const colors = safeArray(data?.preview_colors);
  $('#led-strip-empty').classList.toggle('hidden', colors.length > 0);
  if (!colors.length) return;

  const count = colors.length;
  const columns = Math.min(8, count);
  const rows = Math.max(1, Math.ceil(count / Math.max(1, columns)));
  const marginX = 28;
  const marginTop = 42;
  const marginBottom = 34;
  const usableWidth = width - marginX * 2;
  const usableHeight = Math.max(32, height - marginTop - marginBottom);
  const gapX = Math.max(4, Math.min(14, usableWidth / Math.max(1, columns) * 0.06));
  const gapY = 14;
  const cellWidth = Math.max(8, (usableWidth - gapX * (columns - 1)) / columns);
  const rawCellHeight = (usableHeight - gapY * (rows - 1)) / rows;
  const cellHeight = Math.max(18, Math.min(54, rawCellHeight));
  const stripHeight = cellHeight * rows + gapY * (rows - 1);
  const top = marginTop + Math.max(0, (usableHeight - stripHeight) / 2);

  colors.forEach((packed, index) => {
    const { red, green, blue } = parsePackedColor(packed);
    const column = index % columns;
    const row = Math.floor(index / columns);
    const x = marginX + column * (cellWidth + gapX);
    const y = top + row * (cellHeight + gapY);
    ctx.fillStyle = `rgb(${red}, ${green}, ${blue})`;
    ctx.fillRect(x, y, cellWidth, cellHeight);
    ctx.strokeStyle = 'rgba(7, 95, 137, 0.18)';
    ctx.lineWidth = 1;
    ctx.strokeRect(x, y, cellWidth, cellHeight);
  });

  ctx.fillStyle = '#075f89';
  ctx.font = '12px "Avenir Next", "Segoe UI", sans-serif';
  ctx.textAlign = 'left';
  ctx.fillText(`${count} LED`, marginX, 24);
  ctx.textAlign = 'right';
  const transport = data?.transport || 'spi';
  const bus = data?.spi_bus == null ? '—' : data.spi_bus;
  const device = data?.spi_device == null ? '—' : data.spi_device;
  ctx.fillText(`${transport} ${bus}.${device}`, width - marginX, 24);
}

function markLedStripControlDirty() {
  state.ledStripControlInitialized = true;
  state.ledStripControlDirty = true;
}

async function refreshLedStripStatus() {
  if (!state.selectedLedStripTopic || !state.selectedLedStripType) return null;
  try {
    const info = await api(`/api/led_strip/status?topic=${encodeURIComponent(state.selectedLedStripTopic)}&type=${encodeURIComponent(state.selectedLedStripType)}`);
    state.ledStripData = info;
    renderDetailList($('#led-strip-meta'), [
      { label: 'Topic', value: info.topic || '—' },
      { label: 'Type', value: info.type || '—' },
      { label: 'Frame', value: info.frame_id || '—' },
      { label: 'LED count', value: String(info.led_count ?? '—') },
      { label: 'Lit now', value: String(info.lit_count ?? '—') },
      { label: 'Age', value: info.age_sec == null ? '—' : formatAge(info.age_sec) },
    ]);
    $('#led-strip-status').textContent = info.last_error
      ? `Ошибка: ${info.last_error}`
      : (info.frame_ready ? (info.status_message || 'Данные поступают.') : 'Ожидание первого состояния.');
    $('#led-strip-effect-readout').textContent = `Effect: ${info.effect || '—'}`;
    $('#led-strip-brightness-readout').textContent = `Brightness: ${info.brightness == null ? '—' : formatFloat(info.brightness, 2)}`;
    $('#led-strip-backend').textContent = `Backend: ${info.backend || '—'}`;
    const transportText = info.transport ? `${info.transport} ${info.spi_bus}.${info.spi_device}` : (info.connected ? 'подключено' : 'offline');
    $('#led-strip-connection').textContent = `Состояние: ${transportText}`;
    $('#led-strip-age').textContent = `Возраст: ${info.age_sec == null ? '—' : formatAge(info.age_sec)}`;
    syncLedStripPixelsFromStatus(info);
    drawLedStripVisualization(info);
    return info;
  } catch (error) {
    $('#led-strip-status').textContent = String(error.message || error);
    return null;
  }
}

function startLedStripLoop() {
  stopLedStripLoop();
  state.ledStripTimer = window.setInterval(() => {
    refreshLedStripStatus();
  }, 250);
}

function stopLedStripLoop() {
  if (state.ledStripTimer) {
    window.clearInterval(state.ledStripTimer);
    state.ledStripTimer = null;
  }
}

async function connectLedStrip() {
  const select = $('#led-strip-topic-select');
  state.selectedLedStripTopic = select.value || null;
  state.selectedLedStripType = select.selectedOptions[0]?.dataset.type || null;
  stopLedStripLoop();
  if (!state.selectedLedStripTopic || !state.selectedLedStripType) {
    $('#led-strip-status').textContent = 'Нет выбранного источника.';
    return;
  }
  $('#led-strip-status').textContent = 'Подключение...';
  const info = await refreshLedStripStatus();
  if (!info) return;
  startLedStripLoop();
}

async function sendLedStripCommand(enabledOverride = null) {
  try {
    $('#led-strip-control-status').textContent = 'Отправка команды на LED strip...';
    const payload = await api('/api/led_strip/command', {
      method: 'POST',
      body: JSON.stringify(ledStripCommandPayloadFromForm(enabledOverride)),
    });
    if (payload.settings) {
      state.ledStripSettings = payload.settings;
      setLedStripSettingsForm(payload.settings.parameters || {});
      setLedStripControlForm(payload.settings.parameters || {});
      $('#led-strip-settings-details').textContent = summarizeLedStripSettings(payload.settings);
    }
    state.ledStripControlInitialized = true;
    state.ledStripControlDirty = false;
    state.ledStripPixelsDirty = false;
    $('#led-strip-control-status').textContent = payload.response?.message || 'Команда отправлена.';
    showToast(enabledOverride === false ? 'Лента выключена' : 'Команда для LED strip отправлена');
    await refreshLedStripStatus();
  } catch (error) {
    $('#led-strip-control-status').textContent = String(error.message || error);
    showToast(String(error.message || error), 'error');
  }
}

function syncLedStripPixelsFromFrame(colors = [], { force = false } = {}) {
  const target = ensureLedStripPixels(currentLedStripCount(), colors);
  if (!force && state.ledStripPixelsDirty) {
    return;
  }
  state.ledStripPixels = target;
  state.ledStripManualInitialized = true;
  state.ledStripPixelsDirty = false;
  renderLedStripPixelGrid();
}

function syncLedStripPixelsFromStatus(info = state.ledStripData, { force = false } = {}) {
  const previewColors = safeArray(info?.preview_colors).map((value) => packedColorToHex(value));
  if (previewColors.length) {
    syncLedStripPixelsFromFrame(previewColors, { force });
    return;
  }
  if (!state.ledStripManualInitialized || force) {
    syncLedStripPixelsFromFrame([], { force: true });
  }
}

function renderLedStripStaticPresets() {
  const select = $('#led-strip-custom-preset-select');
  if (!select) return;
  const previousValue = select.value;
  const presets = safeArray(state.ledStripStaticPresets).slice().sort((left, right) => (
    String(left.name || '').localeCompare(String(right.name || ''))
  ));
  select.innerHTML = '';
  const placeholder = document.createElement('option');
  placeholder.value = '';
  placeholder.textContent = presets.length ? 'Выбери сохранённый пресет' : 'Нет сохранённых пресетов';
  select.append(placeholder);
  if (!presets.length) {
    return;
  }
  presets.forEach((preset) => {
    const option = document.createElement('option');
    option.value = preset.name;
    option.textContent = preset.name;
    option.selected = preset.name === previousValue;
    select.append(option);
  });
}

function findLedStripStaticPreset(name) {
  return safeArray(state.ledStripStaticPresets).find((preset) => preset.name === name) || null;
}

function setLedStripFrameEditorStatus(message) {
  const presetStatus = document.querySelector('#led-strip-preset-status');
  if (presetStatus) {
    presetStatus.textContent = message;
  }
}

function renderLedStripPixelGrid() {
  const container = $('#led-strip-pixel-grid');
  if (!container) return;
  state.ledStripPixels = ensureLedStripPixels();
  container.innerHTML = '';
  state.ledStripPixels.forEach((color, index) => {
    const cell = document.createElement('label');
    cell.className = 'led-pixel-cell';

    const number = document.createElement('span');
    number.className = 'led-pixel-index';
    number.textContent = `LED ${index + 1}`;

    const input = document.createElement('input');
    input.type = 'color';
    input.value = normalizeHexColor(color, '#000000');
    input.dataset.index = String(index);
    input.addEventListener('input', (event) => {
      const ledIndex = Number(event.target.dataset.index);
      state.ledStripPixels[ledIndex] = normalizeHexColor(event.target.value, '#000000');
      state.ledStripPixelsDirty = true;
    });
    input.addEventListener('change', (event) => {
      const ledIndex = Number(event.target.dataset.index);
      state.ledStripPixels[ledIndex] = normalizeHexColor(event.target.value, '#000000');
      state.ledStripPixelsDirty = true;
    });

    cell.append(number, input);
    container.append(cell);
  });
}

function fillLedStripPixels(color) {
  state.ledStripPixels = ensureLedStripPixels().map(() => normalizeHexColor(color, '#000000'));
  state.ledStripManualInitialized = true;
  state.ledStripPixelsDirty = true;
  renderLedStripPixelGrid();
}

function ledStripManualFramePayload() {
  const leds = ensureLedStripPixels().reduce((items, color, index) => {
    const rgb = parseHexColorToRgb(color);
    if (rgb.r || rgb.g || rgb.b) {
      items.push({ index, r: rgb.r, g: rgb.g, b: rgb.b });
    }
    return items;
  }, []);
  return { leds };
}

async function ensureLedStripEnabledForManualFrame() {
  if ($('#led-strip-enabled').checked) {
    return;
  }
  $('#led-strip-enabled').checked = true;
  $('#led-strip-control-status').textContent = 'Лента включается для показа кадра...';
  const payload = await api('/api/led_strip/command', {
    method: 'POST',
    body: JSON.stringify(ledStripCommandPayloadFromForm(true)),
  });
  if (payload.settings) {
    state.ledStripSettings = payload.settings;
    setLedStripSettingsForm(payload.settings.parameters || {});
    setLedStripControlForm(payload.settings.parameters || {});
    $('#led-strip-settings-details').textContent = summarizeLedStripSettings(payload.settings);
  }
  state.ledStripControlInitialized = true;
  state.ledStripControlDirty = false;
  $('#led-strip-control-status').textContent = 'Лента включена для показа кадра.';
}

async function applyLedStripManualFrame() {
  try {
    const framePayload = ledStripManualFramePayload();
    setLedStripFrameEditorStatus('Отправка покадрового состояния...');
    await ensureLedStripEnabledForManualFrame();
    const payload = await api('/api/led_strip/leds', {
      method: 'POST',
      body: JSON.stringify(framePayload),
    });
    state.ledStripPixelsDirty = false;
    setLedStripFrameEditorStatus(payload.response?.success === false
      ? 'Кадр отклонён.'
      : 'Кадр показан на ленте.');
    showToast('Покадровое состояние LED strip отправлено');
    await Promise.all([refreshLedStripSettings(), refreshLedStripStatus()]);
  } catch (error) {
    setLedStripFrameEditorStatus(String(error.message || error));
    showToast(String(error.message || error), 'error');
  }
}

function saveCurrentLedStripStaticPreset() {
  const name = $('#led-strip-custom-preset-name').value.trim();
  if (!name) {
    throw new Error('Укажи имя пресета');
  }
  const colors = ensureLedStripPixels();
  const next = safeArray(state.ledStripStaticPresets).filter((preset) => preset.name !== name);
  next.push({ name, colors });
  state.ledStripStaticPresets = next;
  saveLedStripStaticPresets();
  renderLedStripStaticPresets();
  $('#led-strip-custom-preset-select').value = name;
  setLedStripFrameEditorStatus(`Пресет "${name}" сохранён.`);
  showToast(`Сохранён пресет ${name}`);
}

function loadSelectedLedStripStaticPresetIntoEditor() {
  const name = $('#led-strip-custom-preset-select').value;
  const preset = findLedStripStaticPreset(name);
  if (!preset) {
    throw new Error('Выбери сохранённый пресет');
  }
  $('#led-strip-custom-preset-name').value = preset.name;
  syncLedStripPixelsFromFrame(preset.colors, { force: true });
  state.ledStripPixelsDirty = true;
  setLedStripFrameEditorStatus(`Пресет "${preset.name}" загружен в редактор.`);
}

function deleteSelectedLedStripStaticPreset() {
  const name = $('#led-strip-custom-preset-select').value;
  const preset = findLedStripStaticPreset(name);
  if (!preset) {
    throw new Error('Выбери сохранённый пресет');
  }
  state.ledStripStaticPresets = safeArray(state.ledStripStaticPresets)
    .filter((item) => item.name !== preset.name);
  saveLedStripStaticPresets();
  renderLedStripStaticPresets();
  setLedStripFrameEditorStatus(`Пресет "${preset.name}" удалён.`);
  showToast(`Удалён пресет ${preset.name}`);
}

function updateOctolinerSensitivityOutput() {
  $('#octoliner-sensitivity-output').textContent = formatFloat(
    $('#octoliner-setting-sensitivity').value,
    2,
  );
}

function setOctolinerSettingsForm(parameters = {}) {
  $('#octoliner-setting-address').value = String(parameters.i2c_address ?? 42);
  $('#octoliner-setting-bus').value = String(parameters.i2c_bus ?? 1);
  $('#octoliner-setting-poll-rate').value = String(parameters.poll_rate_hz ?? 50);
  $('#octoliner-setting-frame-id').value = parameters.frame_id || 'octoliner_link';
  $('#octoliner-setting-sensitivity').value = String(parameters.sensitivity ?? 0.8);
  $('#octoliner-setting-auto-optimize').checked = Boolean(
    parameters.auto_optimize_on_start ?? false,
  );
  updateOctolinerSensitivityOutput();
}

function octolinerSettingsPayloadFromForm() {
  return {
    poll_rate_hz: Number($('#octoliner-setting-poll-rate').value || '50'),
    frame_id: $('#octoliner-setting-frame-id').value.trim(),
    sensitivity: Number($('#octoliner-setting-sensitivity').value || '0.8'),
    auto_optimize_on_start: $('#octoliner-setting-auto-optimize').checked,
  };
}

function summarizeOctolinerSettings(payload) {
  const parameters = payload?.parameters || {};
  const notes = payload?.notes || {};
  return pretty({
    node_name: payload?.node_name || '/octoliner_node',
    runtime_parameters: payload?.runtime_parameters || [],
    read_only: {
      i2c_address: parameters.i2c_address,
      i2c_bus: parameters.i2c_bus,
      set_sensitivity_service: parameters.set_sensitivity_service,
      optimize_on_black_service: parameters.optimize_on_black_service,
    },
    notes,
  });
}

async function refreshOctolinerSettings() {
  try {
    const payload = await api('/api/octoliner/settings');
    state.octolinerSettings = payload;
    setOctolinerSettingsForm(payload.parameters || {});
    $('#octoliner-settings-details').textContent = summarizeOctolinerSettings(payload);
    $('#octoliner-settings-status').textContent = `Параметры получены из ${payload.node_name || '/octoliner_node'}.`;
    return payload;
  } catch (error) {
    $('#octoliner-settings-status').textContent = String(error.message || error);
    $('#octoliner-settings-details').textContent = 'Не удалось получить параметры Octoliner.';
    return null;
  }
}

async function applyOctolinerSettings() {
  try {
    $('#octoliner-settings-status').textContent = 'Применение параметров Octoliner...';
    const payload = await api('/api/octoliner/settings', {
      method: 'POST',
      body: JSON.stringify(octolinerSettingsPayloadFromForm()),
    });
    state.octolinerSettings = payload;
    setOctolinerSettingsForm(payload.parameters || {});
    $('#octoliner-settings-details').textContent = summarizeOctolinerSettings(payload);
    $('#octoliner-settings-status').textContent = 'Параметры Octoliner применены.';
    showToast('Настройки Octoliner применены');
    await Promise.all([refreshOctolinerSettings(), refreshOctolinerStatus()]);
  } catch (error) {
    $('#octoliner-settings-status').textContent = String(error.message || error);
    showToast(String(error.message || error), 'error');
  }
}

async function optimizeOctoliner() {
  try {
    $('#octoliner-settings-status').textContent = 'Калибровка Octoliner по чёрной поверхности...';
    const payload = await api('/api/octoliner/optimize', {
      method: 'POST',
      body: JSON.stringify({}),
    });
    if (payload.settings) {
      state.octolinerSettings = payload.settings;
      setOctolinerSettingsForm(payload.settings.parameters || {});
      $('#octoliner-settings-details').textContent = summarizeOctolinerSettings(payload.settings);
    }
    $('#octoliner-settings-status').textContent = payload.response?.message || 'Калибровка завершена.';
    showToast('Octoliner откалиброван');
    await refreshOctolinerStatus();
  } catch (error) {
    $('#octoliner-settings-status').textContent = String(error.message || error);
    showToast(String(error.message || error), 'error');
  }
}

function renderOctolinerTopics() {
  const select = $('#octoliner-topic-select');
  const topics = safeArray(state.rosGraph?.topics)
    .filter((item) => safeArray(item.types).includes(OCTOLINER_READING_TYPE))
    .sort((left, right) => String(left.name || '').localeCompare(String(right.name || '')));
  select.innerHTML = '';
  if (!topics.length) {
    const option = document.createElement('option');
    option.value = '';
    option.textContent = 'Нет Octoliner topics';
    select.append(option);
    state.selectedOctolinerTopic = null;
    state.selectedOctolinerType = null;
    $('#octoliner-status').textContent = 'Octoliner не найден.';
    return;
  }

  if (!topics.some((item) => item.name === state.selectedOctolinerTopic)) {
    state.selectedOctolinerTopic = topics[0].name;
    state.selectedOctolinerType = OCTOLINER_READING_TYPE;
  }

  topics.forEach((item) => {
    const option = document.createElement('option');
    option.value = item.name;
    option.dataset.type = OCTOLINER_READING_TYPE;
    option.textContent = `${item.name} (${OCTOLINER_READING_TYPE})`;
    option.selected = item.name === state.selectedOctolinerTopic;
    select.append(option);
  });
}

function drawOctolinerVisualization(data = state.octolinerData) {
  const canvas = $('#octoliner-canvas');
  const ctx = setupCanvas(canvas);
  if (!ctx) return;

  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = '#f9fdff';
  ctx.fillRect(0, 0, width, height);

  const values = safeArray(data?.analog_values);
  $('#octoliner-empty').classList.toggle('hidden', values.length === 8);
  if (values.length !== 8) return;

  const marginX = 60;
  const top = 60;
  const bottom = height - 140;
  const usableWidth = width - marginX * 2;
  const usableHeight = bottom - top;
  const barWidth = usableWidth / 8;
  const pattern = Number(data.pattern || 0);

  ctx.strokeStyle = '#d7e8f0';
  ctx.lineWidth = 1;
  for (let i = 0; i <= 5; i += 1) {
    const y = top + (usableHeight / 5) * i;
    ctx.beginPath();
    ctx.moveTo(marginX, y);
    ctx.lineTo(width - marginX, y);
    ctx.stroke();
  }

  values.forEach((rawValue, index) => {
    const value = Math.max(0, Math.min(1, Number(rawValue || 0)));
    const x = marginX + index * barWidth + 8;
    const w = Math.max(18, barWidth - 16);
    const h = usableHeight * value;
    const y = bottom - h;
    const active = Boolean(pattern & (1 << (7 - index)));
    ctx.fillStyle = active ? '#16b8f3' : '#b8e6f7';
    ctx.fillRect(x, y, w, h);
    ctx.strokeStyle = active ? '#078ac4' : '#9fd7ec';
    ctx.strokeRect(x, y, w, h);
    ctx.fillStyle = '#075f89';
    ctx.font = '12px "Avenir Next", "Segoe UI", sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(`S${index + 1}`, x + w / 2, bottom + 20);
    ctx.fillText(value.toFixed(2), x + w / 2, y - 8);
  });

  const guideY = height - 70;
  const guideLeft = marginX;
  const guideRight = width - marginX;
  ctx.strokeStyle = '#9fd7ec';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(guideLeft, guideY);
  ctx.lineTo(guideRight, guideY);
  ctx.stroke();

  const drawMarker = (position, color, label) => {
    if (!Number.isFinite(Number(position))) return;
    const normalized = (Number(position) + 1) / 2;
    const x = guideLeft + normalized * (guideRight - guideLeft);
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(x, guideY - 26);
    ctx.lineTo(x, guideY + 10);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(x, guideY - 32, 6, 0, Math.PI * 2);
    ctx.fill();
    ctx.font = '12px "Avenir Next", "Segoe UI", sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(label, x, guideY - 42);
  };

  drawMarker(data.line_position, '#c93644', 'line');
  drawMarker(data.tracked_line_position, '#075f89', 'tracked');
}

async function refreshOctolinerStatus() {
  if (!state.selectedOctolinerTopic || !state.selectedOctolinerType) return null;
  try {
    const info = await api(`/api/octoliner/status?topic=${encodeURIComponent(state.selectedOctolinerTopic)}&type=${encodeURIComponent(state.selectedOctolinerType)}`);
    state.octolinerData = info;
    renderDetailList($('#octoliner-meta'), [
      { label: 'Topic', value: info.topic || '—' },
      { label: 'Type', value: info.type || '—' },
      { label: 'Frame', value: info.frame_id || '—' },
      { label: 'Messages', value: String(info.message_count ?? '—') },
      { label: 'Dark sensors', value: String(info.dark_sensor_count ?? '—') },
      { label: 'Age', value: info.age_sec == null ? '—' : formatAge(info.age_sec) },
    ]);
    $('#octoliner-status').textContent = info.last_error
      ? `Ошибка: ${info.last_error}`
      : (info.frame_ready ? 'Данные поступают.' : 'Ожидание первого чтения.');
    $('#octoliner-pattern').textContent = `Pattern: ${info.pattern_bits || '00000000'}`;
    $('#octoliner-visible').textContent = `Линия: ${info.line_visible ? 'видна' : 'не видна'}`;
    $('#octoliner-line').textContent = `Позиция: ${info.line_position == null ? '—' : formatFloat(info.line_position, 2)}`;
    $('#octoliner-tracked').textContent = `Tracked: ${info.tracked_line_position == null ? '—' : formatFloat(info.tracked_line_position, 2)}`;
    $('#octoliner-sensitivity').textContent = `Sensitivity: ${info.sensitivity == null ? '—' : formatFloat(info.sensitivity, 2)}`;
    drawOctolinerVisualization(info);
    return info;
  } catch (error) {
    $('#octoliner-status').textContent = String(error.message || error);
    return null;
  }
}

function startOctolinerLoop() {
  stopOctolinerLoop();
  state.octolinerTimer = window.setInterval(() => {
    refreshOctolinerStatus();
  }, 200);
}

function stopOctolinerLoop() {
  if (state.octolinerTimer) {
    window.clearInterval(state.octolinerTimer);
    state.octolinerTimer = null;
  }
}

async function connectOctoliner() {
  const select = $('#octoliner-topic-select');
  state.selectedOctolinerTopic = select.value || null;
  state.selectedOctolinerType = select.selectedOptions[0]?.dataset.type || null;
  stopOctolinerLoop();
  if (!state.selectedOctolinerTopic || !state.selectedOctolinerType) {
    $('#octoliner-status').textContent = 'Нет выбранного источника.';
    return;
  }
  $('#octoliner-status').textContent = 'Подключение...';
  const info = await refreshOctolinerStatus();
  if (!info) return;
  startOctolinerLoop();
}

function refreshDriveConfigFromConfig(config) {
  if (!config) return;
  const defaults = config.drive_defaults || {};
  const limits = config.drive_limits || {};

  $('#linear-speed').max = String(limits.linear_x ?? 0.35);
  $('#lateral-speed').max = String(limits.linear_y ?? 0.35);
  $('#angular-speed').max = String(limits.angular_z ?? 1.5);

  $('#linear-speed').value = String(defaults.linear_x ?? 0.18);
  $('#lateral-speed').value = String(defaults.linear_y ?? 0.16);
  $('#angular-speed').value = String(defaults.angular_z ?? 0.70);

  updateDriveOutputs();
  renderDetailList($('#drive-meta'), [
    { label: 'Command topic', value: config.command_topic || '/cmd_vel' },
    { label: 'Timeout', value: `${formatFloat(config.drive_command_timeout_sec, 2)} s` },
    { label: 'Linear max', value: formatFloat(limits.linear_x, 2) },
    { label: 'Тип базы', value: 'Дифференциальная, 2 колеса' },
    { label: 'Angular max', value: formatFloat(limits.angular_z, 2) },
  ]);
  $('#drive-topic-label').textContent = config.command_topic || '/cmd_vel';
}

async function refreshDriveConfig() {
  try {
    const payload = await api('/api/drive');
    $('#linear-speed').max = String(payload.limits.linear_x);
    $('#linear-speed').value = String(payload.defaults.linear_x);
    $('#lateral-speed').max = String(payload.limits.linear_y);
    $('#lateral-speed').value = String(payload.defaults.linear_y);
    $('#angular-speed').max = String(payload.limits.angular_z);
    $('#angular-speed').value = String(payload.defaults.angular_z);
    updateDriveOutputs();
    renderDetailList($('#drive-meta'), [
      { label: 'Command topic', value: payload.command_topic || '—' },
      { label: 'Timeout', value: `${formatFloat(payload.timeout_sec, 2)} s` },
      { label: 'Linear max', value: formatFloat(payload.limits.linear_x, 2) },
      { label: 'Тип базы', value: 'Дифференциальная, 2 колеса' },
      { label: 'Angular max', value: formatFloat(payload.limits.angular_z, 2) },
    ]);
    $('#drive-topic-label').textContent = payload.command_topic || '/cmd_vel';
  } catch (error) {
    $('#drive-meta').innerHTML = '';
  }
}

function updateDriveOutputs() {
  $('#linear-speed-output').textContent = `${formatFloat($('#linear-speed').value, 2)} м/с`;
  $('#lateral-speed-output').textContent = `${formatFloat($('#lateral-speed').value, 2)} м/с`;
  $('#angular-speed-output').textContent = `${formatFloat($('#angular-speed').value, 2)} рад/с`;
}

function computeDriveCommand() {
  const linear = Number($('#linear-speed').value);
  const angular = Number($('#angular-speed').value);
  if (state.joystick.active) return { linearX: -state.joystick.y * linear, linearY: 0, angularZ: -state.joystick.x * angular };

  const forward = state.driveKeys.has('KeyW') ? 1 : 0;
  const backward = state.driveKeys.has('KeyS') ? 1 : 0;
  const left = state.driveKeys.has('KeyA') ? 1 : 0;
  const right = state.driveKeys.has('KeyD') ? 1 : 0;

  return {
    linearX: (forward - backward) * linear,
    linearY: 0,
    angularZ: (left - right) * angular,
  };
}

function updateDrivePreview(command) {
  $('#drive-preview').textContent = `linear.x: ${formatFloat(command.linearX, 2)}\nlinear.y: ${formatFloat(command.linearY, 2)}\nangular.z: ${formatFloat(command.angularZ, 2)}`;
}

function connectDriveSocket() {
  if (typeof WebSocket === 'undefined') return;
  if (state.driveSocket && state.driveSocket.readyState < WebSocket.CLOSING) return;
  const url = new URL(location.href);
  url.protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
  url.port = String(state.config?.drive_websocket_port || Number(location.port || 80) + 1);
  url.pathname = '/drive';
  url.search = new URLSearchParams({client_id: state.driveClientId}).toString();
  url.hash = '';
  let socket;
  try { socket = new WebSocket(url); }
  catch (_) { window.setTimeout(connectDriveSocket, 1000); return; }
  state.driveSocket = socket;
  const connectingTimeout = window.setTimeout(() => {
    if (socket.readyState === WebSocket.CONNECTING) socket.close();
  }, 2000);
  socket.addEventListener('open', () => window.clearTimeout(connectingTimeout));
  socket.addEventListener('message', (event) => {
    if (state.driveSocket !== socket) return;
    try {
      const message = JSON.parse(event.data);
      if (Number.isInteger(message.tick)) {
        state.driveTick = message.tick;
        state.driveTickAt = performance.now();
        state.driveReconnectDelay = 0;
        setDriveTransport('WebSocket');
      } else if (message.error) {
        stopDrive();
        $('#drive-preview').textContent = message.error;
      }
    } catch (_) { socket.close(); }
  });
  socket.addEventListener('close', () => {
    window.clearTimeout(connectingTimeout);
    if (state.driveSocket !== socket) return;
    state.driveSocket = null;
    state.driveTick = null;
    setDriveTransport('HTTP резерв');
    // A blocked dedicated WS port is normal on some browsers. Preserve the
    // physical hold state; the same-origin HTTP transport takes over.
    // First reconnect is immediate; back off only while the server stays offline.
    const delay = state.driveReconnectDelay || 0;
    state.driveReconnectDelay = Math.min(1000, Math.max(100, delay * 2));
    window.setTimeout(connectDriveSocket, delay);
  });
}

function setDriveTransport(text) {
  const label = $('#drive-connection');
  if (label) label.textContent = text;
}

function driveSocketReady() {
  return state.driveSocket?.readyState === 1 && state.driveTick !== null
    && performance.now() - state.driveTickAt <= 250;
}

async function sendDriveFallback() {
  if (state.driveHttpPending) return;
  state.driveHttpPending = true;
  const epoch = state.driveInputEpoch;
  try {
    // Ticket age is checked on the robot, so a delayed HTTP request cannot
    // revive an old movement command after a network interruption.
    const ticket = state.driveHttpTicket
      || (await api('/api/drive/ticket', {signal: AbortSignal.timeout(450)})).ticket;
    state.driveHttpTicket = null; // A ticket is single-use even if a request fails.
    if (!ticket || epoch !== state.driveInputEpoch || driveSocketReady()
        || (!state.driveKeys.size && !state.joystick.active)) return;
    const command = computeDriveCommand();
    const result = await api('/api/drive/command', {
      method: 'POST', signal: AbortSignal.timeout(450),
      body: JSON.stringify({client_id: state.driveClientId, sequence: ++state.driveSequence,
        http_ticket: ticket, ui_version: 'wasd-hold-v6',
        linear_x: command.linearX, linear_y: command.linearY, angular_z: command.angularZ}),
    });
    if (result.ticket && epoch === state.driveInputEpoch
        && (state.driveKeys.size || state.joystick.active)) state.driveHttpTicket = result.ticket;
    if (!driveSocketReady()) setDriveTransport('HTTP резерв');
  } catch (_) { if (!driveSocketReady()) setDriveTransport('HTTP резерв'); }
  finally { state.driveHttpPending = false; }
}

function sendDriveCommand() {
  if (!state.driveKeys.size && !state.joystick.active) return;
  const command = computeDriveCommand();
  updateDrivePreview(command);
  const socket = state.driveSocket;
  if (!driveSocketReady()) { sendDriveFallback(); return; }
  // Do not queue stale input when TCP is congested. Next tick samples current keys.
  if (socket.bufferedAmount > 512) return;
  try {
    socket.send(JSON.stringify({
        type: 'drive', tick: state.driveTick,
        client_id: state.driveClientId,
        sequence: ++state.driveSequence,
        linear_x: command.linearX,
        linear_y: command.linearY,
        angular_z: command.angularZ,
    }));
  } catch (_) { socket.close(); }
}

function syncDriveKeyHighlights() {
  $$('#keypad button[data-key]').forEach((button) => {
    button.classList.toggle('active', state.driveKeys.has(button.dataset.key));
  });
}

function startDriveLoop() {
  if (state.driveTimer) return;
  state.driveTimer = window.setInterval(() => {
    if (state.driveKeys.size > 0 || state.joystick.active) {
      sendDriveCommand();
    }
  }, 50);
}

function stopDriveLoop() {
  if (state.driveTimer) {
    window.clearInterval(state.driveTimer);
    state.driveTimer = null;
  }
}

async function stopDrive() {
  state.driveInputEpoch += 1;
  state.driveHttpTicket = null;
  state.joystick = { active: false, x: 0, y: 0, pointer: null };
  $('#drive-joystick-knob').style.transform = '';
  $('#drive-joystick').classList.remove('active');
  stopDriveLoop();
  state.driveKeys.clear();
  syncDriveKeyHighlights();
  updateDrivePreview({ linearX: 0, linearY: 0, angularZ: 0 });
  const payload = {type: 'stop', client_id: state.driveClientId, sequence: ++state.driveSequence};
  if (state.driveSocket?.readyState === 1) {
    try { state.driveSocket.send(JSON.stringify(payload)); } catch (_) { /* HTTP fallback below */ }
  }
  try {
    await api('/api/drive/stop', {
      method: 'POST', keepalive: true, signal: AbortSignal.timeout(1000),
      body: JSON.stringify(payload),
    });
  } catch (error) {
    // no-op
  }
}

async function issueGlobalStop() {
  const driveStop = stopDrive();
  await Promise.allSettled([
    api('/api/stop', {
      method: 'POST',
      body: JSON.stringify({
        source: 'web',
        details: { session_id: state.sessionId, page: state.page },
      }),
    }),
    driveStop,
    api('/api/motion/stop', { method: 'POST', body: '{}' }),
  ]);
  showToast('STOP отправлен', 'error');
  await refreshStatus();
}

function applyRouteDefaults(config) {
  if (!config) return;
  const recommended = config.recommended || {};
  const limits = config.drive_limits || {};
  const angular = recommended.manual_angular_speed_radps ?? config.drive_defaults?.angular_z ?? 0.32;
  const shouldApply = !state.route.draft || !safeArray(state.route.draft.steps).length;
  if (!shouldApply) return;

  $('#route-default-linear').value = String(recommended.manual_linear_speed_mps ?? config.drive_defaults?.linear_x ?? 0.18);
  $('#route-default-approach').value = String(Math.min(recommended.manual_linear_speed_mps ?? 0.18, 0.10));
  $('#route-default-position-tolerance').value = '0.07';
  $('#route-default-angular').value = String(Math.min(angular, limits.angular_z ?? angular));
  $('#route-default-angle-tolerance').value = '3';
  $('#route-default-time').value = '60';

  if (!state.route.draft) {
    state.route.draft = createBlankPlan();
  } else {
    updateDraftDefaults();
  }
}

function currentRouteDefaults() {
  return {
    linear_speed: Number($('#route-default-linear').value || '0.18'),
    approach_speed: Number($('#route-default-approach').value || '0.10'),
    position_tolerance: Number($('#route-default-position-tolerance').value || '0.07'),
    angular_speed: Number($('#route-default-angular').value || '0.32'),
    minimum_angular_speed: 0.10,
    angle_tolerance_deg: Number($('#route-default-angle-tolerance').value || '3'),
    maximum_step_time: 60,
  };
}

function applyDefaultsToInputs(defaults = {}) {
  $('#route-default-linear').value = String(defaults.linear_speed ?? 0.18);
  $('#route-default-approach').value = String(defaults.approach_speed ?? 0.10);
  $('#route-default-position-tolerance').value = String(defaults.position_tolerance ?? 0.07);
  $('#route-default-angular').value = String(defaults.angular_speed ?? 0.32);
  $('#route-default-angle-tolerance').value = String(defaults.angle_tolerance_deg ?? 3);
  $('#route-default-time').value = '60';
}

function normalizePlanName(name) {
  const trimmed = String(name || '').trim();
  if (!trimmed) return 'web_route.yaml';
  return trimmed.endsWith('.yaml') || trimmed.endsWith('.yml') ? trimmed : `${trimmed}.yaml`;
}

function createBlankPlan() {
  return {
    defaults: currentRouteDefaults(),
    steps: [],
  };
}

function createStep(type) {
  const step = { type };
  safeArray(ROUTE_STEP_FIELDS[type]).forEach((field) => {
    if (field.required) {
      step[field.key] = field.defaultValue;
    } else if (field.type === 'checkbox') {
      step[field.key] = field.defaultValue;
    }
  });
  return step;
}

function currentDraftPlan() {
  return state.route.draft || createBlankPlan();
}

function updateRouteStepCount() {
  const count = safeArray(currentDraftPlan().steps).length;
  $('#route-step-count').textContent = `${count} ${count === 1 ? 'шаг' : count < 5 ? 'шага' : 'шагов'}`;
  $('#route-empty').classList.toggle('hidden', count > 0);
}

function updateDraftDefaults() {
  const plan = currentDraftPlan();
  plan.defaults = currentRouteDefaults();
  state.route.draft = plan;
}

function renderRouteSteps() {
  const container = $('#route-step-list');
  container.innerHTML = '';
  const steps = safeArray(currentDraftPlan().steps);

  steps.forEach((step, index) => {
    const card = document.createElement('article');
    card.className = 'route-step';
    card.dataset.index = String(index);

    const header = document.createElement('div');
    header.className = 'route-step-header';

    const title = document.createElement('div');
    title.className = 'route-step-title';
    const number = document.createElement('span');
    number.className = 'route-step-number';
    number.textContent = String(index + 1);
    const label = document.createElement('span');
    label.textContent = ROUTE_STEP_LABELS[step.type] || step.type;
    title.append(number, label);

    const actions = document.createElement('div');
    actions.className = 'route-step-actions';
    actions.innerHTML = `
      <button class="secondary" type="button" data-action="up" ${index === 0 ? 'disabled' : ''}>↑</button>
      <button class="secondary" type="button" data-action="down" ${index === steps.length - 1 ? 'disabled' : ''}>↓</button>
      <button class="danger-secondary" type="button" data-action="delete">✕</button>
    `;

    header.append(title, actions);
    card.append(header);

    const fieldsWrap = document.createElement('div');
    fieldsWrap.className = 'route-step-fields';
    safeArray(ROUTE_STEP_FIELDS[step.type]).forEach((field) => {
      const labelEl = document.createElement('label');
      labelEl.textContent = field.label;
      const input = document.createElement('input');
      input.dataset.field = field.key;
      input.dataset.index = String(index);
      input.type = field.type === 'checkbox' ? 'checkbox' : 'number';
      if (field.type === 'checkbox') {
        input.checked = step[field.key] !== undefined ? Boolean(step[field.key]) : Boolean(field.defaultValue);
      } else {
        if (field.step) input.step = field.step;
        if (field.required) input.required = true;
        if (field.placeholder) input.placeholder = field.placeholder;
        input.value = step[field.key] !== undefined && step[field.key] !== null
          ? String(step[field.key])
          : '';
      }
      labelEl.append(input);
      fieldsWrap.append(labelEl);
    });
    card.append(fieldsWrap);
    container.append(card);
  });

  updateRouteStepCount();
  renderRoutePreview();
}

function renderPlanList() {
  const select = $('#plan-select');
  select.innerHTML = '';
  if (!state.route.plans.length) {
    const option = document.createElement('option');
    option.value = '';
    option.textContent = 'Сохранённых маршрутов нет';
    select.append(option);
    return;
  }
  state.route.plans.forEach((item) => {
    const option = document.createElement('option');
    option.value = item.name;
    option.textContent = `${item.name} · ${item.steps} шагов`;
    option.selected = item.name === state.route.selectedName;
    select.append(option);
  });
}

async function refreshPlanList() {
  try {
    const payload = await api('/api/plans');
    state.route.plans = safeArray(payload.plans);
    if (!state.route.selectedName && state.route.plans.length) {
      state.route.selectedName = state.route.plans[0].name;
    }
    renderPlanList();
  } catch (error) {
    showToast(String(error.message || error), 'error');
  }
}

async function loadPlan(name) {
  const normalized = normalizePlanName(name);
  try {
    const payload = await api(`/api/plans/${encodeURIComponent(normalized)}`);
    state.route.selectedName = payload.name || normalized;
    state.route.draft = payload.plan || createBlankPlan();
    $('#plan-name').value = state.route.selectedName;
    applyDefaultsToInputs(state.route.draft.defaults || {});
    renderPlanList();
    renderRouteSteps();
    showToast(`Маршрут ${state.route.selectedName} загружен`);
    await recordActivity('Загружен маршрут', { name: state.route.selectedName }, 'routes');
  } catch (error) {
    showToast(String(error.message || error), 'error');
  }
}

function newPlan() {
  state.route.selectedName = '';
  state.route.draft = createBlankPlan();
  $('#plan-name').value = 'web_route.yaml';
  renderPlanList();
  renderRouteSteps();
  showToast('Создан новый маршрут');
}

async function savePlan({ silent = false } = {}) {
  const name = normalizePlanName($('#plan-name').value);
  const plan = deepClone(currentDraftPlan());
  plan.defaults = currentRouteDefaults();
  if (!safeArray(plan.steps).length) {
    throw new Error('Маршрут должен содержать хотя бы один шаг');
  }
  await api('/api/plans/save', {
    method: 'POST',
    body: JSON.stringify({ name, plan }),
  });
  state.route.selectedName = name;
  state.route.draft = plan;
  $('#plan-name').value = name;
  await refreshPlanList();
  renderPlanList();
  if (!silent) {
    showToast(`Маршрут ${name} сохранён`);
    await recordActivity('Сохранён маршрут', { name, steps: plan.steps.length }, 'routes');
  }
  return name;
}

async function runSelectedPlan() {
  try {
    const name = await savePlan({ silent: true });
    const payload = await api('/api/motion/start', {
      method: 'POST',
      body: JSON.stringify({ kind: 'plan', name }),
    });
    renderMotionStatus(payload.motion);
    showToast(`Маршрут ${name} запущен`);
    await recordActivity('Запущен маршрут', { name }, 'routes');
  } catch (error) {
    showToast(String(error.message || error), 'error');
  }
}

async function stopMotion() {
  try {
    const payload = await api('/api/motion/stop', { method: 'POST', body: '{}' });
    renderMotionStatus(payload.motion);
    showToast('Маршрут остановлен', 'warn');
  } catch (error) {
    showToast(String(error.message || error), 'error');
  }
}

function routeNavMap() {
  const path = $('#route-nav-map')?.value || state.viz.selectedMap;
  return safeArray(state.viz.maps).find((item) => item.valid && item.path === path) || null;
}

function routeNavPointName(index) {
  return index < 26 ? String.fromCharCode(65 + index) : `P${index + 1}`;
}

function persistRouteNavPoints() {
  localStorage.setItem(STORAGE_KEYS.routeNavPoints, JSON.stringify(state.route.navPoints));
}

function setRouteNavStatus(text, tone = '') {
  const element = $('#route-nav-status');
  if (!element) return;
  element.textContent = text;
  setToneClass(element, tone, text);
}

function renderRouteNavMapSelector() {
  const select = $('#route-nav-map');
  if (!select) return;
  const maps = safeArray(state.viz.maps).filter((item) => item.valid);
  const selected = maps.some((item) => item.path === select.value)
    ? select.value
    : maps.some((item) => item.path === state.viz.selectedMap)
      ? state.viz.selectedMap
      : maps[0]?.path || '';
  select.innerHTML = '';
  if (!maps.length) {
    select.append(new Option('Нет сохранённых карт', ''));
    select.disabled = true;
    renderRouteNavCanvas();
    return;
  }
  maps.forEach((map) => select.append(new Option(map.path, map.path, false, map.path === selected)));
  select.disabled = false;
  loadRouteNavImage();
}

function loadRouteNavImage() {
  const map = routeNavMap();
  if (!map?.image_url) {
    state.route.navImage = null;
    state.route.navImagePath = '';
    renderRouteNavCanvas();
    return;
  }
  if (state.route.navImagePath === map.path && state.route.navImage?.complete) {
    renderRouteNavCanvas();
    return;
  }
  const image = new Image();
  state.route.navImagePath = map.path;
  image.onload = () => {
    state.route.navImage = image;
    renderRouteNavCanvas();
  };
  image.onerror = () => {
    state.route.navImage = null;
    setRouteNavStatus('Не удалось загрузить изображение карты', 'error');
    renderRouteNavCanvas();
  };
  image.src = map.image_url;
}

function routeNavView(canvas, map) {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const origin = Array.isArray(map?.origin) ? map.origin : [0, 0, 0];
  const mapWidth = Math.max(0.5, Number(map?.width_m || 1));
  const mapHeight = Math.max(0.5, Number(map?.height_m || 1));
  const scale = Math.max(1, Math.min((width - 32) / mapWidth, (height - 32) / mapHeight));
  return {
    width,
    height,
    scale,
    centerX: Number(origin[0] || 0) + mapWidth / 2,
    centerY: Number(origin[1] || 0) + mapHeight / 2,
  };
}

function renderRouteNavCanvas() {
  const canvas = $('#route-nav-canvas');
  if (!canvas) return;
  const ctx = setupCanvas(canvas);
  if (!ctx) return;
  const map = routeNavMap();
  const view = routeNavView(canvas, map);
  state.route.navView = view;
  ctx.clearRect(0, 0, view.width, view.height);
  ctx.fillStyle = '#f9fdff';
  ctx.fillRect(0, 0, view.width, view.height);
  drawGrid(ctx, view.width, view.height, view.scale, view.centerX, view.centerY);
  if (map && state.route.navImagePath === map.path && state.route.navImage?.complete) {
    const origin = Array.isArray(map.origin) ? map.origin : [0, 0, 0];
    const x = view.width / 2 + (Number(origin[0] || 0) - view.centerX) * view.scale;
    const y = view.height / 2 - (Number(origin[1] || 0) - view.centerY) * view.scale;
    const mapWidth = Number(map.width_m || 0) * view.scale;
    const mapHeight = Number(map.height_m || 0) * view.scale;
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(-Number(origin[2] || 0));
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(state.route.navImage, 0, -mapHeight, mapWidth, mapHeight);
    ctx.restore();
  }
  const toScreen = (point) => ({
    x: view.width / 2 + (point.x - view.centerX) * view.scale,
    y: view.height / 2 - (point.y - view.centerY) * view.scale,
  });
  if (state.route.navPoints.length > 1) {
    ctx.strokeStyle = '#f0a400';
    ctx.lineWidth = 4;
    ctx.beginPath();
    state.route.navPoints.forEach((point, index) => {
      const screen = toScreen(point);
      if (index === 0) ctx.moveTo(screen.x, screen.y);
      else ctx.lineTo(screen.x, screen.y);
    });
    ctx.stroke();
  }
  state.route.navPoints.forEach((point, index) => {
    const screen = toScreen(point);
    ctx.fillStyle = index === 0 ? '#249255' : '#e14855';
    ctx.beginPath();
    ctx.arc(screen.x, screen.y, 13, 0, Math.PI * 2);
    ctx.fill();
    drawRoverArrow(ctx, screen, Number(point.yaw || 0), '#ffffff', 9);
    ctx.fillStyle = '#063b55';
    ctx.font = '800 15px sans-serif';
    ctx.fillText(routeNavPointName(index), screen.x + 16, screen.y - 12);
  });
}

function renderRouteNavPoints() {
  const container = $('#route-nav-points');
  if (!container) return;
  container.innerHTML = '';
  $('#route-nav-empty').classList.toggle('hidden', state.route.navPoints.length > 0);
  state.route.navPoints.forEach((point, index) => {
    const row = document.createElement('div');
    row.className = 'route-nav-point';
    row.dataset.index = String(index);
    row.innerHTML = `<strong>${routeNavPointName(index)}</strong>
      <label>X, м<input data-axis="x" type="number" step="0.05" value="${Number(point.x).toFixed(2)}"></label>
      <label>Y, м<input data-axis="y" type="number" step="0.05" value="${Number(point.y).toFixed(2)}"></label>
      <label>Курс, °<input data-axis="yaw" type="number" step="1" value="${(Number(point.yaw || 0) * 180 / Math.PI).toFixed(0)}"></label>
      <button class="danger-secondary" data-delete-point="${index}" title="Удалить">✕</button>`;
    container.append(row);
  });
  persistRouteNavPoints();
  renderRouteNavCanvas();
}

function addRouteNavPointFromCanvas(event) {
  const view = state.route.navView;
  if (!view || !routeNavMap()) return;
  const rect = event.currentTarget.getBoundingClientRect();
  const px = (event.clientX - rect.left) * view.width / rect.width;
  const py = (event.clientY - rect.top) * view.height / rect.height;
  const previous = state.route.navPoints.at(-1);
  const x = view.centerX + (px - view.width / 2) / view.scale;
  const y = view.centerY - (py - view.height / 2) / view.scale;
  const yaw = previous ? Math.atan2(y - previous.y, x - previous.x) : 0;
  state.route.navPoints.push({ x, y, yaw });
  renderRouteNavPoints();
}

const routeNavWait = (milliseconds) => new Promise((resolve) => window.setTimeout(resolve, milliseconds));

async function waitForRouteNavGoal(name, timeoutSeconds) {
  const deadline = Date.now() + timeoutSeconds * 1000;
  while (!state.route.navStopRequested && Date.now() < deadline) {
    const status = await api('/api/navigation/status');
    state.viz.navigation = status;
    const goalStatus = String(status.goal_status || '');
    const distance = Number(status.goal?.distance_remaining);
    setRouteNavStatus(`${name}: ${goalStatus}${Number.isFinite(distance) ? ` · ${distance.toFixed(2)} м` : ''}`, 'warn');
    if (goalStatus === 'succeeded') return;
    if (['aborted', 'rejected', 'canceled'].includes(goalStatus) || goalStatus.startsWith('error:')) {
      throw new Error(`${name}: Nav2 завершил цель со статусом ${goalStatus}`);
    }
    await routeNavWait(500);
  }
  if (state.route.navStopRequested) throw new Error('Маршрут остановлен');
  throw new Error(`${name}: превышен лимит ${timeoutSeconds} с`);
}

async function startRouteNavServices() {
  const map = routeNavMap();
  if (!map) throw new Error('Выбери сохранённую карту');
  setRouteNavStatus('Запускаю Nav2…', 'warn');
  await api('/api/navigation/start', { method: 'POST', body: JSON.stringify({ map: map.path }) });
  const deadline = Date.now() + 50000;
  while (Date.now() < deadline) {
    const status = await api('/api/navigation/status');
    state.viz.navigation = status;
    if (status.action_ready && status.localization_ready) {
      setRouteNavStatus('Nav2 готов', 'ok');
      return status;
    }
    await routeNavWait(750);
  }
  throw new Error('Nav2 не стал готов за 50 секунд');
}

async function runRouteNavSequence() {
  if (state.route.navRunning) return;
  if (!state.route.navPoints.length) throw new Error('Добавь хотя бы одну точку на карте');
  const linear = Number($('#route-nav-linear').value);
  const angular = Number($('#route-nav-angular').value);
  const timeoutSeconds = Number($('#route-nav-timeout').value);
  for (const id of ['route-nav-linear', 'route-nav-angular', 'route-nav-timeout']) {
    if (!document.getElementById(id).reportValidity()) return;
  }
  const map = routeNavMap();
  state.route.navRunning = true;
  state.route.navStopRequested = false;
  try {
    await startRouteNavServices();
    for (let index = 0; index < state.route.navPoints.length; index += 1) {
      if (state.route.navStopRequested) break;
      const point = state.route.navPoints[index];
      const name = routeNavPointName(index);
      setRouteNavStatus(`Отправляю точку ${name}…`, 'warn');
      await api('/api/navigation/follow', {
        method: 'POST',
        body: JSON.stringify({ map: map.path, linear_speed: linear, angular_speed: angular, ...point }),
      });
      await waitForRouteNavGoal(name, timeoutSeconds);
    }
    if (!state.route.navStopRequested) {
      setRouteNavStatus('Маршрут полностью выполнен', 'ok');
      showToast('Точки маршрута выполнены по порядку.');
    }
  } finally {
    state.route.navRunning = false;
  }
}

async function stopRouteNavSequence() {
  state.route.navStopRequested = true;
  await api('/api/navigation/cancel', { method: 'POST', body: '{}' });
  setRouteNavStatus('Остановлено', 'warn');
}

function renderRoutePreview() {
  const canvas = $('#route-preview-canvas');
  const ctx = setupCanvas(canvas);
  if (!ctx) return;

  const plan = currentDraftPlan();
  const points = simulatePlan(plan);
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = '#f9fdff';
  ctx.fillRect(0, 0, width, height);

  const bounds = pathBounds(points);
  const margin = 24;
  const usableWidth = Math.max(1, width - margin * 2);
  const usableHeight = Math.max(1, height - margin * 2);
  const rangeX = Math.max(0.5, bounds.maxX - bounds.minX);
  const rangeY = Math.max(0.5, bounds.maxY - bounds.minY);
  const scale = Math.min(usableWidth / rangeX, usableHeight / rangeY);
  const centerX = (bounds.minX + bounds.maxX) / 2;
  const centerY = (bounds.minY + bounds.maxY) / 2;

  const toScreen = (point) => ({
    x: width / 2 + (point.x - centerX) * scale,
    y: height / 2 - (point.y - centerY) * scale,
  });

  drawGrid(ctx, width, height, scale, centerX, centerY);

  const origin = toScreen({ x: 0, y: 0 });
  ctx.strokeStyle = '#c93644';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(origin.x, origin.y, 6, 0, Math.PI * 2);
  ctx.stroke();

  if (points.length > 1) {
    ctx.strokeStyle = '#16b8f3';
    ctx.lineWidth = 3;
    ctx.beginPath();
    points.forEach((point, index) => {
      const screen = toScreen(point);
      if (index === 0) {
        ctx.moveTo(screen.x, screen.y);
      } else {
        ctx.lineTo(screen.x, screen.y);
      }
    });
    ctx.stroke();
  }

  points.forEach((point, index) => {
    const screen = toScreen(point);
    ctx.fillStyle = index === points.length - 1 ? '#075f89' : '#16b8f3';
    ctx.beginPath();
    ctx.arc(screen.x, screen.y, index === points.length - 1 ? 5 : 3, 0, Math.PI * 2);
    ctx.fill();
  });

  const finalPose = points[points.length - 1] || { x: 0, y: 0, yaw: 0 };
  drawRoverArrow(ctx, toScreen(finalPose), finalPose.yaw, '#075f89', 18);
}

function simulatePlan(plan) {
  const pose = { x: 0, y: 0, yaw: 0 };
  const points = [{ ...pose }];
  safeArray(plan.steps).forEach((step) => {
    const type = step.type;
    if (type === 'move') {
      applyRelativeMove(pose, Number(step.forward || 0), Number(step.left || 0));
      points.push({ ...pose });
    } else if (type === 'move_polar') {
      const distance = Number(step.distance || 0);
      const directionRad = Number(step.direction_deg || 0) * Math.PI / 180;
      applyRelativeMove(pose, distance * Math.cos(directionRad), distance * Math.sin(directionRad));
      points.push({ ...pose });
    } else if (type === 'turn') {
      pose.yaw += Number(step.degrees || 0) * Math.PI / 180;
      points.push({ ...pose });
    } else if (type === 'return_to_start') {
      pose.x = 0;
      pose.y = 0;
      if (step.restore_heading !== false) {
        pose.yaw = 0;
      }
      points.push({ ...pose });
    }
  });
  return points;
}

function applyRelativeMove(pose, forward, left) {
  const cosine = Math.cos(pose.yaw);
  const sine = Math.sin(pose.yaw);
  pose.x += cosine * forward - sine * left;
  pose.y += sine * forward + cosine * left;
}

function pathBounds(points) {
  const xs = points.map((point) => point.x);
  const ys = points.map((point) => point.y);
  return {
    minX: Math.min(...xs, -0.25),
    maxX: Math.max(...xs, 0.25),
    minY: Math.min(...ys, -0.25),
    maxY: Math.max(...ys, 0.25),
  };
}

function setupCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  if (!rect.width || !rect.height) return null;
  const dpr = window.devicePixelRatio || 1;
  const width = Math.round(rect.width * dpr);
  const height = Math.round(rect.height * dpr);
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return ctx;
}

function drawGrid(ctx, width, height, scale, centerX, centerY) {
  const spacingMeters = scale >= 180 ? 0.25 : scale >= 100 ? 0.5 : 1.0;
  const left = centerX - width / (2 * scale);
  const right = centerX + width / (2 * scale);
  const bottom = centerY - height / (2 * scale);
  const top = centerY + height / (2 * scale);

  ctx.strokeStyle = '#e3edf2';
  ctx.lineWidth = 1;
  for (let x = Math.floor(left / spacingMeters) * spacingMeters; x <= right; x += spacingMeters) {
    const sx = width / 2 + (x - centerX) * scale;
    ctx.beginPath();
    ctx.moveTo(sx, 0);
    ctx.lineTo(sx, height);
    ctx.stroke();
  }
  for (let y = Math.floor(bottom / spacingMeters) * spacingMeters; y <= top; y += spacingMeters) {
    const sy = height / 2 - (y - centerY) * scale;
    ctx.beginPath();
    ctx.moveTo(0, sy);
    ctx.lineTo(width, sy);
    ctx.stroke();
  }

  ctx.strokeStyle = '#91cfe7';
  ctx.lineWidth = 1.2;
  const axisX = width / 2 + (0 - centerX) * scale;
  const axisY = height / 2 - (0 - centerY) * scale;
  ctx.beginPath();
  ctx.moveTo(axisX, 0);
  ctx.lineTo(axisX, height);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(0, axisY);
  ctx.lineTo(width, axisY);
  ctx.stroke();
}

function drawRoverArrow(ctx, point, yaw, color, size) {
  ctx.save();
  ctx.translate(point.x, point.y);
  ctx.rotate(-yaw);
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.moveTo(size, 0);
  ctx.lineTo(-size * 0.6, size * 0.55);
  ctx.lineTo(-size * 0.25, 0);
  ctx.lineTo(-size * 0.6, -size * 0.55);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function currentVisualizationMap() {
  return safeArray(state.viz.maps).find(
    (item) => item.valid && item.path === state.viz.selectedMap,
  ) || null;
}

function displayedVisualizationMap() {
  if (state.viz.navigation?.mapping?.running && state.viz.liveMap?.ready) {
    return state.viz.liveMap;
  }
  return currentVisualizationMap();
}

function renderVisualizationMapSelector() {
  const select = $('#viz-map-select');
  const visibleToggle = $('#viz-map-visible');
  const validMaps = safeArray(state.viz.maps).filter((item) => item.valid);
  select.innerHTML = '';

  if (!validMaps.length) {
    const option = document.createElement('option');
    option.value = '';
    option.textContent = 'Нет карт в текущей папке навигации';
    select.append(option);
    select.disabled = true;
    visibleToggle.disabled = true;
    state.viz.selectedMap = '';
    $('#viz-map-info').textContent = 'Карта: нет доступных YAML';
    return;
  }

  if (!validMaps.some((item) => item.path === state.viz.selectedMap)) {
    state.viz.selectedMap = validMaps[0].path;
  }

  validMaps.forEach((map) => {
    const option = document.createElement('option');
    option.value = map.path;
    const parts = String(map.path || map.name).split('/');
    const folder = parts.length > 1 ? parts.slice(0, -1).join('/') : map.name;
    option.textContent = `${folder} (${map.resolution} м/px)`;
    option.selected = map.path === state.viz.selectedMap;
    select.append(option);
  });

  select.disabled = false;
  visibleToggle.disabled = false;
  visibleToggle.checked = state.viz.mapVisible;
  localStorage.setItem(STORAGE_KEYS.vizMapSelected, state.viz.selectedMap);
}

function loadVisualizationMapImage(map) {
  if (!map || !map.image_url) return;
  if (
    state.viz.mapImage
    && state.viz.mapImagePath === map.path
    && state.viz.mapImage.complete
  ) {
    return;
  }
  if (state.viz.mapImageLoading && state.viz.mapImagePath === map.path) {
    return;
  }

  state.viz.mapImageLoading = true;
  state.viz.mapImagePath = map.path;
  const image = new Image();
  image.onload = () => {
    state.viz.mapImage = image;
    state.viz.mapImageLoading = false;
    renderVisualization();
  };
  image.onerror = () => {
    state.viz.mapImage = null;
    state.viz.mapImageLoading = false;
    $('#viz-map-info').textContent = `Карта: не удалось загрузить ${map.name}`;
    renderVisualization();
  };
  image.src = map.image_url;
}

async function refreshVisualizationMaps() {
  try {
    const payload = await api('/api/maps');
    state.viz.maps = safeArray(payload.maps);
    state.viz.mapsRoot = payload.root || '';
    renderVisualizationMapSelector();
    renderRouteNavMapSelector();
    const map = currentVisualizationMap();
    if (state.viz.mapVisible && map) {
      loadVisualizationMapImage(map);
    }
    renderVisualization();
    return payload;
  } catch (error) {
    state.viz.maps = [];
    renderVisualizationMapSelector();
    renderRouteNavMapSelector();
    $('#viz-map-info').textContent = String(error.message || error);
    return null;
  }
}

async function refreshLiveMap() {
  try {
    const payload = await api('/api/maps/live');
    state.viz.liveMap = payload.ready ? payload : null;
    if (payload.ready && state.viz.navigation?.mapping?.running) {
      state.viz.mapVisible = true;
      $('#viz-map-visible').checked = true;
      loadVisualizationMapImage(payload);
    }
  } catch (_error) {
    state.viz.liveMap = null;
  }
}

function renderNavigationStatus() {
  const payload = state.viz.navigation;
  if (!payload) return;
  const mapping = Boolean(payload.mapping?.running);
  const navigation = Boolean(payload.navigation?.running);
  const stateLabel = mapping
    ? 'SLAM работает'
    : navigation
      ? payload.action_ready ? 'Nav2 готов' : 'Nav2 запускается'
      : 'Остановлено';
  $('#nav-runtime-state').textContent = stateLabel;
  setToneClass($('#nav-runtime-state'), mapping || navigation ? 'ok' : 'warn', stateLabel);
  $('#mapping-start').disabled = mapping || navigation;
  $('#mapping-save').disabled = !mapping;
  $('#mapping-stop').disabled = !mapping;
  $('#navigation-start').disabled = mapping || navigation || !state.viz.selectedMap;
  $('#navigation-stop').disabled = !navigation;
  $('#navigation-goal').disabled = !state.viz.selectedMap || Boolean(state.viz.followBusy);
  $('#navigation-initial-pose').disabled = !navigation;
  $('#navigation-cancel').disabled = !navigation || !payload.goal;
  const goal = payload.goal;
  const distance = Number(goal?.distance_remaining);
  const planPoints = safeArray(payload.plan?.points).length;
  $('#navigation-status').textContent = mapping
    ? 'SLAM строит карту. Управляй роботом клавишами W/A/S/D и сохрани результат.'
    : navigation
      ? !payload.action_ready
        ? 'Nav2 запускает планировщик и контроллер. Подожди несколько секунд.'
        : !payload.localization_ready
        ? 'Nav2 автоматически устанавливает сохранённую позицию робота.'
        : `Nav2: ${payload.goal_status || 'готов'}${Number.isFinite(distance) ? ` · осталось ${distance.toFixed(2)} м` : ''}${planPoints ? ` · путь ${planPoints} точек` : ''}`
      : 'Выбери старт или цель, затем нажми на карту и протяни мышь в направлении курса.';
  const log = mapping ? payload.mapping?.log : payload.navigation?.log;
  $('#navigation-log').textContent = safeArray(log).slice(-12).join('\n') || '—';
  renderVisualization();
}

async function refreshNavigationStatus() {
  try {
    state.viz.navigation = await api('/api/navigation/status');
    if (state.viz.navigation.mapping?.running) {
      await refreshLiveMap();
    }
    renderNavigationStatus();
  } catch (error) {
    $('#navigation-status').textContent = String(error.message || error);
  }
}

async function navigationAction(path, body = {}) {
  const payload = await api(path, { method: 'POST', body: JSON.stringify(body) });
  if (payload.mapping || payload.navigation) {
    state.viz.navigation = payload;
  }
  await refreshNavigationStatus();
  return payload;
}

function navigationPoseFromInputs(kind = 'goal') {
  const prefix = kind === 'start' ? 'navigation-start' : 'navigation-goal';
  return {
    x: Number($(`#${prefix}-x`).value || 0),
    y: Number($(`#${prefix}-y`).value || 0),
    yaw: Number($(`#${prefix}-yaw`).value || 0) * Math.PI / 180,
  };
}

function setNavigationPoseInputs(kind, pose) {
  const prefix = kind === 'start' ? 'navigation-start' : 'navigation-goal';
  $(`#${prefix}-x`).value = Number(pose.x || 0).toFixed(2);
  $(`#${prefix}-y`).value = Number(pose.y || 0).toFixed(2);
  $(`#${prefix}-yaw`).value = (Number(pose.yaw || 0) * 180 / Math.PI).toFixed(1);
  if (kind === 'start') state.viz.pickedStartPose = { ...pose };
  else state.viz.pickedGoalPose = { ...pose };
}

function setNavigationPickMode(mode) {
  state.viz.pickMode = state.viz.pickMode === mode ? null : mode;
  $('#navigation-pick-start').classList.toggle('active', state.viz.pickMode === 'start');
  $('#navigation-pick-goal').classList.toggle('active', state.viz.pickMode === 'goal');
  $('#odom-canvas').classList.toggle('picking-pose', Boolean(state.viz.pickMode));
  if (state.viz.pickMode) {
    $('#navigation-status').textContent = state.viz.pickMode === 'start'
      ? 'Нажми на стартовую точку и протяни в направлении курса робота.'
      : 'Нажми на цель и протяни в направлении конечного курса.';
  }
}

function canvasPointToWorld(event) {
  const view = state.viz.view;
  if (!view) return null;
  const rect = event.currentTarget.getBoundingClientRect();
  const px = (event.clientX - rect.left) * view.width / rect.width;
  const py = (event.clientY - rect.top) * view.height / rect.height;
  return {
    x: view.centerX + (px - view.width / 2) / view.scale,
    y: view.centerY - (py - view.height / 2) / view.scale,
  };
}

function fitVisualizationMap() {
  const map = displayedVisualizationMap();
  const canvas = $('#odom-canvas');
  if (!map || !canvas.clientWidth || !canvas.clientHeight) return;
  const width = Number(map.width_m || 0);
  const height = Number(map.height_m || 0);
  if (width <= 0 || height <= 0) return;
  const scale = Math.max(60, Math.min(800, Math.floor(Math.min(
    (canvas.clientWidth - 50) / width,
    (canvas.clientHeight - 50) / height,
  ))));
  $('#viz-scale').value = String(scale);
  $('#viz-follow').checked = false;
  state.viz.follow = false;
  renderVisualization();
}

function drawVisualizationMap(ctx, width, height, scale, centerX, centerY) {
  const map = displayedVisualizationMap();
  if (!state.viz.mapVisible || !map) {
    $('#viz-map-info').textContent = state.viz.mapVisible
      ? 'Карта: нет выбранной карты'
      : 'Карта: скрыта';
    return;
  }

  loadVisualizationMapImage(map);
  const image = state.viz.mapImagePath === map.path ? state.viz.mapImage : null;
  const cellCm = Number(map.resolution || 0) * 100;
  $('#viz-map-info').textContent = image?.complete
    ? `Карта: ${map.name}, клетка ${formatFloat(cellCm, 1)} см`
    : `Карта: загрузка ${map.name}…`;
  if (!image?.complete) {
    return;
  }

  const origin = Array.isArray(map.origin) ? map.origin : [0, 0, 0];
  const originX = Number(origin[0] || 0);
  const originY = Number(origin[1] || 0);
  const originYaw = Number(origin[2] || 0);
  const mapWidth = Number(map.width_m || 0) * scale;
  const mapHeight = Number(map.height_m || 0) * scale;
  if (!Number.isFinite(mapWidth) || !Number.isFinite(mapHeight) || mapWidth <= 0 || mapHeight <= 0) {
    return;
  }

  const screenOriginX = width / 2 + (originX - centerX) * scale;
  const screenOriginY = height / 2 - (originY - centerY) * scale;

  ctx.save();
  ctx.translate(screenOriginX, screenOriginY);
  ctx.rotate(-originYaw);
  ctx.globalAlpha = 1;
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(image, 0, -mapHeight, mapWidth, mapHeight);
  ctx.globalAlpha = 1;
  ctx.strokeStyle = '#075f89';
  ctx.lineWidth = 1.2;
  ctx.strokeRect(0, -mapHeight, mapWidth, mapHeight);
  ctx.restore();
}

function renderVisualization() {
  const canvas = $('#odom-canvas');
  const ctx = setupCanvas(canvas);
  if (!ctx) return;

  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = '#f9fdff';
  ctx.fillRect(0, 0, width, height);

  const pose = state.viz.navigation?.navigation?.running
    ? (state.viz.navigation?.robot_pose || state.status?.odom)
    : state.status?.odom;
  const trail = state.viz.trail;
  $('#viz-empty').classList.toggle('hidden', Boolean(pose));
  $('#viz-position').textContent = pose
    ? `Позиция: x=${formatFloat(pose.x, 2)} м, y=${formatFloat(pose.y, 2)} м`
    : 'Позиция: —';
  $('#viz-yaw').textContent = `Курс: ${pose ? formatAngleRad(pose.yaw) : '—'}`;
  $('#viz-points').textContent = `Точек: ${trail.length}`;

  const scale = Number($('#viz-scale').value || state.viz.scale);
  state.viz.scale = scale;
  localStorage.setItem(STORAGE_KEYS.vizScale, String(scale));
  const follow = $('#viz-follow').checked;
  state.viz.follow = follow;
  localStorage.setItem(STORAGE_KEYS.vizFollow, follow ? 'true' : 'false');

  let centerX = pose?.x ?? 0;
  let centerY = pose?.y ?? 0;
  if (!follow) {
    const map = displayedVisualizationMap();
    if (map) {
      const origin = Array.isArray(map.origin) ? map.origin : [0, 0, 0];
      centerX = Number(origin[0] || 0) + Number(map.width_m || 0) / 2;
      centerY = Number(origin[1] || 0) + Number(map.height_m || 0) / 2;
    } else if (trail.length) {
      const bounds = pathBounds(trail);
      centerX = (bounds.minX + bounds.maxX) / 2;
      centerY = (bounds.minY + bounds.maxY) / 2;
    }
  }
  state.viz.view = { width, height, scale, centerX, centerY };

  drawVisualizationMap(ctx, width, height, scale, centerX, centerY);
  drawGrid(ctx, width, height, scale, centerX, centerY);

  const toScreen = (point) => ({
    x: width / 2 + (point.x - centerX) * scale,
    y: height / 2 - (point.y - centerY) * scale,
  });

  const navigationPlan = safeArray(state.viz.navigation?.plan?.points);
  if (navigationPlan.length > 1) {
    ctx.strokeStyle = '#f0a400';
    ctx.lineWidth = 5;
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    ctx.beginPath();
    navigationPlan.forEach((point, index) => {
      const screen = toScreen(point);
      if (index === 0) ctx.moveTo(screen.x, screen.y);
      else ctx.lineTo(screen.x, screen.y);
    });
    ctx.stroke();
  }

  if (trail.length > 1) {
    ctx.strokeStyle = '#16b8f3';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    trail.forEach((point, index) => {
      const screen = toScreen(point);
      if (index === 0) ctx.moveTo(screen.x, screen.y);
      else ctx.lineTo(screen.x, screen.y);
    });
    ctx.stroke();
  }

  if (pose) {
    drawRoverArrow(ctx, toScreen(pose), pose.yaw, '#075f89', 18);
  }
  const start = state.viz.pickedStartPose || state.viz.navigation?.initial_pose;
  if (start) {
    const screen = toScreen(start);
    drawRoverArrow(ctx, screen, start.yaw, '#249255', 16);
    ctx.fillStyle = '#166b3c';
    ctx.font = '700 13px sans-serif';
    ctx.fillText('СТАРТ', screen.x + 18, screen.y - 12);
  }
  const goal = state.viz.pickedGoalPose || state.viz.navigation?.goal;
  if (goal) {
    const target = toScreen(goal);
    ctx.strokeStyle = '#e14855';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(target.x, target.y, 11, 0, Math.PI * 2);
    ctx.moveTo(target.x - 15, target.y);
    ctx.lineTo(target.x + 15, target.y);
    ctx.moveTo(target.x, target.y - 15);
    ctx.lineTo(target.x, target.y + 15);
    ctx.stroke();
    drawRoverArrow(ctx, target, goal.yaw, '#e14855', 14);
    ctx.fillStyle = '#b32131';
    ctx.font = '700 13px sans-serif';
    ctx.fillText('ЦЕЛЬ', target.x + 18, target.y - 12);
  }
}

async function sendHeartbeat() {
  try {
    await api('/api/heartbeat', {
      method: 'POST',
      body: JSON.stringify({
        session_id: state.sessionId,
        page: state.page,
      }),
    });
  } catch (error) {
    // Best effort only.
  }
}

function bindNavigation() {
  $$('.nav-item[data-page]').forEach((button) => {
    button.addEventListener('click', () => {
      if (button.disabled) return;
      const group = button.dataset.navGroup || groupForPage(button.dataset.page);
      setPage(resolveGroupPage(group, button.dataset.page));
    });
  });
  $('#menu-toggle').addEventListener('click', () => {
    $('#sidebar').classList.toggle('open');
  });
}

function bindSectionTabs() {
  $$('.section-tab[data-page]').forEach((button) => {
    button.addEventListener('click', () => {
      setPage(button.dataset.page);
    });
  });
}

function bindOverviewPage() {
  const button = $('#overview-refresh');
  if (button) {
    button.addEventListener('click', async () => {
      await Promise.all([refreshSystem(), refreshStatus()]);
    });
  }
  $('#overview-open-rosboard').addEventListener('click', () => {
    const url = rosboardUrl();
    if (!url) {
      showToast('Rosboard выключен в текущем запуске', 'error');
      return;
    }
    window.open(url, '_blank', 'noopener');
  });
  $('#overview-open-terminal').addEventListener('click', () => {
    setPage('terminal');
  });
}

function bindRosPage() {
  $$('.section-tab[data-ros-tab]').forEach((button) => {
    button.addEventListener('click', () => {
      setRosTab(button.dataset.rosTab);
    });
  });
  $('#nodes-filter').addEventListener('input', renderNodes);
  $('#topics-filter').addEventListener('input', renderTopics);
  $('#services-filter').addEventListener('input', renderServices);
  $('#ros-refresh').addEventListener('click', refreshRosGraph);
  $('#topic-refresh').addEventListener('click', refreshSelectedTopic);
  $('#topic-template').addEventListener('click', restoreTopicTemplate);
  $('#topic-publish').addEventListener('click', publishSelectedTopic);
  $('#service-refresh').addEventListener('click', refreshSelectedService);
  $('#service-template').addEventListener('click', restoreServiceTemplate);
  $('#service-call').addEventListener('click', callSelectedService);
}

function bindCameraPage() {
  for (const [id, action] of [['camera-qr-follow', 'start'], ['camera-qr-stop', 'stop']]) {
    $('#'+id).addEventListener('click', async () => {
      const button = $('#'+id);
      button.disabled = true;
      try {
        await api('/api/qr/approach/'+action, { method: 'POST', body: '{}' });
        $('#camera-qr-follow-status').textContent = action === 'start'
          ? 'Наведение запущено. Остановка после считывания QR.' : 'Остановка отправлена.';
      } catch (error) {
        $('#camera-qr-follow-status').textContent = error.message || String(error);
      } finally { button.disabled = false; }
    });
  }

  $('#camera-refresh-topics').addEventListener('click', async () => {
    await Promise.all([
      refreshRosGraph(),
      refreshCameraSettings(),
      refreshCameraVisionSettings(),
    ]);
    renderCameraTopics();
  });
  $('#camera-connect').addEventListener('click', connectCamera);
  $('#camera-qr-clear').addEventListener('click', async () => {
    await api('/api/qr/clear', { method: 'POST', body: '{}' });
    $('#camera-qr-terminal').textContent = 'Ожидание QR-кода…';
    $('#camera-qr-status').textContent = 'QR: ничего не распознано';
  });
  $$('.camera-mask-controls [data-camera-mask]').forEach((button) => {
    button.classList.toggle('active', button.dataset.cameraMask === state.cameraMask);
    button.addEventListener('click', async () => {
      state.cameraMask = button.dataset.cameraMask || '';
      localStorage.setItem(STORAGE_KEYS.cameraMask, state.cameraMask);
      $$('.camera-mask-controls [data-camera-mask]').forEach((item) => {
        item.classList.toggle('active', item === button);
      });
      if (state.selectedCameraTopic && state.selectedCameraType) {
        await connectCamera();
      }
    });
  });
  $('#sign-detection-start')?.addEventListener('click', async () => {
    try {
      state.signDetection = await api('/api/sign-detection/start', {
        method: 'POST', body: '{}',
      });
      renderSignDetectionStatus();
      showToast('Модель знаков запускается. Загрузка может занять несколько секунд.');
      window.setTimeout(async () => {
        await refreshRosGraph();
        renderCameraTopics();
        await refreshSignDetectionStatus();
      }, 7000);
    } catch (error) {
      showToast(String(error.message || error), 'error');
    }
  });
  $('#sign-detection-stop')?.addEventListener('click', async () => {
    try {
      state.signDetection = await api('/api/sign-detection/stop', {
        method: 'POST', body: '{}',
      });
      renderSignDetectionStatus();
      await refreshRosGraph();
      renderCameraTopics();
    } catch (error) {
      showToast(String(error.message || error), 'error');
    }
  });
  $('#camera-topic-select').addEventListener('change', () => {
    state.selectedCameraTopic = $('#camera-topic-select').value || null;
    state.selectedCameraType = $('#camera-topic-select').selectedOptions[0]?.dataset.type || null;
  });
  $('#camera-vision-enabled').addEventListener('change', toggleCameraVisionEnabled);
  $('#camera-settings-refresh').addEventListener('click', refreshCameraSettings);
  $('#camera-settings-apply').addEventListener('click', applyCameraSettings);
  $('#camera-vision-refresh').addEventListener('click', refreshCameraVisionSettings);
  $('#camera-vision-apply').addEventListener('click', applyCameraVisionSettings);
  $('#camera-vision-open-processed').addEventListener('click', openProcessedCameraStream);
  $('#camera-preset-720p').addEventListener('click', () => applyCameraPreset(1280, 720, 30));
  $('#camera-preset-1080p').addEventListener('click', () => applyCameraPreset(1920, 1080, 30));
  $('#camera-preset-vga').addEventListener('click', () => applyCameraPreset(640, 480, 30));
  $('#camera-frame').addEventListener('load', () => {
    $('#camera-empty').classList.add('hidden');
  });
  $('#camera-frame').addEventListener('error', () => {
    $('#camera-empty').classList.remove('hidden');
  });
}

function bindAudioPage() {
  $('#voice-enabled').addEventListener('change', toggleVoiceEnabled);
  $('#voice-refresh').addEventListener('click', refreshVoiceStatus);
}

function shouldIgnoreDriveKeyEvent(target) {
  if (state.page !== 'drive' && state.page !== 'visualization') return true;
  if (state.page === 'drive' && state.driveMode !== 'keyboard') return true;
  const tag = target?.tagName;
  if ((tag === 'INPUT' && target.type !== 'range') || tag === 'TEXTAREA' || tag === 'SELECT' || target?.isContentEditable) {
    return true;
  }
  return document.activeElement?.id === 'terminal-frame';
}

function bindDrivePage() {
  connectDriveSocket();
  const applyDriveMode = (mode, stop = true) => {
    state.driveMode = mode === 'joystick' ? 'joystick' : 'keyboard';
    localStorage.setItem('frob-drive-mode', state.driveMode);
    $('#drive-keyboard-controls').classList.toggle('hidden', state.driveMode !== 'keyboard');
    $('#drive-joystick-controls').classList.toggle('hidden', state.driveMode !== 'joystick');
    $$('[data-drive-mode]').forEach((button) => {
      button.classList.toggle('active', button.dataset.driveMode === state.driveMode);
    });
    if (stop) stopDrive();
  };
  $$('[data-drive-mode]').forEach((button) => {
    button.addEventListener('click', () => applyDriveMode(button.dataset.driveMode));
  });
  applyDriveMode(state.driveMode, false);

  const joystick = $('#drive-joystick');
  const moveJoystick = (event) => {
    if (!state.joystick.active || event.pointerId !== state.joystick.pointer) return;
    event.preventDefault();
    const rect = joystick.getBoundingClientRect();
    const radius = rect.width / 2 - 30;
    let x = (event.clientX - rect.left - rect.width / 2) / radius;
    let y = (event.clientY - rect.top - rect.height / 2) / radius;
    const length = Math.hypot(x, y);
    if (length > 1) { x /= length; y /= length; }
    if (length < 0.1) { x = 0; y = 0; }
    state.joystick.x = x;
    state.joystick.y = y;
    $('#drive-joystick-knob').style.transform = `translate(${x * radius}px, ${y * radius}px)`;
    updateDrivePreview(computeDriveCommand());
  };
  joystick.addEventListener('pointerdown', (event) => {
    if (state.driveMode !== 'joystick' || state.joystick.active || event.button !== 0) return;
    event.preventDefault();
    state.driveKeys.clear();
    syncDriveKeyHighlights();
    state.joystick.active = true;
    state.joystick.pointer = event.pointerId;
    joystick.setPointerCapture(event.pointerId);
    joystick.classList.add('active');
    moveJoystick(event);
    startDriveLoop();
    sendDriveCommand();
  });
  joystick.addEventListener('pointermove', moveJoystick);
  for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) {
    joystick.addEventListener(name, (event) => {
      if (state.joystick.active && event.pointerId === state.joystick.pointer) stopDrive();
    });
  }

  ['linear-speed', 'lateral-speed', 'angular-speed'].forEach((id) => {
    $(`#${id}`).addEventListener('input', () => {
      updateDriveOutputs();
      updateDrivePreview(computeDriveCommand());
    });
  });

  $('#drive-stop').addEventListener('click', stopDrive);

  $$('#keypad button[data-key]').forEach((button) => {
    let activePointer = null;
    const activate = async (event) => {
      if (state.driveMode !== 'keyboard' || event.button !== 0 || activePointer !== null) return;
      event.preventDefault();
      activePointer = event.pointerId;
      button.setPointerCapture(activePointer);
      state.driveKeys.add(button.dataset.key);
      syncDriveKeyHighlights();
      startDriveLoop();
      await sendDriveCommand();
    };
    const release = async (event) => {
      if (activePointer !== event.pointerId) return;
      activePointer = null;
      state.driveKeys.delete(button.dataset.key);
      syncDriveKeyHighlights();
      if (state.driveKeys.size === 0) {
        stopDriveLoop();
        await stopDrive();
        return;
      }
      await sendDriveCommand();
    };
    button.addEventListener('pointerdown', activate);
    button.addEventListener('pointerup', release);
    button.addEventListener('lostpointercapture', release);
    button.addEventListener('pointercancel', release);
  });

  window.addEventListener('keydown', (event) => {
    if (shouldIgnoreDriveKeyEvent(event.target)) return;
    if (event.code === 'Space') {
      event.preventDefault();
      stopDrive();
      return;
    }
    if (!['KeyW', 'KeyA', 'KeyS', 'KeyD', 'KeyQ', 'KeyE'].includes(event.code)) return;
    event.preventDefault();
    if (event.repeat || state.driveKeys.has(event.code)) return;
    state.driveKeys.add(event.code);
    syncDriveKeyHighlights();
    startDriveLoop();
    sendDriveCommand();
  });

  window.addEventListener('keyup', (event) => {
    if (!state.driveKeys.has(event.code)) return;
    state.driveKeys.delete(event.code);
    syncDriveKeyHighlights();
    if (state.driveKeys.size === 0) {
      stopDriveLoop();
      stopDrive();
      return;
    }
    sendDriveCommand();
  });

  window.addEventListener('blur', stopDrive);
  window.addEventListener('beforeunload', stopDrive);
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) {
      stopDrive();
    }
  });
}

function bindRoutesPage() {
  ['route-default-linear', 'route-default-approach', 'route-default-position-tolerance', 'route-default-angular', 'route-default-angle-tolerance', 'route-default-time']
    .forEach((id) => {
      $(`#${id}`).addEventListener('input', () => {
        updateDraftDefaults();
        renderRoutePreview();
      });
    });

  $$('[data-add-step]').forEach((button) => {
    button.addEventListener('click', () => {
      currentDraftPlan().steps.push(createStep(button.dataset.addStep));
      renderRouteSteps();
    });
  });

  $('#route-step-list').addEventListener('click', (event) => {
    const action = event.target.dataset.action;
    if (!action) return;
    const card = event.target.closest('.route-step');
    const index = Number(card?.dataset.index);
    if (!Number.isFinite(index)) return;
    const steps = currentDraftPlan().steps;

    if (action === 'delete') {
      steps.splice(index, 1);
    } else if (action === 'up' && index > 0) {
      [steps[index - 1], steps[index]] = [steps[index], steps[index - 1]];
    } else if (action === 'down' && index < steps.length - 1) {
      [steps[index + 1], steps[index]] = [steps[index], steps[index + 1]];
    }
    renderRouteSteps();
  });

  $('#route-step-list').addEventListener('input', (event) => {
    const index = Number(event.target.dataset.index);
    const field = event.target.dataset.field;
    if (!Number.isFinite(index) || !field) return;
    const step = currentDraftPlan().steps[index];
    const fieldSpec = safeArray(ROUTE_STEP_FIELDS[step.type]).find((item) => item.key === field);
    if (!fieldSpec) return;

    if (fieldSpec.type === 'checkbox') {
      step[field] = event.target.checked;
    } else {
      const valueText = event.target.value;
      if (valueText === '') {
        if (fieldSpec.required) {
          step[field] = fieldSpec.defaultValue ?? 0;
        } else {
          delete step[field];
        }
      } else {
        step[field] = Number(valueText);
      }
    }
    renderRoutePreview();
  });

  $('#plan-refresh').addEventListener('click', refreshPlanList);
  $('#plan-select').addEventListener('change', (event) => {
    const name = event.target.value;
    if (name) loadPlan(name);
  });
  $('#plan-new').addEventListener('click', newPlan);
  $('#plan-save').addEventListener('click', async () => {
    try {
      await savePlan();
    } catch (error) {
      showToast(String(error.message || error), 'error');
    }
  });
  $('#plan-run').addEventListener('click', runSelectedPlan);
  $('#route-stop').addEventListener('click', stopMotion);
  $('#route-nav-map').addEventListener('change', (event) => {
    state.viz.selectedMap = event.target.value || '';
    localStorage.setItem(STORAGE_KEYS.vizMapSelected, state.viz.selectedMap);
    state.route.navImage = null;
    state.route.navImagePath = '';
    loadRouteNavImage();
  });
  $('#route-nav-refresh').addEventListener('click', refreshVisualizationMaps);
  $('#route-nav-services').addEventListener('click', async () => {
    try {
      await startRouteNavServices();
      showToast('Nav2 и сервисы навигации готовы.');
    } catch (error) {
      setRouteNavStatus(String(error.message || error), 'error');
      showToast(String(error.message || error), 'error');
    }
  });
  $('#route-nav-stop-services').addEventListener('click', async () => {
    try {
      state.route.navStopRequested = true;
      await api('/api/navigation/stop', { method: 'POST', body: '{}' });
      setRouteNavStatus('Nav2 остановлен', 'warn');
    } catch (error) {
      showToast(String(error.message || error), 'error');
    }
  });
  $('#route-nav-canvas').addEventListener('click', addRouteNavPointFromCanvas);
  $('#route-nav-points').addEventListener('input', (event) => {
    const row = event.target.closest('.route-nav-point');
    const index = Number(row?.dataset.index);
    const axis = event.target.dataset.axis;
    if (!Number.isFinite(index) || !axis) return;
    const value = Number(event.target.value);
    if (!Number.isFinite(value)) return;
    state.route.navPoints[index][axis] = axis === 'yaw' ? value * Math.PI / 180 : value;
    persistRouteNavPoints();
    renderRouteNavCanvas();
  });
  $('#route-nav-points').addEventListener('click', (event) => {
    const index = Number(event.target.dataset.deletePoint);
    if (!Number.isFinite(index)) return;
    state.route.navPoints.splice(index, 1);
    renderRouteNavPoints();
  });
  $('#route-nav-clear').addEventListener('click', () => {
    state.route.navPoints = [];
    renderRouteNavPoints();
  });
  $('#route-nav-add-current').addEventListener('click', async () => {
    try {
      const status = await api('/api/navigation/status');
      const pose = status.robot_pose || state.status?.odom;
      if (!pose) throw new Error('Позиция робота пока недоступна');
      state.route.navPoints.push({ x: Number(pose.x), y: Number(pose.y), yaw: Number(pose.yaw || 0) });
      renderRouteNavPoints();
    } catch (error) {
      showToast(String(error.message || error), 'error');
    }
  });
  $('#route-nav-add-zero').addEventListener('click', () => {
    state.route.navPoints.push({ x: 0, y: 0, yaw: 0 });
    renderRouteNavPoints();
  });
  $('#route-nav-run').addEventListener('click', async () => {
    try {
      await runRouteNavSequence();
    } catch (error) {
      setRouteNavStatus(String(error.message || error), 'error');
      showToast(String(error.message || error), 'error');
    }
  });
  $('#route-nav-cancel').addEventListener('click', async () => {
    try {
      await stopRouteNavSequence();
    } catch (error) {
      showToast(String(error.message || error), 'error');
    }
  });
  window.addEventListener('resize', renderRouteNavCanvas);
}

function bindVisualizationPage() {
  $('#viz-scale').value = String(state.viz.scale);
  $('#viz-follow').checked = state.viz.follow;
  $('#viz-map-visible').checked = state.viz.mapVisible;
  $('#viz-scale').addEventListener('input', renderVisualization);
  $('#viz-zoom-out').addEventListener('click', () => {
    $('#viz-scale').value = String(Math.max(60, Number($('#viz-scale').value) - 40));
    renderVisualization();
  });
  $('#viz-zoom-in').addEventListener('click', () => {
    $('#viz-scale').value = String(Math.min(800, Number($('#viz-scale').value) + 40));
    renderVisualization();
  });
  $('#viz-fit-map').addEventListener('click', fitVisualizationMap);
  $('#viz-follow').addEventListener('change', renderVisualization);
  $('#viz-map-visible').addEventListener('change', (event) => {
    state.viz.mapVisible = event.target.checked;
    localStorage.setItem(STORAGE_KEYS.vizMapVisible, state.viz.mapVisible ? 'true' : 'false');
    const map = currentVisualizationMap();
    if (state.viz.mapVisible && map) {
      loadVisualizationMapImage(map);
    }
    renderVisualization();
  });
  $('#viz-map-select').addEventListener('change', (event) => {
    state.viz.selectedMap = event.target.value || '';
    localStorage.setItem(STORAGE_KEYS.vizMapSelected, state.viz.selectedMap);
    state.viz.mapImage = null;
    state.viz.mapImagePath = '';
    const map = currentVisualizationMap();
    if (state.viz.mapVisible && map) {
      loadVisualizationMapImage(map);
    }
    renderVisualization();
  });
  $('#viz-map-refresh').addEventListener('click', refreshVisualizationMaps);
  $('#viz-map-delete').addEventListener('click', async () => {
    const selected = state.viz.selectedMap;
    if (!selected || !window.confirm(`Удалить карту ${selected}?`)) return;
    try {
      await navigationAction('/api/maps/delete', { map: selected });
      state.viz.selectedMap = '';
      state.viz.mapImage = null;
      state.viz.mapImagePath = '';
      await refreshVisualizationMaps();
      showToast('Карта удалена.');
    } catch (error) {
      showToast(String(error.message || error), 'error');
    }
  });
  $('#viz-clear').addEventListener('click', () => {
    state.viz.trail = [];
    renderVisualization();
  });
  $('#mapping-start').addEventListener('click', async () => {
    try {
      await navigationAction('/api/mapping/start');
      showToast('SLAM запущен. Используй W/A/S/D для построения карты.');
    } catch (error) {
      showToast(String(error.message || error), 'error');
    }
  });
  $('#mapping-save').addEventListener('click', async () => {
    try {
      const payload = await navigationAction('/api/mapping/save', {
        name: $('#mapping-name').value.trim(),
      });
      await refreshVisualizationMaps();
      if (payload.map) {
        state.viz.selectedMap = payload.map;
        state.viz.mapVisible = true;
        $('#viz-map-visible').checked = true;
        renderVisualizationMapSelector();
      }
      showToast(`Карта сохранена: ${payload.map || ''}`);
    } catch (error) {
      showToast(String(error.message || error), 'error');
    }
  });
  $('#mapping-stop').addEventListener('click', async () => {
    try {
      await navigationAction('/api/mapping/stop');
    } catch (error) {
      showToast(String(error.message || error), 'error');
    }
  });
  $('#navigation-start').addEventListener('click', async () => {
    try {
      await navigationAction('/api/navigation/start', { map: state.viz.selectedMap });
      showToast('Nav2 запускается. Укажи начальную позицию перед первой целью.');
    } catch (error) {
      showToast(String(error.message || error), 'error');
    }
  });
  $('#navigation-stop').addEventListener('click', async () => {
    try {
      await navigationAction('/api/navigation/stop');
    } catch (error) {
      showToast(String(error.message || error), 'error');
    }
  });
  $('#navigation-cancel').addEventListener('click', async () => {
    try {
      await navigationAction('/api/navigation/cancel');
    } catch (error) {
      showToast(String(error.message || error), 'error');
    }
  });
  $('#navigation-initial-pose').addEventListener('click', async () => {
    try {
      const pose = navigationPoseFromInputs('start');
      await navigationAction('/api/navigation/initial-pose', pose);
      state.viz.pickedStartPose = pose;
      showToast('Начальная позиция отправлена в AMCL.');
    } catch (error) {
      showToast(String(error.message || error), 'error');
    }
  });
  for (const id of ['navigation-linear-speed', 'navigation-angular-speed']) {
    const input = document.getElementById(id);
    const saved = localStorage.getItem(`rover_web.${id}`);
    if (saved !== null && Number(saved) >= Number(input.min) && Number(saved) <= Number(input.max)) input.value = saved;
    input.addEventListener('change', () => {
      if (input.value && input.checkValidity()) localStorage.setItem(`rover_web.${id}`, input.value);
    });
  }
  $('#navigation-goal').addEventListener('click', async () => {
    if (state.viz.followBusy) return;
    for (const id of ['navigation-linear-speed', 'navigation-angular-speed']) {
      const input = document.getElementById(id);
      if (!input.value || !input.reportValidity()) return;
    }
    state.viz.followBusy = true;
    $('#navigation-goal').disabled = true;
    try {
      const pose = navigationPoseFromInputs('goal');
      $('#navigation-status').textContent = 'Запускаю Nav2, восстанавливаю позицию и строю путь…';
      await navigationAction('/api/navigation/follow', {
        map: state.viz.selectedMap,
        initial_pose: state.viz.pickedStartPose || undefined,
        linear_speed: Number($('#navigation-linear-speed').value),
        angular_speed: Number($('#navigation-angular-speed').value),
        ...pose,
      });
      state.viz.pickedGoalPose = pose;
      showToast('Nav2 принял цель. Следование запущено.');
    } catch (error) {
      showToast(String(error.message || error), 'error');
    } finally {
      state.viz.followBusy = false;
      $('#navigation-goal').disabled = !state.viz.selectedMap;
    }
  });
  $('#navigation-confirm-goal').addEventListener('click', () => {
    const pose = navigationPoseFromInputs('goal');
    state.viz.pickedGoalPose = pose;
    $('#navigation-status').textContent = `Цель подтверждена: x=${pose.x.toFixed(2)}, y=${pose.y.toFixed(2)}, курс ${(pose.yaw * 180 / Math.PI).toFixed(1)}°. Теперь нажми «Начать следование к точке».`;
    renderVisualization();
  });
  $('#navigation-pick-start').addEventListener('click', () => setNavigationPickMode('start'));
  $('#navigation-pick-goal').addEventListener('click', () => setNavigationPickMode('goal'));
  ['start', 'goal'].forEach((kind) => {
    ['x', 'y', 'yaw'].forEach((axis) => {
      $(`#navigation-${kind}-${axis}`).addEventListener('input', () => {
        const pose = navigationPoseFromInputs(kind);
        if (kind === 'start') state.viz.pickedStartPose = pose;
        else state.viz.pickedGoalPose = pose;
        renderVisualization();
      });
    });
  });
  $('#odom-canvas').addEventListener('pointerdown', (event) => {
    if (!state.viz.pickMode) return;
    const point = canvasPointToWorld(event);
    if (!point) return;
    event.preventDefault();
    event.currentTarget.setPointerCapture(event.pointerId);
    state.viz.pickStart = point;
    setNavigationPoseInputs(state.viz.pickMode, { ...point, yaw: 0 });
    renderVisualization();
  });
  $('#odom-canvas').addEventListener('pointermove', (event) => {
    if (!state.viz.pickMode || !state.viz.pickStart) return;
    const point = canvasPointToWorld(event);
    if (!point) return;
    const dx = point.x - state.viz.pickStart.x;
    const dy = point.y - state.viz.pickStart.y;
    const current = navigationPoseFromInputs(state.viz.pickMode);
    const yaw = Math.hypot(dx, dy) > 0.03 ? Math.atan2(dy, dx) : current.yaw;
    setNavigationPoseInputs(state.viz.pickMode, { ...state.viz.pickStart, yaw });
    renderVisualization();
  });
  $('#odom-canvas').addEventListener('pointerup', (event) => {
    if (!state.viz.pickMode || !state.viz.pickStart) return;
    const kind = state.viz.pickMode;
    const pose = navigationPoseFromInputs(kind);
    state.viz.pickStart = null;
    setNavigationPickMode(null);
    $('#navigation-status').textContent = `${kind === 'start' ? 'Старт' : 'Цель'}: x=${pose.x.toFixed(2)}, y=${pose.y.toFixed(2)}, курс ${(pose.yaw * 180 / Math.PI).toFixed(1)}°.`;
    renderVisualization();
  });
}

function bindLidarPage() {
  $('#lidar-refresh-topics').addEventListener('click', async () => {
    await Promise.all([refreshRosGraph(), refreshLidarSettings()]);
    renderLidarTopics();
  });
  $('#lidar-connect').addEventListener('click', connectLidar);
  $('#lidar-topic-select').addEventListener('change', () => {
    state.selectedLidarTopic = $('#lidar-topic-select').value || null;
    state.selectedLidarType = $('#lidar-topic-select').selectedOptions[0]?.dataset.type || null;
  });
  $('#lidar-settings-refresh').addEventListener('click', refreshLidarSettings);
  $('#lidar-settings-apply').addEventListener('click', applyLidarSettings);
}

function bindLedStripPage() {
  $('#led-strip-refresh-topics').addEventListener('click', async () => {
    await Promise.all([refreshRosGraph(), refreshLedStripSettings()]);
    renderLedStripTopics();
  });
  $('#led-strip-connect').addEventListener('click', connectLedStrip);
  $('#led-strip-topic-select').addEventListener('change', () => {
    state.selectedLedStripTopic = $('#led-strip-topic-select').value || null;
    state.selectedLedStripType = $('#led-strip-topic-select').selectedOptions[0]?.dataset.type || null;
  });
  $('#led-strip-custom-preset-select').addEventListener('change', () => {
    if (!$('#led-strip-custom-preset-select').value) return;
    try {
      loadSelectedLedStripStaticPresetIntoEditor();
    } catch (error) {
      showToast(String(error.message || error), 'error');
      setLedStripFrameEditorStatus(String(error.message || error));
    }
  });
  $('#led-strip-settings-refresh').addEventListener('click', refreshLedStripSettings);
  $('#led-strip-settings-apply').addEventListener('click', applyLedStripSettings);
  $('#led-strip-apply').addEventListener('click', () => sendLedStripCommand());
  $('#led-strip-off').addEventListener('click', () => {
    $('#led-strip-enabled').checked = false;
    sendLedStripCommand(false);
  });
  $('#led-strip-brightness').addEventListener('input', updateLedStripBrightnessOutput);
  $('#led-strip-speed').addEventListener('input', updateLedStripSpeedOutput);
  $('#led-strip-manual-fill-primary').addEventListener('click', () => {
    fillLedStripPixels($('#led-strip-primary-color').value);
  });
  $('#led-strip-manual-fill-secondary').addEventListener('click', () => {
    fillLedStripPixels($('#led-strip-secondary-color').value);
  });
  $('#led-strip-manual-clear').addEventListener('click', () => {
    fillLedStripPixels('#000000');
    setLedStripFrameEditorStatus('Редактор очищен.');
  });
  $('#led-strip-manual-sync').addEventListener('click', () => {
    syncLedStripPixelsFromStatus(state.ledStripData, { force: true });
    setLedStripFrameEditorStatus('Редактор синхронизирован с текущим превью.');
  });
  $('#led-strip-manual-apply').addEventListener('click', applyLedStripManualFrame);
  $('#led-strip-custom-preset-save').addEventListener('click', () => {
    try {
      saveCurrentLedStripStaticPreset();
    } catch (error) {
      showToast(String(error.message || error), 'error');
      setLedStripFrameEditorStatus(String(error.message || error));
    }
  });
  $('#led-strip-custom-preset-delete').addEventListener('click', () => {
    try {
      deleteSelectedLedStripStaticPreset();
    } catch (error) {
      showToast(String(error.message || error), 'error');
      setLedStripFrameEditorStatus(String(error.message || error));
    }
  });
  [
    'led-strip-enabled',
    'led-strip-effect',
    'led-strip-brightness',
    'led-strip-speed',
    'led-strip-primary-color',
    'led-strip-secondary-color',
  ].forEach((id) => {
    $(`#${id}`).addEventListener('input', markLedStripControlDirty);
    $(`#${id}`).addEventListener('change', markLedStripControlDirty);
  });
}

function bindOctolinerPage() {
  $('#octoliner-refresh-topics').addEventListener('click', async () => {
    await Promise.all([refreshRosGraph(), refreshOctolinerSettings()]);
    renderOctolinerTopics();
  });
  $('#octoliner-connect').addEventListener('click', connectOctoliner);
  $('#octoliner-topic-select').addEventListener('change', () => {
    state.selectedOctolinerTopic = $('#octoliner-topic-select').value || null;
    state.selectedOctolinerType = $('#octoliner-topic-select').selectedOptions[0]?.dataset.type || null;
  });
  $('#octoliner-settings-refresh').addEventListener('click', refreshOctolinerSettings);
  $('#octoliner-settings-apply').addEventListener('click', applyOctolinerSettings);
  $('#octoliner-optimize').addEventListener('click', optimizeOctoliner);
  $('#octoliner-setting-sensitivity').addEventListener('input', updateOctolinerSensitivityOutput);
}

function currentHackathonFile() {
  return safeArray(state.hackathonFiles).find(
    (item) => item.path === state.selectedHackathonFile,
  ) || null;
}

function resetHackathonViewer(message = 'Файл ещё не выбран.') {
  $('#hackathon-file-frame').src = 'about:blank';
  $('#hackathon-file-frame').removeAttribute('sandbox');
  $('#hackathon-file-frame').classList.add('hidden');
  $('#hackathon-markdown-view').innerHTML = '';
  $('#hackathon-markdown-view').classList.add('hidden');
  $('#hackathon-file-empty').textContent = message;
  $('#hackathon-file-empty').classList.remove('hidden');
}

function renderHackathonFileSelector() {
  const select = $('#hackathon-file-select');
  const openButton = $('#hackathon-open-file');
  const files = safeArray(state.hackathonFiles);
  select.innerHTML = '';

  if (!files.length) {
    const option = document.createElement('option');
    option.value = '';
    option.textContent = 'Нет доступных файлов';
    select.append(option);
    select.disabled = true;
    openButton.disabled = true;
    state.selectedHackathonFile = '';
    localStorage.removeItem(STORAGE_KEYS.hackathonFile);
    $('#hackathon-file-status').textContent = 'В папке хакатона пока нет файлов.';
    resetHackathonViewer('В папке хакатона пока нет файлов.');
    return;
  }

  if (!files.some((item) => item.path === state.selectedHackathonFile)) {
    state.selectedHackathonFile = files[0].path;
  }

  files.forEach((item) => {
    const option = document.createElement('option');
    option.value = item.path;
    option.textContent = `${item.path} (${hackathonKindLabel(item.kind)})`;
    if (item.path === state.selectedHackathonFile) {
      option.selected = true;
    }
    select.append(option);
  });

  select.disabled = false;
  openButton.disabled = false;
  localStorage.setItem(STORAGE_KEYS.hackathonFile, state.selectedHackathonFile);
}

async function refreshHackathonFiles() {
  try {
    const payload = await api('/api/hackathon/files');
    state.hackathonFiles = safeArray(payload.files);
    renderHackathonFileSelector();
    return payload;
  } catch (error) {
    state.hackathonFiles = [];
    renderHackathonFileSelector();
    $('#hackathon-file-status').textContent = String(error.message || error);
    showToast(String(error.message || error), 'error');
    return null;
  }
}

async function openSelectedHackathonFile() {
  const select = $('#hackathon-file-select');
  const selectedPath = (select.value || state.selectedHackathonFile || '').trim();
  if (!selectedPath) {
    resetHackathonViewer('Файл ещё не выбран.');
    $('#hackathon-file-status').textContent = 'Выбери файл для просмотра.';
    return;
  }

  state.selectedHackathonFile = selectedPath;
  localStorage.setItem(STORAGE_KEYS.hackathonFile, selectedPath);
  const file = currentHackathonFile();
  if (!file) {
    resetHackathonViewer('Файл не найден в текущем списке.');
    $('#hackathon-file-status').textContent = 'Файл не найден в текущем списке.';
    return;
  }

  const url = hackathonFileUrl(file.path);
  $('#hackathon-file-status').textContent = `Открывается ${file.path}...`;

  try {
    if (String(file.kind) === 'markdown') {
      const response = await fetch(url, { cache: 'no-store' });
      if (!response.ok) {
        throw new Error(`Не удалось открыть файл (${response.status})`);
      }
      const text = await response.text();
      $('#hackathon-file-frame').src = 'about:blank';
      $('#hackathon-file-frame').removeAttribute('sandbox');
      $('#hackathon-file-frame').classList.add('hidden');
      $('#hackathon-markdown-view').innerHTML = renderMarkdownDocument(text);
      $('#hackathon-markdown-view').classList.remove('hidden');
    } else {
      $('#hackathon-markdown-view').innerHTML = '';
      $('#hackathon-markdown-view').classList.add('hidden');
      if (String(file.kind) === 'html') {
        $('#hackathon-file-frame').setAttribute('sandbox', '');
      } else {
        $('#hackathon-file-frame').removeAttribute('sandbox');
      }
      $('#hackathon-file-frame').src = url;
      $('#hackathon-file-frame').classList.remove('hidden');
    }

    $('#hackathon-file-empty').classList.add('hidden');
    $('#hackathon-file-status').textContent = `Открыт файл: ${file.path} · ${hackathonKindLabel(file.kind)} · ${formatBytes(file.size_bytes)}`;
  } catch (error) {
    resetHackathonViewer('Не удалось открыть выбранный файл.');
    $('#hackathon-file-status').textContent = String(error.message || error);
    showToast(String(error.message || error), 'error');
  }
}

function bindHackathonPage() {
  $('#hackathon-refresh-files').addEventListener('click', async () => {
    const payload = await refreshHackathonFiles();
    if (payload) {
      await openSelectedHackathonFile();
    }
  });
  $('#hackathon-open-file').addEventListener('click', openSelectedHackathonFile);
  $('#hackathon-file-select').addEventListener('change', async (event) => {
    state.selectedHackathonFile = event.target.value || '';
    localStorage.setItem(STORAGE_KEYS.hackathonFile, state.selectedHackathonFile);
    await openSelectedHackathonFile();
  });
}

let robotTerminal = null;
let terminalInputQueue = Promise.resolve();
const terminalCommandHints = [
  ['Файлы', 'ls -lah'],
  ['Текущий каталог', 'pwd'],
  ['ROS: узлы', 'ros2 node list'],
  ['ROS: топики', 'ros2 topic list'],
  ['Одометрия', 'ros2 topic echo /odom --once'],
  ['Лидар', 'ros2 topic hz /scan'],
  ['Сервисы робота', 'systemctl --user --no-pager status frob-bringup frob-dashboard'],
  ['Журнал робота', 'journalctl --user -u frob-bringup -n 50 --no-pager'],
  ['Нагрузка', 'top'],
  ['Память', 'free -h'],
  ['IP-адреса', 'hostname -I'],
  ['ROS workspace', 'cd ~/RTC_Evacuation/ros2/src/ros2_ws'],
];
function sendTerminalInput(data) {
  const sessionId = state.terminalSession;
  terminalInputQueue = terminalInputQueue.catch(() => {}).then(() => api('/api/terminal/session/input', {
    method: 'POST', body: JSON.stringify({ data, session_id: sessionId }),
  })).catch((error) => {
    $('#terminal-title').textContent = `Ошибка ввода: ${String(error.message || error)}`;
  });
  return terminalInputQueue;
}
function resizeRobotTerminal() {
  if (!robotTerminal || state.page !== 'terminal') return;
  const box = $('#terminal-command-output');
  const columns = Math.max(20, Math.floor((box.clientWidth - 30) / 9.65));
  const rows = Math.max(10, Math.floor((box.clientHeight - 24) / 19.2));
  robotTerminal.resize(columns, rows);
  api('/api/terminal/session/resize', {
    method: 'POST', body: JSON.stringify({ columns, rows, session_id: state.terminalSession }),
  }).catch(() => {});
}
function ensureRobotTerminal() {
  if (robotTerminal) return;
  robotTerminal = new Terminal({
    cursorBlink: true, fontSize: 16, lineHeight: 1.2,
    fontFamily: 'monospace', scrollback: 3000,
    theme: { background: '#0d141c', foreground: '#dbe7f3', cursor: '#39c4f0', selectionBackground: '#35516b' },
  });
  robotTerminal.open($('#terminal-command-output'));
  robotTerminal.onData(sendTerminalInput);
  new ResizeObserver(resizeRobotTerminal).observe($('#terminal-command-output'));
}
function bindTerminalPage() {
  $('#terminal-session-select').value = state.terminalSession;
  $('#terminal-session-select').addEventListener('change', (event) => {
    stopTerminalPolling();
    state.terminalSession = event.target.value;
    sessionStorage.setItem('frob-shell', state.terminalSession);
    history.replaceState(null, '', `?shell=${state.terminalSession}`);
    state.terminalStarted = false;
    state.terminalCursor = 0;
    robotTerminal?.reset();
    startTerminalSession();
  });
  $('#terminal-new-window').addEventListener('click', () => {
    const next = Number(state.terminalSession.split('-')[1]) % 4 + 1;
    window.open(`/?shell=terminal-${next}`, `frob-terminal-${next}`, 'popup,width=1100,height=800');
  });
  $('#terminal-close-session').addEventListener('click', async () => {
    try {
      await api('/api/terminal/session/close', {
        method: 'POST', body: JSON.stringify({ session_id: state.terminalSession }),
      });
      state.terminalStarted = false;
      showToast('Сессия завершена. Кнопка перезапуска откроет новую.');
    } catch (error) { showToast(String(error.message || error), 'error'); }
  });
  $('#terminal-reload').addEventListener('click', () => startTerminalSession(true));
  $('#terminal-command-clear').addEventListener('click', () => {
    ensureRobotTerminal();
    robotTerminal.clear();
    robotTerminal.focus();
  });
  $('#terminal-interrupt').addEventListener('click', () => {
    sendTerminalInput('\x03');
    robotTerminal?.focus();
  });
  for (const [label, command] of terminalCommandHints) {
    const button = document.createElement('button');
    button.className = 'terminal-hint';
    button.textContent = label;
    button.title = command;
    const code = document.createElement('code');
    code.textContent = command;
    button.append(code);
    button.addEventListener('click', () => {
      sendTerminalInput('\x15' + command);
      robotTerminal?.focus();
    });
    $('#terminal-suggestions-list').append(button);
  }
}

let llmPublishQueue = Promise.resolve();
function publishLlmMessages() {
  const messages = state.llmMessages.slice(-100).map((message, index, all) => {
    const copy = { ...message };
    if (index < all.length - 6) delete copy.imageData;
    return copy;
  });
  llmPublishQueue = llmPublishQueue.catch(() => {}).then(() => api('/api/llm/publish', {
    method: 'POST', body: JSON.stringify({ messages }),
  })).catch(() => {});
}

function saveLlmMessages() {
  publishLlmMessages();
  state.llmMessages = state.llmMessages.slice(-200);
  const stored = state.llmMessages.map((message, index, messages) => {
    if (!message.imageData || index >= messages.length - 6) return message;
    const copy = { ...message };
    delete copy.imageData;
    return copy;
  });
  try {
    localStorage.setItem(STORAGE_KEYS.llmMessages, JSON.stringify(stored));
  } catch (_error) {
    const withoutImages = stored.map((message) => {
      const copy = { ...message };
      delete copy.imageData;
      return copy;
    });
    localStorage.setItem(STORAGE_KEYS.llmMessages, JSON.stringify(withoutImages.slice(-100)));
  }
}

function addLlmMessage(role, text, extra = {}) {
  state.llmMessages.push({
    role,
    text,
    timestamp: Date.now(),
    ...extra,
  });
  saveLlmMessages();
  renderLlmMessages();
}

function sameEvacuationCell(left, right) {
  return Array.isArray(left) && Array.isArray(right)
    && Number(left[0]) === Number(right[0])
    && Number(left[1]) === Number(right[1]);
}

function updateLlmRouteMap(model, pose, path, label = 'Маршрут обновляется по мере движения робота.') {
  if (!model || !Array.isArray(pose?.cell)) return;
  const route = safeArray(path).map((point) => [Number(point[0]), Number(point[1])]);
  const currentCell = [Number(pose.cell[0]), Number(pose.cell[1])];
  let visiblePath = route;
  if (visiblePath.length && sameEvacuationCell(currentCell, visiblePath.at(-1))) {
    visiblePath = [currentCell];
  } else if (!visiblePath.length || !sameEvacuationCell(currentCell, visiblePath[0])) {
    visiblePath = [currentCell, ...visiblePath];
  }
  const routeModel = JSON.parse(JSON.stringify(model));
  routeModel.chatMinimal = true;
  const routePose = JSON.parse(JSON.stringify(pose));
  const signature = JSON.stringify({
    cell: routePose.cell,
    heading: routePose.heading,
    path: visiblePath,
    markers: routeModel.markers,
    person: routeModel.person,
  });
  if (signature === state.llmRouteSignature) return;
  state.llmRouteSignature = signature;
  state.llmRouteMap = { model: routeModel, pose: routePose, path: visiblePath, label };
  const canvas = document.createElement('canvas');
  drawEvacuationMap(canvas, routeModel, routePose, visiblePath);
  const imageData = canvas.toDataURL('image/jpeg', .88);
  let message = [...state.llmMessages].reverse().find((entry) => entry.liveRouteMap);
  if (!message) {
    message = {
      role: 'assistant',
      text: label,
      timestamp: Date.now(),
      liveRouteMap: true,
      imageAlt: 'Текущий маршрут робота',
    };
    state.llmMessages.push(message);
  }
  Object.assign(message, {
    text: label,
    imageData,
    routeModel,
    routePose,
    routePath: visiblePath,
    updatedAt: Date.now(),
  });
  saveLlmMessages();
  renderLlmMessages();
}

function pickLlmPhrase(stage) {
  const phrases = LLM_PHRASES[stage] || [];
  if (!phrases.length) return '';
  const previous = state.llmLastPhrase[stage];
  const candidates = phrases.length > 1
    ? phrases.filter((phrase) => phrase !== previous)
    : phrases;
  const phrase = candidates[Math.floor(Math.random() * candidates.length)];
  state.llmLastPhrase[stage] = phrase;
  return phrase;
}

function updateLlmMissionStatus() {
  const status = $('#llm-mission-status');
  if (!status) return;
  status.textContent = 'Сохрани карту и отправь задание. Робот построит путь и начнёт движение.';
}

async function beginLlmQrMission() {
  if (state.llmMission?.stage === 'moving' || state.llmMission?.stage === 'waiting_qr') {
    addLlmMessage('assistant', 'Задание уже выполняется.');
    return;
  }
  let status = { text: '', history: [] };
  try {
    status = await api('/api/qr/status');
  } catch (_error) {
    // The poller will retry while the mission is active.
  }
  state.llmMission = {
    stage: 'moving',
    waitingQr: false,
    startedAt: Date.now(),
    baselineText: String(status.text || ''),
    baselineHistoryLength: safeArray(status.history).length,
  };
  localStorage.setItem(STORAGE_KEYS.llmMission, JSON.stringify(state.llmMission));
  addLlmMessage('assistant', pickLlmPhrase('startQr'));
  updateLlmMissionStatus();
  try {
    await api('/api/motion/start', {
      method: 'POST',
      body: JSON.stringify({
        kind: 'move',
        forward: 0.1,
        left: 0.0,
        speed: 0.18,
        approach_speed: 0.18,
        position_tolerance: 0.015,
        correction_speed: 0.18,
      }),
    });
  } catch (error) {
    state.llmMission = null;
    localStorage.removeItem(STORAGE_KEYS.llmMission);
    updateLlmMissionStatus();
    addLlmMessage('assistant', `Не удалось начать движение: ${String(error.message || error)}`);
  }
}

function addLlmLiveCamera() {
  if (state.llmMessages.some((message) => message.liveCamera)) return;
  addLlmMessage('assistant', 'Включил видеопоток. Покажи QR-код камере.', {
    liveCamera: true,
  });
}

function blobToDataUrl(blob) {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.addEventListener('load', () => resolve(String(reader.result || '')));
    reader.addEventListener('error', () => reject(reader.error || new Error('Не удалось прочитать кадр')));
    reader.readAsDataURL(blob);
  });
}

async function captureLlmQrFrame() {
  const topic = encodeURIComponent('/camera/qr_annotated');
  const type = encodeURIComponent('sensor_msgs/msg/Image');
  let lastError = new Error('Кадр ещё не готов');
  for (let attempt = 0; attempt < 4; attempt += 1) {
    const response = await fetch(`/api/camera/frame?topic=${topic}&type=${type}&t=${Date.now()}`, {
      cache: 'no-store',
    });
    if (response.ok) return blobToDataUrl(await response.blob());
    lastError = new Error(`HTTP ${response.status}`);
    await new Promise((resolve) => window.setTimeout(resolve, 250));
  }
  throw lastError;
}

async function refreshLlmQrMission() {
  if (!state.llmMission || state.llmQrBusy) return;
  if (Date.now() - Number(state.llmMission.startedAt || 0) > 15 * 60 * 1000) {
    state.llmMission = null;
    localStorage.removeItem(STORAGE_KEYS.llmMission);
    updateLlmMissionStatus();
    return;
  }
  state.llmQrBusy = true;
  try {
    if (state.llmMission.stage === 'moving') {
      const status = await api('/api/status');
      const motion = status.motion || {};
      if (motion.running || Date.now() - Number(state.llmMission.startedAt || 0) < 900) {
        return;
      }
      if (motion.return_code == null) return;
      state.llmMission.stage = 'waiting_qr';
      state.llmMission.waitingQr = true;
      localStorage.setItem(STORAGE_KEYS.llmMission, JSON.stringify(state.llmMission));
      addLlmLiveCamera();
      updateLlmMissionStatus();
      return;
    }
    if (state.llmMission.stage !== 'waiting_qr') return;
    const payload = await api('/api/qr/status');
    const text = String(payload.text || '').trim();
    const history = safeArray(payload.history);
    const fresh = text && Number(payload.age_sec) <= 2.0;
    const isNew = (
      history.length > Number(state.llmMission.baselineHistoryLength || 0)
      || text !== String(state.llmMission.baselineText || '')
    );
    if (!fresh || !isNew) return;

    let imageData = '';
    try {
      imageData = await captureLlmQrFrame();
    } catch (_error) {
      imageData = '';
    }
    state.llmMessages = state.llmMessages.filter((message) => !message.liveCamera);
    saveLlmMessages();
    renderLlmMessages();
    state.llmMission = { stage: 'complete', completedAt: Date.now() };
    localStorage.removeItem(STORAGE_KEYS.llmMission);
    updateLlmMissionStatus();

    addLlmMessage(
      'assistant',
      `${pickLlmPhrase('qrRead')}\nСодержимое: ${text}`,
      imageData ? { imageData, imageAlt: `QR-код: ${text}` } : {},
    );
    window.setTimeout(() => {
      addLlmMessage('assistant', 'Пора выполнять задание.');
    }, 650);
  } catch (_error) {
    // Keep waiting through temporary Wi-Fi or camera gaps.
  } finally {
    state.llmQrBusy = false;
  }
}

function isQrTechnicalMessage(text) {
  return /^(Тест без движения:|Начинаю движение к|Еду вперёд на|Поворот на|Маршрут остановлен или истёк|Ищу QR-код и навожусь|Плавно еду вперёд и ищу QR|QR не найден за|QR найден\. (Приближаюсь|Выравниваю)|QR потерян\.|Жду устойчивое обнаружение QR|Сканирую пространство камерой|Цель сместилась\.)/.test(String(text || ''));
}
function cleanLlmText(text) {
  return String(text || '').replace(/(?:[:;]1111)/g, '').replace('Рядом обнаружены:', 'Рядом:');
}

function renderLlmMessages() {
  const container = $('#llm-chat-messages');
  container.innerHTML = '';
  if (!state.llmMessages.length) {
    const empty = document.createElement('div');
    empty.className = 'llm-chat-empty';
    empty.textContent = 'Сообщений пока нет.';
    container.append(empty);
    return;
  }
  state.llmMessages.forEach((message) => {
    if (message.role === 'assistant' && isQrTechnicalMessage(message.text)) return;
    const item = document.createElement('div');
    const role = message.role === 'assistant' ? 'assistant' : 'user';
    item.className = `llm-message ${role}`;
    if (message.liveRouteMap) item.classList.add('route-map-message');
    const author = document.createElement('strong');
    author.textContent = role === 'assistant' ? 'Робот' : 'Ты';
    const text = document.createElement('div');
    text.textContent = cleanLlmText(message.text);
    item.append(author, text);
    if (message.liveCamera) {
      const stream = document.createElement('img');
      const topic = encodeURIComponent('/camera/qr_annotated');
      const type = encodeURIComponent('sensor_msgs/msg/Image');
      stream.src = `/api/camera/stream?preview=1&neural=1&topic=${topic}&type=${type}&llm=${Date.now()}`;
      stream.alt = 'Видеопоток с QR и распознаванием объектов';
      item.append(stream);
    } else if (message.imageData) {
      const image = document.createElement('img');
      image.src = message.imageData;
      image.alt = message.imageAlt || 'Кадр с распознанным QR-кодом';
      item.append(image);
    }
    const time = document.createElement('small');
    time.textContent = new Date(message.timestamp).toLocaleString('ru-RU');
    item.append(time);
    container.append(item);
  });
  container.scrollTop = container.scrollHeight;
}

let evacuationChatBusy = false;
let evacuationPollBusy = false;
async function pollEvacuationChat() {
  if (evacuationPollBusy) return;
  evacuationPollBusy = true;
  try {
    const status = await api('/api/evacuation/status');
    const key = `frob-evac-events-${status.epoch}`;
    let cursor = Number(localStorage.getItem(key) || 0);
    for (const event of status.events) {
      if (event.id <= cursor) continue;
      const extra = { timestamp: event.timestamp };
      if (event.imageData) {
        extra.imageData = event.imageData;
        extra.imageAlt = event.imageAlt || 'Считанный QR-код';
      }
      if (event.map) {
        updateLlmRouteMap(event.map, event.pose || status.pose, event.path || [],
          `Текущий путь · робот в клетке (${(event.pose || status.pose).cell.join(', ')})`);
      }
      if (!isQrTechnicalMessage(event.text)) addLlmMessage('assistant', cleanLlmText(event.text), extra);
      cursor = event.id;
      localStorage.setItem(key, String(cursor));
    }
    if (state.llmRouteMap && Array.isArray(status.pose?.cell)) {
      updateLlmRouteMap(
        state.llmRouteMap.model,
        status.pose,
        state.llmRouteMap.path,
        `Текущий путь · робот в клетке (${status.pose.cell.join(', ')})`,
      );
    }
    $('#llm-mission-status').textContent = status.running
      ? `Выполняется маршрут, последняя достигнутая клетка (${status.pose.cell.join(', ')})`
      : 'Ожидание задания.';
  } catch (_error) {} finally { evacuationPollBusy = false; }
}
async function sendEvacuationChat(text) {
  if (evacuationChatBusy) { addLlmMessage('assistant', 'Запрос уже обрабатывается.'); return; }
  evacuationChatBusy = true;
  try {
    if (/(^|\s)(стоп|stop|останови|остановить|отмени|отмена)(\s|[.!?,]|$)/i.test(text)) {
      await api('/api/evacuation/stop', { method: 'POST', body: '{}' });
      addLlmMessage('assistant', 'Остановка маршрута отправлена.');
      return;
    }
    if (!text.trim()) return;
    const previewOnly = false;
    const prepared = await api('/api/mission/prepare', { method: 'POST', body: JSON.stringify({ text }) });
    if (prepared.keywords?.length) {
      addLlmMessage('assistant', `Ключевые слова: ${prepared.keywords.join(' · ')}`);
    }
    const plan = prepared.plan;
    addLlmMessage('assistant', prepared.reply);
    const visibleMap = JSON.parse(JSON.stringify(plan.map));
    visibleMap.markers = Object.fromEntries(Object.entries(visibleMap.markers || {})
      .filter(([name]) => (plan.discovered || []).includes(name)));
    visibleMap.person = (plan.discovered || []).includes('person') ? visibleMap.person : null;
    visibleMap.chatMinimal = true;
    const neighborhood = ['blue_area', 'yellow_area'].includes(plan.target);
    const initialPath = neighborhood ? plan.path : plan.path.slice(0, 2);
    state.llmRouteSignature = '';
    updateLlmRouteMap(
      visibleMap,
      plan.start,
      initialPath,
      neighborhood ? 'Путь до окрестностей дома' : 'Следующий участок маршрута',
    );
    if (previewOnly) {
      return;
    } else {
      const launch = await api('/api/evacuation/start', { method: 'POST', body: JSON.stringify({ plan_id: plan.id }) });
      addLlmMessage('assistant', ['Начинаю движение по маршруту.', 'Принято, выдвигаюсь к первой точке.', 'Поехали. Направляюсь к ориентиру.'][Math.floor(Math.random()*3)]);
    }
    await pollEvacuationChat();
  } catch (error) { addLlmMessage('assistant', String(error.message || error)); }
  finally { evacuationChatBusy = false; }
}

function bindLlmPage() {
  state.llmMission = null;
  localStorage.removeItem(STORAGE_KEYS.llmMission);
  state.llmMessages = state.llmMessages.filter((message) => !/^Объекты на карте:|^На карте пока нет отмеченных объектов/.test(String(message.text || '')));
  const savedRoute = [...state.llmMessages].reverse().find((message) => (
    message.liveRouteMap && message.routeModel && message.routePose
  ));
  if (savedRoute) {
    state.llmRouteMap = {
      model: savedRoute.routeModel,
      pose: savedRoute.routePose,
      path: safeArray(savedRoute.routePath),
      label: savedRoute.text,
    };
    state.llmRouteSignature = '';
  }
  saveLlmMessages();
  window.setInterval(() => { if (state.page === 'llm') pollEvacuationChat(); }, 1000);
  if (state.llmMessages.length) publishLlmMessages();
  const send = async () => {
    const input = $('#llm-chat-input');
    const text = input.value.trim();
    if (!text) return;
    input.value = '';
    addLlmMessage('user', text);
    await sendEvacuationChat(text);
  };
  $('#llm-chat-send').addEventListener('click', send);
  $('#llm-chat-clear').addEventListener('click', async () => {
    try {
      await api('/api/evacuation/new-dialog', { method: 'POST', body: '{}' });
    } catch (error) {
      addLlmMessage('assistant', error.message || String(error));
      return;
    }
    state.llmMessages = [];
    state.llmRouteMap = null;
    state.llmRouteSignature = '';
    publishLlmMessages();
    localStorage.removeItem(STORAGE_KEYS.llmMessages);
    state.llmMission = null;
    localStorage.removeItem(STORAGE_KEYS.llmMission);
    updateLlmMissionStatus();
    renderLlmMessages();
  });
  $('#llm-chat-input').addEventListener('keydown', (event) => {
    if (event.key === 'Enter' && !event.shiftKey) {
      event.preventDefault();
      send();
    }
  });
  renderLlmMessages();
  updateLlmMissionStatus();
}

function bindDiagnosticsPage() {
  $('#diagnostics-refresh').addEventListener('click', async () => {
    await Promise.all([refreshSystem(), refreshStatus(), refreshActivity()]);
  });
}

function bindSettingsPage() {
  bindCompactMode();
  bindServoUsage();
}

function refreshPeriodicData() {
  refreshStatus();
  if (state.page === 'overview' || state.page === 'diagnostics' || !state.system) {
    refreshSystem();
  }
  if (state.page === 'ros' || !state.rosGraph) {
    refreshRosGraph();
    refreshSelectedTopic();
    refreshSelectedService();
  }
  if (state.page === 'diagnostics') {
    refreshActivity();
  }
  if (state.page === 'camera' && state.selectedCameraTopic && state.selectedCameraType && !state.cameraTimer) {
    connectCamera();
  }
  if (state.page === 'camera') {
    refreshSignDetectionStatus();
  }
  if (state.page === 'lights' && state.selectedLedStripTopic && state.selectedLedStripType && !state.ledStripTimer) {
    connectLedStrip();
  }
  if (state.page === 'octoliner' && state.selectedOctolinerTopic && state.selectedOctolinerType && !state.octolinerTimer) {
    connectOctoliner();
  }
  if (state.page === 'lidar' && state.selectedLidarTopic && state.selectedLidarType && !state.lidarTimer) {
    connectLidar();
  }
  if (state.page === 'audio') {
    refreshVoiceStatus();
  }
  if (state.page === 'visualization') {
    refreshNavigationStatus();
  }
}

async function initialize() {
  bindNavigation();
  bindSectionTabs();
  bindOverviewPage();
  bindRosPage();
  bindCameraPage();
  bindAudioPage();
  bindDrivePage();
  bindRoutesPage();
  bindVisualizationPage();
  bindLidarPage();
  bindHackathonPage();
  bindLlmPage();
  bindTerminalPage();
  bindDiagnosticsPage();
  bindSettingsPage();

  state.route.draft = createBlankPlan();
  updateDrivePreview({ linearX: 0, linearY: 0, angularZ: 0 });
  renderRouteSteps();
  renderRouteNavPoints();

  await Promise.all([
    refreshIdentityAndConfig(),
    refreshSystem(),
    refreshStatus(),
    refreshRosGraph(),
    refreshCameraSettings(),
    refreshCameraVisionSettings(),
    refreshSignDetectionStatus(),
    refreshVoiceStatus(),
    refreshLidarSettings(),
    refreshDriveConfig(),
    refreshVisualizationMaps(),
    refreshNavigationStatus(),
    refreshPlanList(),
    refreshHackathonFiles(),
    refreshActivity(),
  ]);

  if (!state.route.plans.length) {
    newPlan();
  } else if (state.route.selectedName) {
    await loadPlan(state.route.selectedName);
  }

  setPage(['overview', 'ros', 'camera', 'drive', 'routes', 'visualization', 'lidar', 'audio', 'actuators', 'hackathon', 'llm', 'terminal', 'diagnostics', 'settings'].includes(state.page)
    ? state.page
    : 'overview');
  setRosTab(state.rosTab);
  renderNodeDetails();
  renderSettings();
  renderVisualization();
  drawLidarVisualization();
  renderRoutePreview();
  await openSelectedHackathonFile();

  await recordActivity('Web session started', {
    session_id: state.sessionId,
    user_agent: navigator.userAgent,
  }, 'web');

  await sendHeartbeat();

  window.setInterval(sendHeartbeat, 5000);
  window.setInterval(refreshPeriodicData, 1500);
  window.setInterval(() => {
    if (state.page === 'overview' || state.page === 'settings') {
      refreshIdentityAndConfig();
    }
  }, 12000);
  window.addEventListener('resize', () => {
    renderVisualization();
    drawLidarVisualization();
    drawOctolinerVisualization();
    renderRoutePreview();
  });
}

initialize().catch((error) => {
  state.apiHealthy = false;
  state.rosHealthy = false;
  updateHealthIndicators();
  showToast(`Ошибка инициализации: ${error.message || error}`, 'error');
  console.error(error);
});
