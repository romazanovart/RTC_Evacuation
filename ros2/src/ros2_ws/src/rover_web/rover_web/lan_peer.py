"""Find a known LAN device after DHCP changes; never identify it by old IP alone.

Only directly connected physical LANs are searched. A peer must match both the
configured MAC and the read-only HTTP service identity. Discovery never retries
motion requests and does not transmit user data to candidate devices.
"""
from concurrent.futures import ThreadPoolExecutor
import ipaddress
import json
from pathlib import Path
import socket
import subprocess
import threading
import time
from urllib.request import ProxyHandler, build_opener


def ip_json(*args):
    return json.loads(subprocess.check_output(['ip', '-j', *args], timeout=2))


def local_lans():
    result = []
    for interface in ip_json('address', 'show'):
        name = interface['ifname']
        if ('UP' not in interface.get('flags', []) or
                not (Path('/sys/class/net') / name / 'device').exists()):
            continue
        for address in interface.get('addr_info', []):
            if address.get('family') != 'inet' or address.get('scope') != 'global':
                continue
            network = ipaddress.ip_network(f"{address['local']}/{address['prefixlen']}", strict=False)
            # Bound discovery to 1022 hosts; still accept known neighbors on larger LANs.
            result.append((name, address['local'], network))
    return result


class LANPeer:
    def __init__(self, mac, port, service):
        self.mac = mac.lower()
        self.port = port
        self.service = service
        self._address = None
        self._verified = 0.
        self._retry_after = 0.
        self._lock = threading.Lock()
        self._http = build_opener(ProxyHandler({}))

    def _neighbors(self, lans):
        return [row['dst'] for row in ip_json('neigh', 'show')
                if row.get('lladdr', '').lower() == self.mac
                and any(row.get('dev') == name and ':' not in row['dst']
                        and ipaddress.ip_address(row['dst']) in network
                        for name, _, network in lans)]

    def _alive(self, address):
        try:
            with self._http.open(f'http://{address}:{self.port}/api/health', timeout=.8) as response:
                return json.loads(response.read(4096)).get('service') == self.service
        except (OSError, ValueError):
            return False

    def _probe(self, address):
        try:
            with socket.create_connection((address, self.port), timeout=.35):
                pass
        except OSError:
            pass

    def invalidate(self):
        with self._lock:
            self._address = None
            self._verified = self._retry_after = 0.

    def resolve(self):
        with self._lock:
            now = time.monotonic()
            lans = local_lans()
            candidates = self._neighbors(lans)
            if self._address in candidates and now - self._verified < 2.:
                return f'http://{self._address}:{self.port}'
            for address in candidates:
                if self._alive(address):
                    self._address, self._verified = address, time.monotonic()
                    self._retry_after = 0.
                    return f'http://{address}:{self.port}'
            self._address = None
            if now >= self._retry_after:
                addresses = {str(host) for _, own, network in lans if network.num_addresses <= 1024
                             for host in network.hosts() if str(host) != own}
                with ThreadPoolExecutor(max_workers=48) as pool:
                    list(pool.map(self._probe, sorted(addresses)))
                for address in self._neighbors(lans):
                    if self._alive(address):
                        self._address, self._verified = address, time.monotonic()
                        self._retry_after = 0.
                        return f'http://{address}:{self.port}'
                self._retry_after = time.monotonic() + 10.
            raise OSError(f'Устройство {self.mac} не найдено в текущей локальной сети')


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--mac', default='00:00:00:00:00:00')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--service', default='rover_web')
    args = parser.parse_args()
    print(LANPeer(args.mac, args.port, args.service).resolve())
