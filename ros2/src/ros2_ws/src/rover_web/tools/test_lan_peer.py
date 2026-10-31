import ipaddress
import time
import unittest
from unittest.mock import patch
from rover_web.lan_peer import LANPeer


class PeerTests(unittest.TestCase):
    def setUp(self):
        self.lans = patch('rover_web.lan_peer.local_lans', return_value=[
            ('wlan0', '192.168.45.2', ipaddress.ip_network('192.168.45.0/30'))])
        self.lans.start()
        self.addCleanup(self.lans.stop)
        self.peer = LANPeer('aa:bb:cc:dd:ee:ff', 8765, 'rover_web')

    def test_new_dhcp_address_replaces_old(self):
        self.peer._address = '192.168.44.251'
        self.peer._verified = time.monotonic()
        with patch.object(self.peer, '_neighbors', return_value=['192.168.45.1']), \
                patch.object(self.peer, '_alive', return_value=True) as alive:
            self.assertEqual(self.peer.resolve(), 'http://192.168.45.1:8765')
            alive.assert_called_once_with('192.168.45.1')

    def test_wrong_device_at_old_ip_is_rejected(self):
        with patch('rover_web.lan_peer.ip_json', return_value=[
            {'dst':'192.168.45.1','dev':'wlan0','lladdr':'00:00:00:00:00:01'},
            {'dst':'10.1.0.3','dev':'tun0','lladdr':self.peer.mac},
        ]):
            self.assertEqual(self.peer._neighbors([
                ('wlan0','192.168.45.2',ipaddress.ip_network('192.168.45.0/30'))]), [])

    def test_offline_backoff_and_reappearance(self):
        with patch.object(self.peer, '_neighbors', return_value=[]) as neighbors, \
                patch.object(self.peer, '_probe') as probe, \
                patch.object(self.peer, '_alive', return_value=True):
            with self.assertRaises(OSError): self.peer.resolve()
            count = probe.call_count
            self.assertEqual(count, 1)
            with self.assertRaises(OSError): self.peer.resolve()
            self.assertEqual(probe.call_count, count)
            # Newly learned MAC is used immediately even during scan backoff.
            neighbors.return_value = ['192.168.45.1']
            self.assertEqual(self.peer.resolve(), 'http://192.168.45.1:8765')

    def test_wrong_service_is_rejected(self):
        with patch.object(self.peer, '_neighbors', return_value=['192.168.45.1']), \
                patch.object(self.peer, '_probe'), patch.object(self.peer, '_alive', return_value=False):
            with self.assertRaises(OSError): self.peer.resolve()

    def test_transport_failure_invalidates_but_never_replays_post(self):
        import field_console
        with patch.object(field_console, 'ROBOT') as peer, \
                patch.object(field_console, 'urlopen', side_effect=TimeoutError('lost reply')) as request:
            peer.resolve.return_value = 'http://192.168.45.1:8765'
            with self.assertRaises(OSError): field_console.robot_request('/api/evacuation/start', {'plan_id':'x'})
            request.assert_called_once()
            peer.invalidate.assert_called_once()


if __name__ == '__main__': unittest.main()
