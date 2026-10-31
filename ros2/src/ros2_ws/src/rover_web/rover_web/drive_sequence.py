"""Reject reordered browser commands, especially commands arriving after STOP.

The caller holds its drive lock across acceptance AND command application.
Sequence entries are not expired: a delayed packet must not resurrect a command.
"""
import hashlib
import hmac
import secrets
import time


def legacy_browser_control(payload, user_agent):
    """Reload the pre-fallback UI, while accepting the new ticketed fallback."""
    return bool(payload.get('client_id') and 'sequence' in payload
                and 'http_ticket' not in payload and 'Mozilla/' in user_agent)


class DriveLease:
    """Short-lived server tickets for the same-origin HTTP control fallback."""
    def __init__(self, clock=time.monotonic_ns):
        self.clock = clock
        self.key = secrets.token_bytes(32)

    def issue(self):
        stamp = str(self.clock())
        signature = hmac.new(self.key, stamp.encode(), hashlib.sha256).hexdigest()
        return f'{stamp}.{signature}'

    def valid(self, ticket):
        if not isinstance(ticket, str) or len(ticket) > 100:
            return False
        try:
            stamp, signature = ticket.split('.')
            age = self.clock() - int(stamp)
        except (ValueError, TypeError):
            return False
        if len(signature) != 64 or not signature.isascii():
            return False
        expected = hmac.new(self.key, stamp.encode(), hashlib.sha256).hexdigest()
        return 0 <= age <= 250_000_000 and hmac.compare_digest(signature, expected)


class DriveSequence:
    def __init__(self, limit=256):
        self.latest = {}
        self.limit = limit

    def accept(self, client, sequence):
        # Preserve compatibility with existing non-browser clients.
        if client is None and sequence is None:
            return True
        if (not isinstance(client, str) or not 1 <= len(client) <= 80
                or type(sequence) is not int or not 0 <= sequence <= 2**53 - 1):
            raise ValueError('Invalid drive command sequence')
        if client not in self.latest and len(self.latest) >= self.limit:
            raise ValueError('Too many drive sessions; restart dashboard')
        if sequence <= self.latest.get(client, -1):
            return False
        self.latest[client] = sequence
        return True
