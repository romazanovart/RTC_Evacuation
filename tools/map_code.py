import base64
import hashlib
import json
import zlib


def encode_map(model):
    raw = json.dumps({'version': 1, 'map': model}, ensure_ascii=False, sort_keys=True, separators=(',', ':')).encode()
    packed = zlib.compress(raw, 9)
    body = base64.urlsafe_b64encode(packed).decode().rstrip('=')
    return 'FROBMAP1.' + body + '.' + hashlib.sha256(packed).hexdigest()[:16]


def decode_map(code):
    prefix, body, checksum = code.strip().split('.')
    if prefix != 'FROBMAP1' or len(body) > 100000:
        raise ValueError('Неверный формат карты')
    packed = base64.b64decode(body + '=' * (-len(body) % 4), altchars=b'-_', validate=True)
    if hashlib.sha256(packed).hexdigest()[:16] != checksum:
        raise ValueError('Код карты повреждён')
    decoder = zlib.decompressobj()
    raw = decoder.decompress(packed, 100001)
    if len(raw) > 100000 or not decoder.eof or decoder.unused_data:
        raise ValueError('Неверный размер карты')
    result = json.loads(raw)
    if result.get('version') != 1 or not isinstance(result.get('map'), dict):
        raise ValueError('Неверная версия карты')
    return result['map']


if __name__ == '__main__':
    import sys
    print(json.dumps(decode_map(sys.stdin.read()), ensure_ascii=False, indent=2))
