"""Receive a stopped robot's camshot image; validate size, CRC and footer.

Requires pyserial and Pillow. No driving command or firmware upload is issued.
Outputs belong in ignored local_workspace unless explicitly curated as evidence.
"""
import argparse
import binascii
import json
from pathlib import Path
import time


def parse_header(line):
    if not line.startswith(b'[CAMSHOT] '):
        raise ValueError('Not a camshot header')
    fields = dict(token.split('=', 1) for token in line.decode('ascii').split()[1:])
    for key in ('v', 'width', 'height', 'bytes', 'frame', 'exposure', 'msb_first', 'rotate180'):
        fields[key] = int(fields[key])
    fields['crc32'] = int(fields['crc32'], 16)
    if (fields['v'] != 1 or fields['width'] != 320 or fields['height'] != 240 or
            fields['bytes'] != 153600 or fields['msb_first'] not in (0, 1) or
            fields['rotate180'] not in (0, 1)):
        raise ValueError('Unsupported image header')
    return fields


def validate_payload(fields, payload):
    if len(payload) != fields['bytes']:
        raise ValueError('Incomplete image')
    if binascii.crc32(payload) != fields['crc32']:
        raise ValueError('Image CRC mismatch')


def decode_image(fields, payload):
    from PIL import Image
    validate_payload(fields, payload)
    rgb = bytearray()
    for i in range(0, len(payload), 2):
        a, b = payload[i:i+2]
        value = (a << 8 | b) if fields['msb_first'] else (b << 8 | a)
        r, g, blue = (value >> 11) & 31, (value >> 5) & 63, value & 31
        rgb.extend(((r << 3) | (r >> 2), (g << 2) | (g >> 4), (blue << 3) | (blue >> 2)))
    image = Image.frombytes('RGB', (fields['width'], fields['height']), bytes(rgb))
    if fields['rotate180']:
        image = image.transpose(Image.Transpose.ROTATE_180)
    return image


def decode_wire(wire):
    start = wire.find(b'[CAMSHOT] ')
    if start < 0:
        raise ValueError('No image header in serial archive')
    end = wire.index(b'\n', start)
    fields = parse_header(wire[start:end+1])
    payload_end = end+1+fields['bytes']
    payload = wire[end+1:payload_end]
    validate_payload(fields, payload)
    if wire[payload_end:payload_end+15] != b'\n[CAMSHOT END]\n':
        raise ValueError('Image footer missing')
    return fields, payload


def capture(port_name, output, label):
    import serial
    output.mkdir(parents=True, exist_ok=True)
    base = output / label
    wire = base.with_suffix('.serial.bin')
    if wire.exists():
        raise ValueError('Capture label already exists')
    port = serial.Serial(port_name, 115200, timeout=.2)
    try:
        with wire.open('wb') as archive:
            def read(count):
                data = port.read(count)
                archive.write(data)
                return data
            port.write(b'c0\n')
            until = time.monotonic()+3
            while time.monotonic() < until:
                read(max(1, port.in_waiting))
            port.write(b'camshot\n')
            deadline = time.monotonic()+15
            line = bytearray()
            fields = None
            while time.monotonic() < deadline:
                byte = read(1)
                if not byte:
                    continue
                line.extend(byte)
                if byte == b'\n':
                    if line.startswith(b'[CAMSHOT ERROR]'):
                        raise RuntimeError(line.decode('ascii', errors='replace').strip())
                    if line.startswith(b'[CAMSHOT] '):
                        fields = parse_header(bytes(line))
                        break
                    line.clear()
                if len(line) > 4096:
                    raise ValueError('Unframed response')
            if fields is None:
                raise TimeoutError('No camshot header')
            payload = bytearray()
            deadline = time.monotonic()+12
            while len(payload) < fields['bytes'] and time.monotonic() < deadline:
                payload.extend(read(min(4096, fields['bytes']-len(payload))))
            validate_payload(fields, payload)
            footer = bytearray()
            expected = b'\n[CAMSHOT END]\n'
            while len(footer) < len(expected) and time.monotonic() < deadline:
                footer.extend(read(len(expected)-len(footer)))
            if footer != expected:
                raise ValueError('Image footer missing')
        base.with_suffix('.rgb565').write_bytes(payload)
        base.with_suffix('.json').write_text(json.dumps(fields, indent=2)+'\n')
        decode_image(fields, payload).save(base.with_suffix('.png'))
        print(json.dumps(fields))
        print(base.with_suffix('.png').as_posix())
    finally:
        port.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('label')
    parser.add_argument('--port')
    parser.add_argument('--replay', type=Path, help='Decode archived serial data offline')
    parser.add_argument('--output', type=Path, default=Path('local_workspace/camera-images-20260927'))
    args = parser.parse_args()
    if args.replay:
        fields, payload = decode_wire(args.replay.read_bytes())
        args.output.mkdir(parents=True, exist_ok=True)
        base = args.output / args.label
        decode_image(fields, payload).save(base.with_suffix('.png'))
        base.with_suffix('.json').write_text(json.dumps(fields, indent=2)+'\n')
        print(base.with_suffix('.png').as_posix())
        raise SystemExit(0)
    if not args.port:
        import serial.tools.list_ports
        ports = [p.device for p in serial.tools.list_ports.comports() if p.vid == 0x2341]
        if len(ports) != 1:
            parser.error('Specify --port; expected exactly one Arduino USB serial device')
        args.port = ports[0]
    capture(args.port, args.output, args.label)
