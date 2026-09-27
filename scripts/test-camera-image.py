import binascii
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('capture', Path(__file__).with_name('capture-camera-image.py'))
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


class ImageTransferTests(unittest.TestCase):
    def setUp(self):
        self.payload = bytearray(153600)
        self.payload[:2] = b'\xf8\x00'  # red at native upper-left
        self.fields = capture.parse_header(
            ('[CAMSHOT] v=1 width=320 height=240 bytes=153600 crc32=%08x '
             'frame=1 exposure=135 msb_first=1 rotate180=1\n' % binascii.crc32(self.payload)).encode())

    def test_rotation_and_rgb565_expansion(self):
        image = capture.decode_image(self.fields, self.payload)
        self.assertEqual(image.getpixel((319, 239)), (255, 0, 0))
        self.assertEqual(image.getpixel((0, 0)), (0, 0, 0))

    def test_corruption_and_truncation_rejected(self):
        with self.assertRaises(ValueError):
            capture.validate_payload(self.fields, self.payload[:-1])
        self.payload[2] = 1
        with self.assertRaises(ValueError):
            capture.validate_payload(self.fields, self.payload)

    def test_invalid_size_rejected(self):
        with self.assertRaises(ValueError):
            capture.parse_header(b'[CAMSHOT] v=1 width=320 height=240 bytes=999 crc32=0 frame=1 exposure=135 msb_first=1 rotate180=1')

    def test_actual_usb_captures_match_archived_images(self):
        from PIL import Image
        root = Path(__file__).resolve().parents[1] / 'simulation/evidence/camera_diagnostics'
        for label in ('prototype', '01', '02'):
            base = root / ('20260927_green_camshot_' + label)
            fields, payload = capture.decode_wire(base.with_suffix('.serial.bin').read_bytes())
            self.assertEqual(capture.decode_image(fields, payload).tobytes(),
                             Image.open(base.with_suffix('.png')).convert('RGB').tobytes())


if __name__ == '__main__':
    unittest.main()
