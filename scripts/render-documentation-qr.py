"""Write a print-resolution QR code for a URL supplied on standard input."""

from pathlib import Path
import sys

import cv2


def main():
    destination = Path(sys.argv[1])
    url = sys.stdin.read().strip()
    params = cv2.QRCodeEncoder_Params()
    params.correction_level = cv2.QRCodeEncoder_CORRECT_LEVEL_M
    matrix = cv2.QRCodeEncoder_create(params).encode(url)
    x, y, width, height = cv2.boundingRect(cv2.bitwise_not(matrix))
    matrix = matrix[y : y + height, x : x + width]
    matrix = cv2.copyMakeBorder(matrix, 4, 4, 4, 4, cv2.BORDER_CONSTANT, value=255)
    image = cv2.resize(matrix, None, fx=16, fy=16, interpolation=cv2.INTER_NEAREST)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(destination), image):
        raise RuntimeError(f"Cannot write QR code to {destination}")


if __name__ == "__main__":
    main()
