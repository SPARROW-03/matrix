#!/usr/bin/env python3
"""
crop_template.py

Opens a saved frame (from capture_frame.py) in a window where you draw a
box tightly around just the digit with your mouse, then saves the crop
as a template.

Usage:
    python3 crop_template.py raw_digit_1.png templates/digit_1.png

Controls in the window:
    - Click and drag a box around the digit
    - Press ENTER or SPACE to confirm
    - Press 'c' to cancel selection and retry
"""

import sys
import os
import cv2


def main():
    if len(sys.argv) < 3:
        print('Usage: python3 crop_template.py <input_frame.png> <output_template.png>')
        sys.exit(1)

    in_path, out_path = sys.argv[1], sys.argv[2]
    img = cv2.imread(in_path)
    if img is None:
        print(f'Could not read {in_path}')
        sys.exit(1)

    roi = cv2.selectROI('Select digit region, then press ENTER', img, showCrosshair=True)
    cv2.destroyAllWindows()

    x, y, w, h = roi
    if w == 0 or h == 0:
        print('Empty selection, nothing saved.')
        sys.exit(1)

    crop = img[y:y+h, x:x+w]
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    cv2.imwrite(out_path, crop)
    print(f'Saved template to {out_path} ({w}x{h})')


if __name__ == '__main__':
    main()