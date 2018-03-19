#!/usr/bin/env python

from argparse import ArgumentParser
from Image import open
from pykeyboard import PyKeyboard

def press_q():
    k = PyKeyboard()
    k.tap_key(0x18) # 0x18 causes 'q' to be pressed

def main(args):
    # Press q to close the previous window image
    press_q()
    # Open the given image in a new window
    open(args.image).show()

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("image", help="Path to the image to display")
    args = parser.parse_args()
    main(args)
