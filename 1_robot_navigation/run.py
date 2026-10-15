# -*- coding: utf-8 -*-
"""Entry point for Mobile Robot Navigation GUI."""
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import gui_robot

if __name__ == "__main__":
    gui_robot.main()
