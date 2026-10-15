# -*- coding: utf-8 -*-
"""Forwarder to 1_robot_navigation/gui_robot.py"""
import os
import sys

BASE = os.path.dirname(os.path.abspath(__file__))
sub = os.path.join(BASE, "1_robot_navigation")
if sub not in sys.path:
    sys.path.insert(0, sub)
import gui_robot

if __name__ == "__main__":
    gui_robot.main()
