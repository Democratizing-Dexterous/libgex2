import sys

sys.path.append("../")  # replace with the actual path to libgex2
from libgex2 import Hand
import numpy as np


hand = Hand("/dev/ttyUSB0")  # or using serial_number='XXXX'
hand.connect()

hand.home()
