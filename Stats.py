# ==============================================================
#                        S t a t s . p y
# ==============================================================

# Author:  Dean Earl Wright III
# Created: 25 August 2011
# Purpose: Reimplementation of the Q-learning perturbation
#          testbed for multiple metacognition levels.

# ==============================================================
#                                                        imports
# ==============================================================

import unittest

from Constants import *
from Vector import Vector

# ==============================================================
#                                                          Stats
# ==============================================================
class Stats(object):
    def __init__(self):
        self.__data = {}
        self.__knts = {}

    def __getitem__(self, key): return self.__data[key]

    def add(self, key, data):
        if key in self.__data:
            self.__data[key] = self.__data[key] + data
            self.__knts[key] = self.__knts[key] + 1
        else:
            self.__data[key] = Vector(data)
            self.__knts[key] = 1

    def average(self, key):
        return self.__data[key] / Vector(len(self.__data[key])*(float(self.__knts[key]),))

    def __len__(self):
        return len(self.__data)

# ==============================================================
#                                                     unit tests
# ==============================================================
class TestStats(unittest.TestCase):

    def testEmptyConstructor(self):
        v = Stats()
        self.assertEqual(len(v),             0)


# ==============================================================
#                                                           main
# ==============================================================
def main():
    unittest.main()

# ==============================================================
#                                          module initialization
# ==============================================================
if __name__ == "__main__":
    main()

# ==============================================================
# end                    S t a t s . p y                     end
# ==============================================================
