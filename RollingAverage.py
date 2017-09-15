# ==============================================================
#               R o l l i n g A v e r a g e . p y
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

# ==============================================================
#                                                 RollingAverage
# ==============================================================
class RollingAverage(object):

    def __init__(self, n=AVERAGE_WINDOW):
        self.__values = []
        self.__n      = n
        self.__total  = 0.0

    def add(self, number):
        if len(self.__values) == self.__n:
            self.__total = self.__total - self.__values[0]
            self.__values = self.__values[1:]

        self.__values.append(number)
        self.__total = self.__total + number

    def __len__(self): return len(self.__values)
    def count(self):   return len(self.__values)
    def n(self):       return self.__n
    def total(self):   return self.__total
    def average(self):
        if 0 == len(self.__values):
            return 0.0
        else:
            return self.__total / float(len(self.__values))

# ==============================================================
#                                                     unit tests
# ==============================================================
class TestRollingAverage(unittest.TestCase):

    def testEmptyConstructor(self):
        r = RollingAverage()
        self.assertEqual(len(r),          0)
        self.assertEqual(r.count(),       0)
        self.assertEqual(r.n(), AVERAGE_WINDOW)
        self.assertEqual(r.total(),       0)
        self.assertEqual(r.average(),     0)

    def testConstructor(self):
        r = RollingAverage(n=10)
        self.assertEqual(len(r),          0)
        self.assertEqual(r.count(),       0)
        self.assertEqual(r.n(),          10)
        self.assertEqual(r.total(),       0)
        self.assertEqual(r.average(),     0)

    def testAverages(self):
        r = RollingAverage(n=10)
        self.assertEqual(len(r),          0)
        self.assertEqual(r.count(),       0)
        self.assertEqual(r.n(),          10)
        self.assertEqual(r.total(),       0)
        self.assertEqual(r.average(),     0)
        r.add(1)
        r.add(2)
        r.add(0)
        r.add(3)
        r.add(4)
        self.assertEqual(len(r),          5)
        self.assertEqual(r.count(),       5)
        self.assertEqual(r.n(),          10)
        self.assertEqual(r.total(),      10)
        self.assertEqual(r.average(),     2)
        r.add(2)
        r.add(2)
        r.add(2)
        r.add(2)
        r.add(2)
        self.assertEqual(len(r),         10)
        self.assertEqual(r.count(),      10)
        self.assertEqual(r.n(),          10)
        self.assertEqual(r.total(),      20)
        self.assertEqual(r.average(),     2)
        r.add(3)
        self.assertEqual(len(r),         10)
        self.assertEqual(r.count(),      10)
        self.assertEqual(r.n(),          10)
        self.assertEqual(r.total(),      22)
        self.assertEqual(r.average(),   2.2)
        r.add(3)
        r.add(4)
        r.add(4)
        r.add(5)
        self.assertEqual(len(r),         10)
        self.assertEqual(r.count(),      10)
        self.assertEqual(r.n(),          10)
        self.assertEqual(r.total(),      29)
        self.assertEqual(r.average(),   2.9)
        r.add(0)
        r.add(0)
        r.add(0)
        r.add(0)
        r.add(0)
        r.add(0)
        r.add(0)
        r.add(0)
        r.add(0)
        r.add(0)
        self.assertEqual(len(r),         10)
        self.assertEqual(r.count(),      10)
        self.assertEqual(r.n(),          10)
        self.assertEqual(r.total(),       0)
        self.assertEqual(r.average(),     0)

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
# end           R o l l i n g A v e r a g e . p y            end
# ==============================================================
