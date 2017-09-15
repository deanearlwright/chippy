# ==============================================================
#                       V e c t o r . p y
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
#                                                         Vector
# ==============================================================
class Vector(object):

    def __init__(self, data=None):
        if data:
            self.__vec = list(data)
        else:
            self.__vec = []

    def __repr__(self):
        return 'Vector(%s)' % repr(self.__vec)

    def __add__(self, other):
        return Vector(map(lambda x, y: x+y, self, other))

    def __div__(self, other):
        return Vector(map(lambda x, y: x/y, self, other))

    def __getitem__(self, index):
        return self.__vec[index]

    def __len__(self):
        return len(self.__vec)

    def __str__(self):
        return str(self.__vec)

# ==============================================================
#                                                     unit tests
# ==============================================================
class TestVector(unittest.TestCase):

    def testEmptyConstructor(self):
        v = Vector()
        self.assertEqual(len(v),             0)
        self.assertEqual(str(v),          '[]')
        self.assertEqual(repr(v), 'Vector([])')

    def testConstructor(self):
        v = Vector([1,2,3])
        self.assertEqual(len(v),             3)
        self.assertEqual(v[1],               2)
        self.assertEqual(str(v),          '[1, 2, 3]')
        self.assertEqual(repr(v), 'Vector([1, 2, 3])')

    def testAdd(self):
        v1 = Vector([1,2,3])
        v2 = Vector([22,9,13])
        v = v1 + v2
        self.assertEqual(len(v),             3)
        self.assertEqual(v[1],              11)
        self.assertEqual(str(v),          '[23, 11, 16]')
        self.assertEqual(repr(v), 'Vector([23, 11, 16])')

    def testDiv(self):
        v1 = Vector([6,6,16])
        v2 = Vector([2,3,4])
        v = v1 / v2
        self.assertEqual(len(v),             3)
        self.assertEqual(v[1],               2)
        self.assertEqual(str(v),          '[3, 2, 4]')
        self.assertEqual(repr(v), 'Vector([3, 2, 4])')

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
# end                   V e c t o r . p y                    end
# ==============================================================
