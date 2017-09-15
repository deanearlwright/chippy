# ==============================================================
#                       W a l k e r . p y
# ==============================================================

# Author:  Dean Earl Wright III
# Created: 23 August 2011
# Purpose: Reimplementation of the Q-learning perturbation
#          testbed for multiple metacognition levels.

# ==============================================================
#                                                        imports
# ==============================================================
import unittest
import random

from Constants import *
import Grid

# ==============================================================
#                                                         Walker
# ==============================================================
class Walker(object):
    "A grid crawling agent"

    def __init__(self, grid=None, start=None):
        self.__score   = 0
        self.__count   = 0
        self.__grid    = None
        self.__loc     = None
        self.set_grid(grid,start)

    def set_grid(self, grid=None, start=None):
        self.__grid = grid
        if grid == None:
            if start == None:
                self.__loc = (0,0)
            else:
                self.__loc = start
        else:
            if start == None:
                self.__loc = (grid.n()/2, grid.n()/2)
            else:
                self.__loc = start

    def __str__(self):
        return "Walker (c=%d,s=%d,l=%s %s)" % \
               (self.__count, self.__score,
                str(self.__loc), str(self.__grid))

    def score(self):   return self.__score
    def count(self):   return self.__count
    def grid(self):    return self.__grid
    def loc(self):     return self.__loc

    def policy(self):
        if self.__grid:
            return self.__grid[self.__loc].q()
        else:
            return None

    def move(self, direction=None):
        "Move to the given, best, or random direction"

        # 2. Pick a direction if none given
        if None == direction:
            suggest = self.suggest()
            direction = suggest[random.randint(0, len(suggest))]

        # 3. Move and get reward and new location
        result = self.__grid.move(self.__loc,
                                  direction)
        #print 'walker.move old=',self.__loc, 'dir=',direction, 'rwd=',reward, 'loc=',loc

        # 3. Set new location
        self.__loc = result[RESULT_FINAL_LOC]

        # 4. Accumulate the reward
        self.__score = self.__score + result[RESULT_ACT_REWARD]

        # 5. Count the number of moves
        self.__count = self.__count + 1

        # 6. Return original loc, direction,
        #           expected and actual rewards
        #           and new location
        return result

    def reset(self):
        if self.__grid:
            self.__grid.reset()

    def suggest(self):
        return self.__grid.suggest(self.__loc)

    def csv_format(self, result):

        # 1. Grid build it, so Grid can format it
        return self.__grid.csv_format(result)

    def csv_header(self):

        # 1. Grid build it, so Grid can format it
        return self.__grid.csv_header()

# ==============================================================
#                                                     unit tests
# ==============================================================
class TestWalker(unittest.TestCase):

    def testEmptyConstructor(self):
        w = Walker()
        self.assertEqual(w.score(),          0)
        self.assertEqual(w.count(),          0)
        self.assertEqual(w.loc(),        (0,0))
        self.assertEqual(str(w),  "Walker (c=0,s=0,l=(0, 0) None)")

    def testConstructor(self):
        g = Grid.Grid()
        w = Walker(grid=g)
        self.assertEqual(w.score(),          0)
        self.assertEqual(w.count(),          0)
        self.assertEqual(w.loc(),        (4,4))
        self.assertEqual(str(w),  "Walker (c=0,s=0,l=(4, 4) Grid (n=8,r1=10,r2=-10))")

    def testMoves(self):
        g = Grid.Grid()
        w = Walker(grid=g, start=(0,0))
        self.assertEqual(str(w),  "Walker (c=0,s=0,l=(0, 0) Grid (n=8,r1=10,r2=-10))")
        self.assertEqual(w.move(DIR_S), ((0, 0), 1, (0, 1), None, 0, (0, 1)))
        self.assertEqual(w.move(DIR_S)[-2:], (0,(0,2)))
        self.assertEqual(w.move(DIR_S)[-2:], (0,(0,3)))
        self.assertEqual(w.move(DIR_S)[-2:], (0,(0,4)))
        self.assertEqual(str(w),  "Walker (c=4,s=0,l=(0, 4) Grid (n=8,r1=10,r2=-10))")
        self.assertEqual(w.move(DIR_E)[-2:], (0,(1,4)))
        self.assertEqual(w.move(DIR_E)[-2:], (0,(2,4)))
        self.assertEqual(w.move(DIR_E)[-2:], (0,(3,4)))
        self.assertEqual(w.move(DIR_E)[-2:], (0,(4,4)))
        self.assertEqual(w.move(DIR_E)[-2:], (0,(5,4)))
        self.assertEqual(w.move(DIR_E)[-2:], (0,(6,4)))
        self.assertEqual(w.move(DIR_E)[-2:], (0,(7,4)))
        self.assertEqual(w.move(DIR_E)[-2:], (0,(7,4)))
        self.assertEqual(w.move(DIR_E)[-2:], (0,(7,4)))
        self.assertEqual(str(w),  "Walker (c=13,s=0,l=(7, 4) Grid (n=8,r1=10,r2=-10))")
        self.assertEqual(w.move(DIR_S)[-2:], (0,(7,5)))
        self.assertEqual(w.move(DIR_S)[-2:], (0,(7,6)))
        self.assertEqual(w.move(DIR_S), ((7, 6), 1, (7, 7), None, -10, (0, 0)))
        self.assertEqual(str(w),  "Walker (c=16,s=-10,l=(0, 0) Grid (n=8,r1=10,r2=-10))")
        self.assertEqual(w.score(), -10)
        self.assertEqual(w.move(DIR_N)[-2:], (0,(0,0)))
        self.assertEqual(str(w),  "Walker (c=17,s=-10,l=(0, 0) Grid (n=8,r1=10,r2=-10))")
        self.assertEqual(w.score(), -10)
        self.assertEqual(w.count(), 17)

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
# end                  W a l k e r . p y                     end
# ==============================================================
