# ==============================================================
#                         G r i d . p y
# ==============================================================

# Author:  Dean Earl Wright III
# Created: 23 August 2011
# Purpose: Reimplementation of the Q-learning perturbation
#          testbed for multiple metacognition levels.

# ==============================================================
#                                                        imports
# ==============================================================
import unittest

import Square
from Constants import *

# ==============================================================
#                                                           Grid
# ==============================================================
class Grid(object):
    "Multiple squares arranged in an n by n matrix with two rewards"

    def __init__(self, n=8, r1=10, r2=-10, r=None):
        "Initialize a n-armed bandit"

        # 1. Set local values
        self.__n_1     = n-1
        self.__squares = {}
        self.__r1      = r1
        self.__r2      = r2
        if r:
            self.__r1 = r[0]
            self.__r2 = r[1]

        # 2. Create all the individual squares
        for x in range(n):
            for y in range(n):
                s = Square.Square(x=x, y=y)
                self.__squares[(x,y)] = s

        # 3. Set the rewards
        self.set_rewards(self.__r1,self.__r2)

        # 4. Set the transporters
        self.__squares[(0,0)].set_jump((self.__n_1,self.__n_1))
        self.__squares[(self.__n_1,self.__n_1)].set_jump((0,0))

    def __str__(self):
        return "Grid (n=%d,r1=%d,r2=%d)" % \
               (self.__n_1+1, self.__r1, self.__r2)

    def n(self):       return self.__n_1+1
    def r1(self):      return self.__r1
    def r2(self):      return self.__r2
    def rewards(self): return (self.__r1, self.__r2)

    def set_rewards(self, r1=0, r2=0, r=None):
        #print 'grid.set_rewards(r1=',r1,',r2=',r2,',r=',r,')'
        self.__r1      = r1
        self.__r2      = r2
        if r:
            self.__r1 = r[0]
            self.__r2 = r[1]
        self.__squares[(0,0)].set_reward(self.__r1)
        self.__squares[(self.__n_1,self.__n_1)].set_reward(self.__r2)

    def set_expected(self, index, reward=None):
        #print 'Grid.set_expected: Setting expected reward at', index, 'to', reward
        self.__squares[index].set_expected(reward)

    def reset_expected(self, index):
        self.set_expected(index)

    def __getitem__(self, index):
        return self.__squares[index]

    def __len__(self):
        return len(self.__squares)

    def move(self, at, direction):
        "From square 'at' move in direction 'direction'"

        # 1. Determine new square
        new_x = at[0] + DIR_DELTA_X[direction]
        if new_x < 0:          new_x = 0
        if new_x > self.__n_1: new_x = self.__n_1
        new_y = at[1] + DIR_DELTA_Y[direction]
        if new_y < 0:          new_y = 0
        if new_y > self.__n_1: new_y = self.__n_1
        new_xy = (new_x, new_y)

        # 2. No reward or jumps if new position is same as old
        if at == new_xy:
            return (at, direction, at, 0, 0, at)

        # 3. Get reward for moving to the new square
        reward   = self.__squares[new_xy].reward()
        expected = self.__squares[new_xy].expected()

        # 4. Implement jumps at reward squares
        jump = self.__squares[new_xy].jump()
        if jump:
            final_xy = jump
        else:
            final_xy = new_xy

        # 5. Return reward and new location
        return at,direction,new_xy,expected,reward,final_xy

    def reset(self):
        for s in self.__squares.itervalues():
            s.reset()

    def suggest(self, loc):
        return self.__squares[loc].suggest()

    def switch(self):
        "Switch the values of the two rewards"
        old_r1 = self.__r1
        old_r2 = self.__r2
        self.set_rewards(old_r2, old_r1)

    def picture(self, out_f, arrow=True, which=DRAW_QMAX):

        #  1. Output the initial picture element
        out_f.write("    \\begin{picture}(%d,%d\n"
                    % (self.n(),self.n()))
        out_f.write("    \\thicklines\n")

        # 2. Loop for every square
        for s in self.__squares:

            # 3. Have the square draw itself
            s.draw(out_f, arrow, which)

        # 4. Output the final picture element
        out_f.write("    \\end{picture}\n")

    def csv_format(self, result):

        # 1. Nothing gets you nothing
        if not result or len(result) == 0:
            return ''

        # 2. Get parts
        start_x  = result[RESULT_START_LOC][0]
        start_y  = result[RESULT_START_LOC][1]
        reward_x = result[RESULT_REWARD_LOC][0]
        reward_y = result[RESULT_REWARD_LOC][1]
        final_x  = result[RESULT_FINAL_LOC][0]
        final_y  = result[RESULT_FINAL_LOC][1]
        if result[RESULT_EXP_REWARD] == None:
            exp_reward = result[RESULT_ACT_REWARD]
        else:
            exp_reward = result[RESULT_EXP_REWARD]

        # 3. Format all of those integers
        formatted = ('%d,%d,%d,%d,%d,%d,%d,%d,%d' %
                     (start_x, start_y,
                      result[RESULT_DIRECTION],
                      reward_x, reward_y,
                      exp_reward, result[RESULT_ACT_REWARD],
                      final_x, final_y))

        # 4. Return the formatted string
        return formatted

    def csv_header(self):

        # 1. Return string of column names
        return "START_X,START_Y,DIRECTION,REWARD_X,REWARD_Y," \
               "EXPECTED,ACTUAL,FINAL_X,FINAL_Y"


# ==============================================================
#                                                     unit tests
# ==============================================================
class TestGrid(unittest.TestCase):

    def testEmptyConstructor(self):
        g = Grid()
        self.assertEqual(g.n(),              8)
        self.assertEqual(g.r1(),            10)
        self.assertEqual(g.r2(),           -10)
        self.assertEqual(g.rewards(), (10,-10))
        self.assertEqual(len(g),            64)
        self.assertEqual(str(g),   "Grid (n=8,r1=10,r2=-10)")
        g.set_rewards()
        self.assertEqual(g.n(),              8)
        self.assertEqual(g.r1(),             0)
        self.assertEqual(g.r2(),             0)
        self.assertEqual(g.rewards(),    (0,0))
        self.assertEqual(len(g),            64)
        self.assertEqual(str(g),   "Grid (n=8,r1=0,r2=0)")

    def testConstructor(self):
        g = Grid(n=6, r1=4, r2=3)
        self.assertEqual(g.n(),              6)
        self.assertEqual(g.r1(),             4)
        self.assertEqual(g.r2(),             3)
        self.assertEqual(g.rewards(),    (4,3))
        self.assertEqual(len(g),            36)
        self.assertEqual(str(g),   "Grid (n=6,r1=4,r2=3)")
        g.set_rewards(r1=2, r2=6)
        self.assertEqual(g.n(),              6)
        self.assertEqual(g.r1(),             2)
        self.assertEqual(g.r2(),             6)
        self.assertEqual(g.rewards(),    (2,6))
        self.assertEqual(len(g),             36)
        self.assertEqual(str(g),   "Grid (n=6,r1=2,r2=6)")

    def testMoves(self):
        g = Grid(r1=1,r2=2)
        self.assertEqual(g.n(),              8)
        self.assertEqual(g.r1(),             1)
        self.assertEqual(g.r2(),             2)
        self.assertEqual(g.move((3,3),0), ((3,3),0,(3,2),None,0,(3,2)))
        self.assertEqual(g.move((3,3),1), ((3,3),1,(3,4),None,0,(3,4)))
        self.assertEqual(g.move((3,3),2)[-2:], (0,(4,3)))
        self.assertEqual(g.move((3,3),3)[-2:], (0,(2,3)))
        self.assertEqual(g.move((3,7),0)[-2:], (0,(3,6)))
        self.assertEqual(g.move((3,7),1)[-2:], (0,(3,7)))
        self.assertEqual(g.move((3,7),2)[-2:], (0,(4,7)))
        self.assertEqual(g.move((3,7),3)[-2:], (0,(2,7)))
        self.assertEqual(g.move((3,0),0)[-2:], (0,(3,0)))
        self.assertEqual(g.move((3,0),1)[-2:], (0,(3,1)))
        self.assertEqual(g.move((3,0),2)[-2:], (0,(4,0)))
        self.assertEqual(g.move((3,0),3)[-2:], (0,(2,0)))
        self.assertEqual(g.move((7,3),0)[-2:], (0,(7,2)))
        self.assertEqual(g.move((7,3),1)[-2:], (0,(7,4)))
        self.assertEqual(g.move((7,3),2)[-2:], (0,(7,3)))
        self.assertEqual(g.move((7,3),3)[-2:], (0,(6,3)))
        self.assertEqual(g.move((0,3),0)[-2:], (0,(0,2)))
        self.assertEqual(g.move((0,3),1)[-2:], (0,(0,4)))
        self.assertEqual(g.move((0,3),2)[-2:], (0,(1,3)))
        self.assertEqual(g.move((0,3),3)[-2:], (0,(0,3)))
        self.assertEqual(g.move((0,1),0), ((0,1),0,(0,0),None,1,(7,7)))
        self.assertEqual(g.move((1,0),3)[-2:], (1,(7,7)))
        self.assertEqual(g.move((6,7),2)[-2:], (2,(0,0)))
        self.assertEqual(g.move((7,6),1)[-2:], (2,(0,0)))
        self.assertEqual(g.move((0,0),0)[-2:], (0,(0,0)))
        self.assertEqual(g.move((0,0),DIR_N)[-2:], (0,(0,0)))

    def testSquares(self):
        g = Grid(r1=1,r2=2)
        self.assertEqual(g.n(),              8)
        self.assertEqual(g.r1(),             1)
        self.assertEqual(g.r2(),             2)
        self.assertEqual(g[(0,0)].reward(),  1)
        self.assertEqual(g[(7,7)].reward(),  2)
        self.assertEqual(g[(3,4)].x(),       3)
        self.assertEqual(g[(3,5)].y(),       5)
        self.assertEqual(g[(6,7)].loc(), (6,7))
        g.switch()
        self.assertEqual(g.n(),              8)
        self.assertEqual(g.r1(),             2)
        self.assertEqual(g.r2(),             1)
        self.assertEqual(g[(0,0)].reward(),  2)
        self.assertEqual(g[(7,7)].reward(),  1)

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
# end                     G r i d . p y                      end
# ==============================================================
