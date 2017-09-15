# ==============================================================
#                       S q u a r e . p y
# ==============================================================

# Author:  Dean Earl Wright III
# Created: 23 August 2011
# Purpose: Reimplementation of the Q-learning perturbation
#          testbed for multiple metacognition levels.

# ==============================================================
#                                                        imports
# ==============================================================
import math
import unittest
import random
from Constants import *

# ==============================================================
#                                                         Square
# ==============================================================
class Square(object):
    "A single square of a grid world."

    def __init__(self, reward=0, x=0, y=0,
                       jump=None, letter=None,
                       underline=False):

        self.__reward    = reward
        self.__x         = x
        self.__y         = y
        self.__jump      = jump
        self.__q         = [0.0,0.0,0.0,0.0]
        self.__expected  = None
        self.__visits    = 0
        self.__letter    = letter
        self.__underline = underline
        self.__value     = None

    def __str__(self):
        return "[%d,%d] r=%d" % (self.__x, self.__y, self.__reward)

    def reward(self):   return self.__reward
    def x(self):        return self.__x
    def y(self):        return self.__y
    def loc(self):      return (self.__x, self.__y)
    def jump(self):     return self.__jump
    def q(self):        return self.__q
    def expected(self): return self.__expected
    def visits(self):   return self.__visits

    def set_reward(self, reward):
        self.__reward = reward

    def set_expected(self, reward):
        self.__expected = reward

    def set_jump(self, jump):
        self.__jump = jump

    def set_letter(self, letter):
        self.__letter = letter

    def set_underline(self, underline):
        self.__underline = underline

    def __setitem__(self, index, value):
        self.__q[index] = value

    def __getitem__(self, index):
        return self.__q[index]

    def reset(self):
        self.__q = [0.0,0.0,0.0,0.0]
        self.__expected = None
        self.__visits   = 0

    def suggest(self):
        "Suggest the highest ranked move"

        # 1. Start by picking a random direction
        pick = random.randint(0,3)
        result = [pick]
        value  = self.__q[pick]

        # 2. Loop for the other directions
        for direction in range(4):
            if direction == pick: continue

            # 3. If this direction is better, save it
            v = self.__q[direction]
            if v > value:
                value = v
                result = [direction]

            # 4. If the same, keep both (or more)
            elif v == value:
                result.append(direction)

        # 5. If there is more than one best direction, shuffle them
        if len(result) > 1:
            random.shuffle(result)

        # 6. Return the best direction (or directions)
        return result

    def max(self):
        value  = self.__q[0]
        for direction in [1,2,3]:
            v = self.__q[direction]
            if v > value:
                value = v
        return value

    def min(self):
        value  = self.__q[0]
        for direction in [1,2,3]:
            v = self.__q[direction]
            if v < value:
                value = v
        return value

    def visit(self, times=1):
        self.__visits += times

    def draw(self, out_f, arrow=True, which=DRAW_QMAX):

        # 1. Get values and direction for squeare
        smax = self.max();
        smin = self.min();

        #  2. Output as a comment all four values
        out_f("        %% --- [%d,%d]" % (self.__x,self.__y))
        for direction in DIRECTIONS:
            out_f(" %s:%d" % (DIR_LETTER[direction],
                              self.__q[direction]))
            out_f.write(" r:%d v:%d\n" % (self.__reward,
                                          self.__visits))

            # 3. Output the box
            out_f.write("        \\put(%d,%d){"
                        % (self.__x, self.__y))
            out_f.write("\\framebox(1.0,1.0){")

            # 4. Output the letter or policy value or visits of whatever
            if self.__letter:
                out_f.write("{\\LARGE\\bfseries{%s}}"
                            % self.__letter)
            else:
                if self.__underline:
                    out_f.write("\\underline{")
                if which ==  DRAW_QMAX:
                    if 0.1 < abs(smax) or 0.1 < abs(smin):
                        out_f.write("%4f" % smax)
                elif which == DRAW_VALUE:
                    if self.__value == None:
                        out_f.write("    ")
                    else:
                        out_f.write("%4f" % self.__value)
                elif which == DRAW_LOGV:
                    if self.__visits:
                        out_f.write("%f" % math.log(self.__visits))
                    else:
                        out_f.write("--")
                elif which == DRAW_REWARD:
                    out_f.write("%4d" % self.__reward)
                else:
                    out_f.write("{\\LARGE\\bfseries{?}}")
                if self.__underline:
                    out_f.write("}")
            out_f.write("}}\n")

            # 5. Output an arrow or arrows if and as appropiate
            if arrow and (0.1 < smax - smin):
                for direction in DIRECTIONS:
                    if 0.05 > smax - self.__q[direction]:
                        out_f.write("        \\put(%f,%f)"
                                    % (self.__x+0.5, self.__y+0.5))
                        out_f.write("{\\vector(%d,%d){.5}\n"
                                    % (DIR_DELTA_X[direction],
                                       DIR_DELTA_Y[direction]))

# ==============================================================
#                                                     unit tests
# ==============================================================
class TestSquare(unittest.TestCase):

    def testEmptyConstructor(self):
        s = Square()
        self.assertEqual(s.reward(),        0)
        self.assertEqual(s.x(),             0)
        self.assertEqual(s.y(),             0)
        self.assertEqual(s.loc(),       (0,0))
        self.assertEqual(str(s),  "[0,0] r=0")
        self.assertEqual(s.jump(),       None)
        s.set_reward(10)
        self.assertEqual(s.reward(),       10)
        self.assertEqual(s.x(),             0)
        self.assertEqual(s.y(),             0)
        self.assertEqual(s.loc(),       (0,0))
        self.assertEqual(str(s), "[0,0] r=10")
        self.assertEqual(s.jump(),       None)
        s.set_jump((3,3))
        self.assertEqual(s.reward(),       10)
        self.assertEqual(s.x(),             0)
        self.assertEqual(s.y(),             0)
        self.assertEqual(s.loc(),       (0,0))
        self.assertEqual(str(s), "[0,0] r=10")
        self.assertEqual(s.jump(),      (3,3))

    def testConstructor(self):
        s = Square(reward=10, x=4, y=3)
        self.assertEqual(s.reward(),        10)
        self.assertEqual(s.x(),              4)
        self.assertEqual(s.y(),              3)
        self.assertEqual(s.loc(),        (4,3))
        self.assertEqual(str(s),  "[4,3] r=10")
        self.assertEqual(s.jump(),       None)
        s.set_reward(6)
        self.assertEqual(s.reward(),         6)
        self.assertEqual(s.x(),              4)
        self.assertEqual(s.y(),              3)
        self.assertEqual(s.loc(),        (4,3))
        self.assertEqual(str(s),   "[4,3] r=6")
        self.assertEqual(s.jump(),       None)

    def testSuggestions(self):
        s = Square(reward=10, x=4, y=3)
        self.assertEqual(len(s.suggest()), 4)
        s[0] = 3.4
        self.assertEqual(s.suggest(), [0])
        s[2] = 4.3
        self.assertEqual(s.suggest(), [2])
        s[1] = 1.2
        self.assertEqual(s.suggest(), [2])
        s[3] = 1.2
        self.assertEqual(s.suggest(), [2])
        s[1] = 4.3
        sug = s.suggest()
        self.assert_(sug == [1,2] or sug == [2,1])

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
# end                   S q u a r e . p y                    end
# ==============================================================
