# ==============================================================
#                     Q L e a r n e r . p y
# ==============================================================

# Author:  Dean Earl Wright III
# Created: 23 August 2011
# Purpose: Reimplementation of the Q-learning perturbation
#          testbed for multiple metacognition levels.

# ==============================================================
#                                                        imports
# ==============================================================
import random
import unittest

import Grid
import Walker
from Constants import *

# ==============================================================
#                                                       QLearner
# ==============================================================
class QLearner(Walker.Walker):
    def __init__(self, grid=None, start=None,
                  alpha=DEFAULT_ALPHA,
                  gamma=DEFAULT_GAMMA,
                  epsilon=DEFAULT_EPSILON):

        Walker.Walker.__init__(self, grid=grid, start=start)
        self.__alpha   = alpha
        self.__gamma   = gamma
        self.__epsilon = epsilon

    def __str__(self):
        return "QLearner (a=%f,g=%f,e=%f,c=%d,s=%f %s)" % \
               (self.__alpha,self.__gamma,self.__epsilon, self.count(),
                self.score(), str(self.grid()))

    def epsilon(self): return self.__epsilon
    def alpha(self): return self.__alpha
    def gamma(self): return self.__gamma

    def set_epsilon(self, epsilon): self.__epsilon = epsilon
    def set_alpha(self, alpha): self.__alpha = alpha
    def set_gamma(self, gamma): self.__gamma = gamma

    def move(self, direction=None):
        "Move in the direction with the best expected value or explore"

        # 1. Remember the previous location
        previous = self.loc()
        policy   = self.policy()

        # 2. If forced move use it
        if direction != None:
            move_type = MOVE_TYPE_FORCED
        else:

            # 3. Determine on or off policy move
            if self.__epsilon > random.random():
                direction = random.choice(DIRECTIONS)
                move_type = MOVE_TYPE_RANDOM
            else:
                direction = self.suggest()[0]
                move_type = MOVE_TYPE_POLICY

        # 4. Move in specified direction
        result = list(Walker.Walker.move(self, direction))

        # 5. Adjust the action expected rewards
        self.qreward(direction, previous,
                     result[RESULT_ACT_REWARD],
                     result[RESULT_FINAL_LOC])

        # 6. Append move type and policy to result
        result.append(move_type)
        result.extend(policy)

        # 7. Return movement results
        return result

    def qreward(self, a, s, r, sp):
        "Spread out the reward over the past moves"
        oldQsa = self.grid()[s][a]
        maxQspap = self.grid()[sp].max()
        newQsa = oldQsa + self.__alpha*(r + (self.__gamma*maxQspap) - oldQsa)
        #print "Q([%d,%d],%d) = %f = %f + %f(%f + %f(%f-%f))" % \
        #    (s[0], s[1], a, newQsa, oldQsa,
        #     self.__alpha, r, self.__gamma, maxQspap, oldQsa)
        self.grid()[s][a] = newQsa
        return newQsa

    def increase_epsilon(self, e):
        epsilon = self.__epsilon + e
        if epsilon > MAX_EPSILON:
            epsilon = MAX_EPSILON
        if epsilon < MIN_EPSILON:
            epsilon = MIN_EPSILON
        self.__epsilon = epsilon

    def decrease_epsilon(self, e):
        self.increase_epsilon(-e)

    def csv_format(self, result):

        # 1. Let our parent do most of the work
        walker = Walker.Walker.csv_format(self, result)

        # 2. Get the QLearner parts
        qlearn = ',%s,%f,%f,%f,%f' % \
               (result[RESULT_MOVE_TYPE],
                result[RESULT_POLICY_N],
                result[RESULT_POLICY_S],
                result[RESULT_POLICY_E],
                result[RESULT_POLICY_W])

        # 3. Return formatted result
        return walker+qlearn

    def csv_header(self):

        # 1. Get the Walker header
        walker = Walker.Walker.csv_header(self)

        # 2. Get the QLearner parts
        qlearn = ',WHY,Q_N,Q_S,Q_E,Q_W'

        # 3. Return it all
        return walker+qlearn

# ==============================================================
#                                                     unit tests
# ==============================================================
class TestQLearner(unittest.TestCase):

    def testEmptyConstructor(self):
        q = QLearner()
        self.assertEqual(q.score(),          0)
        self.assertEqual(q.count(),          0)
        self.assertEqual(q.loc(),        (0,0))
        self.assertEqual(q.epsilon(),     0.05)
        self.assertEqual(q.alpha(),        0.5)
        self.assertEqual(q.gamma(),        0.9)
        self.assertEqual(str(q),  "QLearner (a=0.500000,g=0.900000,e=0.050000,c=0,s=0.000000 None)")

    def testConstructor(self):
        g = Grid.Grid()
        q = QLearner(grid=g, alpha=0.8, gamma=0.7, epsilon=0.1)
        self.assertEqual(q.score(),          0)
        self.assertEqual(q.count(),          0)
        self.assertEqual(q.loc(),        (4,4))
        self.assertEqual(q.epsilon(),      0.1)
        self.assertEqual(q.alpha(),        0.8)
        self.assertEqual(q.gamma(),        0.7)
        self.assertEqual(str(q),  "QLearner (a=0.800000,g=0.700000,e=0.100000,c=0,s=0.000000 Grid (n=8,r1=10,r2=-10))")

    def testLearning(self):
        g = Grid.Grid()
        q = QLearner(grid=g, start=(0,0))
        self.assertEqual(q.score(),          0)
        self.assertEqual(q.count(),          0)
        self.assertEqual(q.loc(),        (0,0))
        q.move(DIR_S)
        q.move(DIR_E)
        q.move(DIR_N)
        q.move(DIR_W)
        self.assertEqual(q.count(),          4)
        self.assertEqual(q.score(),         10)
        self.assertEqual(q.loc(),        (7,7))
        self.assertEqual(g[(1,0)].q(), [0.0,0.0,0.0,5.0])
        q.move(DIR_W)
        q.move(DIR_W)
        q.move(DIR_W)
        q.move(DIR_W)
        q.move(DIR_W)
        q.move(DIR_W)
        q.move(DIR_N)
        q.move(DIR_N)
        q.move(DIR_N)
        q.move(DIR_N)
        q.move(DIR_N)
        q.move(DIR_N)
        q.move(DIR_N)
        q.move(DIR_W)
        self.assertEqual(q.loc(),        (7,7))
        self.assertEqual(g[(1,0)].q(), [0.0,0.0,0.0,7.5])
        self.assertEqual(g[(1,1)].q(), [2.25,0.0,0.0,0.0])

    def xtest10K(self):
        g = Grid.Grid()
        q = QLearner(grid=g)
        self.assertEqual(q.score(),          0)
        self.assertEqual(q.count(),          0)
        self.assertEqual(q.loc(),        (4,4))
        for _ in xrange(10000):
            q.move()
        self.assertEqual(q.count(),       10000)

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
# end                Q L e a r n e r . p y                   end
# ==============================================================
