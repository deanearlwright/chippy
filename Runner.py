# ==============================================================
#                       R u n n e r . p y
# ==============================================================

# Author:  Dean Earl Wright III
# Created: 28 August 2011
# Purpose: Reimplementation of the Q-learning perturbation
#          testbed for multiple metacognition levels.

# ==============================================================
#                                                        imports
# ==============================================================
import unittest

import QLearner
from Constants import *


# ==============================================================
#                                                         Runner
# ==============================================================
class Runner(QLearner.QLearner):
    """QLearner that invokes metacognition"""

    def __init__(self, grid=None, start=None,
                  alpha=DEFAULT_ALPHA,
                  gamma=DEFAULT_GAMMA,
                  epsilon=DEFAULT_EPSILON):

        QLearner.QLearner.__init__(self, grid=grid, start=start,
                                   alpha=alpha, gamma=gamma, epsilon=epsilon)
        self.adj_epsilon = None
        self.initial_epsilon = epsilon

    def move(self, direction=None):
        "Move in the direction with the best expected value or explore"

        # 1. If Adjusting learning, do it
        if self.adj_epsilon:
            self.decrease_epsilon(self.adj_epsilon)

        # 2. Move like a QLearner
        result = QLearner.QLearner.move(self, direction)

        # 3. If new reward, remember it
        if result[RESULT_EXP_REWARD] == None:
            self.set_expected(result)

        # 2. Get suggestion from Get suggested direction
        suggestion = self.monitor(result)
        result.append(suggestion)

        # 3. Implement the suggestion
        if suggestion == SUGGEST_NONE:
            pass
        elif suggestion == SUGGEST_LEARN:
            print "Runner.move: implemeting learning"
            self.increase_epsilon(LEARN_EPSILON)
            self.adj_epsilon = LEARN_ADJUST
        elif suggestion == SUGGEST_RESET:
            print "Runner.move: implemeting reset"
            self.reset()

        # 4. Return reward and new location
        return result

    def monitor(self, results):

        # 1. Not a very good metacognition
        return SUGGEST_NONE

    def reset(self):
        QLearner.QLearner.reset(self)
        self.set_epsilon(self.initial_epsilon)
        self.adj_epsilon = None

    def set_expected(self, result):
        self.grid().set_expected(result[RESULT_REWARD_LOC], result[RESULT_ACT_REWARD])

    def csv_format(self, result):

        # 1. Let our parent do most of the work
        qlearn = QLearner.QLearner.csv_format(self, result)

        # 2. Get the Suggestion
        runner = ',%d' % result[RESULT_SUGGESTION]

        # 3. Return formatted result
        return qlearn + runner

    def csv_header(self):

        # 1. Get the QLearner header
        qlearn = QLearner.QLearner.csv_header(self)

        # 2. Get the runner parts
        runner = ',SUGGESTION'

        # 3. Return it all
        return qlearn + runner

# ==============================================================
#                                                     unit tests
# ==============================================================
class TestRunner(unittest.TestCase):

    def testEmptyConstructor(self):
        pass

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
# end                      R u n n e r . p y                 end
# ==============================================================
