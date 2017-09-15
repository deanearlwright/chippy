# ==============================================================
#                   E x p e r i m e n t . p y
# ==============================================================

# Author:  Dean Earl Wright III
# Created: 26 August 2011
# Purpose: Reimplementation of the Q-learning perturbation
#          testbed for multiple metacognition levels.

# ==============================================================
#                                                        imports
# ==============================================================

import unittest

from Constants import *
import Grid
import QLearner

# ==============================================================
#                                                      constants
# ==============================================================
CSV_HEADER = "STEP,"

# ==============================================================
#                                                     Experiment
# ==============================================================
class Experiment(object):

    def __init__(self, walker=None,
                       grid=None,
                       rewards=((10,-10),(-10,10)),
                       switch=SWITCH_TURN,
                       stop=STOP_TURN):

        self.walker  = walker
        self.grid    = grid
        self.rewards = rewards
        self.switch  = switch
        self.stop    = stop
        if walker:
            walker.set_grid(grid)

    def run(self, csv_f=None, csv_b=None, csv_e=None):

        # 1. Can't run if no one or no where
        if not self.walker or not self.grid or not self.walker.grid():
            return None

        # 2. Set the initial goals
        self.grid.set_rewards(r=self.rewards[INITIAL])

        # 3. Start with no rewards
        rewards = 0

        # 4. Walk a mile in chippy's shoes
        for step in xrange(self.stop):

            # 5. Execute a single move
            result = self.walker.move()

            # 6. Record the move in the results
            rewards += result[RESULT_ACT_REWARD]
            if csv_f:
                if csv_b:
                    csv_f.write(csv_b)
                csv_f.write('%d,' % step)
                csv_f.write(self.walker.csv_format(result))
                if csv_e:
                    csv_f.write(csv_e)

            # 7. Switch the rewards if it is time
            if step == self.switch:
                #print 'Switching rewards at step', step
                self.grid.set_rewards(r=self.rewards[CHANGED])

        # 8. Return the total rewards
        return rewards

    def reset(self):

        if self.walker:
            self.walker.reset()

    def csv_header(self):
        return CSV_HEADER + self.walker.csv_header()

# ==============================================================
#                                                     unit tests
# ==============================================================
class TestExperiment(unittest.TestCase):

    def testEmptyConstructor(self):
        e = Experiment()
        self.assertEqual(e.walker,   None)
        self.assertEqual(e.grid,     None)
        self.assertEqual(e.rewards, ((10,-10),(-10,10)))
        self.assertEqual(e.switch,  SWITCH_TURN)
        self.assertEqual(e.stop,    STOP_TURN)

    def testNoPerturbShort(self):
        g = Grid.Grid()
        l = QLearner.QLearner()
        e = Experiment(l,g,((9,2),(2,9)),1000,2000)
        self.assertEqual(e.walker,   l)
        self.assertEqual(e.grid,     g)
        self.assertEqual(e.rewards, ((9,2),(2,9)))
        self.assertEqual(e.switch,  1000)
        self.assertEqual(e.stop,    2000)
        r = e.run()
        self.assertTrue(r > 0)

    def xxx_testNoPerturbLong(self):
        g = Grid.Grid()
        l = QLearner.QLearner()
        e = Experiment(g,l)
        r = e.run()
        self.assertEqual(len(r), STOP_TURN)

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
# end              E x p e r i m e n t . p y                 end
# ==============================================================
