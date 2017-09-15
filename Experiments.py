# ==============================================================
#                  E x p e r i m e n t s . p y
# ==============================================================

# Author:  Dean Earl Wright III
# Created: 28 August 2011
# Purpose: Reimplementation of the Q-learning perturbation
#          testbed for multiple metacognition levels.

# ==============================================================
#                                                        imports
# ==============================================================

import sys
import unittest

from Constants import *
import Grid
import QLearner
import Experiment

# ==============================================================
#                                                      constants
# ==============================================================
CSV_HEADER = "EXP,NUM,"

# ==============================================================
#                                                    Experiments
# ==============================================================
class Experiments(object):

    def __init__(self, walker=None,
                       grid=None,
                       switch=SWITCH_TURN,
                       stop=STOP_TURN,
                       repeats=REPEATS):

        self.walker  = walker
        self.grid    = grid
        self.switch  = switch
        self.stop    = stop
        self.repeats = repeats
        if walker:
            walker.set_grid(grid)

    def run(self, csv_f=None, csv_b=None, csv_e=None,
                  experiments=None, verbose=False):

        # 1. Can't run if no one or no where
        if not self.walker or not self.grid or not self.walker.grid():
            return None

        # 2. Start with no results
        results = []

        # 3. Loop for all possible experiments
        if experiments == None:
            exps = range(len(EXPERIMENTS))
        else:
            exps = [experiments]
        for exp in exps:

            # 4. Determine the rewards for the experiment
            initial_rewards = REWARDS[EXPERIMENTS[exp][INITIAL]]
            changed_rewards = REWARDS[EXPERIMENTS[exp][CHANGED]]
            rewards=(initial_rewards, changed_rewards)

            # 5. Create the experiment
            experiment = Experiment.Experiment(walker=self.walker,
                                               grid=self.grid,
                                               rewards=rewards,
                                               switch=self.switch,
                                               stop=self.stop)
            if verbose:
                print "%s %2d" % (self.walker, exp),
                sys.stdout.flush()


            # 6. Loop for the number of repeats
            prefix = None
            rewards = 0
            for num in range(self.repeats):


                # 7. Run the experiment
                experiment.reset()
                if csv_f:
                    if csv_b:
                        prefix = '%s%d,%d,' % (csv_b, exp, num)
                    else:
                        prefix = '%d,%d,' % (exp, num)
                if verbose:
                    sys.stdout.write('.')
                    sys.stdout.flush()
                rewards += experiment.run(csv_f=csv_f,
                                          csv_b=prefix,
                                          csv_e=csv_e)

            # 8. Save the results
            if verbose:
                print rewards
            results.append(rewards)

        # 9. Return the total reward
        return results

    def csv_header(self):
        exp = Experiment.Experiment(walker=self.walker,
                                    grid=self.grid)
        return CSV_HEADER + exp.csv_header()

# ==============================================================
#                                                     unit tests
# ==============================================================
class TestExperiment(unittest.TestCase):

    def testEmptyConstructor(self):
        e = Experiments()
        self.assertEqual(e.walker,   None)
        self.assertEqual(e.grid,     None)
        self.assertEqual(e.switch,  SWITCH_TURN)
        self.assertEqual(e.stop,    STOP_TURN)
        self.assertEqual(e.repeats, REPEATS)

    def testNoPerturbShort(self):
        g = Grid.Grid()
        l = QLearner.QLearner()
        e = Experiments(l,g,1000,2000,5)
        self.assertEqual(e.walker,   l)
        self.assertEqual(e.grid,     g)
        self.assertEqual(e.switch,  1000)
        self.assertEqual(e.stop,    2000)
        self.assertEqual(e.repeats, 5)
        r = e.run()
        self.assertEqual(len(r), len(EXPERIMENTS))

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
# end             E x p e r i m e n t s . p y                end
# ==============================================================
