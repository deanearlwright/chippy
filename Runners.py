# ==============================================================
#                       R u n n e r s . p y
# ==============================================================

# Author:  Dean Earl Wright III
# Created: 23 August 2011
# Purpose: Reimplementation of the Q-learning perturbation
#          testbed for multiple metacognition levels.

# ==============================================================
#                                                        imports
# ==============================================================
import unittest

import Runner
from Constants import *


# ==============================================================
#                                                Level 0: Bereft
# ==============================================================
class Level0(Runner.Runner):

    def __str__(self): return 'Level0'

    def monitor(self, results):
        return SUGGEST_NONE

# ==============================================================
#                                            Level 1 Hardcoded A
# ==============================================================
class Level1HCA(Runner.Runner):

    def __str__(self): return 'Level1 HC A'

    def monitor(self,results):

        # 1. If unexpected reward, reset
        if results[RESULT_EXP_REWARD] != None and \
           results[RESULT_EXP_REWARD] != results[RESULT_ACT_REWARD]:
            return SUGGEST_RESET

        # 2. Else no suggestion
        return SUGGEST_NONE

# ==============================================================
#                                            Level 1 Hardcoded B
# ==============================================================
class Level1HCB(Runner.Runner):

    def __str__(self): return 'Level1 HC B'

    def monitor(self,results):

        # 1. If unexpected reward, learn
        if results[RESULT_EXP_REWARD] != None and \
           results[RESULT_EXP_REWARD] != results[RESULT_ACT_REWARD]:
            self.grid().set_expected(results[RESULT_REWARD_LOC],
                                     results[RESULT_ACT_REWARD])
            return SUGGEST_LEARN

        # 2. Else no suggestion
        return SUGGEST_NONE

# ==============================================================
#                                              Hardcoded Level 2
# ==============================================================
class Level2HC(Runner.Runner):

    def __str__(self): return 'Level2'

    def monitor(self,results):

        # 1. If unexpected reward, evaluate
        if results[RESULT_EXP_REWARD] != None and \
           results[RESULT_EXP_REWARD] != results[RESULT_ACT_REWARD]:

            # 2. If was positive and now negative, reset
            if results[RESULT_EXP_REWARD] > 0 and \
                results[RESULT_EXP_REWARD] < 0:
                return SUGGEST_RESET

            # 3. If reward is less that before, learn
            if results[RESULT_EXP_REWARD] > results[RESULT_EXP_REWARD]:
                self.grid().set_expected(results[RESULT_REWARD_LOC],
                                         results[RESULT_ACT_REWARD])
                return SUGGEST_LEARN

        # 4. Else no suggestion
        return SUGGEST_NONE

# ==============================================================
#                                              Hardcoded Level 3
# ==============================================================
class Level3HC(Runner.Runner):

    def __init__(self):

        # 1. Initialize our parent
        Runner.Runner.__init__(self)

        # 2. Times we had a smaller reward than expected
        self.suggested_learning = 0

    def __str__(self): return 'Level3'

    def reset(self):
        Runner.Runner.reset(self)
        self.suggested_learning = 0

    def monitor(self,results):

        # 1. Assume that there is nothing to do
        suggestion = SUGGEST_NONE

        # 2. If unexpected reward, evaluate
        if results[RESULT_EXP_REWARD] != None and \
           results[RESULT_EXP_REWARD] != results[RESULT_ACT_REWARD]:
            print 'Runners.Level3HC.monitor: unexpected reward %s at %s expected %s' % (
                results[RESULT_ACT_REWARD],
                results[RESULT_REWARD_LOC],
                results[RESULT_EXP_REWARD])

            # 3. If was positive and now negative, reset
            if results[RESULT_EXP_REWARD] > 0 and \
                results[RESULT_EXP_REWARD] < 0:
                suggestion = SUGGEST_RESET

            # 4. If reward is less that before, learn
            if results[RESULT_EXP_REWARD] > results[RESULT_EXP_REWARD]:
                self.grid().set_expected(results[RESULT_REWARD_LOC],
                                         results[RESULT_ACT_REWARD])
                suggestion = SUGGEST_LEARN

                # 5. Unless we suggested learning too often
                self.suggested_learning += 1
                if self.suggested_learning > LEARNING_LIMIT:
                    suggestion = SUGGEST_RESET
                    self.suggested_learning = 0

        # 6. Return the suggestion
        if suggestion != 0:
            print results, '-->', suggestion
        return suggestion

# ==============================================================
#                                                    MCL Level 2
# ==============================================================
class MCL2(Runner.Runner):

    def __str__(self): return 'MCL2'

    def monitor(self,results):
        return SUGGEST_NONE

# ==============================================================
#                                                    MCL Level 3
# ==============================================================
class MCL3(Runner.Runner):

    def __str__(self): return 'MCL3'

    def monitor(self,results):
        return SUGGEST_NONE

# ==============================================================
#                                                  RunnerFactory
# ==============================================================
CLASSES = {
    LEVEL0HC:   Level0,
    LEVEL1HCA:  Level1HCA,
    LEVEL1HCB:  Level1HCB,
    LEVEL2HC:   Level2HC,
    LEVEL3HC:   Level3HC,
    LEVEL2MCL:  MCL2,
    LEVEL3MCL:  MCL3
}

def RunnerFactory(number):

    return CLASSES[number]()

# ==============================================================
#                                                     unit tests
# ==============================================================
class TestRunners(unittest.TestCase):

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
# end                     R u n n e r s . p y                end
# ==============================================================
