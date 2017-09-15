# ==============================================================
#                     C o n s t a n t s . p y
# ==============================================================

# Author:  Dean Earl Wright III
# Created: 24 August 2011
# Purpose: Reimplementation of the Q-learning perturbation
#          testbed for multiple metacognition levels.

# ==============================================================
#                                                     directions
# ==============================================================
DIR_N = 0
DIR_S = 1
DIR_E = 2
DIR_W = 3
DIRECTIONS  = [DIR_N, DIR_S, DIR_E, DIR_W]
DIR_LETTER = 'NSEW'
DIR_DELTA_X = [ 0,  0,  1, -1]
DIR_DELTA_Y = [-1,  1,  0,  0]

# ==============================================================
#                                                   move results
# ==============================================================
RESULT_START_LOC  = 0
RESULT_DIRECTION  = 1
RESULT_REWARD_LOC = 2
RESULT_EXP_REWARD = 3
RESULT_ACT_REWARD = 4
RESULT_FINAL_LOC  = 5
RESULT_MOVE_TYPE  = 6
RESULT_POLICY_N   = 7
RESULT_POLICY_S   = 8
RESULT_POLICY_E   = 9
RESULT_POLICY_W   = 10
RESULT_SUGGESTION = 11

RESULT_NAMES = [
    "START",
    "DIR",
    "MOVE",
    "EXPECTED",
    "ACTUAL",
    "JUMP",
    "TYPE",
    "QN",
    "QS",
    "QE",
    "QW",
    "SUGGESTION"
    ]

MOVE_TYPE_FORCED = 'F'
MOVE_TYPE_POLICY = 'P'
MOVE_TYPE_RANDOM = 'R'


# ==============================================================
#                                            Grid/Square picture
# ==============================================================
DRAW_QMAX   = 0
DRAW_VALUE  = 1
DRAW_LOGV   = 2
DRAW_REWARD = 3

# ==============================================================
#                                                      Q Learner
# ==============================================================
DEFAULT_ALPHA   = 0.5
DEFAULT_GAMMA   = 0.9
DEFAULT_EPSILON = 0.05

DELTA_EPSILON   = 0.001
MIN_EPSILON     = 0.01
MAX_EPSILON     = 0.99

# ==============================================================
#                                                    experiments
# ==============================================================
REWARDS = [(10,-10), # 0
           (25,  5), # 1
           (35, 15), # 2
           (19, 21), # 3
           (15, 35), # 4
           ( 5, 25), # 5
           (-10,10), # 6
           (21, 19)] # 7

EXPERIMENTS = [
#   ----------------- second ----------------------
#   (25,5)  (35,15) (19,21) (15,35) (5,25) (-10,10)   first
    (0, 1), (0, 2), (0, 3),                 (0, 6), # (10,-10) 4
            (1, 2), (1, 3),         (1, 5), (1, 6), # (25,5)   4
    (2, 1),         (2, 3), (2, 4),         (2, 6), # (35,15)  4
    (3, 1), (3, 2), (3, 7),                 (3, 6), # (19,21)  4
    (4, 1),         (4, 3),                 (4, 6), # (15,35)  3
            (5, 2), (5, 3),                 (5, 6)] #  (5,25)  3
#                                                             ---
#                                                             22
__EXPERIMENTS = [
    (0,1), (0,2), (0,3), (0,4), (0,5), (0,6), # (10,-10) 6
           (1,2), (1,3), (1,4), (1,5), (1,6), # (25,5)  5
                  (2,3), (2,4), (2,5), (2,6), # (35,15) 4
                  (3,7), (3,4), (3,5), (3,6), # (19,21)  4
                                (4,5), (4,6), # (15,35)  2
                                       (5,6)] #  (5,25)  1
                                              #         ---
                                              #         22

INITIAL = 0
CHANGED = 1

R1 = 0
R2 = 1
RR = 2

SWITCH_TURN = 10000
STOP_TURN   = 20000

REPEATS     = 20

# ==============================================================
#                                                rolling average
# ==============================================================
AVERAGE_WINDOW = 200

# ==============================================================
#                                                  metacognition
# ==============================================================
SUGGEST_NONE  = 0
SUGGEST_RESET = 1
SUGGEST_LEARN = 2

LEVEL0HC   = 0
LEVEL1HCA  = 1
LEVEL1HCB  = 2
LEVEL2HC   = 3
LEVEL3HC   = 4
LEVEL2MCL  = 5
LEVEL3MCL  = 6

LEVELS = [LEVEL0HC, LEVEL1HCA, LEVEL1HCB, LEVEL2HC, LEVEL3HC]
#          LEVEL2MCL, LEVEL3MCL]

LEARN_EPSILON  = 0.20
LEARN_ADJUST   = 0.00025
LEARNING_LIMIT = 3

# ==============================================================
# end                C o n s t a n t s . p y                 end
# ==============================================================
