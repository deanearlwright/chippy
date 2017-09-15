# ==============================================================
#                   D i s s e r t a t i o n . p y
# ==============================================================

# Author:  Dean Earl Wright III
# Created: 9 September 2011
# Purpose: Reimplementation of the Q-learning perturbation
#          testbed for multiple metacognition levels.

# ==============================================================
#                                                        imports
# ==============================================================
import sys
import time
import unittest
from optparse import OptionParser

from Constants import *
import Experiments
import Runners
import Grid

# ==============================================================
#                                                      constants
# ==============================================================
CSV_HEADER = "RUNNER,"

# ==============================================================
#                                                       unittest
# ==============================================================
class Test_Dissertation(unittest.TestCase):

    def test_somethine(self):
        pass

# ==============================================================
#                                               parseCommandLine
# ==============================================================
def parseCommandLine():

    # 1. Create the command line parser
    usage = """usage: %prog [options]"""
    version = "%prog beta"
    parser = OptionParser(usage=usage, version=version)
    parser.add_option("-u", "--unittest", default=False,
                      action="store_true", dest="unittest",
                      help="Execute unittests")
    parser.add_option("-v", "--verbose", default=False,
                      action="store_true", dest="verbose",
                      help="Print status messages to stdout")
    parser.add_option("-c", "--csv_name", default=None,
                      action="store", dest="csv_name",
                      help="CSV file name (default = TIMESTAMP.csv")
    parser.add_option("-e", "--experiment", default=None,
                      action="store", dest="exp_number", type="int",
                      help="Experiment number (0-21), default=ALL")
    parser.add_option("-l", "--level", default=None, type="int",
                      action="store", dest="level",
                      help="Runner to use (0-4), default=ALL")
    parser.add_option("-r", "--repeats", default=REPEATS, type="int",
                      action="store", dest="repeats",
                      help="Times to rerun the experiment")

    # 2. Get the options and arguments
    options, args = parser.parse_args()

    # 3. Validate arguments
    if args:
        parser.error('No arguments expected')

    # 4. Validate options
    if options.csv_name == None:
        options.csv_name = '%s.csv' % time.strftime('%Y%m%d%H%M')
    if options.level == None:
        options.level = LEVELS
    else:
        options.level = [int(options.level)]

    # 5. Return validated options and arguments
    return options, args

# ==============================================================
#                                                       open_csv
# ==============================================================
def open_csv(csv_name):

    # 1. Open the experiment output file
    csv_f = open(csv_name, 'w')

    # 2. Output the header line
    grid = Grid.Grid()
    runner = Runners.RunnerFactory(1)
    exps   = Experiments.Experiments(walker=runner,
                                     grid=grid)
    csv_f.write('%s%s\n' % (CSV_HEADER, exps.csv_header()))

    # 3. Return the opened csv file
    return csv_f

# ==============================================================
#                                                           main
# ==============================================================
def main():

    # 1. Process command line arguments
    options, args = parseCommandLine()

    # 2. Perform unit tests if requested
    if options.unittest:
        unittest.main(argv=["Dissertation.py"])
        sys.exit(2)

    # 3. Create a Chippy world
    grid = Grid.Grid()

    # 4. Open csv file for results
    csv_f = open_csv(options.csv_name)

    # 5. Loop for all of the runners
    for number in options.level:

        # 6. Set up experiments for that Runner
        runner = Runners.RunnerFactory(number)
        exps   = Experiments.Experiments(walker=runner,
                                         grid=grid,
                                         repeats=options.repeats)
        csv_b = '%d,' % number
        csv_e = '\n'

        # 7. Run the experiments
        result = exps.run(csv_f, csv_b, csv_e,
                          experiments=options.exp_number,
                          verbose=options.verbose)

        # 8. Output quick counts
        if options.verbose:
            print number, result

    # 9. Close the csv file
    csv_f.close()

# ==============================================================
#                                          module initialization
# ==============================================================
if __name__ == "__main__":
    main()

# ==============================================================
# end              D i s s e r a t i o n . p y               end
# ==============================================================
