#!/usr/local/bin/python
# ======================================================
#                    R e s u l t s . p y
# ======================================================

__author__  = 'Dean Earl Wright III <dean[three][at]UMBC[dot]edu>'
__date__    = '12 September 2011'
__version__ = '$Revision: 0 $'

"""Results.py

Create a database for Chippy dissertation results.
"""

# ======================================================
#                                                imports
# ======================================================
import os
import sys
import csv
import math
import time
import sqlite3
import unittest
from optparse import OptionParser

from Constants import *

import statlib.anova as anova

# ======================================================
#                                              constants
# ======================================================
DEFAULT_DB = ":memory:"

SAMPLE_INPUT_FILE = """
RUNNER,EXP,NUM,STEP,START_X,START_Y,DIRECTION,REWARD_X,REWARD_Y,EXPECTED,ACTUAL,FINAL_X,FINAL_Y,WHY,Q_N,Q_S,Q_E,Q_W,SUGGESTION
0,0,0,0,4,4,2,5,4,0,0,5,4,P,0.000000,0.000000,0.000000,0.000000,0
0,0,0,1,5,4,3,4,4,0,0,4,4,P,0.000000,0.000000,0.000000,0.000000,0
0,0,0,2,4,4,0,4,3,0,0,4,3,P,0.000000,0.000000,0.000000,0.000000,0
0,0,0,3,4,3,1,4,4,0,0,4,4,P,0.000000,0.000000,0.000000,0.000000,0
"""

EXPECTED_FIELDS = 19

NUM_WORDS = ['zero', 'one', 'two', 'three', 'four', 'five', 'six',
             'seven', 'eight', 'nine', 'ten', 'eleven', 'twelve']

LATEX_START = """
%
% Tables generated from the Rover experiments.
%
\\documentclass[12pt]{article}
\\usepackage{rotating}  % for sidewaystable environment
\\begin{document}
"""

LATEX_FINISH = """
\\end{document}
"""

LATEX_DASHES = 50

P_BY_P_TABLE_BEGIN = """
\\begin{table}
  \\centering
  \\begin{tabular}{%s}
    \\hline
    \\emph{1/2} & %s & \\emph{Avg} \\\\
    \\hline
"""


P_BY_P_TABLE_END = """
    \\hline
  \\end{tabular}
  \\caption%s
  \\label{tab:%s}
\\end{table}
"""

IGNORE_BEGIN = set(['picture', 'center', 'tabular',
                    'description', 'document'])

FIG_LABEL = 'fig:res:'
TAB_LABEL = 'tab:res:'

FIG_BEGIN = set(['figure',])
TAB_BEGIN = set(['table', 'longtable','sidewaystable'])

CSV_FILE_TYPE = '.csv'

# ======================================================
#                                                    SQL
# ======================================================
CRE8_RWDS  = """
create table rewards (
    exp_id integer primary key,
    ir1    integer,
    ir2    integer,
    cr1    integer,
    cr2 integer)"""
INSERT_RWD = """
insert into rewards (exp_id, ir1, ir2, cr1, cr2)
             values (%d,%d,%d,%d,%d)"""

CRE8_FNS   = """
create table filenames (
    fn_id integer primary key autoincrement,
    filename text)"""
INSERT_FN  = "insert into filenames (filename) values ('%s')"
ALL_FN     = "select fn_id, filename from filenames"
SELECT_FN  = "select fn_id from filenames where filename = '%s'"
COUNT_FNS  = "select count(fn_id) from filenames"
KNT_EXP_FN = "select count(exp_num) from experiments where file_id = ? and repeat=0"

CRE8_EXPS = """
create table experiments (
    exp_num    integer primary key autoincrement,
    file_id    integer key,
    runner_id  integer key,
    exp_id     integer key,
    repeat     integer,
    tot_rwds   integer,
    pos_rwds   integer,
    neg_rwds   integer,
    move_f     integer,
    move_p     integer,
    move_r     integer,
    sug_0      integer,
    sug_1      integer,
    sug_2      integer)"""

INSERT_EXP = """
insert into experiments (
    file_id,
    runner_id,
    exp_id,
    repeat,
    tot_rwds,
    pos_rwds,
    neg_rwds,
    move_f,
    move_p,
    move_r,
    sug_0,
    sug_1,
    sug_2) values (?,?,?,?,?,?,?,?,?,?,?,?,?)"""

COUNT_EXPS = "select count(exp_num) from experiments"

# ======================================================
#                                                Results
# ======================================================
class Results(object):

    def __init__(self, files=None,
                 db_name=DEFAULT_DB,
                 verbose=False):

        # 1. Initialize attributes
        self.db_name = db_name
        self.conn = sqlite3.connect(db_name)
        self.cursor = self.conn.cursor()
        self.fieldnames = None

        # 2. Create table for reward values
        self.cursor.execute(CRE8_RWDS)
        self.load_rewards()

        # 2. Create table for filenames and moves
        self.cursor.execute(CRE8_FNS)
        self.cursor.execute(CRE8_EXPS)

        # 3. Add data from csv files (if any given)
        if files:
            self.add_files(files, verbose=verbose)

    def load_rewards(self):

        # 1. Loop for all possible experiments
        for exp in range(len(EXPERIMENTS)):

            # 2. Determine the rewards for the experiment
            irwds = REWARDS[EXPERIMENTS[exp][INITIAL]]
            crwds = REWARDS[EXPERIMENTS[exp][CHANGED]]

            # 3. Write row in table
            self.cursor.execute(INSERT_RWD % (exp,
                                              irwds[0],
                                              irwds[1],
                                              crwds[0],
                                              crwds[1]))
        # 4. Commit the rewards table
        self.conn.commit()


    def add_files(self, files, verbose=False):

        # 1. If chatty, output header
        if verbose:
            print "File Count Name"

        # 2. Loop for all of the files
        for file_name in files:

            # 3. Add data from one file
            self.add_file(file_name, verbose=verbose)

    def add_file(self, file_name, verbose=False):

        # 1. Save file name in filename table
        self.cursor.execute(INSERT_FN % file_name)
        file_id = self.find_file(file_name)

        # 2. Open the csv file
        csv_obj = csv.DictReader(open(file_name, "rb"))
        last_exp = None
        exps = 0
        last_info = [0,0,0, 0,0,0, 0,0,0]

        # 3. Loop for every line in the file
        rows = 0
        for line in csv_obj:
            if len(line) != EXPECTED_FIELDS:
                print 'Results.add_file: Expected %d fields but got %d' % (
                    EXPECTED_FIELDS, len(line))
                sys.exit(1)
            if not line: continue
            rows += 1

            # 4. Process line
            this_exp  = [int(line['RUNNER']),
                         int(line['EXP']),
                         int(line['NUM'])]
            this_info = [int(line['ACTUAL']),0,0, 0,0,0, 0,0,0]
            if this_info[0] > 0:
                this_info[1] = 1
            elif this_info[0] < 0:
                this_info[2] = 1
            if line['WHY'] == 'F':
                this_info[3] = 1
            elif line['WHY'] == 'P':
                this_info[4] = 1
            elif line['WHY'] == 'R':
                this_info[5] = 1
            if line['SUGGESTION'] == '0':
                this_info[6] = 1
            elif line['SUGGESTION'] == '1':
                this_info[7] = 1
            elif line['SUGGESTION'] == '2':
                this_info[8] = 1

            # 5. If same experiment, update counts
            if this_exp == last_exp:
                for i in range(9):
                    last_info[i] += this_info[i]

            # 6. Else write exp data
            else:
                if last_exp:
                    self.add_experiment(file_id,
                                        last_exp,
                                        last_info)
                    exps += 1
                last_exp = this_exp
                last_info = this_info

        # 7. Output last info (if any)
        if last_exp:
            self.add_experiment(file_id,
                                last_exp,
                                last_info)
            exps += 1

        # 8. Commit the file
        self.conn.commit()

        # 9. Announce file (if desired)
        if verbose:
            print "%4d %5d %s" % (file_id, exps, file_name)

    def add_experiment(self, file_id,
                             exp, info):
        parms = [file_id]
        parms.extend(exp)
        parms.extend(info)
        self.cursor.execute(INSERT_EXP, parms)

    def find_file(self, filename):
        self.cursor.execute(SELECT_FN % filename)
        return int(self.cursor.next()[0])

    def number_files(self):
        self.cursor.execute(COUNT_FNS)
        return int(self.cursor.next()[0])

    def number_experiments(self):
        self.cursor.execute(COUNT_EXPS)
        return int(self.cursor.next()[0])

    def get_filename_info(self):

        # 1. Start with nothing
        result = {}

        # 2. Loop for all of the files
        for row in self.cursor.execute(ALL_FN):

            # 3, Save filename
            result[row[0]] = row[1]

        # 4. Loop for all file numbers
        for k in result:

            # 5. Get number of experiments in the file
            self.cursor.execute(KNT_EXP_FN, k)
            knt = int(self.cursor.next()[0])

            # 6. Save the number of experiments
            result[k] = (result[k], knt)

        # 7. Return experiment
        return result

# ======================================================
#                                               unittest
# ======================================================
class Test_Results(unittest.TestCase):

    def test_convert_file(self):
        pass

# ======================================================
#                                       parseCommandLine
# ======================================================
def parseCommandLine():

    # 1. Create the command line parser
    usage = """usage: %prog [options] csv-files
sample: python %prog -t results.tex -s ../current ../Experiments/*.csv"""
    version = "%prog beta"
    parser = OptionParser(usage=usage, version=version)
    parser.add_option("-u", "--unittest", default=False,
                      action="store_true", dest="unittest",
                      help="Execute unittests")
    parser.add_option("-v", "--verbose", default=False,
                      action="store_true", dest="verbose",
                      help="Print status messages to stdout")
    parser.add_option("-d", "--db_name", default=DEFAULT_DB,
                      action="store", dest="db_name",
                      help="Database name (default = :memory:")
    parser.add_option("-t", "--tex_file", default=None,
                      action="store", dest="tex_file_name",
                      help="Output tex file")
    parser.add_option("-q", "--quick", default=False,
                      action="store_true", dest="quick",
                      help="Display quick tables")
    parser.add_option("-s", "--split", default=None,
                      action="store", dest="split",
                      help="Split tables/figures into directory")
    parser.add_option("-c", "--csv", default=None,
                      action="store", dest="csv",
                      help="Directory for CSV tables and histograms")


    # 2. Get the options and arguments
    options, args = parser.parse_args()

    # 3. Validate arguments
    if options.unittest:
        if args:
            parser.error('Arguments are not allowed for unit test')
    elif not args:
        parser.error("At least one experment csv file name must be specified")
    for filename in args:
        if not os.path.isfile(filename):
            parser.error("File %s not found" % filename)

    # 4. Check split directory (if any)
    if options.split:
        if not options.tex_file_name:
            parser.error("Split (-s) requires latex file (-t) specified")
        if not os.path.isdir(options.split):
            parser.error("Split directory (%s) not found" % options.split)

    # 5. Check CSV directory (if any)
    if options.csv:
        if not options.tex_file_name:
            parser.error("CSV (-c) requires latex file (-t) specified")
        if not os.path.isdir(options.csv):
            parser.error("CSV directory (%s) not found" % options.csv)

    # 6. Return validated options and arguments
    return options, args

# ======================================================
#                                                   main
# ======================================================
def main():

    # 1. Get the command line options
    options, args = parseCommandLine()

    # 2. Perform unit tests if requested
    if options.unittest:
        unittest.main(argv=["Results.py"])
        sys.exit(2)

    # 3. Read in the csv experiments to sqlite tables
    results = Results(args, db_name=options.db_name, verbose=options.verbose)
    if options.verbose:
        print

    # 4. Output some general information
    if options.verbose:
        print "Number of experiment files  = %6d" % results.number_files()
        print "Total number of experiments = %6d" % results.number_experiments()
        print

    # 5. Output quick counts (if desired by -q)
    if options.quick:
        output_quick(results)

    # 6. Output latex tables (if desired by -t).
    if options.tex_file_name:
        latex_tables(results, options.tex_file_name, options.csv)

    # 7. Split latex file into individual files
    if options.split:
        split_tables(options.split, options.tex_file_name)

# ======================================================
#                                           latex_tables
# ======================================================
def latex_tables(results, file_name, csv=None):

    # 1. Open Latex output file
    latex = open(file_name, 'w')
    latex_start(latex, file_name)

    # 2. Write table of experiment files
    latex.write("""

Table~\\ref{tab:res:files} [tab:res:files] lists the experimental files that were created by the mars rover simulation.
There are useful for seeing which experimental runs produced the tables in this
appendix but are not likely to be included in the final paper.

""")

    # 14. Close latex file
    latex.write("""

That's all the tables for now.  I would like to do a few more but don't know
what would be useful.

""")
    latex_finish(latex, file_name)
    latex.close()

# ======================================================
#                                            latex_start
# ======================================================
def latex_start(latex, file_name):

    # 1. Output the name of the file
    latex_filename(latex, file_name, top=True)

    # 2. Output date / time generated
    latex.write('\n%% Generated %s\n\n' % time.ctime())

    # 3. Output fixed text
    latex.write(LATEX_START)

# ======================================================
#                                           latex_finish
# ======================================================
def latex_finish(latex, file_name):

    # 1. Output fixed text
    latex.write(LATEX_FINISH)

    #  2. Output the name of the file
    latex_filename(latex, file_name, top=False)

# ======================================================
#                                         latex_filename
# ======================================================
def latex_filename(latex, file_name, top=True):

    # 1. Output a line of dashes
    dashes = LATEX_DASHES * '='
    latex.write('%% %s\n' % dashes)

    # 2. Explode name
    name = ' '.join([x for x in file_name])
    name = name.center(LATEX_DASHES)
    if not top:
        name = 'END ' + name[4:-4] + ' END'
    latex.write('%% %s\n' % name)

    # 3. Output a second line of dashes
    latex.write('%% %s\n' % dashes)

# ======================================================
#                                 latex_experiment_files
# ======================================================
def latex_experiment_files(latex, info, comment=False):

    # 1. Comment out (if desired)
    if comment:
        latex.write("\\begin{comment}\n")

    # 2. Output the table header
    latex.write("""

% --------------------------------------------------
%        E x p e r i m e n t  C S V  F i l e s
% --------------------------------------------------

\\begin{center}
\\begin{longtable}{|c|l|r|}
\\caption{Files with experiment results}  \\\\ % label was here
\\hline
\\emph{Number} & \\emph{File} & \\emph{Experiments} \\\\
\\hline
\\endhead
\\hline
\\multicolumn{3}{|r|}{{Continued on next page}} \\\\
\\hline
\\endfoot
\\hline \\hline
\\endlastfoot
\\label{tab:res:files}
""")

    # 3. Get Experiment file information
    keys = info.keys()
    keys.sort()

    # 4. Loop for all of the rows in the file information
    for k in keys:

        # 5. Output one row
        latex.write('%s & %s & %s \\\\\n' % (k, info[k][0], info[k][1]))

    # 6. Close it up
    latex.write("""
\\hline
\\end{longtable}
\\end{center}


% --------------------------------------------------
% end    E x p e r i m e n t  C S V  F i l e s   end
% --------------------------------------------------

""")

    # 7. Close comment (if needed)
    if comment:
        latex.write("\\end{comment}\n")

# ======================================================
#                               latex_experiment_numbers
# ======================================================
def latex_experiment_numbers(latex, results, comment=False):

    # 1. Comment out table (if desired)
    if comment:
        latex.write("\\begin{comment}\n")

    # 2. Output the table header
    latex.write("""

% --------------------------------------------------
%         E x p e r i m e n t  N u m b e r s
% --------------------------------------------------

\\begin{table}[htb]
\\caption{Experiment Numbers}
\\label{tab:res:numbers}
\\begin{center}
\\begin{tabular}{|l|r|}
\\hline
\\emph{What} & \\emph{Number} \\\\
\\hline
""")

    # 3. Output experiment file information

    latex_number(latex,
                 'Number of experiment files',
                 results.number_files(),
                 'NumTables')
    latex_number(latex,
                 "Total number of experiments",
                 results.number_experiments(),
                 'NumExperiments')
    latex_number(latex,
                 "Maximum number of steps",
                 results.max_steps(),
                 'MaxNumSteps')

    # 4. Close it up
    latex.write("""
\\hline
\\end{tabular}
\\end{center}
\\end{table}

% --------------------------------------------------
% end     E x p e r i m e n t  N u m b e r s     end
% --------------------------------------------------

""")

    # 5. Close comment (if needed)
    if comment:
        latex.write("\\end{comment}\n")

# ======================================================
#                                           latex_number
# ======================================================
def latex_number(latex, title, number,
                 command=None, csv=None):

    # 1. Output table line
    latex.write('%s & %s \\\\\n' % (title, number))

    # 2. Output new command (if needed)
    if command:
        newcommand(latex, command, number)

    # 3. Output csv line (if needed)
    if csv:
        csv.write('"%s",%s\n' % (title, number))

# ======================================================
#                                             newcommand
# ======================================================
def newcommand(latex, command, number):
    latex.write('\\newcommand{\\res%s}{%s}\n'
                % (command, number))


# ======================================================
#                                          block_comment
# ======================================================
def block_comment(latex=None, comment='', top=True):

    # 1. Output a line of dashes
    dashes = LATEX_DASHES * '-'
    latex.write('% ' + dashes + '\n')

    # 2. Output the comment
    text = comment.center(LATEX_DASHES)
    if not top:
        text = 'end ' + text[4:-4] + ' end'
    latex.write('%% %s\n' % text)

    # 3. Output a final line of dashes
    latex.write('% ' + dashes + '\n')


# ======================================================
#                                           split_tables
# ======================================================
def split_tables(split_dir, tex_file_name):

    # 1. Start with nothing
    tables = {}
    figures = {}
    duplicates = set()
    begin = None
    label = None
    lines = []
    begun_at = 0
    knt_lines = 0
    knt_ignored = 0
    knt_written = 0
    knt_blocks = 0
    knt_duplicates = 0
    knt_errors = 0
    new_commands = []

    # 2. Loop for all of the lines in the input file
    for line in open(tex_file_name):
        knt_lines += 1

        # 3. If label, remember it
        sline = line.strip()
        if sline.startswith('\\label'):
            new_label = sline.split('{')[1].split('}')[0]
            if label:
                print ("*** Label %s at line %d found while processing %s"
                       % (new_label, knt_lines, label))
                knt_errors += 1
            elif not begin:
                print ("*** Label %s at line %d found before begin"
                       % (new_label, knt_lines))
                knt_errors += 1
            else:
                if new_label.startswith(FIG_LABEL):
                    if begin not in FIG_BEGIN:
                        print ("*** Figure label %s at line %d in non-figure %s block started at %d"
                               % (new_label, knt_lines, begin, begun_at))
                        knt_errors += 1
                        continue
                    if new_label in figures:
                        print ("*** Figure label %s at line %d already used at line %d"
                               % (new_label, knt_lines, figures[new_label]))
                        knt_errors += 1
                        duplicates.add(new_label)
                        continue
                    figures[new_label] = new_label
                elif new_label.startswith(TAB_LABEL):
                    if begin not in TAB_BEGIN:
                        print ("*** Table label %s at line %d in non-table %s block started at %d"
                               % (new_label, knt_lines, begin, begun_at))
                        knt_errors += 1
                        continue
                    if new_label in tables:
                        print ("*** Table label %s at line %d already used at line %d"
                               % (new_label, knt_lines, figures[new_label]))
                        knt_errors += 1
                        duplicates.add(new_label)
                        continue
                    tables[new_label] = new_label
                else:
                    print ("*** Unkown label style %s at line %d in non-table %s block started at %d"
                           % (new_label, knt_lines, begin, begun_at))
                    knt_errors += 1
                    continue
                label = new_label
                lines.append(line)

        # 4. If matching end, stop collecting and write out lines
        elif begin and sline.startswith('\\end{'+begin+'}'):
            if not label:
                print ("*** %s block started at line %d and ended at line %d has no label"
                       % (begin, begun_at, knt_lines))
                knt_errors += 1
            else:
                lines.append(line)
                knt_written += write_split_file(split_dir, label, lines)
            begin = None
            label = None
            lines = []
            knt_blocks += 1

        # 5. If begin, start collecting
        elif sline.startswith('\\begin{'):
            new_begin = sline.split('{')[1].split('}')[0]
            if new_begin in IGNORE_BEGIN:
                if begin: lines.append(line)
                continue
            if begin:
                print ("*** Begin %s at line %d found while processing %s begun at line %d"
                       % (new_begin, knt_lines, begin, begun_at))
                knt_errors += 1
            else:
                begin = new_begin
                lines = [line]
                begun_at = knt_lines

        # 6. Remember new commands separately
        elif sline.startswith('\\newcommand'):
            new_commands.append(sline + '\n')

        # 7. Remember line (if collecting)
        elif begin:
            lines.append(line)

        # 8. Else ignore line
        else:
            knt_ignored += 1

    # 9. Check that last block was finished
    if begin:
        print ("*** %s block started at line %d never finished"
               % (begin, begun_at))

    # 10. Output new commands (if any)
    if new_commands:
        knt_written += write_split_file(split_dir,
                                        'res_new_commands',
                                        new_commands)

    # 10. Output infomation
    print "------------------------ split to %s" % split_dir
    print "%6d lines read" %  knt_lines
    print "%6d lines ignored" % knt_ignored
    print "%6d lines written" % knt_written
    print "%6d blocks found" % knt_blocks
    print "%6d tables" % len(tables)
    print "%6d figures" % len(figures)
    if new_commands:
        print "%6d new commands" % len(new_commands)
    if knt_errors:
        print "%6d errors in %s" % (knt_errors, tex_file_name)
    if knt_duplicates:
        print "%6d duplicates: %s" % (knt_duplicates, ','.join(duplicates))

# ======================================================
#                                       write_split_file
# ======================================================
def write_split_file(split_dir, label, lines):
    #print "Writing %d lines to %s" % (len(lines), label)

    # 1. Check lines and label
    if not lines:
        print ('!!! Attempting to write %d lines to %s'
               % (len(lines), label))
        return
    if not label:
        print ('!!! Attempting to write %d lines with no label'
               % (len(lines)))
        return

    # 2. Open the output file
    output_file_name = label.replace(':','_') + '.tex'
    output_file_path = os.path.join(split_dir, output_file_name)
    out_f = open(output_file_path, 'w')

    # 3. Write the lines to it
    out_f.writelines(lines)

    # 4. Close the file
    out_f.close()

    # 5. And return the number of lines written
    return len(lines)

# ======================================================
#                                  module initialization
# ======================================================
if __name__ == "__main__":
    main()

# ======================================================
# end                R e s u l t s . p y             end
# ======================================================
