// ====================================================================
//                       c h i p p y M A . c p p
// ====================================================================

// Author:  Dean Earl Wright III
// Created: 10 December 2008
// Purpose: Reimplementation of the Q-learning perturbation testbed
//          use for the initial Meta-cognitive Loop testing.  This
//          version uses the MCL multiagent API.
// Updated: June 2009
//          To use lastest version of MCL Multiagent API
// --------------------------------------------------------------------
//                                                              imports
// --------------------------------------------------------------------

#define WIN32_LEAN_AND_MEAN
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <cstdlib>
#include <ctime>
#include <cassert>
#include <math.h>

#define USEMCL2
#ifdef USEMCL2
#include "mcl_multiagent_api.h"
//#include "APICodes.h"
#endif

using namespace std;

// --------------------------------------------------------------------
//                                                            constants
// --------------------------------------------------------------------
#define ROLLING_AVERAGE_SIZE 200

#define EXPECTED_STATE_UNKNOWN -1
#define EXPECTED_REWARD_UNKNOWN -1000

#define PERTURB_NONE 0
#define PERTURB_TTR 1
#define PERTURB_PRF 2
#define PERTURB_EXP 3

#define MIN_ACTION_NUMBER 2000

#define EXP_REPEAT 20 
#define EXP_STEPS  20000
#define EXP_PERTURB 10000

#define MAX_EXPECTATIONS 10
#define EGK 1

// --------------------------------------------------------------------
//                                                           directions
// --------------------------------------------------------------------
// E and W are OK but N and S are backwards -- Need to change
#define DIR_N 0
#define DIR_S 1
#define DIR_E 2
#define DIR_W 3
#define DIR_NUM 4
#define DIR_LETTER "NSEW"
static int DIR_DELTA_X[DIR_NUM] = { 0,  0,  1, -1};
static int DIR_DELTA_Y[DIR_NUM] = { 1, -1,  0,  0};

// --------------------------------------------------------------------
//                                                            locations
// --------------------------------------------------------------------

#define LOC_RAN -1
#define LOC_MIN -2
#define LOC_MAX -3
#define LOC_CTR -4

static int LOC_LL[2] = {LOC_MIN, LOC_MIN};
static int LOC_UR[2] = {LOC_MAX, LOC_MAX};
static int LOC_UL[2] = {LOC_MIN, LOC_MAX};
static int LOC_LR[2] = {LOC_MAX, LOC_MIN};
//static int LOC_RR[2] = {LOC_RAN, LOC_RAN};

static int *CLOCKWISE[DIR_NUM] = {LOC_LL, LOC_UL, LOC_UR, LOC_LR};

// --------------------------------------------------------------------
//                                                                 draw
// --------------------------------------------------------------------
#define DRAW_QMAX   0
#define DRAW_VALUE  1
#define DRAW_LOGV   2
#define DRAW_REWARD 3

// --------------------------------------------------------------------
//                                                         command line
// chippy2008 -h            display help
//            -u            perform all unittests
//            -t <name>     perform specified unittest
//            -e            perform all experiments
//            -g <name> -w <name>  perform specified experiment
//               -v         verbose
//               -p         output policy
// --------------------------------------------------------------------
#define CMD_NONE 0
#define CMD_HELP 1
#define CMD_UNITTESTS 2
#define CMD_1_UNITTEST 3
#define CMD_EXPERIMENTS 4
#define CMD_1_EXPERIMENT 5

// --------------------------------------------------------------------
//                                                              walkers
// --------------------------------------------------------------------
#define WALK_NONE 0
#define WALK_WALKER 1
#define WALK_QLEARNER 2
#define WALK_SIMPLE 3
#define WALK_SENSITIVE 4
#define WALK_SOPHISTICATED 5
#define WALK_BAYES1 6
#define WALK_BAYES2 7

const char *walker_initials[] = {
    "??", "WA", "QL", "SI", "SE", "SO", "B1", "B2"
};

// --------------------------------------------------------------------
//                                                                grids
// --------------------------------------------------------------------
#define GRID_NONE 0
#define GRID_CHIPPY 1
#define GRID_CLASSIC 2
#define GRID_CORNER 3
#define GRID_ROTATE 4
#define GRID_PCHIPPY 5
#define GRID_PCLASSIC 6
#define GRID_PCORNER 7
#define GRID_PROTATE 8

const char *grid_initials[] = {
    "??", "CH", "CL", "CO", "RO", "PC", "PL", "PO", "PR"
};

// ====================================================================
//                                                              randint
// Return a random integer
// ====================================================================
int randint(int low, int high)
{
    int result = low + rand() % (high - low + 1);
    //int result = low + random() % (high - low + 1);
    //cout << "randint(" << low << "," << high << ")=" << result << endl;
    return result;
}

// ====================================================================
//                                                         orient_value
// Decode a location value based on size of grid
// ====================================================================
int orient_value(int value, int n)
{
    switch(value) {
        case LOC_RAN:
            return randint(1, n-2);
        case LOC_MIN:
            return 0;
        case LOC_MAX:
            return n-1;
        case LOC_CTR:
            return n/2;
    }
    return value;
}

// ====================================================================
//                                                               Square
// A single square of a grid world.
// ====================================================================
class Square
{
    int    x;
    int    y;
    double q[DIR_NUM];
    
    char    letter;
    double  value;
    int     visits;
    bool    underline;
    double  reward;
    
public:    
    Square(int xx = 0, int yy = 0)
    {
            x    = xx;
            y    = yy;

            letter = '\0';
            visits = 0;
            value  = 0.0;
            underline = false;
            reward = 0.0;
            
            reset();
    }    
    
    int    get_x()      const { return x; }
    int    get_y()      const { return y; }

    double get_q(int index) const { return q[index]; }
    void set_q(int index, double value) {q[index] = value; }

    void set_letter(char l='\0') { letter = l; }
    void set_value(double v=0.0) { value = v; }
    void set_underline(bool u=true) { underline = u; }
    void set_visits(int v=0) {visits = v; }
    void set_reward(double r=0.0) {reward = r; }
    void visit() {++visits; }
    
    void reset() {for (int i = 0; i < DIR_NUM; ++i) q[i] = 0.0; }
    
    int suggest()
    {
        // "Suggest the highest ranked move"
        int picked[DIR_NUM] = {0};
        int dir, d; 
        
        //  1. Start by picking a random direction
        int pick = randint(0, DIR_NUM-1);
        picked[pick] = 1;
        double value  = q[pick];
        
        // 2. Loop for the other directions
        for (dir = 0; dir < DIR_NUM; ++dir)
        {
            if (dir == pick) continue;
            
            // 3. If this direction is better, save it
            if (q[dir] > value)
			{
				for (d = 0; d < DIR_NUM; ++d) picked[d] = 0;
                value = q[dir];
                picked[dir] = 1;
				pick = dir;
			}
            
            // 4. If the same, keep both (or more)    
            else if (q[dir] == value)
            {
                picked[dir] = 1;
				pick = dir;
			}
        }
        
        // 5. Count the number of "best" directions
		int num = 0;
		for (dir = 0; dir < DIR_NUM; ++dir)
			if (picked[dir] == 1) ++num;
        
        // 6. Pick just one if there are multiple
        if (num > 1) 
        {
		    num = rand() % num;
		    for (dir = 0; dir < DIR_NUM; ++dir)
            { 
			    if (picked[dir] == 1)
				{
					--num;
					if (num < 0) 
                    {
                        pick = dir;
                        break;
                    }
				}
            }
        } 
        
		// 7. Return one of the "best" directions
        return pick;
    }
    
    double max()
    {   
        double value  = q[0];
        for (int dir = 1; dir < DIR_NUM; ++dir)
            if (q[dir] > value) value = q[dir];
        return value;
    }    

    double min()
    {   
        double value  = q[0];
        for (int dir = 1; dir < DIR_NUM; ++dir)
            if (q[dir] < value) value = q[dir];
        return value;
    }    
    
    void draw(ostream& out, bool arrow=true, int which=DRAW_QMAX)
    {
            // 1. Get values and direction for squeare
            double smax = max();
            double smin = min();
            int    dir;
            
            // 2. Output as a comment all four values
            out << "        % --- [" << x << "," << y << "]";
            for (dir = 0; dir < DIR_NUM; ++dir)
            {
                out << " " << DIR_LETTER[dir] << ":" << q[dir];
            }
            out << " r:" << reward << " v:" << visits << endl;
            
            // 3. Output the box
            out << "        \\put(" << x << "," << y << "){";
            out << "\\framebox(1.0,1.0){";
            
            // 4. Output the letter or policy value or visits of whatever
            if ('\0' != letter)
            {
                out << "{\\LARGE\\bfseries{" << letter << "}}";
            }
            else
            {   
                if (underline) out << "\\underline{";
                switch (which)
                {
                    case DRAW_QMAX:  
                        if ((0.1 < fabs(smax)) || (0.1 < fabs(smin)))
                        {
                            out << setprecision(fabs(smax)>0?3:2) << setw(4) << smax;  
                        }
                        break;
                    case DRAW_VALUE: 
                        out << setprecision(fabs(value)>0?3:2) << setw(4) << value;  
                        break;     
                    case DRAW_LOGV:  
                        out << setprecision(3) << setw(4) << (visits==0?0:log(visits));  
                        break;
                    case DRAW_REWARD:  
                        out << setprecision(3) << setw(4) << reward;  
                        break;
                    default:
                        out << "{\\LARGE\\bfseries{?}}";  
                }         
                if (underline) out << "}";
            }
            out << "}}" << endl;
            
            // 5. Output an arrow or arrows if and as appropiate
            if (arrow && 0.1 < smax - smin)
            {
                for (dir = 0; dir < DIR_NUM; ++dir)
                {
                    if (0.05 > smax - q[dir])
                    {
                        out << "        \\put(" << x+0.5 << "," << y+0.5 
                        << "){\\vector(" << DIR_DELTA_X[dir] 
                        << "," << DIR_DELTA_Y[dir] << "){.5}}" 
                        << endl;
                    }       
                }                       
            }
    }
    
};

// ====================================================================
//                                                                 Goal
// Reward and move
// ====================================================================
class Goal
{
    int    r;
    int    x;
    int    y;
    int    nx;
    int    ny;
    int    n;
    int    o_x;
    int    o_y;
    int    j_x;
    int    j_y;
    
public:    
    Goal(int rr=0, int xx = 0, int yy = 0, int nxx=0, int nyy=0)
    {
        r   = rr;  
        x   = xx;
        y   = yy;
        nx  = nxx;
        ny  = nyy;
        n   = 0;
        o_x = 0;
        o_y = 0;
    }    

    int    get_reward() const { return r;   }
    void   set_reward(int rr) { r = rr;     }
    int    get_x()      const { return x;   }
    int    get_y()      const { return y;   }
    int    get_newx()   const { return nx;  }
    int    get_newy()   const { return ny;  }
    int    get_n()      const { return n;   }
    int    get_ox()     const { return o_x; }
    int    get_oy()     const { return o_y; }
    
    void orient(int nn=0) {
        n = nn;
        o_x = orient_value(x, n);
        o_y = orient_value(y, n);
    }
        
    int    get_jumpx() const { return orient_value(nx, n); }
    int    get_jumpy() const { return orient_value(ny, n); }

    void   set_x(int xx)  { x = xx; }
    void   set_y(int yy)  { y = yy; }
};

// ====================================================================
//                                                                 Grid
// Multiple squares arranged in an n by n matrix with two rewards
// ====================================================================
class Grid
{
    int      n;
    Square **squares;
    Goal   **goals;
    
public:
    Grid(int nn=8, Goal **g=NULL)
    {
        // 1. Create the grid of squares
        n = nn;
        squares = (Square **)malloc(sizeof(Square *)*n*n);
            
        // 2. Create all the individual squares
        for (int x = 0; x < n; ++x)
        {
            for (int y = 0; y < n; ++y)
            {  
                Square *s = new Square(x, y);
                squares[x*n+y] = s;
            }
        }
        
        // 3. Orient the goals on the grid
        set_goals(g);
    }

    virtual ~Grid(){
        // 1. Loop for all of the squares in the grid
        for (int x = 0; x < n; ++x)
        {
            for (int y = 0; y < n; ++y)
            {  
                // 2. And delete them
                delete squares[x*n+y];
            }
        }
        
        // 3. Delete the square pointers
        free(squares);
    }
    
    int    get_n()     const { return n; }
    Goal **get_goals() const { return goals; }

    void set_goals(Goal **g=NULL)
    {
        // 1. Save pointer to NULL terminated array of goals 
        goals = g;
        
        // 2. Orient each of the Goals  
        if (goals) {
            for (Goal **g = goals; *g != NULL; ++g) {
                (*g)->orient(n);
            }
        }
    }
    
    Square *square(int x, int y)
    {    
        return squares[x*n + y];
    }
    
    Goal *goal_at(int x, int y)
    {
        // 1. Loop for all the goals  
        if (goals) {
            for (Goal **g = goals; *g != NULL; ++g) {
                
                // 2. If this is the one, return it
                if (x == (*g)->get_ox() &&
                    y == (*g)->get_oy()) {
                    return *g;
                }
            }
        }
        
        // 3. If no matching goal, return the bad news
        return NULL;
    }
    
    virtual Goal* move(int x, int y, int dir, int *n_x, int *n_y)
    {
        //"From square 'at' move in direction 'dir'"
        Goal *g;
        
        // 1. Determine new square
        int new_x = x + DIR_DELTA_X[dir];
        if (new_x < 0)          new_x = 0;
        if (new_x >= n)         new_x = n-1;
        int new_y = y + DIR_DELTA_Y[dir];
        if (new_y < 0)          new_y = 0;
        if (new_y >= n)         new_y = n-1;
        *n_x = new_x;
        *n_y = new_y;
        
        // 2. No reward or jumps if new position is same as old
        if (x == new_x && y == new_y) return NULL;
        
        // 3. If no goal, no reward or jump
        if (NULL == (g = goal_at(new_x, new_y))) return NULL;
            
        // 4. Implement jumps at reward squares
        *n_x = orient_value(g->get_newx(), n);
        *n_y = orient_value(g->get_newy(), n);
        
        // 5. Return goal
        return g;
    }
    
    void reset()
    {  
        for (int i = 0; i < n*n; ++i) squares[i]->reset();
    }
    
    int suggest(int x, int y)
    { 
        return square(x, y)->suggest();
    }
    
    virtual int perturb(void) { return 0; }
    virtual void restore(void) {  }
    virtual const char* name(void)    const { return "Grid"; }
    virtual const char* initials(void) const { return "GR"; }
    
    virtual int reinit(void) {
        reset();
        restore();
        return 1;
    }

    void picture(ostream& out, bool arrow=true, int which=DRAW_QMAX)
    {
        // 1. Output the initial picture element
        out << "    \\begin{picture}(" << n << "," << n << ")" << endl;
        out << "    \\thicklines" << endl;
        
        // 2. Loop for every row
        for (int y = 0; y < n; ++y)
        {
            // 3. Loop for every column
            for (int x = 0; x < n; ++x)
            {
                // 4. Have square draw itself
                square(x, y)->draw(out, arrow, which);
            }
        }
        
        // 9. Output the final picture element
        out << "    \\end{picture}" << endl;
    }
};

// ====================================================================
//                                                               Chippy
// n by n Grid with rewards bottom-left, top-right and random jumping
// ====================================================================
class Chippy : public Grid
{
    int r1;
    int r2;
    Goal *g3[3];
    
public:
    Chippy(int n=8, int rr1=10, int rr2=-10) : Grid(n)
    {
        // 1. Save the reward values
        r1 = rr1;
        r2 = rr2;
        
        // 2. Create the goals
        g3[0] = new Goal(r1, LOC_MIN, LOC_MIN, LOC_RAN, LOC_RAN);
        g3[1] = new Goal(r2, LOC_MAX, LOC_MAX, LOC_RAN, LOC_RAN);
        g3[2] = NULL;
        
        // 3. Add them to the grid
        set_goals(g3);
    }

    virtual ~Chippy()
    {
        delete g3[0];
        delete g3[1];
    }

    int get_r1() { return r1; }
    int get_r2() { return r2; }
    Goal *get_g1() { return get_goals()[0]; }
    Goal *get_g2() { return get_goals()[1]; }
    virtual const char* name(void)    const { return "ChippyFixed"; }
    virtual const char* initials(void) const { return "CH"; }
};

// ====================================================================
//                                                        ChippyClassic
// n by n Grid with rewards bottom-left, top-right and random jumping
// ====================================================================
class ChippyClassic : public Chippy
{
public:
    ChippyClassic(int n=8, int r1=10, int r2=-10) 
    : Chippy(n, r1, r2) {
    }
     
    virtual int perturb() {
        
        // 1. Get the current reward values
        int cr1 = get_g1()->get_reward();
        int cr2 = get_g2()->get_reward();
        
        // 2. Swap the values in the goals
        get_g1()->set_reward(cr2);
        get_g2()->set_reward(cr1);
        
        // 3. Return 1 if perturbed
        if (get_r1() == cr1) return 1;
        return 0;
    }
    
    virtual void restore() 
    {
        get_g1()->set_reward(get_r1());
        get_g2()->set_reward(get_r2());
    }
    virtual const char* name(void)    const { return "ChippyClassic"; }
    virtual const char* initials(void) const { return "CL"; }
};

// ====================================================================
//                                                         ChippyCorner
// n by n Grid: rewards bottom-left, top-right and goal-to-goal jumping
// ====================================================================
class ChippyCorner : public ChippyClassic
{
    Goal *g3[3];
    
public:
    ChippyCorner(int n=8, int r1=10, int r2=-10) : ChippyClassic(n,r1,r2)
    {
        // 1. Create the goals
        g3[0] = new Goal(r1, LOC_MIN, LOC_MIN, LOC_MAX, LOC_MAX);
        g3[1] = new Goal(r2, LOC_MAX, LOC_MAX, LOC_MIN, LOC_MIN);
        g3[2] = NULL;
            
        // 2. Add them to the grid
        set_goals(g3);
    }
    
    ~ChippyCorner()
    {
        delete g3[0];
        delete g3[1];
    }

    virtual const char* name(void)    const { return "ChippyCorner"; }
    virtual const char* initials(void) const { return "CO"; }
};

// ====================================================================
//                                                         ChippyRotate
// n by n Grid: goal BL/TR --> TL/BR --> TR/BL --> BR/TL --> ...
// ====================================================================
class ChippyRotate : public Chippy
{
    int state;    
    
public:
    ChippyRotate(int n=8, int r1=10, int r2=-10) : Chippy(n,r1,r2) {
        move_goals_to(0);
    };

    void move_goals_to(int newstate)
    {
        // 1. Save new state
        state = newstate;
        
        // 2. Set locatation in goals
        get_goals()[0]->set_x(CLOCKWISE[state][0]);
        get_goals()[0]->set_y(CLOCKWISE[state][1]);
        int other = (state+2) % 4;
        get_goals()[1]->set_x(CLOCKWISE[other][0]);
        get_goals()[1]->set_y(CLOCKWISE[other][1]);
        
        // 3. Tell the grid
        set_goals(get_goals());
    }
    
    virtual int perturb() {
        
        // 1. Move goals
        move_goals_to((state + 1) % 4);
        
        // 2. Return new state
        return state;
    }
    
    virtual void restore() 
    {
        // 1. Start back at the beginning
        move_goals_to(0);        
    }
    virtual const char* name(void)    const { return "ChippyRotate"; }
    virtual const char* initials(void) const { return "CR"; }
};


// ====================================================================
//                                                               Walker
// A grid crawling agent
// ====================================================================
class Walker
{
protected:
    double score;
    int    count;
    Grid  *grid;
    int    x;
    int    y;
    int    startx;
    int    starty;
    int    last_x;
    int    last_y;
    int    verbose;
public:
    Walker(Grid *g = NULL, int sx = LOC_CTR, int sy = LOC_CTR)
    {    
            score   = 0;
            count   = 0;
            grid    = g;
            start_at(sx, sy);
    }
    virtual ~Walker()
    {
    }

    int     get_score()    const { return score; }
    int     get_count()    const { return count; }
    Square* square(int x, int y) const { return grid->square(x,y); }
    int     get_x()        const { return x; }
    int     get_y()        const { return y; }
    Grid*   get_grid()     const { return grid; }
    void    set_grid(Grid *g) {
        grid = g;
    }
    int     get_last_x()        const { return last_x; }
    int     get_last_y()        const { return last_y; }
    void set_verbose(int v) {
        verbose = v;
    }
    
    void start_at(int sx=LOC_CTR, int sy=LOC_CTR) 
    {
        if (grid == NULL)
        {
            if (sx < 0 || sy < 0)
            {
                startx = 0;
                starty = 0;
            }
            else
            {
                startx = sx;
                starty = sy;
            }
        }
        else
        {
            startx = orient_value(sx, grid->get_n());
            starty = orient_value(sy, grid->get_n());
        }
        x = startx;
        y = starty;
        last_x = x;
        last_y = y;
    }
        
    virtual Goal* move(int dir=-1)
    {
        // Move to the given, best, or random direction"
        int new_x;
        int new_y;
        Goal* g;
        
        //  1. Pick a direction if none given
        if (-1 == dir)
        {    
            dir = suggest();
        }
        
        // 2. Move and get reward and new location
        g = grid->move(x, y, dir, &new_x, &new_y);
        
        // 3. Set new location
        last_x = x;
        last_y = y;
        x = new_x;
        y = new_y;
        
        // 4. Count the number of moves
        ++count;

        // 5. Accumulate the reward
        if (g != NULL)
            score += g->get_reward();
        
        // 6. Return goal achieved (if any)
        return g;
    }
    
    int suggest()
    {
        return grid->suggest(x, y);
    }
    
    virtual void reset()
    {
        grid->reset();
    }
    
    virtual const char* name(void)    const { return "Walker"; }
    virtual const char* initials(void) const { return "WA"; }
    
    virtual int reinit(void) {
        count = 0;
        score = 0;
        start_at(startx, starty);
        if (grid)
            return grid->reinit();
        else return 1;
    }

    void picture(ostream& out, bool arrow=true, int which=DRAW_QMAX)
    {
        grid->picture(out, arrow, which);
    }
    
};

// ====================================================================
//                                                             QLearner
// A grid walker that learns
// ====================================================================
#define MIN_EPSILON 0.05
#define MAX_EPSILON 0.99
class QLearner : public Walker
{
    double alpha;
    double gamma;
    double epsilon;
    double start_alpha;
    double start_gamma;
    double start_epsilon;
    int    policy_number;
    
public:
    QLearner(Grid *gr = NULL, int sx = LOC_CTR, int sy = LOC_CTR, 
             double a = 0.5, double g = 0.9, double e = 0.05) 
        : Walker(gr, sx, sy)
    {
        alpha   = a;
        gamma   = g;
        epsilon = e;
        start_alpha   = a;
        start_gamma   = g;
        start_epsilon = e;
        policy_number = 0;
    }
    
    double get_epsilon()    const { return epsilon; }
    double get_alpha()      const { return alpha; }
    double get_gamma()      const { return gamma; } 
    int    get_policy_number() const {
        return policy_number;
    }
    void set_epsilon(double e)     { epsilon = e; }
    void set_alpha(double a)       { alpha = a; }
    void set_gamma(double g)       { gamma = g; }
    
    virtual Goal* move(int dir=-1)
    {
        // Move in the direction with the best expected value or explore
        int reward = 0;
        
        // 1. Remember the previous location
        int prev_x = get_x();
        int prev_y = get_y();
        
        // 2. Get suggested direction
        if (dir == -1)
        {
            dir = suggest();
            
            // 3. If exploring, get a random direction
            if ((epsilon*10000) > (rand() % 10000))
            { 
                dir = rand() % DIR_NUM;
            }
        }
        
        // 4. Move in specified direction     
        Goal* goal = Walker::move(dir);
        if (goal != NULL) reward = goal->get_reward(); 

        // 5. Adjust the action expected rewards
        double newQsa = qreward(dir, prev_x, prev_y, reward, get_x(), get_y());
        square(prev_x, prev_y)->set_q(dir, newQsa);
        
        // 6. Return goal (if any)
        return goal;
    }
    
    double qreward(int a, int s_x, int s_y, int r, int sp_x, int sp_y)
    {
        //Spread out the reward over the past move
        double oldQsa = square(s_x, s_y)->get_q(a);
        double maxQspap = square(sp_x, sp_y)->max();
        double newQsa = oldQsa + alpha*(r + (gamma*maxQspap) - oldQsa);
        //printf("Q([%d,%d],%d) = %f = %f + %f(%f + %f*%f-%f)\n",
        //       s_x, s_y, a, newQsa, oldQsa, alpha, r, gamma, maxQspap, oldQsa);
        return newQsa; 
    }
    virtual const char* name(void)    const { return "QLearner"; }
    virtual const char* initials(void) const { return "QL"; }
    virtual int reinit(void) {
        alpha   = start_alpha;
        gamma   = start_gamma;
        epsilon = start_epsilon;
        policy_number = 0;
        return Walker::reinit();
    }
    void increase_epsilon(double e){
        epsilon += e;
        if (epsilon > MAX_EPSILON) epsilon = MAX_EPSILON;
        if (epsilon < MIN_EPSILON) epsilon = MIN_EPSILON;
    }
    void decrease_epsilon(double e)
    {
        increase_epsilon(-e);
    }
    void increment_policy()
    {
        epsilon = start_epsilon;
        ++policy_number;
        Walker::reset();
    }
};        


// ====================================================================
//                                                          Expectation
// Base class for hand coded MCL exceptions
// ====================================================================
class Expectation
{
    int number;
public:
    Expectation() {
        number = 0;
    }
    virtual ~Expectation()
    {
    }

    virtual int check() {
        return 0;
    }
    virtual int check(Goal *g) {
        return 0;
    }
    
    virtual int get_x() const {
        return -1;
    }
    virtual int get_y() const {
        return -1;
    }
    
    void set_number(int n) {
        number = n;
    }
    int get_number(void) {
        return number;
    }
};

// ====================================================================
//                                                  RewardAtExpectation
// Expectation for receiving a reward at a square
// ====================================================================
class RewardAtExpectation : public Expectation
{
    int reward;
    int atx;
    int aty;
public:
    RewardAtExpectation(int r, int x, int y) {
        reward = r;
        atx = x;
        aty = y;
    }
    
    virtual int get_reward() const {
        return reward;
    } 
    virtual int get_x() const {
        return atx;
    }
    virtual int get_y() const {
        return aty;
    }
    virtual int check(Goal *g) {
        if (g == NULL) return 0;
        if (atx != g->get_ox()) return 0;
        if (aty != g->get_oy()) return 0;
        if (reward == g->get_reward()) return 0;
        return 1;
    }
};

// ====================================================================
//                                                         Expectations
// Bag of expecations
// ====================================================================
class Expectations
{
    Expectation **expect;
    int numexp;
    int maxexp;
public:
    Expectations(int maxnum = MAX_EXPECTATIONS) {
        numexp = 0;
        maxexp = maxnum;
        expect = (Expectation **)calloc(maxnum, sizeof(Expectation *));
    }
    
    virtual ~Expectations() {
        clear();    
    }
    
    virtual void add(Expectation *exp) {
        if (numexp < maxexp) {
            expect[numexp] = exp;
            ++numexp;
            exp->set_number(numexp);
        }
    }
    virtual int check(Goal *g) {
        for (int i = 0; i < numexp; ++i) {
            if (expect[i]->check(g)) {
                return 1;
            }
        }
        return 0;
    }
    virtual void clear() {
        for (int i = 0; i < numexp; ++i)
            delete expect[i];
        numexp = 0;
    }
    virtual Expectation * at(int x, int y) {
        for (int i = 0; i < numexp; ++i) {
            if ((x == expect[i]->get_x()) &&
                (y == expect[i]->get_y())) {
                return expect[i];
            }
        }
        return NULL;
    }
};

// ====================================================================
//                                                          QLMCLSimple
// A grid walker that learns with a modest amount of meta-congnition
// ====================================================================
class QLMCLSimple : public QLearner
{
protected:    
    Expectations* expectations;
    int    violations;
    int    threshold;
    int    start_threshold;  
    int    resets;
public:
        QLMCLSimple(Grid *gr = NULL, int th = 3,
                    int sx = LOC_CTR, int sy = LOC_CTR, 
                    double a = 0.5, double g = 0.9, double e = 0.05) 
            : QLearner(gr, sx, sy, a, g, e)
    {
        expectations = new Expectations();
        violations = 0;
        resets = 0;
        threshold  = th;    
        start_threshold  = th;
        verbose = 0;
    }
    virtual ~QLMCLSimple()
    {
        delete expectations;
    }
    
    int get_violations(void) {
        return violations;
    }
    int get_resets(void) {
        return resets;
    }
    int get_threshold(void) {
        return threshold;
    }
    
    virtual Goal* move(int dir = -1)
    {
        RewardAtExpectation* expectation;
        int reward;
        int exp_reward;
        int x;
        int y;
        
        // 1. Move according to what we have learned
        Goal* goal = QLearner::move(dir);
        
        // 2. If threshold is -1, we don't do MCL
        if (-1 == threshold) return goal;

        // 3. Get reward value and location
        if (NULL == goal) {
            reward = 0;
            x = get_x();
            y = get_y();
        } else {
            reward = goal->get_reward();
            x = goal->get_ox();
            y = goal->get_oy();
        }
        if (verbose && (reward != 0)) {
            cout << "step " << get_count() <<": reward = " << reward 
                 << " at (" << x << "," << y << ") ";
        }
        
        // 4. Else get reward from goalWhich reward is this?
        expectation = (RewardAtExpectation *) expectations->at(x, y);
        if (expectation == NULL) {
            exp_reward = 0;
        } else {
            if ((x == get_last_x()) && (y == get_last_y())) {
                exp_reward = 0;
            } else {
                exp_reward = expectation->get_reward();
            } 
        }
        if (verbose && (reward != 0)) {
            cout << "step " << get_count() << ": ";
            if (expectation == NULL) cout << "No expectation" << endl;
            else cout << "Expectation of " << expectation->get_reward() << endl;
        }
        
        // 5. Just store the reward if this is the first time
        if ((NULL == expectation) && (reward != 0))
        {
            expectation = new RewardAtExpectation(reward, x, y);
            expectations->add((Expectation*)expectation);
            if (verbose) {
                cout << "step " << get_count() << ": " 
                << "Adding expectation of reward " << reward 
                << " at (" << x << "," << y << ")" << endl;
            }
            return goal;
        }
        
        // 6. Note: If this is the expected reward, all is well
        if (reward == exp_reward)
        {
            if (expectation) {
                violations = 0;
                if (verbose) {
                    cout << "step " << get_count() << ": " 
                    << "Got expected reward of " << reward << " at (" 
                    << x << "," << y << ") Resetting violations" << endl;
                }
            }
            return goal;
        }
        
        // 7. Assess: Increment the number of violations
        ++violations;
        if (verbose) {
            cout << "step " << get_count() << ": " 
            << "Got reward of " << reward   
            << " at (" << x << "," << y << ")" 
            << " instead of " << exp_reward << endl;
            cout << "Incremented violations to " << violations << endl;
        }
        
        // 8. Guide: If too many, reset the learner
        if (violations >= threshold)
        {
            if (verbose) {
                cout << "Resetting, violations (" << violations 
                << ") >= threshold of " << threshold << endl;    
            }
            reset();
            increment_policy();
        }
        
        // 9. Return this reward, and hope for better days
        return goal;
    }
    
    virtual void reset() {
        if (verbose) {
            cout << "step " << get_count() << ": MCLSimple::reset()" << endl;
        }
        violations = 0;
        ++resets;
        expectations->clear();
    }
    
    virtual const char* name(void)    const { return "MCLSimple"; }
    virtual const char* initials(void) const { return "SI"; }
    virtual int reinit(void) {
        violations = 0;
        resets = 0;
        threshold  = start_threshold; 
        expectations->clear();
        return QLearner::reinit();
    }
};        

// ====================================================================
//                                                       QLMCLSensitive
// A grid walker that learns with a modest amount of meta-congnition
// ====================================================================
class QLMCLSensitive : public QLMCLSimple
{
    int expectedState;
    int expectedReward;
    int totalReward;
    double performance;
    double highPerformance;
    int lastRewardTurn;
    double rewardDistance;
    int numRewards;
    int actionNumber;
    double averageReward;
    
public:
    QLMCLSensitive(Grid *gr = NULL, int th = 3,
                int sx = LOC_CTR, int sy = LOC_CTR, 
                double a = 0.5, double g = 0.9, double e = 0.05) 
    : QLMCLSimple(gr, th, sx, sy, a, g, e)    {
        expectedState = EXPECTED_STATE_UNKNOWN;
        expectedReward = EXPECTED_REWARD_UNKNOWN;
        highPerformance = 0.0;
        performance = 0.0;
        totalReward = 0;
        lastRewardTurn = 0;
        rewardDistance = 0;
        numRewards = 0;
        actionNumber = 0;
    }

    virtual int reinit(void) {
        Reset();
        return QLMCLSimple::reinit();
    }
    
    virtual Goal* move(int dir = -1)
    {
        RewardAtExpectation* expectation;
        int reward;
        int x;
        int y;
        
        // 1. Move according to what we have learned
        Goal* goal = QLearner::move(dir);
        
        // 2. If threshold is -1, we don't do MCL
        if (-1 == threshold) return goal;
        
        // 3. Get reward value and location
        if (NULL == goal) {
            reward = 0;
            x = get_x();
            y = get_y();
        } else {
            reward = goal->get_reward();
            x = goal->get_ox();
            y = goal->get_oy();
        }
        if (verbose && (reward != 0)) {
            cout << "step " << get_count() <<": reward = " << reward 
            << " at (" << x << "," << y << ") ";
        }
        
        // 4. Else get reward from goal?
        expectation = (RewardAtExpectation *) expectations->at(x, y);
        if (expectation == NULL) {
            expectedReward = EXPECTED_REWARD_UNKNOWN;
        } else {
            if ((x == get_last_x()) && (y == get_last_y())) {
                expectedReward = 0;
            } else {
                expectedReward = expectation->get_reward();
            } 
        }
        if (verbose && (reward != 0)) {
            cout << "step " << get_count() << ": ";
            if (expectation == NULL) cout << "No expectation" << endl;
            else cout << "Expectation of " << expectedReward << endl;
        }
        
        // 5. Just store the reward if this is the first time
        if ((NULL == expectation) && (reward != 0))
        {
            expectation = new RewardAtExpectation(reward, x, y);
            expectations->add((Expectation*)expectation);
            if (verbose) {
                cout << "step " << get_count() << ": " 
                << "Adding expectation of reward " << reward 
                << " at (" << x << "," << y << ")" << endl;
            }
            return goal;
        }
        
        // 6. Invoke MCL
        expectedState = x*100+y;
        Compare(x*100+y, reward, get_count());
        
        // 9. Return this reward, and hope for better days
        return goal;
    }
    
    void Reset(void)
    {    
        if (verbose) {
            cout << "MCLSensitive::Reset()" << endl;
        }
        expectations->clear();
        reset();
        resets += 1;
        
        performance = 0.0;
        highPerformance = 0.0;
        totalReward = 0;
        lastRewardTurn = 0;
        rewardDistance = 0;
        numRewards = 0;
        actionNumber = 0;
        expectedState = EXPECTED_STATE_UNKNOWN;
        expectedReward = EXPECTED_REWARD_UNKNOWN;
    }
    
    virtual void Note(int inReward) 
    {
        ++violations;
    }
    
    virtual void Assess(int inType, int inReward)
    {
        if (verbose) {
            cout << "Assess(" << inType << ", " << inReward << ")" << endl;
            cout << "  averageReward = " << averageReward
                << ", expectedReward = " << expectedReward << endl;
        }
        if (violations > threshold)
            {
                if (verbose) {
                    cout << "Assess: violation (" << violations
                    << ") > threshold of " << threshold
                    << ", increment_policy and reset" << endl;
                }
                increment_policy();
                Reset();
            }
        
    } // end Assess
    
    virtual void Guide(void)
    {
        decrease_epsilon(0.0003);
    }
    
    virtual void Compare(int inState, int inReward, int inTurn)
    {
        int pType;
        
        // 1. Assume that nothing is wrong
        pType = PERTURB_NONE;

        // 2. Increase MCL action number and decrement counters
        ++actionNumber;
               
        // 3. Check for Perturbation 1
        //    It has been a long time since the last reward
        if (inReward != EXPECTED_REWARD_UNKNOWN)
        {
            if (inReward == 0)
            {
                if (((actionNumber - lastRewardTurn) > (3 * rewardDistance))
                    && (actionNumber > MIN_ACTION_NUMBER))
                {
                    if (verbose) {
                        cout << "Perturbation 1: "
                        << "actionNumber = " << actionNumber
                        << ", lastRewardTurn = " << lastRewardTurn
                        << ", rewardDistance = " << rewardDistance
                        << endl;
                    }
                    Note(inReward);
                    pType = PERTURB_TTR;
                }
            } else {
                ++numRewards;
                totalReward += inReward;
                lastRewardTurn = actionNumber;
            }
        }
        
        if (numRewards > 0) {
            rewardDistance = actionNumber / double(numRewards);
            averageReward = totalReward / double(numRewards);
        }
        
        performance = totalReward / double(actionNumber);
        
        // 4. Check for perturbation 2
        //    Overall performance has dropped below 80% of high performance
        if (actionNumber > MIN_ACTION_NUMBER) 
        {
            if (performance > highPerformance) 
                highPerformance = performance;
            else if ((highPerformance != 0) &&
                     (performance < (highPerformance * 0.8)))
            {
                if (verbose) {
                    cout << "Perturbation 2: " 
                    << "actionNumber = " << actionNumber
                    << ", performance = " << performance
                    << ", highPerformance = " << highPerformance
                    << endl;
                }
                Note(inReward);
                pType = PERTURB_PRF;
            }
        }
        
        // 5. Perturbation 3 
        //    Ending up in an unexpected state or with an unexpected rewar
        if ( ( (expectedState != EXPECTED_STATE_UNKNOWN) &&
               (expectedReward != EXPECTED_REWARD_UNKNOWN)) && 
             ( (expectedState != inState) ||
               (expectedReward != inReward))) {
            if (actionNumber > MIN_ACTION_NUMBER) {
                if (verbose) {
                    cout << "Perturbation 3: " 
                    << "actionNumber = " << actionNumber
                    << ", expectedState = " << expectedState
                    << ", inState = " << inState
                    << ", expectedReward = " << expectedReward
                    << ", inReward = " << inReward
                    << endl;
                }
                Note(inReward);
                pType = PERTURB_EXP;
            } 
        }

        // 6. If there has been a perturbation, assess it
        if (pType != PERTURB_NONE) 
        {
            if (verbose) {
                cout << "Compare(" << inState 
                << ", " << inReward
                << ", " << inTurn 
                << ") = " << pType << endl;
            }
            Assess(pType, inReward);
        }
        
        // 7. Guide 
        Guide();
    }    
    virtual const char* name(void)    const { return "MCLSensitive"; }
    virtual const char* initials(void) const { return "SE"; }
    
};

// ====================================================================
//                                                   QLMCLSophisticated
// A grid walker that learns with a modest amount of meta-congnition
// Based on Visual Basic version: mcl_module3.cls
// ====================================================================
class QLMCLSophisticated : public QLMCLSimple
{
    int mvarMCL_threshold;
    int mvarMCL_excitation;
    int expectedState;
    int expectedReward;
    int totalReward;
    double performance;
    double highPerformance;
    int lastRewardTurn;
    double rewardDistance;
    int numRewards;
    int actionNumber;
    int countdown1;
    int countdown2;
    int lastPerturbation;
    double degreePerturbation;
    double averageReward;
    int negReward;
    int negRewardSet;
public:
    QLMCLSophisticated(Grid *gr = NULL, int th = 3,
                   int sx = LOC_CTR, int sy = LOC_CTR, 
                   double a = 0.5, double g = 0.9, double e = 0.05) 
    : QLMCLSimple(gr, th, sx, sy, a, g, e)
    {
        expectedState = EXPECTED_STATE_UNKNOWN;
        expectedReward = EXPECTED_REWARD_UNKNOWN;
        mvarMCL_excitation = 0;
        mvarMCL_threshold = 3;
        highPerformance = 0.0;
        performance = 0.0;
        totalReward = 0;
        lastRewardTurn = 0;
        rewardDistance = 0;
        numRewards = 0;
        actionNumber = 0;
        countdown1 = 0;
        countdown2 = 0;
        lastPerturbation = 0;
        degreePerturbation = 0.0;
        negReward = 0;
        negRewardSet = 0;
    }
    virtual const char* name(void)    const { return "MCLSophisticated"; }
    virtual const char* initials(void) const { return "SO"; }
    virtual int reinit(void) {
        Reset();
        return QLMCLSimple::reinit();
    }

    virtual Goal* move(int dir = -1)
    {
        RewardAtExpectation* expectation;
        int reward;
        int x;
        int y;
        
        // 1. Move according to what we have learned
        Goal* goal = QLearner::move(dir);
        
        // 2. If threshold is -1, we don't do MCL
        if (-1 == threshold) return goal;
        
        // 3. Get reward value and location
        if (NULL == goal) {
            reward = 0;
            x = get_x();
            y = get_y();
        } else {
            reward = goal->get_reward();
            x = goal->get_ox();
            y = goal->get_oy();
        }
        if (verbose && (reward != 0)) {
            cout << "step " << get_count() <<": reward = " << reward 
            << " at (" << x << "," << y << ") ";
        }
        
        // 4. Else get reward from goal?
        expectation = (RewardAtExpectation *) expectations->at(x, y);
        if (expectation == NULL) {
            expectedReward = EXPECTED_REWARD_UNKNOWN;
        } else {
            if ((x == get_last_x()) && (y == get_last_y())) {
                expectedReward = 0;
            } else {
                expectedReward = expectation->get_reward();
            } 
        }
        if (verbose && (reward != 0)) {
            cout << "step " << get_count() << ": ";
            if (expectation == NULL) cout << "No expectation" << endl;
            else cout << "Expectation of " << expectedReward << endl;
        }
        
        // 5. Just store the reward if this is the first time
        if ((NULL == expectation) && (reward != 0))
        {
            expectation = new RewardAtExpectation(reward, x, y);
            expectations->add((Expectation*)expectation);
            if (verbose) {
                cout << "step " << get_count() << ": " 
                << "Adding expectation of reward " << reward 
                << " at (" << x << "," << y << ")" << endl;
            }
            return goal;
        }
        
        // 6. Invoke MCL
        expectedState = x*100+y;
        Compare(x*100+y, reward, get_count());
            
        // 7. Return this reward, and hope for better days
        return goal;
    }

    void Reset(void)
    {    
        if (verbose) {
            cout << "MCLSophisticated::Reset()" << endl;
        }
        expectations->clear();
        reset();
        resets += 1;

        mvarMCL_excitation = 0;
        performance = 0.0;
        highPerformance = 0.0;
        totalReward = 0;
        lastRewardTurn = 0;
        rewardDistance = 0;
        numRewards = 0;
        actionNumber = 0;
        expectedState = EXPECTED_STATE_UNKNOWN;
        expectedReward = EXPECTED_REWARD_UNKNOWN;
        lastPerturbation = 0;
        degreePerturbation = 0.0;
        negReward = 0;
        negRewardSet = 0;
    }
    
    virtual void Note(int inReward) 
    {
        ++mvarMCL_excitation;
        
        if (((actionNumber - lastPerturbation) > 300) &&
            (lastPerturbation != 0))
        {
                --mvarMCL_excitation;
                degreePerturbation -= 0.02;
                if (degreePerturbation < 0)
                    degreePerturbation = 0;
        }
        
        lastPerturbation = actionNumber;
        if (verbose) {
            cout << "Note: lastPerturbation = " << lastPerturbation
            << ", degreePerturbation = " << degreePerturbation
            << ", mvarMCL_excitation = " << mvarMCL_excitation
            << endl;
        }
    }

    virtual void Assess(int inType, int inReward)
    {
        if (verbose) {
            cout << "Assess(" << inType << ", " << inReward << ")" << endl;
            cout << "  neg/avg/exp/Reward = " << negReward 
                 << "/" << averageReward
                << "/" << expectedReward << endl;
        }
        if (negReward)
        {
            switch(inType)
            {
                case PERTURB_TTR:
                case PERTURB_PRF:    
                    increase_epsilon(0.2);
                    degreePerturbation += 2;
                    break;
                case PERTURB_EXP:
                    if ((expectedReward < 0) &&
                        (inReward > 0)) // valence change - to +
                    {
                        if (inReward > averageReward) {
                            degreePerturbation += 8;       
                        } else {
                            increase_epsilon(0.3);
                            degreePerturbation += 3;
                        }
                    } else if ((expectedReward > 0) &&
                        (inReward < 0)) // valence change + to -
                    {
                        if (expectedReward > averageReward) {
                            increase_epsilon(0.3);
                            degreePerturbation += 3;
                        } else {
                            increase_epsilon(0.2);
                            degreePerturbation += 2;
                        }
                    } else { // both rewards positive
                        if ((expectedReward > averageReward) && 
                            (expectedReward > inReward)) {
                            if ((double(inReward) / double(expectedReward)) < 0.75) {
                                increase_epsilon(0.3);
                                degreePerturbation += 3;
                            } else {
                                increase_epsilon(0.1);
                                degreePerturbation += 1;
                            }  
                        } else {
                            increase_epsilon(0.1);
                            degreePerturbation += 1;
                        }
                    }
                    break;    
            } // end switch
            if (degreePerturbation > 7)
            {
                if (verbose) {
                    cout << "Assess: negReward - degreePerturbation > 7"
                    << ", increment_policy and reset" << endl;
                }
                increment_policy();
                Reset();
            }
            if (mvarMCL_excitation >= mvarMCL_threshold)
            {
                if (verbose) cout << "excitation ("
                    << mvarMCL_excitation 
                    << ") > threshold ("
                    << mvarMCL_threshold
                    << "), reset" << endl;
                Reset();
            }
        } else { // end if (negReward)
            switch(inType)
            {
                case PERTURB_TTR:
                case PERTURB_PRF:    
                    degreePerturbation += 2;
                    break;
                case PERTURB_EXP:
                    if ((expectedReward < 0) &&
                        (inReward > 0)) // valence change - to +
                    {
                        degreePerturbation += 4;
                    }
                    else if ((expectedReward > 0) &&
                          (inReward < 0)) // valence change + to -
                    {
                        if (expectedReward > averageReward) {
                            degreePerturbation += 3;
                        } else {
                            degreePerturbation += 2;
                        }
                    } else { // both rewards positive
                        if ((expectedReward > averageReward) && 
                            (expectedReward > inReward)) 
                        {
                            if ((double(inReward) / double(expectedReward)) < 0.75) {
                                degreePerturbation += 3;
                            } else {
                                degreePerturbation += 1;
                            }  
                        } else {
                            degreePerturbation += 1;
                        }
                    }
                    break;    
            } // end switch
            if (degreePerturbation > 7)
            {
                if (verbose) {
                    cout << "Assess: !negReward - degreePerturbation > 7"
                    << ", increment_policy and reset" << endl;
                }
                increment_policy();
                Reset();
            }
            if (mvarMCL_excitation >= mvarMCL_threshold)
            {
                increase_epsilon(double(degreePerturbation) / 10.0);
                Reset();
            }
        } // end else (negReward)
        if (verbose) {
            cout << "Assess: degreePerturbation = " << degreePerturbation
            << ", MCL_excitation = " << mvarMCL_excitation << endl;
        }
    } // end Assess

    virtual void Guide(void)
    {
        decrease_epsilon(0.0003);
    }
        
    virtual void Compare(int inState, int inReward, int inTurn)
    {
        int pType;
        
        // 1. Assume that nothing is wrong
        pType = PERTURB_NONE;
        
        // 2. Increase MCL action number and decrement counters
        ++actionNumber;
        if (countdown1 > 0) --countdown1;
        if (countdown2 > 0) --countdown2;
                
        // 3. Check for Perturbation 1
        //    It has been a long time since the last reward
        if (inReward != EXPECTED_REWARD_UNKNOWN)
        {
            if (inReward == 0)
            {
                if (((actionNumber - lastRewardTurn) > (3 * rewardDistance)) &&
                    (countdown1 = 0) && (actionNumber > MIN_ACTION_NUMBER))
                {
                    if (verbose) {
                        cout << "Perturbation 1: "
                        << "actionNumber = " << actionNumber
                        << ", lastRewardTurn = " << lastRewardTurn
                        << ", rewardDistance = " << rewardDistance
                        << endl;
                    }
                    Note(inReward);
                        pType = PERTURB_TTR;
                        countdown1 = 50;
                }
            } else {
                ++numRewards;
                totalReward += inReward;
                lastRewardTurn = actionNumber;
            }
        }
        
        if (numRewards > 0) {
            rewardDistance = actionNumber / double(numRewards);
            averageReward = totalReward / double(numRewards);
        }
        
        performance = totalReward / double(actionNumber);
        
        // 4. Check for perturbation 2
        //    Overall performance has dropped below 80% of high performance
        if (actionNumber > MIN_ACTION_NUMBER) 
        {
            if (performance > highPerformance) 
                highPerformance = performance;
            else if ((highPerformance != 0) &&
                     (performance < (highPerformance * 0.8)) &&
                     (countdown2 == 0))
            {
                if (verbose) {
                    cout << "Perturbation 2: " 
                    << "actionNumber = " << actionNumber
                    << ", performance = " << performance
                    << ", highPerformance = " << highPerformance
                    << endl;
                }
                Note(inReward);
                pType = PERTURB_PRF;
                countdown2 = 50;
            }
        }
        
        // 5. Perturbation 3 
        //    Ending up in an unexpected state or with an unexpected reward
        if ((expectedState == EXPECTED_STATE_UNKNOWN) ||
            (expectedReward == EXPECTED_REWARD_UNKNOWN)) {
            if (mvarMCL_excitation < 0) mvarMCL_excitation = 0;
        } else if ((expectedState != inState) ||
                   (expectedReward != inReward)) {
            if (actionNumber > MIN_ACTION_NUMBER) {
                if (verbose) {
                    cout << "Perturbation 3: "
                    << "actionNumber = " << actionNumber
                    << ", expectedState = " << expectedState
                    << ", inState = " << inState
                    << ", expectedReward = " << expectedReward
                    << ", inReward = " << inReward
                    << endl;
                }
                Note(inReward);
                pType = PERTURB_EXP;
            } 
        } else {
            if (mvarMCL_excitation < 0) mvarMCL_excitation = 0;
        }
        
        // 6. If there has been a perturbation, assess it
        if (pType != PERTURB_NONE) 
        {
            if (verbose) {
                cout << "Compare(" << inState 
                << ", " << inReward
                << ", " << inTurn 
                << ") = " << pType << endl;
            }
            Assess(pType, inReward);
        }
        
        // 7. Guide 
        Guide();
        
        // 8. Remember valiance of reward
        if (!negRewardSet)
        {
            if ((inReward < 0) && 
                (inReward != EXPECTED_REWARD_UNKNOWN))
                {
                    negReward = 1;
                    negRewardSet = 1;
                }
            if (actionNumber > MIN_ACTION_NUMBER)
            {
                negRewardSet = 1;
            }
        }
    }
};

// ====================================================================
//                                                          QLMCLBayes1
// A grid walker that learns with a modest amount of meta-congnition
// ====================================================================
class QLMCLBayes1 : public QLMCLSimple
{
    double sensors[10];
    int    expected[5];
#ifdef USEMCL2
    mclMA::observables::update _update;
    string mcl_key;
#endif
public:
    QLMCLBayes1(Grid *gr = NULL, int th = 3,
                   int sx = LOC_CTR, int sy = LOC_CTR, 
                   double a = 0.5, double g = 0.9, double e = 0.05) 
    : QLMCLSimple(gr, th, sx, sy, a, g, e)
    {

#ifdef USEMCL2
        // 1. Introduce ourselves to MCL
        mcl_key = "QLMCLBayes1";
        mclMA::setOutput("QLMCLBayes1.html");
        mclMA::initializeMCL(mcl_key, 0); 
        
        // 2. Define properties
        mclMA::setPropertyDefault(mcl_key, PCI_INTENTIONAL,         PC_NO);
        mclMA::setPropertyDefault(mcl_key, PCI_EFFECTORS_CAN_FAIL,  PC_NO);
        mclMA::setPropertyDefault(mcl_key, PCI_SENSORS_CAN_FAIL,    PC_NO);
        mclMA::setPropertyDefault(mcl_key, PCI_PARAMETERIZED,       PC_YES);
        mclMA::setPropertyDefault(mcl_key, PCI_DECLARATIVE,         PC_NO);
        mclMA::setPropertyDefault(mcl_key, PCI_RETRAINABLE,         PC_YES);
        mclMA::setPropertyDefault(mcl_key, PCI_HLC_CONTROLLING,     PC_NO);
        mclMA::setPropertyDefault(mcl_key, PCI_HTN_IN_PLAY,         PC_NO);
        mclMA::setPropertyDefault(mcl_key, PCI_PLAN_IN_PLAY,        PC_NO);
        mclMA::setPropertyDefault(mcl_key, PCI_ACTION_IN_PLAY,      PC_NO);
        
        mclMA::setPropertyDefault(mcl_key, CRC_IGNORE,              PC_YES);
        mclMA::setPropertyDefault(mcl_key, CRC_NOOP,                PC_YES);
        mclMA::setPropertyDefault(mcl_key, CRC_TRY_AGAIN,           PC_YES);
        mclMA::setPropertyDefault(mcl_key, CRC_SOLICIT_HELP,        PC_NO);
        mclMA::setPropertyDefault(mcl_key, CRC_RELINQUISH_CONTROL,  PC_NO);
        mclMA::setPropertyDefault(mcl_key, CRC_SENSOR_DIAG,         PC_NO); 
        mclMA::setPropertyDefault(mcl_key, CRC_EFFECTOR_DIAG,       PC_NO);
        mclMA::setPropertyDefault(mcl_key, CRC_ACTIVATE_LEARNING,   PC_NO); 
        mclMA::setPropertyDefault(mcl_key, CRC_ADJ_PARAMS,          PC_NO); 
        mclMA::setPropertyDefault(mcl_key, CRC_REBUILD_MODELS,      PC_YES);
        mclMA::setPropertyDefault(mcl_key, CRC_REVISIT_ASSUMPTIONS, PC_NO); 
        mclMA::setPropertyDefault(mcl_key, CRC_AMEND_CONTROLLER,    PC_NO);
        mclMA::setPropertyDefault(mcl_key, CRC_REVISE_EXPECTATIONS, PC_NO); 
        mclMA::setPropertyDefault(mcl_key, CRC_ALG_SWAP,            PC_NO); 
        mclMA::setPropertyDefault(mcl_key, CRC_CHANGE_HLC,          PC_NO);
        
        // 3. Define the sensors and property values
        mclMA::observables::declare_observable_self(mcl_key, "step",    0.0);     // [0]
        mclMA::observables::declare_observable_self(mcl_key, "reward",  0.0);   // [1]
        mclMA::observables::declare_observable_self(mcl_key, "expect1", 0.0);  // [2]
        mclMA::observables::declare_observable_self(mcl_key, "expect2", 0.0);  // [3]
        mclMA::observables::declare_observable_self(mcl_key, "expect3", 0.0);  // [4]
        mclMA::observables::declare_observable_self(mcl_key, "expect4", 0.0);  // [5]
        
        // 4. Define the property values for the sensors
        mclMA::observables::set_obs_prop_self(mcl_key, "step",     
                             PROP_DT, DT_INTEGER); // [0]
        mclMA::observables::set_obs_prop_self(mcl_key, "reward",   
                             PROP_DT, DT_INTEGER); // [1]
        mclMA::observables::set_obs_prop_self(mcl_key, "expect1",  
                             PROP_DT, DT_INTEGER); // [2]
        mclMA::observables::set_obs_prop_self(mcl_key, "expect2",  
                             PROP_DT, DT_INTEGER); // [3]
        mclMA::observables::set_obs_prop_self(mcl_key, "expect3",  
                             PROP_DT, DT_INTEGER); // [4]
        mclMA::observables::set_obs_prop_self(mcl_key, "expect4",  
                             PROP_DT, DT_INTEGER); // [5]
        
        mclMA::observables::set_obs_prop_self(mcl_key, "step",     
                             PROP_SCLASS, SC_TEMPORAL); // [0]
        mclMA::observables::set_obs_prop_self(mcl_key, "reward",  
                             PROP_SCLASS, SC_REWARD);   // [1] 
        mclMA::observables::set_obs_prop_self(mcl_key, "expect1", 
                             PROP_SCLASS, SC_REWARD);   // [2]
        mclMA::observables::set_obs_prop_self(mcl_key, "expect2", 
                             PROP_SCLASS, SC_REWARD);   // [3]
        mclMA::observables::set_obs_prop_self(mcl_key, "expect3", 
                             PROP_SCLASS, SC_REWARD);   // [4]
        mclMA::observables::set_obs_prop_self(mcl_key, "expect4",  
                             PROP_SCLASS, SC_REWARD);   // [5]
        
        // 5. Define the expectation group.  
        //    We will add the expectations when we get the rewards.
        mclMA::declareExpectationGroup(mcl_key, EGK);
#endif
        sensors[0] = 0;
    }
    
    virtual const char* name(void)    const { return "MCLBayes1"; }
    virtual const char* initials(void) const { return "B1"; }

#ifdef USEMCL2
    ~QLMCLBayes1()
    {
        mclMA::releaseMCL(mcl_key);
    }
    
    virtual Goal* move(int dir = -1)
    {
        int reward;
        int expectedReward;
        int expectedNumber;
        RewardAtExpectation* expectation;
        int x;
        int y;
        
        // 1. Move according to what we have learned
        Goal* goal = QLearner::move(dir);
        
        // 2. If threshold is -1, we don't do MCL
        if (-1 == threshold) return goal;
        
        // 3. Get reward value and location
        if (NULL == goal) {
            reward = 0;
            x = get_x();
            y = get_y();
        } else {
            reward = goal->get_reward();
            x = goal->get_ox();
            y = goal->get_oy();
        }
        if (verbose && (reward != 0)) {
            cout << "step " << get_count() <<": reward = " << reward 
            << " at (" << x << "," << y << ") " << endl;
        }
        
        // 4. What was our expected reward?
        expectation = (RewardAtExpectation *) expectations->at(x, y);
        if (expectation == NULL) {
            expectedReward = EXPECTED_REWARD_UNKNOWN;
            expectedNumber = 0;
        } else {
            if ((x == get_last_x()) && (y == get_last_y())) {
                expectedReward = 0;
                expectedNumber = 0;
            } else {
                expectedReward = expectation->get_reward();
                expectedNumber = expectation->get_number();
            } 
        }
        if (verbose && (reward != 0)) {
            cout << "step " << get_count() << ": ";
            if (expectation == NULL) cout << "No expectation" << endl;
            else cout << "Expectation of " << expectedReward << endl;
        }
        
        // 5. Just store the reward if this is the first time
        if ((NULL == expectation) && (reward != 0)) {
            char sensor_name[15];
            expectation = new RewardAtExpectation(reward, x, y);
            expectations->add((Expectation*)expectation);
            sprintf(sensor_name, "expect%d", expectation->get_number());
            expectedNumber = expectation->get_number();
            if (verbose) {
                cout << "step " << get_count() << ": " 
                << "Added expectation of reward " << reward 
                << " at (" << x << "," << y << ")" 
                << " named " << sensor_name  
                << " number " << expectedNumber << endl;
            }
            expected[expectation->get_number()] = reward;
            mclMA::declareExpectation(mcl_key, EGK, 
                                       sensor_name, 
                                       EC_MAINTAINVALUE, 
                                       (float) reward);
        }
        
        // 6. Set values in update object
        sensors[0] = get_count();
        sensors[1] = reward;
        sensors[2] = expected[1];
        sensors[3] = expected[2];
        sensors[4] = expected[3];
        sensors[5] = expected[4];
        if (expectedNumber > 0) {
            sensors[1+expectedNumber] = reward;
        }
        _update.set_update("step",    sensors[0]);
        _update.set_update("reward",  sensors[1]);
        _update.set_update("expect1", sensors[2]);
        _update.set_update("expect2", sensors[3]);
        _update.set_update("expect3", sensors[4]);
        _update.set_update("expect4", sensors[5]);

        // 7. Tell MCL what we know
        responseVector rv = mclMA::monitor(mcl_key, _update);
        
        // 8. Evaluate the suggestions from MCL
        processSuggestions(rv);
        
        // 9. Return the goal
        return goal;
    }
    
    void processSuggestions(responseVector& m) {
        if (m.size() > 0) {
            for (responseVector::iterator rvi = m.begin();
                 rvi!=m.end();
                 rvi++) {
                mclMonitorResponse *r = (mclMonitorResponse *)(*rvi);
                if (verbose) {
                    cout << "step " << get_count()
                         << ": " << r->rclass();
                    if (r->requiresAction()) cout << " Action";
                    if (r->recommendAbort()) cout << " Abort";
                    cout << " " << r->responseText() << endl;
                    cout << "step " << get_count() << ": ";
                }
                processSuggestion(r);
            } // end for
        } else {
            decrease_epsilon(0.0003);
        }// end if
    } // end processSuggestions

    void processSuggestion(mclMonitorResponse *r) {
        if (r->rclass() == "internalError") 
            processSuggestionInternalError((mclInternalErrorResponse*)r);
        else if (r->rclass() == "noAnomalies")
            processSuggestionOK((mclMonitorOKResponse*)r);
        else if (r->rclass() == "noOperation")
            processSuggestionNOOP((mclMonitorNOOPResponse*)r);
        else if (r->rclass() == "suggestion")
            processSuggestionCorrective((mclMonitorCorrectiveResponse*)r);
        else {
            if (verbose) 
                cout << "Unknown MCL monitor response ("
                     << r->rclass() << ")" << endl;
        }
    } // end processSuggestion

    void processSuggestionInternalError(mclInternalErrorResponse* r)
    {
        if (verbose)
            cout << "mclInternalErrorResponse" << endl;
    } // end processSuggestionInternalError 

    void processSuggestionOK(mclMonitorOKResponse* r)
    {
        if (verbose)
            cout << "mclMonitorOKResponse" << endl;
    } // end processSuggestionOK 

    void processSuggestionNOOP(mclMonitorNOOPResponse* r)
    {
        if (verbose)
            cout << "mclMonitorNOOPResponse" << endl;
    } // end processSuggestionNOOP 


    void processSuggestionCorrective(mclMonitorCorrectiveResponse* r)
    {
        if (verbose)
            cout << "mclMonitorCorrectiveResponse" << endl;

        switch (r->responseCode()) {
            case CRC_IGNORE:
                if (verbose) cout << "Suggestion: Ignore" << endl;
                mclMA::suggestionImplemented(mcl_key, r->referenceCode());
                break;
            case CRC_NOOP:
                if (verbose) cout << "Suggestion: No Op" << endl;
                mclMA::suggestionImplemented(mcl_key, r->referenceCode());
                break;
            case CRC_TRY_AGAIN:
                if (verbose) cout << "Suggestion: Try Again" << endl;
                mclMA::suggestionImplemented(mcl_key, r->referenceCode());
                break;
            case CRC_REBUILD_MODELS:
                if (verbose) cout << "Suggestion: Rebuild Models" << endl;
                reset();
                increment_policy();
                mclMA::declareExpectationGroup(mcl_key, EGK);
                mclMA::suggestionImplemented(mcl_key, r->referenceCode());
                break;
            default:
                cout << "Unexpected Suggestion: ["
                     << r->responseCode() << "] "
                     << r->responseText() << endl;
                if (r->requiresAction()) {
                    mclMA::suggestionIgnored(mcl_key, r->referenceCode());
                } // end if requiresAction
        } // end switch
    } // end processConcreteSuggestion
    
    virtual int reinit(void) {
        reset();
        mclMA::declareExpectationGroup(mcl_key, EGK);
        return QLMCLSimple::reinit();
    }
    
    void resetExpectationGroup(void) {
        mclMA::expectationGroupAborted(mcl_key, EGK);
        for (int i=0; i < 5; ++i) {
            expected[i] = 0;
        }
    }

    void reset(void) {    
        if (verbose) {
            cout << "step " << get_count() << ": MCLBayes1::reset()" << endl;
        }
        resetExpectationGroup();
        QLMCLSimple::reset();
    }
    
#endif    
};


    
// ====================================================================
//                                                          QLMCLBayes2
// A grid walker that learns with a modest amount of meta-congnition
// ====================================================================
class QLMCLBayes2 : public QLMCLSimple
{
    float sensors[10];
    int   total_rewards;
    int   count_rewards;
    int   reward_steps;
    int   last_reward_step;
    bool   expectations_set;
#ifdef USEMCL2
    mclMA::observables::update _update;
    string mcl_key;
#endif
    
public:
    QLMCLBayes2(Grid *gr = NULL, int th = 3,
                int sx = LOC_CTR, int sy = LOC_CTR, 
                double a = 0.5, double g = 0.9, double e = 0.05) 
    : QLMCLSimple(gr, th, sx, sy, a, g, e)
    {
#ifdef USEMCL2
        // 1. Introduce ourselves to MCL
        mcl_key = "QLMCLBayes2";
        mclMA::setOutput("QLMCLBayes2.html");
        mclMA::initializeMCL(mcl_key, 0); 
        
        // 2. Define properties
        mclMA::setPropertyDefault(mcl_key, PCI_INTENTIONAL,         PC_NO);
        mclMA::setPropertyDefault(mcl_key, PCI_EFFECTORS_CAN_FAIL,  PC_NO);
        mclMA::setPropertyDefault(mcl_key, PCI_SENSORS_CAN_FAIL,    PC_NO);
        mclMA::setPropertyDefault(mcl_key, PCI_PARAMETERIZED,       PC_YES);
        mclMA::setPropertyDefault(mcl_key, PCI_DECLARATIVE,         PC_NO);
        mclMA::setPropertyDefault(mcl_key, PCI_RETRAINABLE,         PC_YES);
        mclMA::setPropertyDefault(mcl_key, PCI_HLC_CONTROLLING,     PC_NO);
        mclMA::setPropertyDefault(mcl_key, PCI_HTN_IN_PLAY,         PC_NO);
        mclMA::setPropertyDefault(mcl_key, PCI_PLAN_IN_PLAY,        PC_NO);
        mclMA::setPropertyDefault(mcl_key, PCI_ACTION_IN_PLAY,      PC_NO);
        
        mclMA::setPropertyDefault(mcl_key, CRC_IGNORE,              PC_YES);
        mclMA::setPropertyDefault(mcl_key, CRC_NOOP,                PC_YES);
        mclMA::setPropertyDefault(mcl_key, CRC_TRY_AGAIN,           PC_YES);
        mclMA::setPropertyDefault(mcl_key, CRC_SOLICIT_HELP,        PC_NO);
        mclMA::setPropertyDefault(mcl_key, CRC_RELINQUISH_CONTROL,  PC_NO);
        mclMA::setPropertyDefault(mcl_key, CRC_SENSOR_DIAG,         PC_NO); 
        mclMA::setPropertyDefault(mcl_key, CRC_EFFECTOR_DIAG,       PC_NO);
        mclMA::setPropertyDefault(mcl_key, CRC_ACTIVATE_LEARNING,   PC_YES); 
        mclMA::setPropertyDefault(mcl_key, CRC_ADJ_PARAMS,          PC_NO); 
        mclMA::setPropertyDefault(mcl_key, CRC_REBUILD_MODELS,      PC_YES);
        mclMA::setPropertyDefault(mcl_key, CRC_REVISIT_ASSUMPTIONS, PC_NO); 
        mclMA::setPropertyDefault(mcl_key, CRC_AMEND_CONTROLLER,    PC_NO);
        mclMA::setPropertyDefault(mcl_key, CRC_REVISE_EXPECTATIONS, PC_YES); 
        mclMA::setPropertyDefault(mcl_key, CRC_ALG_SWAP,            PC_NO); 
        mclMA::setPropertyDefault(mcl_key, CRC_CHANGE_HLC,          PC_NO);

        // 3. Define the sensors and property values
        mclMA::observables::declare_observable_self(mcl_key, "step",    0.0);  // [0]
        mclMA::observables::declare_observable_self(mcl_key, "reward",  0.0);  // [1]
        mclMA::observables::declare_observable_self(mcl_key, "valperf", 0.0);  // [2]
        mclMA::observables::declare_observable_self(mcl_key, "kntperf", 0.0);  // [3]
        mclMA::observables::declare_observable_self(mcl_key, "lastrwd", 0.0);  // [4]
        
        // 4. Define the property values for the sensors
        mclMA::observables::set_obs_prop_self(mcl_key, "step",     PROP_DT, DT_INTEGER);  // [0]
        mclMA::observables::set_obs_prop_self(mcl_key, "reward",   PROP_DT, DT_INTEGER);  // [1]
        mclMA::observables::set_obs_prop_self(mcl_key, "valperf",  PROP_DT, DT_RATIONAL); // [2]
        mclMA::observables::set_obs_prop_self(mcl_key, "kntperf",  PROP_DT, DT_RATIONAL); // [3]
        mclMA::observables::set_obs_prop_self(mcl_key, "lastrwd",  PROP_DT, DT_INTEGER);  // [4]
        
        mclMA::observables::set_obs_prop_self(mcl_key, "step",     PROP_SCLASS, SC_TEMPORAL); // [0]
        mclMA::observables::set_obs_prop_self(mcl_key, "reward",   PROP_SCLASS, SC_REWARD);   // [1]
        mclMA::observables::set_obs_prop_self(mcl_key, "valperf",  PROP_SCLASS, SC_REWARD);   // [2]
        mclMA::observables::set_obs_prop_self(mcl_key, "kntperf",  PROP_SCLASS, SC_REWARD);   // [3]
        mclMA::observables::set_obs_prop_self(mcl_key, "lastrwd",  PROP_SCLASS, SC_TEMPORAL); // [4]
        
        // 5. Define the expectation group.  
        //    We will add the expectations when we get the rewards.
        mclMA::declareExpectationGroup(mcl_key, EGK);
#endif
        sensors[0] = 0;
        total_rewards = 0;
        count_rewards = 0;
        reward_steps = 0;
        last_reward_step = 0;
        expectations_set  = false;
    }
    
    virtual const char* name(void)    const { return "MCLBayes2"; }
    virtual const char* initials(void) const { return "B2"; }
    
#ifdef USEMCL2
    ~QLMCLBayes2()
    {
        // We should call mclMA::terminateMCL();
        // but no such function exists 
        mclMA::reSetDefaultPV(mcl_key);
        mclMA::expectationGroupAborted(mcl_key, EGK);
    }

    void set_expectations(float val_perf, float knt_perf)
    {
        mclMA::declareExpectation(mcl_key, EGK, 
                                   "valperf", 
                                   EC_STAYOVER, 
                                   (float) 0.85*val_perf);
        mclMA::declareExpectation(mcl_key, EGK, 
                                   "kntperf", 
                                   EC_STAYUNDER, 
                                   (float) 1.5*knt_perf);
        mclMA::declareExpectation(mcl_key, EGK, 
                                   "lastrwd", 
                                   EC_STAYUNDER, 
                                   (float) 10.0*knt_perf);
        expectations_set = true;
        if (verbose) {
            cout << "Step " << get_count()
            << " setting expectations " 
            << " valperf > " << 0.85*val_perf
            << " kntperf < " << 1.5*knt_perf
            << " lastrwd < " << 10.0*knt_perf
            << endl;
        }
    }

    virtual Goal* move(int dir = -1)
    {
        int reward;
        
        // 1. Move according to what we have learned
        Goal* goal = QLearner::move(dir);
        
        // 2. If threshold is -1, we don't do MCL
        if (-1 == threshold) return goal;
        
        // 3. Total and count rewards
        if (NULL != goal) {
            reward = goal->get_reward();
            if (verbose && (reward != 0)) {
                cout << "step " << get_count() 
                     << ": reward = " << reward 
                     << " at (" << goal->get_ox() 
                     << "," << goal->get_oy() << ") " << endl;
            }
            if (reward > 0) last_reward_step = get_count();
            if (reward != 0) {
                total_rewards += reward;
                count_rewards += 1;
            }
        }
        
        // 4. Compute performance numbers
        ++reward_steps;
        float val_perf = float(total_rewards) / float(reward_steps);
        float knt_perf = float(reward_steps) / float(count_rewards>0?count_rewards:1);
        if (verbose) {
            cout << "step " << get_count()
            << ": val_perf = " << val_perf
            << ", knt_perf = " << knt_perf
            << ", last_reward = " << get_count() - last_reward_step << endl;
        }
        
        // 5. Set values in sensor vector
        sensors[0] = get_count();
        sensors[1] = reward;
        sensors[2] = val_perf;
        sensors[3] = knt_perf;
        sensors[4] = get_count() - last_reward_step;
        
        // 6. Set expectations if needed
        if ((!expectations_set) && (reward_steps > MIN_ACTION_NUMBER))
        {
            set_expectations(val_perf, knt_perf);
        }
        
        // 7. Tell MCL what we know
        _update.set_update("step",    sensors[0]);
        _update.set_update("reward",  sensors[1]);
        _update.set_update("valperf", sensors[2]);
        _update.set_update("kntperf", sensors[3]);
        _update.set_update("lastrwd", sensors[4]);

        responseVector m = mclMA::monitor(mcl_key, _update);
        
        // 8. Evaluate the suggestions from MCL
        processSuggestions(m);
        
        // 9. Return the goal
        return goal;
    }
    
    void processSuggestions(responseVector& m) {
        if (m.size() > 0) {
            for (responseVector::iterator rvi = m.begin();
                 rvi!=m.end();
                 rvi++) {
                mclMonitorResponse *r = (mclMonitorResponse *)(*rvi);
                if (verbose) {
                    cout << "step " << get_count()
                         << ": " << r->rclass();
                    if (r->requiresAction()) cout << " Action";
                    if (r->recommendAbort()) cout << " Abort";
                    cout << " " << r->responseText() << endl;
                    cout << "step " << get_count() << ": ";
                }
                processSuggestion(r);
            } // end for
        } else {
            decrease_epsilon(0.0003);
        }// end if
    } // end processSuggestions

    void processSuggestion(mclMonitorResponse *r) {
        if (r->rclass() == "internalError") 
            processSuggestionInternalError((mclInternalErrorResponse*)r);
        else if (r->rclass() == "noAnomalies")
            processSuggestionOK((mclMonitorOKResponse*)r);
        else if (r->rclass() == "noOperation")
            processSuggestionNOOP((mclMonitorNOOPResponse*)r);
        else if (r->rclass() == "suggestion")
            processSuggestionCorrective((mclMonitorCorrectiveResponse*)r);
        else {
            if (verbose) 
                cout << "Unknown MCL monitor response ("
                     << r->rclass() << ")" << endl;
        }
    } // end processSuggestion

    void processSuggestionInternalError(mclInternalErrorResponse* r)
    {
        if (verbose)
            cout << "mclInternalErrorResponse" << endl;
    } // end processSuggestionInternalError 

    void processSuggestionOK(mclMonitorOKResponse* r)
    {
        if (verbose)
            cout << "mclMonitorOKResponse" << endl;
    } // end processSuggestionOK 

    void processSuggestionNOOP(mclMonitorNOOPResponse* r)
    {
        if (verbose)
            cout << "mclMonitorNOOPResponse" << endl;
    } // end processSuggestionNOOP 


    void processSuggestionCorrective(mclMonitorCorrectiveResponse* r)
    {
        if (verbose)
            cout << "mclMonitorCorrectiveResponse" << endl;

        switch (r->responseCode()) {
            case CRC_IGNORE:
                if (verbose) cout << "Suggestion: Ignore" << endl;
                mclMA::suggestionImplemented(mcl_key, r->referenceCode());
                break;
            case CRC_NOOP:
                if (verbose) cout << "Suggestion: No Op" << endl;
                mclMA::suggestionImplemented(mcl_key, r->referenceCode());
                break;
            case CRC_TRY_AGAIN:
                if (verbose) cout << "Suggestion: Try Again" << endl;
                mclMA::suggestionImplemented(mcl_key, r->referenceCode());
                break;
            case CRC_ACTIVATE_LEARNING:
                if (verbose) cout << "Suggestion: Activate Learning" << endl;
                if (get_epsilon() >= 0.5) {
                    mclMA::suggestionFailed(mcl_key, r->referenceCode());    
                } else {
                    increase_epsilon(0.1);
                    mclMA::suggestionImplemented(mcl_key, r->referenceCode());
                }
                break;
            case CRC_REBUILD_MODELS:
                if (verbose) cout << "Suggestion: Rebuild Models" << endl;
                reset();
                increment_policy();
                mclMA::declareExpectationGroup(mcl_key, EGK);
                mclMA::suggestionImplemented(mcl_key, r->referenceCode());
                break;
            case CRC_REVISE_EXPECTATIONS:
                if (verbose) cout << "Suggestion: Revise Expectations" << endl;
                resetExpectationGroup();                           
                mclMA::declareExpectationGroup(mcl_key, EGK);
                mclMA::suggestionImplemented(mcl_key, r->referenceCode());
                break;
            default:
                cout << "Unexpected Suggestion: ["
                     << r->responseCode() << "] "
                     << r->responseText() << endl;
                if (r->requiresAction()) {
                    mclMA::suggestionIgnored(mcl_key, r->referenceCode());
                } // end if requiresAction
        } // end switch
    } // end processConcreteSuggestion
    
    virtual int reinit(void) {
        reset();
        mclMA::declareExpectationGroup(mcl_key, EGK);
        return QLMCLSimple::reinit();
    }
    
    void resetExpectationGroup(void) {
        mclMA::expectationGroupAborted(mcl_key, EGK);
        total_rewards = 0;
        count_rewards = 0;
        reward_steps = 0;
        last_reward_step = get_count();
        expectations_set = false;
    }
    
    void reset(void) {    
        if (verbose) {
            cout << "step " << get_count() << ": MCLBayes2::reset()" << endl;
        }
        resetExpectationGroup();
        QLMCLSimple::reset();
    }
#endif    
};

// ====================================================================
//                                                       RollingAverage
// ====================================================================
class RollingAverage
{
    double  *values;
    int      n;
    int      index;
    int      count;
    double   total;
    
public:
        RollingAverage(int xn=ROLLING_AVERAGE_SIZE)
    {
            n      = xn;
            index  = 0;
            count  = 0;
            values = (double *) calloc(sizeof(double), n);
            values[0] = 0.0;
            total  = 0.0;
    }
    
    void add(double value)
    {
        if (count == n)
        {
            total = total - values[index];
        }
        else
        {
            ++count;
        }
        
        values[index] = value;
        total = total + value;
        ++index;
        if (index == n)
        {
            index = 0;
        }
    }
    
    int    get_n()        const { return n; }
    int    get_count()    const { return count; }
    double get_total()    const { return total; }
    
    double get_average()
    {    
        if (0 == count)
        {
            return 0.0;
        }
        else
        {
            return total / double(count);
        }
    }
};

// ====================================================================
//                                                              Rewards
// ====================================================================
class Rewards
{
    double  *values;
    int      n;
    int      index;
    int      count;
    double   total;
    char     initials[10];
    char     colname[25];
    char     rowname[25];
    
public:
        Rewards(int xn=1024)
    {
            n      = xn;
            index  = 0;
            count  = 0;
            values = (double *) calloc(sizeof(double),n);
            values[0] = 0.0;
            total = 0.0;
            strcpy(initials, "????");
            strcpy(colname, "????");
            strcpy(rowname, "????");
    }
    
    void append(double value)
    {
        if (index == n)
        {
            n = 2 * n;
            values = (double *) realloc(values, sizeof(double)*n);
        }
        values[index] = value;
        ++index;
        total += value;
        count = 1;
    }
    
    int    get_count()            const { return count; }
    int    get_index()            const { return index; }
    int    get_n()                const { return n; }
    double get_reward(int index)  const { return values[index]; }
    double get_average(int index) const { return values[index] / double(count); }
    double get_total()            const { return total; }
    char * get_initials()         { return initials; }
    char * get_colname()          { return colname; }
    char * get_rowname()          { return rowname; }
    void set_initials(const char*i1=NULL, const char *i2=NULL) 
    {
	    if (i1 == NULL)
	        if (i2 == NULL)
		        strcpy(initials, "????");
	        else
		        strcpy(initials, i2);
	    else {
	        strcpy(initials, i1);
	        if (i2 != NULL) {
                strcat(initials, ":");
		        strcat(initials, i2);
            }
	    }
    }
    void set_colname(const char *name) {
        strcpy(colname, name);
    }
    void set_rowname(const char *name) {
        strcpy(rowname, name);
    }
    
    void add(Rewards *other)
    {
        if (count == 0)
        {
            set(other);
        }
        else
        {
            for (int i = 0; i < index; ++i)
            {   
                values[i] = values[i] + other->get_reward(i);
            }
            ++count;
            total += other->get_total();
        }
    }
    
    void set(Rewards *other)
    {
        if (other->get_n() > n)
        {
            n = other->get_n();
            values = (double *) realloc(values, sizeof(double)*n);
        }
        index = other->get_index();
        for (int i = 0; i < index; ++i)
        {    
            values[i] = other->get_reward(i);
        }
        total = other->get_total();
        count = other->get_count();
    }
};

// ====================================================================
//                                                         grid_factory
// Return an initialized grid object based on grid number
// ====================================================================
Grid* grid_factory(int igrid) {
    switch(igrid) {
        case GRID_NONE: return NULL;
        case GRID_CHIPPY: return new Chippy();
        case GRID_CLASSIC: return new ChippyClassic();
        case GRID_CORNER: return new ChippyCorner();
        case GRID_ROTATE: return new ChippyRotate();
        case GRID_PCHIPPY: return new Chippy(8, 10, 5);
        case GRID_PCLASSIC: return new ChippyClassic(8, 10 ,5);
        case GRID_PCORNER: return new ChippyCorner(8, 10, 5);
        case GRID_PROTATE: return new ChippyRotate(8, 10, 5);
    }
    return NULL;
}

// ====================================================================
//                                                       walker_factory
// Return an initialized walker object based on walker number
// ====================================================================
Walker* walker_factory(int iwalk) {
    switch(iwalk) {
        case WALK_NONE: return NULL;
        case WALK_WALKER: return new Walker();
        case WALK_QLEARNER: return new QLearner();
        case WALK_SIMPLE: return new QLMCLSimple();
        case WALK_SENSITIVE: return new QLMCLSensitive();
        case WALK_SOPHISTICATED: return new QLMCLSophisticated();
        case WALK_BAYES1: return new QLMCLBayes1();
        case WALK_BAYES2: return new QLMCLBayes2();
    }
    return NULL;
}

// ====================================================================
//                                                         write_policy
// Output the policy that has been learned by the walker
// ====================================================================
void write_policy(const char *basename, Walker *w, int steps)
{
    char filename[256];
    
    sprintf(filename, "%s-%s-%s-%d.tex" ,
            basename, w->initials(), w->get_grid()->initials(), steps);
    ofstream tex(filename);
    
    w->picture(tex);
    w->picture(tex,false,DRAW_LOGV);
}

// ====================================================================
//                                                           experiment
// Do a single chippy experiment
// ====================================================================
Rewards *experiment(int steps=EXP_STEPS, 
                    int pstep=0,
                    int mult=0,
                    Walker *w=NULL,
                    const char *basename=NULL,
                    bool policy=false)
{
    RollingAverage *ravg;
    Rewards        *rwds;
    
    // 1. Create the objects
    steps += ROLLING_AVERAGE_SIZE;
    ravg = new RollingAverage();
    rwds = new Rewards(steps+1);
    rwds->set_colname(w->get_grid()->name());
    rwds->set_rowname(w->name());
    rwds->set_initials(w->initials(), w->get_grid()->initials());
    
    // 2. Walk a mile in chippy's shoes
    w->start_at();
    rwds->append(0.0);
    for (int step = 0; step <= steps; ++step)
    {
        // 3. Take a step
        Goal *goal = w->move();
        //cout << step << ": (" << w->get_x() << "," << w->get_y() << ") " << reward << endl;
        
        // 4. Record this reward in the averages
        int reward = (NULL==goal)?0:goal->get_reward();
        ravg->add(reward);
        
        // 5. Record the average in the results
        rwds->append(ravg->get_average());
        
        // 6. Switch the rewards if it is time
        if (((!mult) && step && (pstep == step)) ||
            (mult && step && pstep && (0 == (step%pstep))))
        {
            w->get_grid()->perturb();
            if (policy) write_policy(basename, w, step);
        }    
    }    
    
    // 7. Output final policy
    if (policy) write_policy(basename, w, steps);

    // 8. Delete the rolling average
    delete ravg;
    
    // 9. Return the average rewards at each step
    return rwds;
}     

void write_line(char *basename, Rewards* rwd, int steps, int skip=1) 
{
    // 1. Create Output files
    char filename[40];
    strcpy(filename, basename);
    strcat(filename, ".csv");
    ofstream out(filename, std::ios::app);

    // 2. Compute and output total reward
    float reward = 0.0;
    for (int i=0; i < steps; ++i)
        reward += rwd->get_average(i);
    out << reward;

    // 3. Output average rewards
    for (int step = 0; step <= steps; step += skip)
    {
        out << "," << rwd->get_average(step);
    }
    
    // 4. End off the line
    out << endl;
}

void write_lines(const char *basename, int kntw, int kntg, 
                 Rewards** rewards, int steps, int skip=1)
{
    // 1. Create Output files
    char filename[40];
    strcpy(filename, basename);
    strcat(filename, "l.csv");
    ofstream out(filename);
    
    // 2. Output column headers
    out << "step";
    for (int i=0; i < kntw*kntg; ++i)
        out << "," << rewards[i]->get_initials(); 
    out << endl;
    
    // 3. Loop for all of the steps and output step number
    for (int step = 0; step <= steps; step += skip)
    {
        out << step;
        
        // 4. Loop for all of the experiments
        for (Rewards **ri = rewards; *ri !=NULL; ++ri) {
            
            // 5. Output step average for one experiment
            out << "," << (*ri)->get_average(step);
        }
        
        // 6. End off the row for this step
        out << endl;
    }
}

void write_totals(const char *basename, int kntw, int kntg, 
                  Rewards** rewards, int steps)
{
    // 1. Create Output files
    char filename[256];
    strcpy(filename, basename);
    strcat(filename, "t.csv");
    ofstream out(filename);
    
    // 2. Output column headers
    out << "totals";
    for (int i=0; i < kntg; ++i)
        out << "," << rewards[i]->get_colname(); 
    out << endl;
    
    // 3. Loop for all of the Walkers
    Rewards** ri = rewards;
    for (int w = 0; w < kntw; ++w)
    {
        // 4. Output name of the Walker
        out << (*ri)->get_rowname();
        
        // 5. Loop for all of the grids
        for (int g = 0; g < kntg; ++g, ++ri) {
            
            // 6. Output step average for one experiment
            out << "," << (*ri)->get_total()/(*ri)->get_count();
        }
        
        // 7. End off the row for this walker
        out << endl;
    }
}

char *stripChippy(char *name) {
    return name+6;
}

void write_table_totals(const char *basename, int kntw, int kntg, 
                        Rewards** rewards, int steps)
{
    int g;
    int w;
    
    // 1. Create Output files
    char filename[256];
    strcpy(filename, basename);
    strcat(filename, "t.tex");
    ofstream out(filename);
    
    // 3. Output table header
    out << "\\begin{tabular}{|l";
    for (g = 0; g < kntg; ++g) out << "|r";
    out << "|}" << endl;
    out << "\\hline" << endl;
    
    // 4. Output names of grid worlds
    out << "\\emph{Chippy}";
    for (int i=0; i < kntg; ++i)
        out << " & \\emph{" 
            << stripChippy(rewards[i]->get_colname()) << "}"; 
    out << " \\\\" << endl;
    out << "\\hline" << endl;
    
    // 3. Loop for all of the Walkers
    Rewards** ri = rewards;
    for (w = 0; w < kntw; ++w)
    {
        // 4. Output name of the Walker
        out << "\\emph{" << (*ri)->get_rowname() << "}"; 
        
        // 5. Loop for all of the grids
        for (int g = 0; g < kntg; ++g, ++ri) {
            
            // 6. Output step average for one experiment
            out << " & " << setiosflags(ios::fixed) 
                << setprecision(0)  
                << (*ri)->get_total()/(*ri)->get_count();
        }
        
        // 7. End off the row for this walker
        out << " \\\\" << endl;
    }
    
    out << "\\hline" << endl;
    out << "\\end{tabular}" << endl;

}


// ====================================================================
//                                                          experiments
// Repeat the chippy experiment multiple times
// ====================================================================
void experiments(const char *basename,
                 int repeat=EXP_REPEAT, 
                 int steps=EXP_STEPS, 
                 int pstep=EXP_PERTURB, 
                 int mult=0,
                 int *walkers = NULL, Grid **grids = NULL)
{
    int *wi;
    Grid   **gi;
    Walker *w;
    Grid   *g;
    int     kntw = 0;
    int     kntg = 0;
    int     kntr = 0;
    Rewards** ri; 
    Rewards** rewards;
    int     i;
    
    // 1. Count the number of walkers and grids
    if (NULL != walkers) {
        for (wi = walkers; WALK_NONE != *wi; ++wi)
            ++kntw;
    }
    if (NULL != grids) {
        for (gi = grids; NULL != *gi; ++gi)
            ++kntg;
    }
    kntr = kntw *kntg;
    printf("%d walkers, %d grids, %d rewards\n", kntw, kntg, kntr);
    if (0 == kntr) return;
    
    // 2. Allocate and initialize rewards
    rewards = (Rewards**)calloc(1+kntr, sizeof(Rewards*));
    for (i = 0; i < kntr; ++i) {
        rewards[i] = new Rewards(steps);
    }
    rewards[kntr] = NULL;

    // 3. Loop for all the Walkers
    for (ri = rewards, wi = walkers; WALK_NONE != *wi; ++wi)
    {
        w = walker_factory(*wi);
        printf("walker %s\n", w->name());
        (*ri)->set_rowname(w->name());
               
        // 4. Loop for all the grids
        for (gi = grids; NULL !=*gi; ++gi, ++ri)
        {
            printf("  grid %s\n", (*gi)->name());
            (*ri)->set_colname((*gi)->name());
            (*ri)->set_initials(w->initials(), (*gi)->initials());
            
            // 5. Loop for several time to smooth the curves
            printf("    ");
            bool policy = true;
            for (int num = 0; num < repeat; ++num)
            {
                Rewards *result;
                
                cout << num << " ";
                cout.flush();
                
                // 6. Reinitialize a grid and walker for the repeated experiment
                g = *gi;
                g->reset();
                g->restore();
                w->set_grid(g);

                // 7. Run the experiment, saving the result
                result = experiment(steps, pstep, mult, w, basename, policy);
                (*ri)->add(result);
                delete result;
                policy = false;
            }
            cout << "OK" << endl;    
        }
        delete w;
    }

    // 8. Write the results files
    write_lines(basename, kntw, kntg, rewards, steps, 100);
    write_totals(basename, kntw, kntg, rewards, steps);
    write_table_totals(basename, kntw, kntg, rewards, steps);
    
    // 9. Release allocated storage
    for (i = 0; i < kntr; ++i) {
        delete rewards[i];
    }
    free(rewards);
}



// ====================================================================
//                                                           unit tests
// ====================================================================
void Testrandint();
void Testorient_value();
void TestSquare();
void TestSquare_testEmptyConstructor();
void TestSquare_testConstructor();
void TestSquare_testSuggestions();
void TestGoal();
void TestGoal_testEmptyConstructor();
void TestGoal_testConstructor();
void TestGrid();
void TestGrid_testEmptyConstructor();
void TestGrid_testConstructor();
void TestGrid_testMoves();
void TestGrid_testSquares();
void TestChippy();
void TestChippy_testEmptyConstructor();
void TestChippy_testConstructor();
void TestChippy_testMoves();
void TestChippy_testPerturb();
void TestChippy_testSquares();
void TestClassic();
void TestClassic_testEmptyConstructor();
void TestClassic_testConstructor();
void TestClassic_testMoves();
void TestClassic_testPerturb();
void TestClassic_testSquares();
void TestCorner();
void TestCorner_testEmptyConstructor();
void TestCorner_testConstructor();
void TestCorner_testMoves();
void TestCorner_testPerturb();
void TestCorner_testSquares();
void TestRotate();
void TestRotate_testEmptyConstructor();
void TestRotate_testConstructor();
void TestRotate_testMoves();
void TestRotate_testPerturb();
void TestRotate_testSquares();
void TestWalker();
void TestWalker_testEmptyConstructor();
void TestWalker_testConstructor();
void TestWalker_testMoves();
void TestQLearner();
void TestQLearner_testEmptyConstructor();
void TestQLearner_testConstructor();
void TestQLearner_testLearning();
void TestQLearner_test10k();
void TestQLMCLSimple();
void TestQLMCLSimple_testEmptyConstructor();
void TestQLMCLSimple_testConstructor();
void TestQLMCLSimple_testLearning();
void TestQLMCLSimple_testCH20k();
void TestQLMCLSimple_testCL10k();
void TestQLMCLSimple_testCO10k();
void TestQLMCLSimple_testCR10k();
void TestQLMCLSensitive();
void TestQLMCLSensitive_testEmptyConstructor();
void TestQLMCLSensitive_testConstructor();
void TestQLMCLSensitive_testLearning();
void TestQLMCLSensitive_testCH20k();
void TestQLMCLSensitive_testCL10k();
void TestQLMCLSensitive_testCO10k();
void TestQLMCLSensitive_testCR10k();
void TestQLMCLSophisticated();
void TestQLMCLSophisticated_testEmptyConstructor();
void TestQLMCLSophisticated_testConstructor();
void TestQLMCLSophisticated_testLearning();
void TestQLMCLSophisticated_testCH20k();
void TestQLMCLSophisticated_testCL10k();
void TestQLMCLSophisticated_testCO10k();
void TestQLMCLSophisticated_testCR10k();
void TestQLMCLBayes1();
void TestQLMCLBayes1_testEmptyConstructor();
void TestQLMCLBayes1_testConstructor();
void TestQLMCLBayes1_testLearning();
void TestQLMCLBayes1_testCH20k();
void TestQLMCLBayes1_testCL10k();
void TestQLMCLBayes1_testCO10k();
void TestQLMCLBayes1_testCR10k();
void TestQLMCLBayes2();
void TestQLMCLBayes2_testEmptyConstructor();
void TestQLMCLBayes2_testConstructor();
void TestQLMCLBayes2_testLearning();
void TestQLMCLBayes2_testCH20k();
void TestQLMCLBayes2_testCL10k();
void TestQLMCLBayes2_testCO10k();
void TestQLMCLBayes2_testCR10k();
void TestQLMCLBayes2_testCL10p5();
void TestRollingAverage();
void TestRollingAverage_testEmptyConstructor();
void TestRollingAverage_testConstructor();
void TestRollingAverage_testAverages();
void TestRewards();
void TestRewards_testEmptyConstructor();
void TestRewards_testConstructor();
void TestRewards_testAppend();
void TestRewards_testAdd();

void Testrandint();
void Testorient_value();
void TestSquare();
void TestSquare_testEmptyConstructor();
void TestSquare_testConstructor();
void TestSquare_testSuggestions();
void TestGoal();
void TestGoal_testEmptyConstructor();
void TestGoal_testConstructor();
void TestGrid();
void TestGrid_testEmptyConstructor();
void TestGrid_testConstructor();
void TestGrid_testMoves();
void TestGrid_testSquares();
void TestChippy();
void TestChippy_testEmptyConstructor();
void TestChippy_testConstructor();
void TestChippy_testMoves();
void TestChippy_testPerturb();
void TestChippy_testSquares();
void TestClassic();
void TestClassic_testEmptyConstructor();
void TestClassic_testConstructor();
void TestClassic_testMoves();
void TestClassic_testPerturb();
void TestClassic_testSquares();
void TestCorner();
void TestCorner_testEmptyConstructor();
void TestCorner_testConstructor();
void TestCorner_testMoves();
void TestCorner_testPerturb();
void TestCorner_testSquares();
void TestRotate();
void TestRotate_testEmptyConstructor();
void TestRotate_testConstructor();
void TestRotate_testMoves();
void TestRotate_testPerturb();
void TestRotate_testSquares();
void TestWalker();
void TestWalker_testEmptyConstructor();
void TestWalker_testConstructor();
void TestWalker_testMoves();
void TestQLearner();
void TestQLearner_testEmptyConstructor();
void TestQLearner_testConstructor();
void TestQLearner_testLearning();
void TestQLearner_test10k();
void TestQLMCLSimple();
void TestQLMCLSimple_testEmptyConstructor();
void TestQLMCLSimple_testConstructor();
void TestQLMCLSimple_testLearning();
void TestQLMCLSimple_testCH20k();
void TestQLMCLSimple_testCL10k();
void TestQLMCLSimple_testCO10k();
void TestQLMCLSimple_testCR10k();
void TestQLMCLSensitive();
void TestQLMCLSensitive_testEmptyConstructor();
void TestQLMCLSensitive_testConstructor();
void TestQLMCLSensitive_testLearning();
void TestQLMCLSensitive_testCH20k();
void TestQLMCLSensitive_testCL10k();
void TestQLMCLSensitive_testCO10k();
void TestQLMCLSensitive_testCR10k();
void TestQLMCLSophisticated();
void TestQLMCLSophisticated_testEmptyConstructor();
void TestQLMCLSophisticated_testConstructor();
void TestQLMCLSophisticated_testLearning();
void TestQLMCLSophisticated_testCH20k();
void TestQLMCLSophisticated_testCL10k();
void TestQLMCLSophisticated_testCO10k();
void TestQLMCLSophisticated_testCR10k();
void TestQLMCLBayes1();
void TestQLMCLBayes1_testEmptyConstructor();
void TestQLMCLBayes1_testConstructor();
void TestQLMCLBayes1_testLearning();
void TestQLMCLBayes1_testCH20k();
void TestQLMCLBayes1_testCL10k();
void TestQLMCLBayes1_testCO10k();
void TestQLMCLBayes1_testCR10k();
void TestQLMCLBayes2();
void TestQLMCLBayes2_testEmptyConstructor();
void TestQLMCLBayes2_testConstructor();
void TestQLMCLBayes2_testLearning();
void TestQLMCLBayes2_testCH20k();
void TestQLMCLBayes2_testCL10k();
void TestQLMCLBayes2_testCO10k();
void TestQLMCLBayes2_testCR10k();
void TestQLMCLBayes2_testCL10p5();
void TestRollingAverage();
void TestRollingAverage_testEmptyConstructor();
void TestRollingAverage_testConstructor();
void TestRollingAverage_testAverages();
void TestRewards();
void TestRewards_testEmptyConstructor();
void TestRewards_testConstructor();
void TestRewards_testAppend();
void TestRewards_testAdd();

void unittests()
{
    cout << "Running unittests ... " << endl;
    Testrandint();
    Testorient_value();
    TestSquare();
    TestGoal();
    TestGrid();
    TestClassic();
    TestCorner();
    TestRotate();
    TestWalker();
    TestQLearner();
    TestQLMCLSimple();
    TestQLMCLSensitive();
    TestQLMCLSophisticated();
    TestQLMCLBayes1();
    TestQLMCLBayes2();
    TestRollingAverage();
    TestRewards();
    cout << "OK" << endl;
}

void Testrandint()
{
    int value[11] = {0,0,0,0,0,0,0,0,0,0};
    cout << "  randint ... ";
    for (int i=0; i<1000; ++i) {
        int v = randint(0,10);
        assert(v >= 0);
        assert(v <= 10);
        value[v]++;
        v = randint(1,8);
        assert(v >= 1);
        assert(v <= 8);
    }
    for (int j=0; j<11; ++j) {
        assert(value[j] > 0);
    }
    cout << "OK" << endl;
}

void Testorient_value()
{
    cout << "  orient_value ... ";
    assert(orient_value(1,0) == 1);
    assert(orient_value(1,8) == 1);
    assert(orient_value(8,8) == 8);
    assert(orient_value(8,10) == 8);
    assert(orient_value(LOC_MIN,8) == 0);
    assert(orient_value(LOC_MIN,10) == 0);
    assert(orient_value(LOC_MAX,8) == 7);
    assert(orient_value(LOC_MAX,10) == 9);
    assert(orient_value(LOC_RAN,8) >= 1);
    assert(orient_value(LOC_RAN,10) >= 1);
    assert(orient_value(LOC_RAN,8) < 7);
    assert(orient_value(LOC_RAN,10) < 9);
    assert(orient_value(LOC_CTR,8) == 4);
    assert(orient_value(LOC_CTR,10) == 5);
    assert(orient_value(LOC_CTR,0) == 0);
    cout << "OK" << endl;
}

void TestSquare()
{
    cout << "  Square ... ";
    TestSquare_testEmptyConstructor();
    TestSquare_testConstructor();
    TestSquare_testSuggestions();
    cout << "OK" << endl;
}

void TestSquare_testEmptyConstructor()
{
    Square *s = new Square();
    assert(s->get_x() == 0);
    assert(s->get_y() == 0);
    delete s;
}

void TestSquare_testConstructor()
{
    Square *s = new Square(4, 3);
    assert(s->get_x() == 4);
    assert(s->get_y() == 3);
    delete s;
}

void TestSquare_testSuggestions()
{
    Square *s = new Square(4, 3);
    assert(s->get_x() == 4);
    assert(s->get_y() == 3);
    s->set_q(0, 3.4);
    assert(s->suggest() == 0);
    s->set_q(2, 4.3);
    assert(s->suggest() == 2);
    s->set_q(1, 1.2);
    assert(s->suggest() == 2);
    s->set_q(3, 1.2);
    assert(s->suggest() == 2);
    s->set_q(1, 4.3);
    int sug = s->suggest();
    assert(sug == 1 || sug == 2);
    delete s;
}

void TestGoal()
{
    cout << "  Goal ... ";
    TestGoal_testEmptyConstructor();
    TestGoal_testConstructor();
    cout << "OK" << endl;
}

void TestGoal_testEmptyConstructor()
{
    Goal *g = new Goal();
    assert(g->get_reward() == 0);
    assert(g->get_x() == 0);
    assert(g->get_y() == 0);
    assert(g->get_newx() == 0);
    assert(g->get_newy() == 0);
    assert(g->get_n() == 0);
    assert(g->get_ox() == 0);
    assert(g->get_oy() == 0);
    assert(g->get_jumpx() == 0);
    assert(g->get_jumpy() == 0);
    g->orient(8);
    assert(g->get_n() == 8);
    assert(g->get_ox() == 0);
    assert(g->get_oy() == 0);
    assert(g->get_jumpx() == 0);
    assert(g->get_jumpy() == 0);
    delete g;
}

void TestGoal_testConstructor()
{
    Goal *g = new Goal(1, 2, 3, 4, 5);
    assert(g->get_reward() == 1);
    assert(g->get_x() == 2);
    assert(g->get_y() == 3);
    assert(g->get_newx() == 4);
    assert(g->get_newy() == 5);
    assert(g->get_n() == 0);
    assert(g->get_ox() == 0);
    assert(g->get_oy() == 0);
    assert(g->get_jumpx() == 4);
    assert(g->get_jumpy() == 5);
    g->orient(8);
    assert(g->get_n() == 8);
    assert(g->get_ox() == 2);
    assert(g->get_oy() == 3);
    assert(g->get_jumpx() == 4);
    assert(g->get_jumpy() == 5);
    delete g;
}

void TestGrid()
{
    cout << "  Grid ... ";
    TestGrid_testEmptyConstructor();
    TestGrid_testConstructor();
    TestGrid_testMoves();
    TestGrid_testSquares();
    cout << "OK" << endl;
}

void TestGrid_testEmptyConstructor()
{
    Grid *g = new Grid();
    assert(g->get_n() == 8);
    assert(g->get_goals() == NULL);
    assert(g->goal_at(0,0) == NULL);
    delete g;
}

void TestGrid_testConstructor()
{
    Goal *g1 = new Goal(10,0,0,5,5);
    Goal *g2 = new Goal(-10,5,5,0,0);
    Goal *goals[3] = {g1, g2, NULL};
    
    Grid *g = new Grid(6, NULL);
    assert(g->get_n() == 6);
    assert(g->get_goals() == NULL);
    assert(g->goal_at(0,0) == NULL);
    g->set_goals(goals);
    assert(g->get_n() == 6);
    assert(g->get_goals() == (Goal **)&goals);
    assert(g->goal_at(0,0) == g1);
    assert(g->goal_at(1,1) == NULL);
    assert(g->goal_at(5,5) == g2);
    delete g;
    
    g = new Grid(6, goals);
    assert(g->get_n() == 6);
    assert(g->get_goals() == (Goal **)&goals);
    assert(g->goal_at(0,0) == g1);
    assert(g->goal_at(1,1) == NULL);
    assert(g->goal_at(5,5) == g2);
    delete g;
    
    delete g1;
    delete g2;
}

void TestGrid_testMoves()
{
    int new_x, new_y;
    Goal *g1 = new Goal(1,0,0,7,7);
    Goal *g2 = new Goal(2,7,7,0,0);
    Goal *goals[3] = {g1, g2, NULL};
    
    Grid *g = new Grid(8, goals);
    assert(g->get_goals() == (Goal **)&goals);
    assert(g->get_n()  == 8);
    assert(NULL == g->move(3, 3, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(3, 3, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(3, 3, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(3, 3, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(3, 7, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(6 == new_y);
    assert(NULL == g->move(3, 7, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 7, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 7, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 0, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(3, 0, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(1 == new_y);
    assert(NULL == g->move(3, 0, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(3, 0, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(7, 3, DIR_S, &new_x, &new_y));
    assert(7 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(7, 3, DIR_N, &new_x, &new_y));
    assert(7 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(7, 3, DIR_E, &new_x, &new_y));
    assert(7 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(7, 3, DIR_W, &new_x, &new_y));
    assert(6 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(0, 3, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(0, 3, DIR_N, &new_x, &new_y));
    assert(0 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(0, 3, DIR_E, &new_x, &new_y));
    assert(1 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(0, 3, DIR_W, &new_x, &new_y));
    assert(0 == new_x);
    assert(3 == new_y);
    assert(g1 == g->move(0, 1, DIR_S, &new_x, &new_y));
    assert(7 == new_x);
    assert(7 == new_y);
    assert(g1 == g->move(1, 0, DIR_W, &new_x, &new_y));
    assert(7 == new_x);
    assert(7 == new_y);
    assert(g2 == g->move(6, 7, DIR_E, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    assert(g2 == g->move(7, 6, DIR_N, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(0, 0, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(0, 0, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);

    g->set_goals(NULL);
    assert(NULL == g->move(1, 0, DIR_W, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(6, 7, DIR_E, &new_x, &new_y));
    assert(7 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(7, 6, DIR_N, &new_x, &new_y));
    assert(7 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(0, 0, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(0, 0, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);

    delete g;
    delete g1;
    delete g2;
}

void TestGrid_testSquares()
{
    Grid *g = new Grid(8);
    assert(g->get_n()  == 8);
    assert(g->square(3, 4)->get_x() == 3);
    assert(g->square(3, 4)->get_y() == 4);
    delete g;
}

void TestChippy()
{
    cout << "  Chippy ... ";
    TestChippy_testEmptyConstructor();
    TestChippy_testConstructor();
    TestChippy_testPerturb();
    TestChippy_testMoves();
    TestChippy_testSquares();
    cout << "OK" << endl;
}

void TestChippy_testEmptyConstructor()
{
    Chippy *c = new Chippy();
    assert(c->get_n() == 8);
    assert(c->get_goals() != NULL);
    assert(c->goal_at(0,0) != NULL);
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7) != NULL);
    assert(c->goal_at(7,7)->get_reward() == -10);
    delete c;
}

void TestChippy_testConstructor()
{
    Chippy *c = new Chippy(6,3,4);
    assert(c->get_n() == 6);
    assert(c->get_goals() != NULL);
    assert(c->goal_at(0,0) != NULL);
    assert(c->goal_at(0,0)->get_reward() == 3);
    assert(c->goal_at(5,5) != NULL);
    assert(c->goal_at(5,5)->get_reward() == 4);
    delete c;
}

void TestChippy_testPerturb()
{
    Chippy *c = new Chippy();
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    assert(0 == c->perturb());
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    assert(0 == c->perturb());
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    c->restore();
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    c->restore();
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    assert(0 == c->perturb());
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    c->restore();
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    assert(1 == c->perturb());
    assert(c->goal_at(0,0)->get_reward() == -10);
    assert(c->goal_at(7,7)->get_reward() == 10);
    delete c;
}

void TestChippy_testMoves()
{
    int new_x, new_y;
    Chippy *g = new Chippy();
    assert(g->get_n()  == 8);
    assert(g->goal_at(0,0)->get_reward() == 10);
    assert(g->goal_at(7,7)->get_reward() == -10);
    assert(NULL == g->move(3, 3, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(3, 3, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(3, 3, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(3, 3, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(3, 7, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(6 == new_y);
    assert(NULL == g->move(3, 7, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 7, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 7, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 0, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(3, 0, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(1 == new_y);
    assert(NULL == g->move(3, 0, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(3, 0, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(7, 3, DIR_S, &new_x, &new_y));
    assert(7 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(7, 3, DIR_N, &new_x, &new_y));
    assert(7 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(7, 3, DIR_E, &new_x, &new_y));
    assert(7 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(7, 3, DIR_W, &new_x, &new_y));
    assert(6 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(0, 3, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(0, 3, DIR_N, &new_x, &new_y));
    assert(0 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(0, 3, DIR_E, &new_x, &new_y));
    assert(1 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(0, 3, DIR_W, &new_x, &new_y));
    assert(0 == new_x);
    assert(3 == new_y);
    assert(g->get_goals()[0] == g->move(0, 1, DIR_S, &new_x, &new_y));
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(NULL != g->move(1, 0, DIR_W, &new_x, &new_y));
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(g->goal_at(7,7) == g->move(6, 7, DIR_E, &new_x, &new_y));
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(g->get_goals()[1] == g->move(7, 6, DIR_N, &new_x, &new_y));
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(NULL == g->move(0, 0, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(0, 0, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);

    assert(1 == g->perturb());
    assert(g->get_n()  == 8);
    assert(g->goal_at(0,0)->get_reward() == -10);
    assert(g->goal_at(7,7)->get_reward() == 10);
    assert(NULL == g->move(3, 3, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(3, 3, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(3, 3, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(3, 3, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(3, 7, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(6 == new_y);
    assert(NULL == g->move(3, 7, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 7, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 7, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 0, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(3, 0, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(1 == new_y);
    assert(NULL == g->move(3, 0, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(3, 0, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(7, 3, DIR_S, &new_x, &new_y));
    assert(7 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(7, 3, DIR_N, &new_x, &new_y));
    assert(7 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(7, 3, DIR_E, &new_x, &new_y));
    assert(7 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(7, 3, DIR_W, &new_x, &new_y));
    assert(6 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(0, 3, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(0, 3, DIR_N, &new_x, &new_y));
    assert(0 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(0, 3, DIR_E, &new_x, &new_y));
    assert(1 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(0, 3, DIR_W, &new_x, &new_y));
    assert(0 == new_x);
    assert(3 == new_y);
    assert(g->goal_at(0,0) == g->move(0, 1, DIR_S, &new_x, &new_y));
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(g->get_goals()[0] == g->move(1, 0, DIR_W, &new_x, &new_y));
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(NULL != g->move(6, 7, DIR_E, &new_x, &new_y));
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(g->goal_at(7,7) == g->move(7, 6, DIR_N, &new_x, &new_y));
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(NULL == g->move(0, 0, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(0, 0, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    delete g;
}

void TestChippy_testSquares()
{
    Chippy *g = new Chippy();
    assert(g->get_n()  == 8);
    assert(g->goal_at(0,0)->get_reward() == 10);
    assert(g->goal_at(7,7)->get_reward() == -10);
    assert(g->square(3, 4)->get_x() == 3);
    assert(g->square(3, 4)->get_y() == 4);
    delete g;
}

void TestClassic()
{
    cout << "  Chippy Classic ... ";
    TestClassic_testEmptyConstructor();
    TestClassic_testConstructor();
    TestClassic_testPerturb();
    TestClassic_testMoves();
    TestClassic_testSquares();
    cout << "OK" << endl;
}

void TestClassic_testEmptyConstructor()
{
    ChippyClassic *c = new ChippyClassic();
    assert(c->get_n() == 8);
    assert(c->get_goals() != NULL);
    assert(c->goal_at(0,0) != NULL);
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7) != NULL);
    assert(c->goal_at(7,7)->get_reward() == -10);
    assert(0 == strcmp(c->initials(), "CL"));
    assert(0 == strcmp(c->name(), "ChippyClassic"));
    delete c;
}

void TestClassic_testConstructor()
{
    ChippyClassic *c = new ChippyClassic(6,3,4);
    assert(c->get_n() == 6);
    assert(c->get_goals() != NULL);
    assert(c->goal_at(0,0) != NULL);
    assert(c->goal_at(0,0)->get_reward() == 3);
    assert(c->goal_at(5,5) != NULL);
    assert(c->goal_at(5,5)->get_reward() == 4);
    delete c;
}

void TestClassic_testPerturb()
{
    ChippyClassic *c = new ChippyClassic();
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    assert(1 == c->perturb());
    assert(c->goal_at(0,0)->get_reward() == -10);
    assert(c->goal_at(7,7)->get_reward() == 10);
    assert(0 == c->perturb());
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    c->restore();
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    c->restore();
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    assert(1 == c->perturb());
    assert(c->goal_at(0,0)->get_reward() == -10);
    assert(c->goal_at(7,7)->get_reward() == 10);
    c->restore();
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    assert(1 == c->perturb());
    assert(c->goal_at(0,0)->get_reward() == -10);
    assert(c->goal_at(7,7)->get_reward() == 10);
    delete c;
}

void TestClassic_testMoves()
{
    int new_x, new_y;
    ChippyClassic *g = new ChippyClassic();
    assert(g->get_n()  == 8);
    assert(g->goal_at(0,0)->get_reward() == 10);
    assert(g->goal_at(7,7)->get_reward() == -10);
    assert(NULL == g->move(3, 3, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(3, 3, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(3, 3, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(3, 3, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(3, 7, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(6 == new_y);
    assert(NULL == g->move(3, 7, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 7, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 7, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 0, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(3, 0, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(1 == new_y);
    assert(NULL == g->move(3, 0, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(3, 0, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(7, 3, DIR_S, &new_x, &new_y));
    assert(7 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(7, 3, DIR_N, &new_x, &new_y));
    assert(7 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(7, 3, DIR_E, &new_x, &new_y));
    assert(7 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(7, 3, DIR_W, &new_x, &new_y));
    assert(6 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(0, 3, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(0, 3, DIR_N, &new_x, &new_y));
    assert(0 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(0, 3, DIR_E, &new_x, &new_y));
    assert(1 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(0, 3, DIR_W, &new_x, &new_y));
    assert(0 == new_x);
    assert(3 == new_y);
    assert(g->get_goals()[0] == g->move(0, 1, DIR_S, &new_x, &new_y));
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(NULL != g->move(1, 0, DIR_W, &new_x, &new_y));
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(g->goal_at(7,7) == g->move(6, 7, DIR_E, &new_x, &new_y));
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(g->get_goals()[1] == g->move(7, 6, DIR_N, &new_x, &new_y));
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(NULL == g->move(0, 0, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(0, 0, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);

    assert(1 == g->perturb());
    assert(g->get_n()  == 8);
    assert(g->goal_at(0,0)->get_reward() == -10);
    assert(g->goal_at(7,7)->get_reward() == 10);
    assert(NULL == g->move(3, 3, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(3, 3, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(3, 3, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(3, 3, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(3, 7, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(6 == new_y);
    assert(NULL == g->move(3, 7, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 7, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 7, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 0, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(3, 0, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(1 == new_y);
    assert(NULL == g->move(3, 0, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(3, 0, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(7, 3, DIR_S, &new_x, &new_y));
    assert(7 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(7, 3, DIR_N, &new_x, &new_y));
    assert(7 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(7, 3, DIR_E, &new_x, &new_y));
    assert(7 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(7, 3, DIR_W, &new_x, &new_y));
    assert(6 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(0, 3, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(0, 3, DIR_N, &new_x, &new_y));
    assert(0 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(0, 3, DIR_E, &new_x, &new_y));
    assert(1 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(0, 3, DIR_W, &new_x, &new_y));
    assert(0 == new_x);
    assert(3 == new_y);
    assert(g->goal_at(0,0) == g->move(0, 1, DIR_S, &new_x, &new_y));
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(g->get_goals()[0] == g->move(1, 0, DIR_W, &new_x, &new_y));
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(g->get_goals()[1] == g->move(6, 7, DIR_E, &new_x, &new_y));
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(g->goal_at(7,7) == g->move(7, 6, DIR_N, &new_x, &new_y));
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(NULL == g->move(0, 0, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(0, 0, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    delete g;
}

void TestClassic_testSquares()
{
    ChippyClassic *g = new ChippyClassic();
    assert(g->get_n()  == 8);
    assert(g->goal_at(0,0)->get_reward() == 10);
    assert(g->goal_at(7,7)->get_reward() == -10);
    assert(g->square(3, 4)->get_x() == 3);
    assert(g->square(3, 4)->get_y() == 4);
    delete g;
}

void TestCorner()
{
    cout << "  Chippy Corner ... ";
    TestCorner_testEmptyConstructor();
    TestCorner_testConstructor();
    TestCorner_testPerturb();
    TestCorner_testMoves();
    TestCorner_testSquares();
    cout << "OK" << endl;
}

void TestCorner_testEmptyConstructor()
{
    ChippyCorner *c = new ChippyCorner();
    assert(c->get_n() == 8);
    assert(c->get_goals() != NULL);
    assert(c->goal_at(0,0) != NULL);
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7) != NULL);
    assert(c->goal_at(7,7)->get_reward() == -10);
    delete c;
}

void TestCorner_testConstructor()
{
    ChippyCorner *c = new ChippyCorner(6,3,4);
    assert(c->get_n() == 6);
    assert(c->get_goals() != NULL);
    assert(c->goal_at(0,0) != NULL);
    assert(c->goal_at(0,0)->get_reward() == 3);
    assert(c->goal_at(5,5) != NULL);
    assert(c->goal_at(5,5)->get_reward() == 4);
    delete c;
}

void TestCorner_testPerturb()
{
    ChippyCorner *c = new ChippyCorner();
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    assert(c->goal_at(0,0)->get_jumpx() == 7);
    assert(c->goal_at(0,0)->get_jumpy() == 7);
    assert(c->goal_at(7,7)->get_jumpx() == 0);
    assert(c->goal_at(7,7)->get_jumpy() == 0);
    assert(1 == c->perturb());
    assert(c->goal_at(0,0)->get_reward() == -10);
    assert(c->goal_at(7,7)->get_reward() == 10);
    assert(c->goal_at(0,0)->get_jumpx() == 7);
    assert(c->goal_at(0,0)->get_jumpy() == 7);
    assert(c->goal_at(7,7)->get_jumpx() == 0);
    assert(c->goal_at(7,7)->get_jumpy() == 0);
    assert(0 == c->perturb());
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    assert(c->goal_at(0,0)->get_jumpx() == 7);
    assert(c->goal_at(0,0)->get_jumpy() == 7);
    assert(c->goal_at(7,7)->get_jumpx() == 0);
    assert(c->goal_at(7,7)->get_jumpy() == 0);
    c->restore();
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    assert(c->goal_at(0,0)->get_jumpx() == 7);
    assert(c->goal_at(0,0)->get_jumpy() == 7);
    assert(c->goal_at(7,7)->get_jumpx() == 0);
    assert(c->goal_at(7,7)->get_jumpy() == 0);
    c->restore();
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    assert(c->goal_at(0,0)->get_jumpx() == 7);
    assert(c->goal_at(0,0)->get_jumpy() == 7);
    assert(c->goal_at(7,7)->get_jumpx() == 0);
    assert(c->goal_at(7,7)->get_jumpy() == 0);
    assert(1 == c->perturb());
    assert(c->goal_at(0,0)->get_reward() == -10);
    assert(c->goal_at(7,7)->get_reward() == 10);
    assert(c->goal_at(0,0)->get_jumpx() == 7);
    assert(c->goal_at(0,0)->get_jumpy() == 7);
    assert(c->goal_at(7,7)->get_jumpx() == 0);
    assert(c->goal_at(7,7)->get_jumpy() == 0);
    c->restore();
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    assert(c->goal_at(0,0)->get_jumpx() == 7);
    assert(c->goal_at(0,0)->get_jumpy() == 7);
    assert(c->goal_at(7,7)->get_jumpx() == 0);
    assert(c->goal_at(7,7)->get_jumpy() == 0);
    assert(1 == c->perturb());
    assert(c->goal_at(0,0)->get_reward() == -10);
    assert(c->goal_at(7,7)->get_reward() == 10);
    assert(c->goal_at(0,0)->get_jumpx() == 7);
    assert(c->goal_at(0,0)->get_jumpy() == 7);
    assert(c->goal_at(7,7)->get_jumpx() == 0);
    assert(c->goal_at(7,7)->get_jumpy() == 0);
    delete c;
}

void TestCorner_testMoves()
{
    int new_x, new_y;
    ChippyCorner *g = new ChippyCorner();
    assert(g->get_n()  == 8);
    assert(g->goal_at(0,0)->get_reward() == 10);
    assert(g->goal_at(7,7)->get_reward() == -10);
    assert(NULL == g->move(3, 3, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(3, 3, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(3, 3, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(3, 3, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(3, 7, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(6 == new_y);
    assert(NULL == g->move(3, 7, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 7, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 7, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 0, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(3, 0, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(1 == new_y);
    assert(NULL == g->move(3, 0, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(3, 0, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(7, 3, DIR_S, &new_x, &new_y));
    assert(7 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(7, 3, DIR_N, &new_x, &new_y));
    assert(7 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(7, 3, DIR_E, &new_x, &new_y));
    assert(7 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(7, 3, DIR_W, &new_x, &new_y));
    assert(6 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(0, 3, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(0, 3, DIR_N, &new_x, &new_y));
    assert(0 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(0, 3, DIR_E, &new_x, &new_y));
    assert(1 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(0, 3, DIR_W, &new_x, &new_y));
    assert(0 == new_x);
    assert(3 == new_y);
    assert(g->goal_at(0,0) == g->move(0, 1, DIR_S, &new_x, &new_y));
    assert(7 == new_x);
    assert(7 == new_y);
    assert(g->goal_at(0,0) == g->move(1, 0, DIR_W, &new_x, &new_y));
    assert(7 == new_x);
    assert(7 == new_y);
    assert(g->goal_at(7,7) == g->move(6, 7, DIR_E, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    assert(g->goal_at(7,7) == g->move(7, 6, DIR_N, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(0, 0, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(0, 0, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);

    assert(1 == g->perturb());
    assert(g->get_n()  == 8);
    assert(g->goal_at(0,0)->get_reward() == -10);
    assert(g->goal_at(7,7)->get_reward() == 10);
    assert(g->goal_at(0,0)->get_jumpx() == 7);
    assert(g->goal_at(0,0)->get_jumpy() == 7);
    assert(g->goal_at(7,7)->get_jumpx() == 0);
    assert(g->goal_at(7,7)->get_jumpy() == 0);
    assert(NULL == g->move(3, 3, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(3, 3, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(3, 3, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(3, 3, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(3, 7, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(6 == new_y);
    assert(NULL == g->move(3, 7, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 7, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 7, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 0, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(3, 0, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(1 == new_y);
    assert(NULL == g->move(3, 0, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(3, 0, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(7, 3, DIR_S, &new_x, &new_y));
    assert(7 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(7, 3, DIR_N, &new_x, &new_y));
    assert(7 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(7, 3, DIR_E, &new_x, &new_y));
    assert(7 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(7, 3, DIR_W, &new_x, &new_y));
    assert(6 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(0, 3, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(0, 3, DIR_N, &new_x, &new_y));
    assert(0 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(0, 3, DIR_E, &new_x, &new_y));
    assert(1 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(0, 3, DIR_W, &new_x, &new_y));
    assert(0 == new_x);
    assert(3 == new_y);
    assert(g->goal_at(0,0) == g->move(0, 1, DIR_S, &new_x, &new_y));
    assert(7 == new_x);
    assert(7 == new_y);
    assert(g->goal_at(0,0) == g->move(1, 0, DIR_W, &new_x, &new_y));
    assert(7 == new_x);
    assert(7 == new_y);
    assert(g->goal_at(7,7) == g->move(6, 7, DIR_E, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    assert(g->goal_at(7,7) == g->move(7, 6, DIR_N, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    assert(10 == g->goal_at(7,7)->get_reward());
    assert(NULL == g->move(0, 0, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(0, 0, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    delete g;
}

void TestCorner_testSquares()
{
    ChippyCorner *g = new ChippyCorner();
    assert(g->get_n()  == 8);
    assert(g->goal_at(0,0)->get_reward() == 10);
    assert(g->goal_at(7,7)->get_reward() == -10);
    assert(g->square(3, 4)->get_x() == 3);
    assert(g->square(3, 4)->get_y() == 4);
    delete g;
}

void TestRotate()
{
    cout << "  Chippy Rotate ... ";
    TestRotate_testEmptyConstructor();
    TestRotate_testConstructor();
    TestRotate_testPerturb();
    TestRotate_testMoves();
    TestRotate_testSquares();
    cout << "OK" << endl;
}

void TestRotate_testEmptyConstructor()
{
    ChippyRotate *c = new ChippyRotate();
    assert(c->get_n() == 8);
    assert(c->get_goals() != NULL);
    assert(c->goal_at(0,0) != NULL);
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7) != NULL);
    assert(c->goal_at(7,7)->get_reward() == -10);
    delete c;
}

void TestRotate_testConstructor()
{
    ChippyRotate *c = new ChippyRotate(6,3,4);
    assert(c->get_n() == 6);
    assert(c->get_goals() != NULL);
    assert(c->goal_at(0,0) != NULL);
    assert(c->goal_at(0,0)->get_reward() == 3);
    assert(c->goal_at(5,5) != NULL);
    assert(c->goal_at(5,5)->get_reward() == 4);
    delete c;
}

void TestRotate_testPerturb()
{
    ChippyRotate *c = new ChippyRotate();
    assert(c->goal_at(0,0) != NULL);
    assert(c->goal_at(7,7) != NULL);
    assert(c->goal_at(7,0) == NULL);
    assert(c->goal_at(0,7) == NULL);
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    assert(1 == c->perturb());
    assert(c->goal_at(0,0) == NULL);
    assert(c->goal_at(7,7) == NULL);
    assert(c->goal_at(7,0) != NULL);
    assert(c->goal_at(0,7) != NULL);
    assert(c->goal_at(7,0)->get_reward() == -10);
    assert(c->goal_at(0,7)->get_reward() == 10);
    assert(2 == c->perturb());
    assert(c->goal_at(0,0) != NULL);
    assert(c->goal_at(7,7) != NULL);
    assert(c->goal_at(7,0) == NULL);
    assert(c->goal_at(0,7) == NULL);
    assert(c->goal_at(0,0)->get_reward() == -10);
    assert(c->goal_at(7,7)->get_reward() == 10);
    c->restore();
    assert(c->goal_at(0,0) != NULL);
    assert(c->goal_at(7,7) != NULL);
    assert(c->goal_at(7,0) == NULL);
    assert(c->goal_at(0,7) == NULL);
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    c->restore();
    assert(c->goal_at(0,0) != NULL);
    assert(c->goal_at(7,7) != NULL);
    assert(c->goal_at(7,0) == NULL);
    assert(c->goal_at(0,7) == NULL);
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    assert(1 == c->perturb());
    assert(c->goal_at(0,0) == NULL);
    assert(c->goal_at(7,7) == NULL);
    assert(c->goal_at(7,0) != NULL);
    assert(c->goal_at(0,7) != NULL);
    assert(c->goal_at(7,0)->get_reward() == -10);
    assert(c->goal_at(0,7)->get_reward() == 10);
    assert(2 == c->perturb());
    assert(c->goal_at(0,0) != NULL);
    assert(c->goal_at(7,7) != NULL);
    assert(c->goal_at(7,0) == NULL);
    assert(c->goal_at(0,7) == NULL);
    assert(c->goal_at(0,0)->get_reward() == -10);
    assert(c->goal_at(7,7)->get_reward() == 10);
    assert(3 == c->perturb());
    assert(c->goal_at(0,0) == NULL);
    assert(c->goal_at(7,7) == NULL);
    assert(c->goal_at(7,0) != NULL);
    assert(c->goal_at(0,7) != NULL);
    assert(c->goal_at(0,7)->get_reward() == -10);
    assert(c->goal_at(7,0)->get_reward() == 10);
    assert(0 == c->perturb());
    assert(c->goal_at(0,0) != NULL);
    assert(c->goal_at(7,7) != NULL);
    assert(c->goal_at(7,0) == NULL);
    assert(c->goal_at(0,7) == NULL);
    assert(c->goal_at(0,0)->get_reward() == 10);
    assert(c->goal_at(7,7)->get_reward() == -10);
    delete c;
}

void TestRotate_testMoves()
{
    int new_x, new_y;
    ChippyRotate *g = new ChippyRotate();
    assert(g->get_n()  == 8);
    assert(g->goal_at(0,0)->get_reward() == 10);
    assert(g->goal_at(7,7)->get_reward() == -10);
    assert(NULL == g->move(3, 3, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(3, 3, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(3, 3, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(3, 3, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(3, 7, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(6 == new_y);
    assert(NULL == g->move(3, 7, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 7, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 7, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 0, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(3, 0, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(1 == new_y);
    assert(NULL == g->move(3, 0, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(3, 0, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(7, 3, DIR_S, &new_x, &new_y));
    assert(7 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(7, 3, DIR_N, &new_x, &new_y));
    assert(7 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(7, 3, DIR_E, &new_x, &new_y));
    assert(7 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(7, 3, DIR_W, &new_x, &new_y));
    assert(6 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(0, 3, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(0, 3, DIR_N, &new_x, &new_y));
    assert(0 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(0, 3, DIR_E, &new_x, &new_y));
    assert(1 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(0, 3, DIR_W, &new_x, &new_y));
    assert(0 == new_x);
    assert(3 == new_y);
    assert(g->goal_at(0,0) == g->move(0, 1, DIR_S, &new_x, &new_y));
    assert(g->goal_at(0,0)->get_reward() == 10);
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(g->goal_at(0,0) == g->move(1, 0, DIR_W, &new_x, &new_y));
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(g->goal_at(7,7) == g->move(6, 7, DIR_E, &new_x, &new_y));
    assert(g->goal_at(7,7)->get_reward() == -10);
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(g->goal_at(7,7) == g->move(7, 6, DIR_N, &new_x, &new_y));
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(NULL == g->move(0, 0, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(0, 0, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);

    assert(1 == g->perturb()); 
    assert(g->goal_at(0,0) == NULL);
    assert(g->goal_at(7,7) == NULL);
    assert(g->goal_at(7,0) != NULL);
    assert(g->goal_at(0,7) != NULL);
    assert(g->goal_at(7,0)->get_reward() == -10);
    assert(g->goal_at(0,7)->get_reward() == 10);
    assert(NULL == g->move(3, 3, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(3, 3, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(3, 3, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(3, 3, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(3, 7, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(6 == new_y);
    assert(NULL == g->move(3, 7, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 7, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 7, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(3, 0, DIR_S, &new_x, &new_y));
    assert(3 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(3, 0, DIR_N, &new_x, &new_y));
    assert(3 == new_x);
    assert(1 == new_y);
    assert(NULL == g->move(3, 0, DIR_E, &new_x, &new_y));
    assert(4 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(3, 0, DIR_W, &new_x, &new_y));
    assert(2 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(7, 3, DIR_S, &new_x, &new_y));
    assert(7 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(7, 3, DIR_N, &new_x, &new_y));
    assert(7 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(7, 3, DIR_E, &new_x, &new_y));
    assert(7 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(7, 3, DIR_W, &new_x, &new_y));
    assert(6 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(0, 3, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(2 == new_y);
    assert(NULL == g->move(0, 3, DIR_N, &new_x, &new_y));
    assert(0 == new_x);
    assert(4 == new_y);
    assert(NULL == g->move(0, 3, DIR_E, &new_x, &new_y));
    assert(1 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(0, 3, DIR_W, &new_x, &new_y));
    assert(0 == new_x);
    assert(3 == new_y);
    assert(NULL == g->move(0, 1, DIR_S, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(1, 0, DIR_W, &new_x, &new_y));
    assert(0 == new_x);
    assert(0 == new_y);
    assert(NULL == g->move(6, 7, DIR_E, &new_x, &new_y));
    assert(7 == new_x);
    assert(7 == new_y);
    assert(NULL == g->move(7, 6, DIR_N, &new_x, &new_y));
    assert(7 == new_x);
    assert(7 == new_y);

    assert(g->goal_at(0,7) == g->move(0, 6, DIR_N, &new_x, &new_y));
    assert(g->goal_at(0,7)->get_reward() == 10);
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    assert(g->goal_at(7,0) == g->move(7, 1, DIR_S, &new_x, &new_y));
    assert(g->goal_at(7,0)->get_reward() == -10);
    assert(0 != new_x);
    assert(0 != new_y);
    assert(7 != new_x);
    assert(7 != new_y);
    delete g;
}

void TestRotate_testSquares()
{
    ChippyRotate *g = new ChippyRotate();
    assert(g->get_n()  == 8);
    assert(g->goal_at(0,0)->get_reward() == 10);
    assert(g->goal_at(7,7)->get_reward() == -10);
    assert(g->square(3, 4)->get_x() == 3);
    assert(g->square(3, 4)->get_y() == 4);
    delete g;
}

void TestWalker()
{
    cout << "  Walker ... ";
    TestWalker_testEmptyConstructor();
    TestWalker_testConstructor();
    TestWalker_testMoves();
    cout << "OK" << endl;
}

void TestWalker_testEmptyConstructor()
{
    Walker *w = new Walker();
    assert(0   == w->get_score());
    assert(0   == w->get_count());
    assert(0   == w->get_x());
    assert(0   == w->get_y());
    delete w;
}
void TestWalker_testConstructor()
{
    Grid   *g = new Grid();
    Walker *w = new Walker(g);
    assert(0   == w->get_score());
    assert(0   == w->get_count());
    assert(4   == w->get_x());
    assert(4   == w->get_y());
    delete g;
    delete w;
}

void TestWalker_testMoves()
{
    Goal *g1 = new Goal(10,LOC_MIN,LOC_MIN,LOC_MAX,LOC_MAX);
    Goal *g2 = new Goal(-10,LOC_MAX,LOC_MAX,LOC_MIN,LOC_MIN);
    Goal *goals[3] = {g1, g2, NULL};
    
    Grid   *g = new Grid(8,goals);
    Walker *w = new Walker(g, 0, 0);
    assert(0   == w->get_score());
    assert(0   == w->get_count());
    assert(0   == w->get_x());
    assert(0   == w->get_y());
    assert(NULL== w->move(DIR_N));
    assert(0   == w->get_x());
    assert(1   == w->get_y());
    assert(NULL== w->move(DIR_N));
    assert(0   == w->get_x());
    assert(2   == w->get_y());
    assert(NULL== w->move(DIR_N));
    assert(0   == w->get_x());
    assert(3   == w->get_y());
    assert(NULL== w->move(DIR_N));
    assert(0   == w->get_x());
    assert(4   == w->get_y());
    assert(NULL== w->move(DIR_E));
    assert(1   == w->get_x());
    assert(4   == w->get_y());
    assert(NULL== w->move(DIR_E));
    assert(2   == w->get_x());
    assert(4   == w->get_y());
    assert(NULL== w->move(DIR_E));
    assert(3   == w->get_x());
    assert(4   == w->get_y());
    assert(NULL== w->move(DIR_E));
    assert(4   == w->get_x());
    assert(4   == w->get_y());
    assert(NULL== w->move(DIR_E));
    assert(5   == w->get_x());
    assert(4   == w->get_y());
    assert(NULL== w->move(DIR_E));
    assert(6   == w->get_x());
    assert(4   == w->get_y());
    assert(NULL== w->move(DIR_E));
    assert(7   == w->get_x());
    assert(4   == w->get_y());
    assert(NULL== w->move(DIR_E));
    assert(7   == w->get_x());
    assert(4   == w->get_y());
    assert(NULL== w->move(DIR_E));
    assert(7   == w->get_x());
    assert(4   == w->get_y());
    assert(NULL== w->move(DIR_N));
    assert(7   == w->get_x());
    assert(5   == w->get_y());
    assert(NULL== w->move(DIR_N));
    assert(7   == w->get_x());
    assert(6   == w->get_y());
    assert(g->goal_at(7,7) == w->move(DIR_N));
    assert(g->goal_at(7,7)->get_reward() == -10);
    assert(0   == w->get_x());
    assert(0   == w->get_y());
    assert(NULL== w->move(DIR_S));
    assert(0   == w->get_x());
    assert(0   == w->get_y());
    assert(-10 == w->get_score());
    assert(17  == w->get_count());
    
    delete g1;
    delete g2;
    delete g;
    delete w;
}

void TestQLearner()
{
    cout << "  QLearner ... ";
    TestQLearner_testEmptyConstructor();
    TestQLearner_testConstructor();
    TestQLearner_testLearning();
    TestQLearner_test10k();
    cout << "OK" << endl;
}

void TestQLearner_testEmptyConstructor()
{
    QLearner *q = new QLearner();
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(0    == q->get_x());
    assert(0    == q->get_y());
    assert(0.05 == q->get_epsilon());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    delete q;
}

void TestQLearner_testConstructor()
{
    Grid   *g = new Grid();
    QLearner *q = new QLearner(g, 1, 2, 0.8, 0.7, 0.1);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(1    == q->get_x());
    assert(2    == q->get_y());
    assert(0.1  == q->get_epsilon());
    assert(0.8  == q->get_alpha());
    assert(0.7  == q->get_gamma());
    delete g;
    delete q;
}

void TestQLearner_testLearning()
{
    Goal *g1 = new Goal(10,LOC_MIN,LOC_MIN,LOC_MAX,LOC_MAX);
    Goal *g2 = new Goal(-10,LOC_MAX,LOC_MAX,LOC_MIN,LOC_MIN);
    Goal *goals[3] = {g1, g2, NULL};
    Grid   *g = new Grid(8,goals);
    QLearner *q = new QLearner(g, 0, 0, 0.5, 0.9, 0.05);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(0    == q->get_x());
    assert(0    == q->get_y());
    assert(0.05 == q->get_epsilon());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    q->move(DIR_N);
    q->move(DIR_E);
    q->move(DIR_S);
    q->move(DIR_W);
    assert(10    == q->get_score());
    assert(4    == q->get_count());
    assert(7    == q->get_x());
    assert(7    == q->get_y());
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_W);
    assert(7    == q->get_x());
    assert(7    == q->get_y());
    assert(0.0  == g->square(1,0)->get_q(0));
    assert(0.0  == g->square(1,0)->get_q(1));
    assert(0.0  == g->square(1,0)->get_q(2));
    assert(7.5  == g->square(1,0)->get_q(3)); 
    assert(0.0  == g->square(1,1)->get_q(0));
    assert(2.25 == g->square(1,1)->get_q(1));
    assert(0.0  == g->square(1,1)->get_q(2));
    assert(0.0  == g->square(1,1)->get_q(3));
    
    delete g1;
    delete g2;
    delete g;
    delete q;
}

void TestQLearner_test10k()
{
    Goal *g1 = new Goal(10,LOC_MIN,LOC_MIN,LOC_MAX,LOC_MAX);
    Goal *g2 = new Goal(-10,LOC_MAX,LOC_MAX,LOC_MIN,LOC_MIN);
    Goal *goals[3] = {g1, g2, NULL};
    Grid   *g = new Grid(8,goals);
    QLearner *q = new QLearner(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.05 == q->get_epsilon());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    for (int i = 0; i < 10000; ++i)
        q->move();
    assert(10000 == q->get_count());
    delete g1;
    delete g2;
    delete g;
    delete q;
}
// FGJMjpsw

void TestQLMCLSimple()
{
    cout << "  QLearner MCL Simple ... ";
    TestQLMCLSimple_testEmptyConstructor();
    TestQLMCLSimple_testConstructor();
    TestQLMCLSimple_testLearning();
    TestQLMCLSimple_testCH20k();
    TestQLMCLSimple_testCL10k();
    TestQLMCLSimple_testCO10k();
    TestQLMCLSimple_testCR10k();
    cout << "OK" << endl;
}

void TestQLMCLSimple_testEmptyConstructor()
{
    QLMCLSimple *q = new QLMCLSimple();
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(0    == q->get_x());
    assert(0    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_resets());
    delete q;
}

void TestQLMCLSimple_testConstructor()
{
    Grid   *g = new Grid();
    QLMCLSimple *q = new QLMCLSimple(g,2,4,4,0.8,0.7,0.1);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.8  == q->get_alpha());
    assert(0.7  == q->get_gamma());
    assert(0.1  == q->get_epsilon());
    assert(2    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_resets());
    delete g;
    delete q;
}

void TestQLMCLSimple_testLearning()
{
    Goal *g1 = new Goal(10,LOC_MIN,LOC_MIN,LOC_MAX,LOC_MAX);
    Goal *g2 = new Goal(-10,LOC_MAX,LOC_MAX,LOC_MIN,LOC_MIN);
    Goal *goals[3] = {g1, g2, NULL};
    Grid   *g = new Grid(8,goals);
    QLMCLSimple *q = new QLMCLSimple(g, 2, 0, 0, 0.5, 0.9, 0.05);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(0    == q->get_x());
    assert(0    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    assert(2    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_resets());
    q->move(DIR_N); // 0,0->0,1
    assert(0    == q->get_x());
    assert(1    == q->get_y());
    q->move(DIR_E); // 0,1->1,1
    assert(1    == q->get_x());
    assert(1    == q->get_y());
    q->move(DIR_S); // 1,1->1,0
    assert(1    == q->get_x());
    assert(0    == q->get_y());
    q->move(DIR_W); // 1,0->0,0 jumpto 7,7
    assert(4    == q->get_count());
    assert(7    == q->get_x());
    assert(7    == q->get_y());
    assert(10   == q->get_score());
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_W);
    assert(7    == q->get_x());
    assert(7    == q->get_y());
    assert(0.0  == g->square(1,0)->get_q(0));
    assert(0.0  == g->square(1,0)->get_q(1));
    assert(0.0  == g->square(1,0)->get_q(2));
    assert(7.5  == g->square(1,0)->get_q(3)); 
    assert(0.0  == g->square(1,1)->get_q(0));
    assert(2.25 == g->square(1,1)->get_q(1));
    assert(0.0  == g->square(1,1)->get_q(2));
    assert(0.0  == g->square(1,1)->get_q(3));
    delete g1;
    delete g2;
    delete g;
    delete q;
}

void TestQLMCLSimple_testCH20k()
{
    //cout << "---------- TestQLMCLSimple_testCH20k ----------" << endl;
    Grid   *g = new ChippyClassic(8);
    QLMCLSimple *q = new QLMCLSimple(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    for (int i = 0; i < 20000; ++i)
        q->move();
    assert(20000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_resets());
    assert(0    == q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLSimple_testCL10k()
{
    //cout << "---------- TestQLMCLSimple_testCL10k ----------" << endl;
    Grid   *g = new ChippyClassic(8);
    QLMCLSimple *q = new QLMCLSimple(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    for (int i = 0; i < 5000; ++i)
        q->move();
    assert(5000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_resets());
    g->perturb();
    //cout << "----- perturbation ----- " << endl;
    for (int j = 0; j < 5000; ++j)
        q->move();
    assert(10000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(1    == q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLSimple_testCO10k()
{
    //cout << "---------- TestQLMCLSimple_testCO10k ----------" << endl;
    Grid   *g = new ChippyCorner(8);
    QLMCLSimple *q = new QLMCLSimple(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    for (int i = 0; i < 5000; ++i)
        q->move();
    assert(5000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    g->perturb();
    //cout << "----- perturbation ----- " << endl;
    for (int j = 0; j < 5000; ++j)
        q->move();
    assert(10000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(1    == q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLSimple_testCR10k()
{
    //cout << "---------- TestQLMCLSimple_testCR10k ----------" << endl;
    Grid   *g = new ChippyRotate(8);
    QLMCLSimple *q = new QLMCLSimple(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    for (int i = 0; i < 5000; ++i)
        q->move();
    assert(5000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    g->perturb();
    //cout << "----- perturbation ----- " << endl;
    for (int j = 0; j < 5000; ++j)
        q->move();
    assert(10000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(1    == q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLSensitive()
{
    cout << "  QLearner MCL Sensitive ... ";
    TestQLMCLSensitive_testEmptyConstructor();
    TestQLMCLSensitive_testConstructor();
    TestQLMCLSensitive_testLearning();
    TestQLMCLSensitive_testCH20k();
    TestQLMCLSensitive_testCL10k();
    TestQLMCLSensitive_testCO10k();
    TestQLMCLSensitive_testCR10k();
    cout << "OK" << endl;
}

void TestQLMCLSensitive_testEmptyConstructor()
{
    QLMCLSensitive *q = new QLMCLSensitive();
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(0    == q->get_x());
    assert(0    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    delete q;
}

void TestQLMCLSensitive_testConstructor()
{
    Grid   *g = new Grid();
    QLMCLSensitive *q = new QLMCLSensitive(g,2,4,4,0.8,0.7,0.1);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.8  == q->get_alpha());
    assert(0.7  == q->get_gamma());
    assert(0.1  == q->get_epsilon());
    assert(2    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    delete g;
    delete q;
}

void TestQLMCLSensitive_testLearning()
{
    Goal *g1 = new Goal(10,LOC_MIN,LOC_MIN,LOC_MAX,LOC_MAX);
    Goal *g2 = new Goal(-10,LOC_MAX,LOC_MAX,LOC_MIN,LOC_MIN);
    Goal *goals[3] = {g1, g2, NULL};
    Grid   *g = new Grid(8,goals);
    QLMCLSensitive *q = new QLMCLSensitive(g, 2, 0, 0, 0.5, 0.9, 0.05);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(0    == q->get_x());
    assert(0    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    assert(2    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    q->move(DIR_N); // 0,0->0,1
    assert(0    == q->get_x());
    assert(1    == q->get_y());
    q->move(DIR_E); // 0,1->1,1
    assert(1    == q->get_x());
    assert(1    == q->get_y());
    q->move(DIR_S); // 1,1->1,0
    assert(1    == q->get_x());
    assert(0    == q->get_y());
    q->move(DIR_W); // 1,0->0,0 jumpto 7,7
    assert(4    == q->get_count());
    assert(7    == q->get_x());
    assert(7    == q->get_y());
    assert(10   == q->get_score());
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_W);
    assert(7    == q->get_x());
    assert(7    == q->get_y());
    assert(0.0  == g->square(1,0)->get_q(0));
    assert(0.0  == g->square(1,0)->get_q(1));
    assert(0.0  == g->square(1,0)->get_q(2));
    assert(7.5  == g->square(1,0)->get_q(3)); 
    assert(0.0  == g->square(1,1)->get_q(0));
    assert(2.25 == g->square(1,1)->get_q(1));
    assert(0.0  == g->square(1,1)->get_q(2));
    assert(0.0  == g->square(1,1)->get_q(3));
    delete g1;
    delete g2;
    delete g;
    delete q;
}

void TestQLMCLSensitive_testCH20k()
{
    //cout << "---------- TestQLMCLSensitive_testCH10k ----------" << endl;
    Grid   *g = new ChippyClassic(8);
    QLMCLSensitive *q = new QLMCLSensitive(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    //q->set_verbose(1); // ################# DEBUG ###################
    for (int i = 0; i < 20000; ++i)
        q->move();
    assert(20000 == q->get_count());
    assert(3    == q->get_threshold());
    //cout << "CH20K violations = " << q->get_violations() << endl;
    //cout << "CH20K policy num = " << q->get_policy_number() << endl;
    assert(2 > q->get_violations());
    assert(2 > q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLSensitive_testCL10k()
{
    //cout << "---------- TestQLMCLSensitive_testCL10k ----------" << endl;
    Grid   *g = new ChippyClassic(8);
    QLMCLSensitive *q = new QLMCLSensitive(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    //q->set_verbose(1); // ################# DEBUG ###################
    for (int i = 0; i < 5000; ++i)
        q->move();
    assert(5000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    g->perturb();
    //cout << "----- perturbation ----- " << endl;
    for (int j = 0; j < 5000; ++j)
        q->move();
    assert(10000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(1    == q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLSensitive_testCO10k()
{
    //cout << "---------- TestQLMCLSensitive_testCO10k ----------" << endl;
    Grid   *g = new ChippyCorner(8);
    QLMCLSensitive *q = new QLMCLSensitive(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    for (int i = 0; i < 5000; ++i)
        q->move();
    assert(5000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    g->perturb();
    //cout << "----- perturbation ----- " << endl;
    for (int j = 0; j < 5000; ++j)
        q->move();
    assert(10000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(1    == q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLSensitive_testCR10k()
{
    //cout << "---------- TestQLMCLSensitive_testCR10k ----------" << endl;
    Grid   *g = new ChippyRotate(8);
    QLMCLSensitive *q = new QLMCLSensitive(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    for (int i = 0; i < 5000; ++i)
        q->move();
    assert(5000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    g->perturb();
    //cout << "----- perturbation ----- " << endl;
    for (int j = 0; j < 5000; ++j)
        q->move();
    assert(10000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(1    == q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLSophisticated()
{
    cout << "  QLearner MCL Sophisticated ... ";
    TestQLMCLSophisticated_testEmptyConstructor();
    TestQLMCLSophisticated_testConstructor();
    TestQLMCLSophisticated_testLearning();
    TestQLMCLSophisticated_testCH20k();
    TestQLMCLSophisticated_testCL10k();
    TestQLMCLSophisticated_testCO10k();
    TestQLMCLSophisticated_testCR10k();
    cout << "OK" << endl;
}

void TestQLMCLSophisticated_testEmptyConstructor()
{
    QLMCLSophisticated *q = new QLMCLSophisticated();
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(0    == q->get_x());
    assert(0    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    delete q;
}

void TestQLMCLSophisticated_testConstructor()
{
    Grid   *g = new Grid();
    QLMCLSophisticated *q = new QLMCLSophisticated(g,2,4,4,0.8,0.7,0.1);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.8  == q->get_alpha());
    assert(0.7  == q->get_gamma());
    assert(0.1  == q->get_epsilon());
    assert(2    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    delete g;
    delete q;
}

void TestQLMCLSophisticated_testLearning()
{
    Goal *g1 = new Goal(10,LOC_MIN,LOC_MIN,LOC_MAX,LOC_MAX);
    Goal *g2 = new Goal(-10,LOC_MAX,LOC_MAX,LOC_MIN,LOC_MIN);
    Goal *goals[3] = {g1, g2, NULL};
    Grid   *g = new Grid(8,goals);
    QLMCLSophisticated *q = new QLMCLSophisticated(g, 2, 0, 0, 0.5, 0.9, 0.05);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(0    == q->get_x());
    assert(0    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    assert(2    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    q->move(DIR_N); // 0,0->0,1
    assert(0    == q->get_x());
    assert(1    == q->get_y());
    q->move(DIR_E); // 0,1->1,1
    assert(1    == q->get_x());
    assert(1    == q->get_y());
    q->move(DIR_S); // 1,1->1,0
    assert(1    == q->get_x());
    assert(0    == q->get_y());
    q->move(DIR_W); // 1,0->0,0 jumpto 7,7
    assert(4    == q->get_count());
    assert(7    == q->get_x());
    assert(7    == q->get_y());
    assert(10   == q->get_score());
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_W);
    assert(7    == q->get_x());
    assert(7    == q->get_y());
    assert(0.0  == g->square(1,0)->get_q(0));
    assert(0.0  == g->square(1,0)->get_q(1));
    assert(0.0  == g->square(1,0)->get_q(2));
    assert(7.5  == g->square(1,0)->get_q(3)); 
    assert(0.0  == g->square(1,1)->get_q(0));
    assert(2.25 == g->square(1,1)->get_q(1));
    assert(0.0  == g->square(1,1)->get_q(2));
    assert(0.0  == g->square(1,1)->get_q(3));
    delete g1;
    delete g2;
    delete g;
    delete q;
}

void TestQLMCLSophisticated_testCL10k()
{
    //cout << "---------- TestQLMCLSophisticated_testCL10k ----------" << endl;
    Grid   *g = new ChippyClassic(8);
    QLMCLSophisticated *q = new QLMCLSophisticated(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    for (int i = 0; i < 5000; ++i)
        q->move();
    assert(5000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    g->perturb();
    //cout << "----- perturbation ----- " << endl;
    for (int j = 0; j < 5000; ++j)
        q->move();
    assert(10000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(1    == q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLSophisticated_testCH20k()
{
    //cout << "---------- TestQLMCLSophisticated_testCH20k ----------" << endl;
    Grid   *g = new ChippyClassic(8);
    QLMCLSophisticated *q = new QLMCLSophisticated(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    for (int i = 0; i < 20000; ++i)
        q->move();
    assert(20000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLSophisticated_testCO10k()
{
    //cout << "---------- TestQLMCLSophisticated_testCO10k ----------" << endl;
    Grid   *g = new ChippyCorner(8);
    QLMCLSophisticated *q = new QLMCLSophisticated(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    for (int i = 0; i < 5000; ++i)
        q->move();
    assert(5000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    g->perturb();
    //cout << "----- perturbation ----- " << endl;
    for (int j = 0; j < 5000; ++j)
        q->move();
    assert(10000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(1    == q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLSophisticated_testCR10k()
{
    //cout << "---------- TestQLMCLSophisticated_testCR10k ----------" << endl;
    Grid   *g = new ChippyRotate(8);
    QLMCLSophisticated *q = new QLMCLSophisticated(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    for (int i = 0; i < 5000; ++i)
        q->move();
    assert(5000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    g->perturb();
    //cout << "----- perturbation ----- " << endl;
    for (int j = 0; j < 5000; ++j)
        q->move();
    assert(10000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(2    >  q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLBayes1()
{
    cout << "  QLearner MCL Bayes 1 ... ";
    TestQLMCLBayes1_testEmptyConstructor();
    TestQLMCLBayes1_testConstructor();
    TestQLMCLBayes1_testLearning();
    TestQLMCLBayes1_testCH20k();
    TestQLMCLBayes1_testCL10k();
    TestQLMCLBayes1_testCO10k();
    TestQLMCLBayes1_testCR10k();
    cout << "OK" << endl;
}

void TestQLMCLBayes1_testEmptyConstructor()
{
    QLMCLBayes1 *q = new QLMCLBayes1();
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(0    == q->get_x());
    assert(0    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    delete q;
}

void TestQLMCLBayes1_testConstructor()
{
    Grid   *g = new Grid();
    QLMCLBayes1 *q = new QLMCLBayes1(g,2,4,4,0.8,0.7,0.1);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.8  == q->get_alpha());
    assert(0.7  == q->get_gamma());
    assert(0.1  == q->get_epsilon());
    assert(2    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    delete g;
    delete q;
}

void TestQLMCLBayes1_testLearning()
{
    Goal *g1 = new Goal(10,LOC_MIN,LOC_MIN,LOC_MAX,LOC_MAX);
    Goal *g2 = new Goal(-10,LOC_MAX,LOC_MAX,LOC_MIN,LOC_MIN);
    Goal *goals[3] = {g1, g2, NULL};
    Grid   *g = new Grid(8,goals);
    QLMCLBayes1 *q = new QLMCLBayes1(g, 2, 0, 0, 0.5, 0.9, 0.05);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(0    == q->get_x());
    assert(0    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    assert(2    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    q->move(DIR_N); // 0,0->0,1
    assert(0    == q->get_x());
    assert(1    == q->get_y());
    q->move(DIR_E); // 0,1->1,1
    assert(1    == q->get_x());
    assert(1    == q->get_y());
    q->move(DIR_S); // 1,1->1,0
    assert(1    == q->get_x());
    assert(0    == q->get_y());
    q->move(DIR_W); // 1,0->0,0 jumpto 7,7
    assert(4    == q->get_count());
    assert(7    == q->get_x());
    assert(7    == q->get_y());
    assert(10   == q->get_score());
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_W);
    assert(7    == q->get_x());
    assert(7    == q->get_y());
    assert(0.0  == g->square(1,0)->get_q(0));
    assert(0.0  == g->square(1,0)->get_q(1));
    assert(0.0  == g->square(1,0)->get_q(2));
    assert(7.5  == g->square(1,0)->get_q(3)); 
    assert(0.0  == g->square(1,1)->get_q(0));
    assert(2.25 == g->square(1,1)->get_q(1));
    assert(0.0  == g->square(1,1)->get_q(2));
    assert(0.0  == g->square(1,1)->get_q(3));
    delete g1;
    delete g2;
    delete g;
    delete q;
}

void TestQLMCLBayes1_testCH20k()
{
    //cout << "---------- TestQLMCLBayes1_testCH20k ----------" << endl;
    Grid   *g = new ChippyClassic(8);
    QLMCLBayes1 *q = new QLMCLBayes1(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    for (int j = 0; j < 20000; ++j)
        q->move();
    assert(20000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLBayes1_testCL10k()
{
    //cout << "---------- TestQLMCLBayes1_testCL10k ----------" << endl;
    Grid   *g = new ChippyClassic(8);
    QLMCLBayes1 *q = new QLMCLBayes1(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    for (int i = 0; i < 5000; ++i)
        q->move();
    assert(5000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    //q->set_verbose(1);
    g->perturb();
    //cout << "----- perturbation ----- " << endl;
    for (int j = 0; j < 5000; ++j)
        q->move();
    assert(10000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    <  q->get_policy_number());
    assert(4    >  q->get_policy_number());
    //cout << "----- MCLBayes1 CL score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLBayes1_testCO10k()
{
    //cout << "---------- TestQLMCLBayes1_testCO10k ----------" << endl;
    Grid   *g = new ChippyCorner(8);
    QLMCLBayes1 *q = new QLMCLBayes1(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    for (int i = 0; i < 5000; ++i)
        q->move();
    assert(5000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    g->perturb();
    //cout << "----- perturbation ----- " << endl;
    for (int j = 0; j < 5000; ++j)
        q->move();
    assert(10000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    <  q->get_policy_number());
    assert(4    >  q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLBayes1_testCR10k()
{
    //cout << "---------- TestQLMCLBayes1_testCR10k ----------" << endl;
    Grid   *g = new ChippyRotate(8);
    QLMCLBayes1 *q = new QLMCLBayes1(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    for (int i = 0; i < 5000; ++i)
        q->move();
    assert(5000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    g->perturb();
    //cout << "----- perturbation ----- " << endl;
    for (int j = 0; j < 5000; ++j)
        q->move();
    assert(10000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    <  q->get_policy_number());
    assert(4    >  q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLBayes2()
{
    cout << "  QLearner MCL Bayes 2 ... ";
    TestQLMCLBayes2_testEmptyConstructor();
    TestQLMCLBayes2_testConstructor();
    TestQLMCLBayes2_testLearning();
    TestQLMCLBayes2_testCH20k();
    TestQLMCLBayes2_testCL10k();
    TestQLMCLBayes2_testCO10k();
    TestQLMCLBayes2_testCR10k();
    TestQLMCLBayes2_testCL10p5();
    cout << "OK" << endl;
}

void TestQLMCLBayes2_testEmptyConstructor()
{
    QLMCLBayes2 *q = new QLMCLBayes2();
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(0    == q->get_x());
    assert(0    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    delete q;
}

void TestQLMCLBayes2_testConstructor()
{
    Grid   *g = new Grid();
    QLMCLBayes2 *q = new QLMCLBayes2(g,2,4,4,0.8,0.7,0.1);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.8  == q->get_alpha());
    assert(0.7  == q->get_gamma());
    assert(0.1  == q->get_epsilon());
    assert(2    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    delete g;
    delete q;
}

void TestQLMCLBayes2_testLearning()
{
    Goal *g1 = new Goal(10,LOC_MIN,LOC_MIN,LOC_MAX,LOC_MAX);
    Goal *g2 = new Goal(-10,LOC_MAX,LOC_MAX,LOC_MIN,LOC_MIN);
    Goal *goals[3] = {g1, g2, NULL};
    Grid   *g = new Grid(8,goals);
    QLMCLBayes2 *q = new QLMCLBayes2(g, 2, 0, 0, 0.5, 0.9, 0.05);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(0    == q->get_x());
    assert(0    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    assert(2    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    q->move(DIR_N); // 0,0->0,1
    assert(0    == q->get_x());
    assert(1    == q->get_y());
    q->move(DIR_E); // 0,1->1,1
    assert(1    == q->get_x());
    assert(1    == q->get_y());
    q->move(DIR_S); // 1,1->1,0
    assert(1    == q->get_x());
    assert(0    == q->get_y());
    q->move(DIR_W); // 1,0->0,0 jumpto 7,7
    assert(4    == q->get_count());
    assert(7    == q->get_x());
    assert(7    == q->get_y());
    assert(10   == q->get_score());
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_W);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_S);
    q->move(DIR_W);
    assert(7    == q->get_x());
    assert(7    == q->get_y());
    assert(0.0  == g->square(1,0)->get_q(0));
    assert(0.0  == g->square(1,0)->get_q(1));
    assert(0.0  == g->square(1,0)->get_q(2));
    assert(7.5  == g->square(1,0)->get_q(3)); 
    assert(0.0  == g->square(1,1)->get_q(0));
    assert(2.25 == g->square(1,1)->get_q(1));
    assert(0.0  == g->square(1,1)->get_q(2));
    assert(0.0  == g->square(1,1)->get_q(3));
    delete g1;
    delete g2;
    delete g;
    delete q;
}

void TestQLMCLBayes2_testCH20k()
{
    //cout << "---------- TestQLMCLBayes2_testCH20k ----------" << endl;
    Grid   *g = new ChippyClassic(8);
    QLMCLBayes2 *q = new QLMCLBayes2(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    for (int i = 0; i < 20000; ++i)
        q->move();
    assert(20000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLBayes2_testCL10k()
{
    //cout << "---------- TestQLMCLBayes2_testCL10k ----------" << endl;
    Grid   *g = new ChippyClassic(8);
    QLMCLBayes2 *q = new QLMCLBayes2(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    for (int i = 0; i < 5000; ++i)
        q->move();
    assert(5000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    g->perturb();
    //cout << "----- perturbation ----- " << endl;
    //q->set_verbose(1);
    for (int j = 0; j < 5000; ++j)
        q->move();
    assert(10000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    <  q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLBayes2_testCL10p5()
{
    //cout << "---------- TestQLMCLBayes2_testCL10p5 ----------" << endl;
    Grid   *g = new ChippyClassic(8, 10, 5);
    QLMCLBayes2 *q = new QLMCLBayes2(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    //q->set_verbose(1);
    for (int i = 0; i < 5000; ++i)
        q->move();
    assert(5000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    g->perturb();
    //cout << "----- perturbation ----- " << endl;
    for (int j = 0; j < 5000; ++j)
        q->move();
    assert(10000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLBayes2_testCO10k()
{
    //cout << "---------- TestQLMCLBayes2_testCO10k ----------" << endl;
    Grid   *g = new ChippyCorner(8);
    QLMCLBayes2 *q = new QLMCLBayes2(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    for (int i = 0; i < 5000; ++i)
        q->move();
    assert(5000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    g->perturb();
    //cout << "----- perturbation ----- " << endl;
    for (int j = 0; j < 5000; ++j)
        q->move();
    assert(10000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    <= q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestQLMCLBayes2_testCR10k()
{
    //cout << "---------- TestQLMCLBayes2_testCR10k ----------" << endl;
    Grid   *g = new ChippyRotate(8);
    QLMCLBayes2 *q = new QLMCLBayes2(g);
    assert(0    == q->get_score());
    assert(0    == q->get_count());
    assert(4    == q->get_x());
    assert(4    == q->get_y());
    assert(0.5  == q->get_alpha());
    assert(0.9  == q->get_gamma());
    assert(0.05 == q->get_epsilon());
    for (int i = 0; i < 5000; ++i)
        q->move();
    assert(5000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    == q->get_policy_number());
    g->perturb();
    //cout << "----- perturbation ----- " << endl;
    for (int j = 0; j < 5000; ++j)
        q->move();
    assert(10000 == q->get_count());
    assert(3    == q->get_threshold());
    assert(0    == q->get_violations());
    assert(0    <  q->get_policy_number());
    //cout << "----- score = " << q->get_score() << " ----- " << endl;
    delete g;
    delete q;
}

void TestRollingAverage()
{
    cout << "  RollingAverage ... ";
    TestRollingAverage_testEmptyConstructor();
    TestRollingAverage_testConstructor();
    TestRollingAverage_testAverages();
    cout << "OK" << endl;
}

void TestRollingAverage_testEmptyConstructor()
{
    RollingAverage *r = new RollingAverage();
    assert(0    == r->get_count());
    assert(ROLLING_AVERAGE_SIZE  == r->get_n());
    assert(0.0  == r->get_total());
    assert(0.0  == r->get_average());
    delete r;
}

void TestRollingAverage_testConstructor()
{
    RollingAverage *r = new RollingAverage(10);
    assert(0    == r->get_count());
    assert(10   == r->get_n());
    assert(0.0  == r->get_total());
    assert(0.0  == r->get_average());
    delete r;
}

void TestRollingAverage_testAverages()
{
    RollingAverage *r = new RollingAverage(10);
    assert(0    == r->get_count());
    assert(10   == r->get_n());
    assert(0.0  == r->get_total());
    assert(0.0  == r->get_average());
    r->add(1);
    r->add(2);
    r->add(0);
    r->add(3);
    r->add(4);
    assert(5    == r->get_count());
    assert(10   == r->get_n());
    assert(10.0 == r->get_total());
    assert(2.0  == r->get_average());
    r->add(2);
    r->add(2);
    r->add(2);
    r->add(2);
    r->add(2);
    assert(10   == r->get_count());
    assert(10   == r->get_n());
    assert(20.0 == r->get_total());
    assert(2.0  == r->get_average());
    r->add(3);
    assert(10   == r->get_count());
    assert(10   == r->get_n());
    assert(22.0 == r->get_total());
    assert(fabs(2.2-r->get_average())<0.1);
    r->add(3);
    r->add(4);
    r->add(4);
    r->add(5);
    assert(10   == r->get_count());
    assert(10   == r->get_n());
    assert(29.0 == r->get_total());
    assert(fabs(2.9-r->get_average())<0.1);
    r->add(0);
    r->add(0);
    r->add(0);
    r->add(0);
    r->add(0);
    r->add(0);
    r->add(0);
    r->add(0);
    r->add(0);
    r->add(0);
    assert(10   == r->get_count());
    assert(10   == r->get_n());
    assert(0.0  == r->get_total());
    assert(0.0  == r->get_average());
    delete r;
}

void TestRewards()
{
    cout << "  Rewards ... ";
    TestRewards_testEmptyConstructor();
    TestRewards_testConstructor();
    TestRewards_testAppend();
    TestRewards_testAdd();
    cout << "OK" << endl;
}

void TestRewards_testEmptyConstructor()
{
    Rewards *r = new Rewards();
    assert(0    == r->get_count());
    assert(0    == r->get_index());
    assert(1024 == r->get_n());
    assert(0.0  == r->get_total());
    assert(0 == strcmp("????", r->get_initials()));
    assert(0 == strcmp("????", r->get_initials()));
    assert(0 == strcmp("????", r->get_initials()));
    delete r;
}

void TestRewards_testConstructor()
{
    Rewards *r = new Rewards(10);
    assert(0    == r->get_count());
    assert(0    == r->get_index());
    assert(10   == r->get_n());
    assert(0.0  == r->get_total());
    assert(0 == strcmp("????", r->get_initials()));
    assert(0 == strcmp("????", r->get_initials()));
    assert(0 == strcmp("????", r->get_initials()));
    delete r;
}

void TestRewards_testAppend()
{
    Rewards *r = new Rewards(10);
    assert(0    == r->get_count());
    assert(0    == r->get_index());
    assert(10   == r->get_n());
    assert(0.0  == r->get_total());
    r->append(1);
    assert(1    == r->get_count());
    assert(1    == r->get_index());
    assert(10   == r->get_n());
    assert(1.0  == r->get_total());
    assert(1.0  == r->get_reward(0));
    assert(1.0  == r->get_average(0));
    r->append(2);
    assert(1    == r->get_count());
    assert(2    == r->get_index());
    assert(10   == r->get_n());
    assert(3.0  == r->get_total());
    assert(2.0  == r->get_reward(1));
    assert(2.0  == r->get_average(1));
    r->append(3);
    assert(1    == r->get_count());
    assert(3    == r->get_index());
    assert(10   == r->get_n());
    assert(6.0  == r->get_total());
    r->append(4);
    r->append(5);
    r->append(6);
    assert(1    == r->get_count());
    assert(6    == r->get_index());
    assert(10   == r->get_n());
    assert(21.0  == r->get_total());
    r->append(3);
    r->append(2);
    r->append(1);
    assert(1    == r->get_count());
    assert(9    == r->get_index());
    assert(10   == r->get_n());
    assert(27.0  == r->get_total());
    r->append(4);
    r->append(3);
    r->append(1.7);
    assert(1    == r->get_count());
    assert(12    == r->get_index());
    assert(20   == r->get_n());
    assert(35.7  == r->get_total());
    assert(1.0  == r->get_reward(0));
    assert(2.0  == r->get_reward(1));
    assert(3.0  == r->get_reward(2));
    assert(4.0  == r->get_reward(3));
    assert(5.0  == r->get_reward(4));
    assert(6.0  == r->get_reward(5));
    assert(3.0  == r->get_reward(6));
    assert(2.0  == r->get_reward(7));
    assert(1.0  == r->get_reward(8));
    assert(4.0  == r->get_reward(9));
    assert(3.0  == r->get_reward(10));
    assert(1.7  == r->get_reward(11));
    assert(1.7  == r->get_average(11));
    delete r;
}

void TestRewards_testAdd()
{
    Rewards *r1 = new Rewards(10);
    Rewards *r2 = new Rewards(10);
    Rewards *r3 = new Rewards(10);
    r2->append(1);
    r2->append(2);
    r2->append(3);
    r2->append(4);
    r2->append(3);
    r2->append(2);
    r2->append(1);
    assert(7    == r2->get_index());
    assert(1.0  == r2->get_reward(0));
    assert(2.0  == r2->get_reward(1));
    assert(3.0  == r2->get_reward(2));
    assert(4.0  == r2->get_reward(3));
    assert(3.0  == r2->get_reward(4));
    assert(2.0  == r2->get_reward(5));
    assert(1.0  == r2->get_reward(6));
    assert(1.0  == r2->get_average(6));
    r3->append(1);
    r3->append(2);
    r3->append(1);
    r3->append(2);
    r3->append(1);
    r3->append(2);
    assert(6    == r3->get_index());
    assert(1.0  == r3->get_reward(0));
    assert(2.0  == r3->get_reward(1));
    assert(1.0  == r3->get_reward(2));
    assert(2.0  == r3->get_reward(3));
    assert(1.0  == r3->get_reward(4));
    assert(2.0  == r3->get_reward(5));
    assert(2.0  == r3->get_average(5));
    assert(0    == r1->get_count());
    assert(1    == r2->get_count());
    assert(1    == r3->get_count());
    assert(0    == r1->get_total());
    assert(16   == r2->get_total());
    assert(9    == r3->get_total());
    r1->add(r2);
    //cout << "after r1->add(r2) r1->get_index() = " << r1->get_index() << endl;
    assert(7    == r1->get_index());
    assert(1.0  == r1->get_reward(0));
    assert(2.0  == r1->get_reward(1));
    assert(3.0  == r1->get_reward(2));
    assert(4.0  == r1->get_reward(3));
    assert(3.0  == r1->get_reward(4));
    assert(2.0  == r1->get_reward(5));
    assert(2.0  == r1->get_average(5));
    assert(1.0  == r1->get_reward(6));
    assert(1.0  == r1->get_average(6));
    r1->add(r3);
    assert(2    == r1->get_count());
    assert(25   == r1->get_total());
    assert(1.0  == r1->get_average(0));
    assert(2.0  == r1->get_average(2));
    assert(2.0  == r1->get_reward(0));  // 1 + 1
    assert(4.0  == r1->get_reward(1));  // 2 + 2 
    assert(4.0  == r1->get_reward(2));  // 3 + 1
    assert(6.0  == r1->get_reward(3));  // 4 + 2
    assert(4.0  == r1->get_reward(4));  // 3 + 1
    assert(4.0  == r1->get_reward(5));  // 2 + 2
    assert(1.0  == r1->get_reward(6));  // 1 + 0
    assert(2.0  == r1->get_average(5));
    assert(7    == r1->get_index());
    //cout << "after r1->add(r3) r1->get_reward(5) = " << r1->get_reward(5) << endl;
    //cout << "after r1->add(r3) r1->get_average(5) = " << r1->get_average(5) << endl;
    assert(2.0  == r1->get_average(5));
    assert(0.5  == r1->get_average(6));
    delete r1;
    delete r2;
    delete r3;
}    
// --------------------------------------------------------------------
//                                                       do_experiments
// --------------------------------------------------------------------
void do_experiments(const char *basename, int repeats=EXP_REPEAT,
                    int n = 8, int r1=10, int r2=-10)
{
    Grid* grids[] = {
        new Chippy(n, r1, r2), 
	    new ChippyClassic(n, r1, r2), 
        new ChippyCorner(n, r1, r2), 
	    new ChippyRotate(n, r1, r2),
	    NULL
    };
    Grid **g;
    int kntg = 0;
    
    int walkers[] = {
        WALK_QLEARNER, 
	    WALK_SIMPLE,
        WALK_SENSITIVE,
        WALK_SOPHISTICATED,
        WALK_BAYES1,
        WALK_BAYES2,
        WALK_NONE
    };
    
    int *w;
    int kntw= 0;
    
    // 1. Count the walkers and 
    for (w = walkers; *w != WALK_NONE; ++w) ++kntw;
    for (g = grids; *g != NULL; ++g) ++kntg;
    //cout << "grids=" << kntg << ", walkers=" << kntw << endl;
    
    // 2. Execute the experiments
    experiments(basename, 
                repeats, EXP_STEPS, EXP_PERTURB, 0, 
                walkers, grids);
    
    // 3. Delete allocated objects
    for (g = grids; *g != NULL; ++g) delete *g;
    
}

struct unittest_reference {
    char *name;
    void (*test)(void);
};

unittest_reference unit_tests[] = {
    {"UNKNOWN", NULL},
    {"randint", Testrandint},
    {"orient_value", Testorient_value},
    {"Square", TestSquare},
    {"Goal", TestGoal},
    {"Grid", TestGrid},
    {"Chippy", TestChippy},
    {"Classic", TestClassic},
    {"Corner", TestCorner},
    {"Rotate", TestRotate},
    {"Walker", TestWalker},
    {"QLearner", TestQLearner},
    {"MCLSimple", TestQLMCLSimple},
    {"MCLSensitive", TestQLMCLSensitive},
    {"MCLSophisticated", TestQLMCLSophisticated},
    {"MCLBayes1", TestQLMCLBayes1},
    {"B1EmptyConstructor", TestQLMCLBayes1_testEmptyConstructor},
    {"B1Constructor", TestQLMCLBayes1_testConstructor},
    {"B1Learning", TestQLMCLBayes1_testLearning},
    {"B1CH20k", TestQLMCLBayes1_testCH20k},
    {"B1CL10k", TestQLMCLBayes1_testCL10k},
    {"B1CO10k", TestQLMCLBayes1_testCO10k},
    {"B1CR10k", TestQLMCLBayes1_testCR10k},
    {"MCLBayes2", TestQLMCLBayes2},
    {"B2EmptyConstructor", TestQLMCLBayes2_testEmptyConstructor},
    {"B2Constructor", TestQLMCLBayes2_testConstructor},
    {"B2Learning", TestQLMCLBayes2_testLearning},
    {"B2CH20k", TestQLMCLBayes2_testCH20k},
    {"B2CL10k", TestQLMCLBayes2_testCL10k},
    {"B2CO10k", TestQLMCLBayes2_testCO10k},
    {"B2CR10k", TestQLMCLBayes2_testCR10k},
    {"B2CL10p5", TestQLMCLBayes2_testCL10p5},
    {"RollingAverage", TestRollingAverage},
    {"Rewards", TestRewards},
    {"", NULL}
};

struct grid_reference {
    char *name;
    Grid *grid;
};

grid_reference grids[] = {
    {"UNKNOWN", NULL},
    {"Fixedpn", new Chippy(8, 10, -10)}, 
	{"Classicpn", new ChippyClassic(8, 10, -10)}, 
    {"Cornerpn", new ChippyCorner(8, 10, -10)}, 
	{"Rotatepn", new ChippyRotate(8, 10, -10)},
    {"Fixedpp", new Chippy(8, 10, 5)}, 
	{"Classicpp", new ChippyClassic(8, 10, 5)}, 
    {"Cornerpp", new ChippyCorner(8, 10, 5)}, 
	{"Rotatepp", new ChippyRotate(8, 10, 5)},
    {"", NULL}
};

struct walk_reference {
    char *name;
    int walk;
};

walk_reference walkers[] = {
    {"None", WALK_NONE},
    {"Walker", WALK_WALKER},
    {"QLearner", WALK_QLEARNER}, 
    {"Simple", WALK_SIMPLE},
    {"Sensitive", WALK_SENSITIVE},
    {"Sophisticated", WALK_SOPHISTICATED},
    {"Bayes1", WALK_BAYES1},
    {"Bayes2", WALK_BAYES2},
    {"", WALK_NONE}    
};

// --------------------------------------------------------------------
//                                                        do_experiment
// --------------------------------------------------------------------
void do_experiment(int grid_index, 
                   int walk_index,
                   int steps=EXP_STEPS,
                   int pstep=EXP_STEPS/2,
                   int mult=0,
                   int verbose=false, 
                   int policy=false) {
    char basename[20];
    
    // 1. Get the grid and walkers
    Grid *g = grid_factory(grid_index);
    Walker *w = walker_factory(walk_index);
    w->set_grid(g);
    if (verbose) w->set_verbose(1);
    
    // 2. Set the base name for the output files
    strcpy(basename, "chippy2009_");
    strcat(basename, grid_initials[grid_index]);
    strcat(basename, "_");
    strcat(basename, walker_initials[walk_index]);
    
    // 3. Conduct the experiment
    Rewards *rwds = experiment(steps, pstep, mult,
                               w, basename, policy);
    
    // 4. Output the rewards received
    write_line(basename, rwds, steps, 50); 
}
    
// --------------------------------------------------------------------
//                                                 process_command_line
// --------------------------------------------------------------------
int process_command_line(int argc, char **argv, 
                         int *itest, int *igrid, int *iwalk,
                         int *repeats, bool *verbose, bool *policy)
{
    int command = CMD_NONE;
    *itest = 0;
    *igrid = 0;
    *iwalk = 0;
    *repeats = EXP_REPEAT;
    *verbose = false;
    *policy = false;
    
    for (int i=1; i < argc; ++i) {
        if ('-' == argv[i][0]) {
            switch (argv[i][1]) {
                case 'h':
                    command = CMD_HELP;
                    break;
                case 'u':
                    command = CMD_UNITTESTS;
                    break;
                case 'e':
                    command = CMD_EXPERIMENTS;
                    break;
                case 'v':
                    *verbose = true;
                    break;
                case 'p':
                    *policy = true;
                    break;
                case 't':
                    command = CMD_1_UNITTEST;
                    ++i;
                    if (i < argc) {
                        for (int t = 1; unit_tests[t].test != NULL; ++t) {
                            if (0 == strcmp(argv[i], unit_tests[t].name)) {
                                *itest = t;
                                break;
                            }
                        }
                    }    
                    break;
                case 'g':
                    command = CMD_1_EXPERIMENT;
                    ++i;
                    if (i < argc) {
                        for (int g = 1; grids[g].grid != NULL; ++g) {
                            if (0 == strcmp(argv[i], grids[g].name)) {
                                *igrid = g;
                                break;
                            }
                        }
                    }
                    break;
                case 'w':        
                    command = CMD_1_EXPERIMENT;
                    ++i;
                    if (i < argc) {
                        for (int w = 1; walkers[w].walk != WALK_NONE; ++w) {
                            if (0 == strcmp(argv[i], walkers[w].name)) {
                                *iwalk = walkers[w].walk;
                                break;
                            }
                        }
                    }
                    break;
                case 'r':        
                    ++i;
                    if (i < argc) {
                        *repeats = int(argv[i]);
                    }
                    break;
                default:
                    cout << "unknown option (" 
                    << argv[i][1] << ")" << endl;
            }
        }
    }
    return command;
}

// --------------------------------------------------------------------
//                                                         display_help
// --------------------------------------------------------------------
void display_help(void)
{
    cout << endl;
    cout << "chippy <options> <command>" << endl;
    cout << "  <command> = -h   Display this message" << endl;
    cout << "              -u   Execute all unit tests" << endl;
    cout << "              -e   Execute all experiments" << endl;
    cout << "              -t   Execute specified unittest" << endl;
    cout << "              -g   Execute experiment using specified grid" << endl; 
    cout << "              -w   Execute experiment using specified walker" << endl;
    cout << "  <options> = -r   Specify number of times experiment is repeated" << endl;
    cout << "              -v   Adds extra trace/debug information" << endl;
    cout << "              -p   Write policy file" << endl;
    cout << endl;
}

// --------------------------------------------------------------------
//                                                                 main
// --------------------------------------------------------------------
int main(int argc, char **argv)
{
    int test_index = 0;
    int grid_index = 0;
    int walk_index = 0;
    int repeats = 0;
    bool policy = false;
    bool verbose = false;
    //char *argv1t[] = {"chippyMA","-v","-t","B1CL10k",NULL}; // argc=4 
    //char *argv2t[] = {"chippyMA","-v","-t","B2CL10k",NULL}; // argc=4 
    //char *argvu[] = {"chippyMA","-u",NULL}; // argc=2 
    
    // 1. Announce ourselves
    cout << "chippy 2009.7 - MCL2 w/obserables" << endl;
    
    // 2. Randomize the random number generator
    srand(time(0)); 

    // 3. Process command line
    //argc = 2;
    //argv = argvu;
    int cmd_type = process_command_line(argc, argv,
                                        &test_index, &grid_index, &walk_index,
                                        &repeats, &verbose, &policy);
    
    // 4. Execute command
    switch (cmd_type) {
        case CMD_NONE:
            cout << "Use -h to see available commands" << endl;
            //unittests();
            //do_experiments("chippy10n10", 3);
            //do_experiments("chippy10p5", 3, 8, 10, 5);
            break;
        case CMD_HELP:
            display_help();
            break;
        case CMD_UNITTESTS:
            unittests();
            break;
        case CMD_1_UNITTEST:
            if (0 == test_index) {
                cerr << "No unit test specified" << endl;
                cerr << "Valid unit tests are:" << endl;
                for (int t=1; unit_tests[t].test != NULL; ++t) {
                    cerr << "  " << unit_tests[t].name << endl;
                }
            } else {
                if (verbose) {
                    cout << "unit test # " << test_index
                    << " " << unit_tests[test_index].name 
                    << endl;
                }
                unit_tests[test_index].test();
            }
            break;
        case CMD_1_EXPERIMENT:
            if (0 == grid_index) {
                cerr << "No grid specified" << endl;
                cerr << "Valid grid names are:" << endl;
                for (int g=1; grids[g].grid != NULL; ++g) {
                    cerr << "  " << grids[g].name << endl;
                }
            } else if (0 == walk_index) {
                cerr << "No walker specified" << endl;
                cerr << "Valid walker names are:" << endl;
                for (int w=1; walkers[w].walk != WALK_NONE; ++w) {
                    cerr << "  " << walkers[w].name << endl;
                }
            } else {
                if (verbose) {
                    cout << "experiment: grid test # " << grid_index
                    << " " << grids[grid_index].name
                    << ", walker # " << walk_index
                    << " " << walkers[walk_index].name
                    << endl;
                }    
                do_experiment(grid_index, walk_index, 
                              EXP_STEPS, EXP_STEPS/2, 0,
                              verbose, policy);
            }
            break;
        default:
            cerr << "Unimplemented command" << endl;
    }
    
    // 4. Return success
    return 0;
}

// ====================================================================
// end                 c h i p p y 2 0 0 8 . c p p                  end
// ====================================================================

