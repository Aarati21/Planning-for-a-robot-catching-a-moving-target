/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <iostream> 
#include <math.h>
#include <mex.h>
#include <cstdio>
#include <queue>
#include <vector>
#include <chrono>
#include <unordered_map>


/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, x_size, y_size) ((Y-1)*x_size + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 9
using namespace std;



bool goal_reached = false;
vector<int>H_VAL;
static vector<int>H_VAL1;
auto start = chrono::high_resolution_clock::now();
int* steps;
int x_size, y_size, collision_thresh;
double* map;


// 8-connected grid
// added 9th position to account for staying in the same place
const int dX[NUMOFDIRS] = { -1, -1, -1,  0,  0,  1, 1, 1, 0 };
const int dY[NUMOFDIRS] = { -1,  0,  1, -1,  1, -1, 0, 1, 0 };


// check whether cell is vacant, has obstacle and whether time is valid

bool cell_validity(int x, int y, int curr_time, int target_steps) {

    if ((x > 0) && (x <= x_size) && (y > 0) && (y <= y_size) && (curr_time <= target_steps) )
        {
         return true;
        }
    else
    {
        return false;
    }
};




class state {
    
public:
    int x, y, t, g, f, a;
    state(int x, int y, int t, int g, int f, int a): x(x), y(y), t(t), g(g), f(f), a(a) {}
    bool operator==(const state& s) const
    {
        return (x == s.x && y == s.y && t == s.t);
    }
 };


class stateG {  
public:
    int x, y, s, h;
    stateG(int x, int y, int s, int h) : x(x), y(y), s(s), h(h) {}
   
};



struct Point {
    int x, y, n;
    Point(int x, int y, int n) : x(x), y(y), n(n) {}
};

struct min_heap_h {
    bool operator()(stateG const& p1, stateG const& p2)
    {
        return p1.h > p2.h;
    }
};

struct min_heap_f {
    bool operator()(state const& p1, state const& p2)
    {
        return p1.f > p2.f;
    }
};

struct max_heap_g {
    bool operator()(state const& p1, state const& p2)
    {
        return p1.g < p2.g;
    }
};

struct min_heap_heur {
    bool operator() (tuple<int, int,  double, int> p1, tuple<int, int,  double, int> p2) {
        return get<2>(p1) > get<2>(p2);
    }
};
vector<int> computeHeuristic(int goalX, int goalY, int curr_time, int target_steps) {

    int map_size = x_size * y_size;
    priority_queue<stateG, vector<stateG>, min_heap_h> OPEN;
    vector<int> H_VALUES(map_size, -1);
    vector<bool> EXPANDED(map_size, 0);

    
    stateG starget = stateG(goalX, goalY, 0, 0);
    H_VALUES[GETMAPINDEX(starget.x, starget.y, x_size, y_size)] = 0;
    OPEN.push(starget);
    

    while (OPEN.empty() == 0) {
        stateG s = OPEN.top();
        OPEN.pop();
       
        int map_index1 = GETMAPINDEX(s.x, s.y, x_size, y_size);
        if (!(EXPANDED[map_index1])) {
            EXPANDED[map_index1] = 1;
            H_VALUES[map_index1] = s.h;
            
            for (int i = 0; i < 8; ++i) {
                int X = s.x + dX[i];
                int Y = s.y + dY[i];
                int TS = s.s + 1;
                    if (cell_validity(X, Y, TS, target_steps)) {
                        int cell_cost = ((int)map[GETMAPINDEX(X, Y, x_size, y_size)]);
                            if (cell_cost < collision_thresh){
                            int map_index2 = GETMAPINDEX(X, Y, x_size, y_size);
                            int cost = (int)map[map_index2];
                            if ((H_VALUES[map_index2] > cost + s.h) || (!(EXPANDED[map_index2]))) {
                                H_VALUES[map_index2] = cost + s.h;
                                OPEN.push(stateG(X, Y, TS, H_VALUES[map_index2]));
                                steps[map_index2] = TS;
                            }
                            }

                    }
            }

        }

    }
    return H_VALUES;
}



Point Astar3D(int x_start, int y_start, int targetX, int targetY, int curr_time, int target_steps) {

    int map_size = x_size * y_size;
    vector<int> G_VALUES(map_size, -1);
    vector<bool> EXPANDED(map_size, 0);
    priority_queue<state, vector<state>, min_heap_f> OPEN;
    priority_queue<state, vector<state>, max_heap_g> CLOSED;
    state s_start = state(x_start, y_start, curr_time, 0, H_VAL1[GETMAPINDEX(x_start, y_start, x_size, y_size)], -1);

    OPEN.push(s_start);
    G_VALUES[GETMAPINDEX(s_start.x, s_start.y, x_size, y_size)] = 0;

    bool GOAL_EXPANDED = false;
    while (!OPEN.empty() && !GOAL_EXPANDED) {

        state s = OPEN.top();
        OPEN.pop();

        if (!EXPANDED[GETMAPINDEX(s.x, s.y, x_size, y_size)]) {
            EXPANDED[GETMAPINDEX(s.x, s.y, x_size, y_size)] = 1;
            CLOSED.push(s);
            
            for (int i = 0; i < 9; ++i) {
                int X = s.x + dX[i];
                int Y = s.y + dY[i];
                int T = s.t + 1;

                if (cell_validity(X, Y, T, target_steps)) {

                    int cell_cost = ((int)map[GETMAPINDEX(X, Y, x_size, y_size)]);
                    if (cell_cost < collision_thresh) {


                        int new_index = GETMAPINDEX(s.x + dX[i], s.y + dY[i], x_size, y_size);

                        if (!EXPANDED[new_index] && (G_VALUES[new_index] == -1 || (G_VALUES[new_index] > (((int)map[new_index]) + s.g)))) {
                            int g_new = ((int)map[new_index]) + s.g;
                            G_VALUES[new_index] = g_new;
                            OPEN.push(state(s.x + dX[i], s.y + dY[i], s.t + 1, g_new, g_new + H_VAL1[new_index], i));
                        }
                    }
                }
            }

            if ((s.x == targetX) && (s.y == targetY) ) {
                GOAL_EXPANDED = true;

                while (!(s == CLOSED.top())) 
                    CLOSED.pop();

                state sn = CLOSED.top();
                CLOSED.pop();
               
                while (CLOSED.empty() == 0) {
                                       
                    state sp = state(sn.x - dX[sn.a], sn.y - dY[sn.a], sn.t - 1, 0, 0, 0);
                    while (!(sp == CLOSED.top())) CLOSED.pop();
                    sp = CLOSED.top();
                    CLOSED.pop();
                    if (sp.a == -1) 
                        return Point(sn.x, sn.y, 0);
                    else sn = sp;
                }
                return Point(s.x, s.y, 0);
            }

        }
    }
    
}













static void planner(
    double* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    double* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    double* action_ptr
)
{
   
   
    steps = new int[x_size * y_size];
    priority_queue<tuple<int, int, double, int>, vector<tuple<int, int, double, int>>, min_heap_heur> lowest_cost;
    static int Goal, robot_len, GoalX, GoalY;
    
    
    // calculate goal position
   if (curr_time == 0){
        H_VAL = computeHeuristic(robotposeX, robotposeY, curr_time, target_steps);
        
        for (int i = 0; i < target_steps; i++) {
           
            int index = GETMAPINDEX(target_traj[i], target_traj[i+ target_steps], x_size, y_size);
            
            if (steps[index] <=i) {
              
                lowest_cost.push(make_tuple(target_traj[i], target_traj[i + target_steps], H_VAL[index] + (i - steps[index]), i));
            }
        }
        
            
            GoalX = get<0>(lowest_cost.top());
            GoalY = get<1>(lowest_cost.top());
            int goalindex = GETMAPINDEX(GoalX, GoalY, x_size, y_size);
            robot_len = steps[goalindex];
            H_VAL1 = computeHeuristic(GoalX, GoalY, curr_time, robot_len);
        
    }
   
   
   // If goal is reached stay in same position
    if ((robotposeX == GoalX) && (robotposeY == GoalY)) {
        
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        return;
    }
    
    //perform A * 3D search
    
    Point RobotPose = Astar3D(robotposeX, robotposeY, GoalX, GoalY, curr_time, robot_len);

    action_ptr[0] = RobotPose.x;
    action_ptr[1] = RobotPose.y;
      
    
    
    return;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    x_size = mxGetM(MAP_IN);
    y_size = mxGetN(MAP_IN);
    map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
   
    return;   
}