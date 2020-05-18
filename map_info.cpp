#include <mex.h>
#include "map_info.h"

// constructor
MAP_INFO::MAP_INFO(double* map_in,
	int collision_thresh_in,
	int x_size_in,
	int y_size_in,
	int rX,
	int rY,
	double* tgt_traj,
	int tgt_step,
	int curr_time_in
) {
	// map size
	x_size = x_size_in;
	y_size = y_size_in;
	map_size = x_size * y_size;

	// robot position
	robotX = rX;
	robotY = rY;

	// copy map_in into map
	map = new double[map_size];
	// double* map = (double *)malloc(sizeof(double) * (x_size_in * y_size_in));
	memcpy(map, map_in, sizeof(double) * (map_size));

	// initialize a heuristic array that is the same size as the map array
	heuristic = new double[map_size];
	// mexPrintf("Allocated Space for heuristic\n");
	memset(heuristic, DBL_MAX, map_size);

	// // initialize updated set
	// heuristic_updated = new double[map_size];
	// memset(heuristic_updated, -1, map_size);
	// mexPrintf("initialized all heuristic fields to %lf \n", heuristic[0]);

	// update target information
	target_steps = tgt_step;						// total steps 
	target_traj = new double[target_steps * 2];		// copy target trajectory into object
	memcpy(target_traj, tgt_traj, sizeof(double) * (target_steps * 2));
	curr_time = curr_time_in;						// current step of the target

	targetX = target_traj[curr_time];
	targetY = target_traj[curr_time + target_steps];

	collision_thresh = collision_thresh_in;

	// flags
	isHComputed = 0;

	// steps to track Dijkstra steps
	steps = new int[map_size];
	memset(steps, INT_MAX, map_size);
}

// destructor
MAP_INFO::~MAP_INFO() {
	isHComputed = 0;
	delete[] map;
	delete[] heuristic;
	// delete [] heuristic_updated;
	delete[] steps;
}

/*****************************************************/
/**					Utility Functions				**/
/*****************************************************/

bool MAP_INFO::isComputed() {
	return isHComputed;
}

void MAP_INFO::Update(int rX, int rY) {
	robotX = rX;
	robotY = rY;
	return;
}

bool MAP_INFO::isOutofBound(int X_one, int Y_one) {
	return (X_one < 1 || X_one > x_size || Y_one < 1 || Y_one > y_size);
}

void MAP_INFO::WriteHeuristic2File(std::string fname) {
	std::ofstream myfile;
	myfile.open(fname);
	for (int i = 0; i < map_size; i++) {
		myfile << i << "," << heuristic[i] << "\n";
	}
	myfile << "\n";
	myfile.close();
	return;
}

// modify heuristic at index
void MAP_INFO::setHeuristicAt(int index, double H) {
	heuristic[index] = H;
	return;
}

// return heuristic from private
double MAP_INFO::getHeuristicAt(int X_one, int Y_one) {
	// return heuristic value with 0-indexing
	return heuristic[GETMAPINDEX(X_one, Y_one, x_size, y_size)];
}

// return heuristic from private
double MAP_INFO::getMapGAt(int X_one, int Y_one) {
	// return heuristic value with 0-indexing
	return map[GETMAPINDEX(X_one, Y_one, x_size, y_size)];
}

void MAP_INFO::findBestFeasiblePoints() {
	// find feasibility set
	for (int i = 0; i < target_steps; i++) {
		// target index at every time step
		int target = GETMAPINDEX(target_traj[i], target_traj[i + target_steps], x_size, y_size);
		if (steps[target] <= i) {
			lowest_cost.push(std::make_tuple(target, heuristic[target] + (i - steps[target]), i));
		}
	}
	return;
}

int MAP_INFO::getBestIndex() {
	return std::get<0>(lowest_cost.top());
}

int MAP_INFO::getBestTotLen() {
	return std::get<2>(lowest_cost.top());
}

int MAP_INFO::getBestLen() {
	return steps[getBestIndex()];
}

int MAP_INFO::nextBestIndex() {
	lowest_cost.pop();
	return getBestIndex();
}

int MAP_INFO::nextBestLen() {
	lowest_cost.pop();
	return getBestLen();
}

int MAP_INFO::getMapX() {
	return x_size;
}

int MAP_INFO::getMapY() {
	return y_size;
}

int MAP_INFO::getCollisionThresh() {
	return collision_thresh;
}

void MAP_INFO::reinitHeuristic() {
	// memset(heuristic, DBL_MAX, map_size);

	for (int i = 0; i < map_size; i++) {
		heuristic[i] = DBL_MAX;
	}
	return;
}

void MAP_INFO::reinitSteps() {
	// memset(heuristic, DBL_MAX, map_size);

	for (int i = 0; i < map_size; i++) {
		steps[i] = DBL_MAX;
	}
	return;
}

/*************************************************************/
/**				Heuristic Calculation Functions				**/
/*************************************************************/

// function to compute Dijkstra heuristic
void MAP_INFO::ComputeDHeuristic(int goalX, int goalY) {
	// declare a minHeap utilizing greater comparison so that lowest value node is always on the top
	std::priority_queue<std::tuple<int, double, int>, std::vector<std::tuple<int, double, int>>, CompareTuple> Dijkstra_pq;

	// visited set contains 0-indexing integers
	std::vector<int> visited(map_size, 0);

	// initialize heuristic for the goal node
	int currMapIndex = GETMAPINDEX(goalX, goalY, x_size, y_size);

	// re-initialize for heuristic just in case
	reinitHeuristic();

	// heuristic[currMapIndex] = 0;

	// push the goal node into the priority_queue
	std::tuple<int, double, int> start = std::make_tuple(currMapIndex, 0.0, 0);
	Dijkstra_pq.push(start);

	// 8 connected grid (from top left to bottom right, traversing horizontally)
	int dirX[8] = { -1,  0,  1, -1,  1, -1,  0,  1 };
	int dirY[8] = { -1, -1, -1,  0,  0,  1,  1,  1 };

	// Dijkstra Loop
	while (!Dijkstra_pq.empty()) {
		// mexPrintf("Size of the priority_queue: %d.\n", Dijkstra_pq.size());
		// get the top DijkstraNode 
		std::tuple<int, double, int> tmp = Dijkstra_pq.top();
		Dijkstra_pq.pop();

		int tmpIndex = std::get<0>(tmp);
		double tmpVal = std::get<1>(tmp);
		int tmpX = tmpIndex % x_size + 1;
		int tmpY = tmpIndex / x_size + 1;
		int tmpS = std::get<2>(tmp);

		if (visited[tmpIndex] == 0) {
			// mexPrintf("Not visited.\n");
			visited[tmpIndex] = 1;
			for (int i = 0; i < 8; i++) {
				int neiX = tmpX + dirX[i];							// 1-indexing
				int neiY = tmpY + dirY[i];							// 1-indexing
				int neiMapIndex = GETMAPINDEX(neiX, neiY, x_size, y_size);	// 0-indexing
				// mexPrintf("Neighbor node indices: (%d, %d, %d). \n", neiX, neiY, neiMapIndex);
				if (!isOutofBound(neiX, neiY) && visited[neiMapIndex] == 0) {
					if (map[neiMapIndex] < collision_thresh) {
						// update heuristics with the smaller value b/t current val and tmp->val+map_cost 
						if ((tmpVal + map[neiMapIndex]) < heuristic[neiMapIndex]) {
							heuristic[neiMapIndex] = tmpVal + map[neiMapIndex];
							Dijkstra_pq.push(std::make_tuple(neiMapIndex, heuristic[neiMapIndex], tmpS + 1));
							steps[neiMapIndex] = tmpS + 1;
						}
						// steps[neiMapIndex] = MIN(tmpS + 1, steps[neiMapIndex]);
					}
					else {
						visited[neiMapIndex] = 1;					// 0-indexing
					}
				}
			}
		}
	}
	// print set element for debug
	// mexPrintf("Number of elements in the visited set: %d.\n", visited.size());
	isHComputed = 1;
	return;
}

// double MAP_INFO::ComputeManhattan(int curX, int curY, int goalX, int goalY){
// 	return (std::abs(curX - goalX) + std::abs(curY + goalY)); 
// }

// void MAP_INFO::ComputeMHeuristic(int goalX, int goalY){
// 	if (!isHComputed){
// 		for (int i = 1; i <= x_size; i++){
// 			for (int j = 1; j <= y_size; j++){
// 				int currMapIndex = GETMAPINDEX(i, j, x_size, y_size);
// 				if (map[currMapIndex] < collision_thresh){
// 					heuristic[currMapIndex] = ComputeManhattan(i, j, goalX, goalY);
// 				}
// 			}
// 		}
// 		isHComputed = 1;
// 	} else {
// 		for (int i = 1; i <= x_size; i++){
// 			for (int j = 1; j <= y_size; j++){
// 				int currMapIndex = GETMAPINDEX(i, j, x_size, y_size);
// 				if (map[currMapIndex] < collision_thresh){
// 					heuristic_updated[currMapIndex] = ComputeManhattan(i, j, goalX, goalY);
// 				}
// 			}
// 		}
// 		isHComputed = 1;
// 	}
// 	return;
// }

// void MAP_INFO::UpdateMHeuristic(int goalX, int goalY, int curr_index){
// 	heuristic[curr_index] = heuristic[GETMAPINDEX(goalX, goalY, x_size, y_size)] - heuristic[curr_index];
// 	return;
// }

// void MAP_INFO::CorrectMHeuristic(int curr_index, int new_goalX, int new_goalY){
// 	// to use this function, new_goalX and new_goalY need to be supplied. 
// 	// before use, updated heuristic need to be calculated.
// 	heuristic[curr_index] = MAX(heuristic_updated[curr_index], 
// 		(heuristic[curr_index] - heuristic[GETMAPINDEX(new_goalX, new_goalY, x_size, y_size)]));
// 	return;
// }