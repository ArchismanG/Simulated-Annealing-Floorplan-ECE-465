#pragma once

#include <stdint.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <vector>
#include <list>
#include <algorithm>
#include <queue>
#include <sstream>
#include <cstring>
#include <cmath>
#include <limits>
#include <utility>
#include <filesystem>
#include <random>
#include <iomanip>
#include <thread>
#include <chrono> 
#include <mutex>  

using namespace std;
extern int x_outline, y_outline;
extern std::mutex UI_MUTEX;
extern std::vector<double> LIVE_TEMPS;
extern std::vector<bool> THREADS_DONE;
// extern int num_chiplets;

struct Chiplet
{
	// Structure store simple information about the modules
	int id;
	string name = "";

	// Size information
	int x_dim;
	int y_dim; // Assuming rigid modules for now

	// No orientation information for now, rotations are only 0 or 90 degrees
	// Only need to swap dimention information since individual pins of chiplets are not specified

	// Power density information
	double powerDensity = 1.00;

	// Bottom Left coordinates
	double x_left = 0.0;
	double y_left = 0.0;

	//Centroid Coordiates
	double x_centroid = 0.0;
	double y_centroid = 0.0;
};

struct Net
{
	int size;
	std::vector<int> chipletIDs; // Stores the nets
};

struct FloorplanState
{
	std::vector<Chiplet> chiplets;
	std::vector<std::vector<int>> relationMatrix;
	std::vector<int> id_relation;

	double area = 0.0;
	double hpwl = 0.0;
	double xExceed = 0.0;
	double yExceed = 0.0;
}; // Used in simulated annealing to keep track of current, best, and new solution

std::vector<Chiplet> parseChiplets(const std::string& filename);
std::vector<Net> parseNets(const std::string& filename, const std::vector<Chiplet>& blocks);
void sortChiplets(std::vector<Chiplet>& chiplets, int low, int high);
int findChipletId(const std::vector<Chiplet>& blocks, const std::string& targetName);
void parsePowerFile(const std::string& filename, std::vector<Chiplet>& chiplets);
bool getDatasetLocs(std::string& blkf, std::string& ntf, std::string& pwrf, int dataset_no);
void compileFloorplan(std::vector<Chiplet>& chiplets, const std::vector<std::vector<int>>& relationMatrix, const std::vector<int>& id_relation);
void initializeFloorplan(const std::vector<Chiplet>& chiplets, std::vector<std::vector<int>>& relationMatrix, std::vector<int>& id_relation);
void calculateAreaMetrics(const std::vector<Chiplet>& chiplets, double& totalArea, double& xExceed, double& yExceed);
void calculateHPWL(const std::vector<Chiplet>& chiplets, const std::vector<Net>& nets, double& totalHPWL);
double calculateCost(const FloorplanState& state, const FloorplanState& bestState);
void simulatedAnnealing(int threadID, FloorplanState initialState, const std::vector<Net>& nets, FloorplanState& BestState,
	double startTemp, double coolRate, double finTemp, int mpt);
void exportFloorplan(const FloorplanState& final_state, const std::string& outfile, const double tot_x, const double tot_y);