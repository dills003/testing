//Taken from quiz
#ifndef COST_H
#define COST_H

#include <vector>

using namespace std;

double LaneSpeed(const vector<vector<double> > lane, double desiredSpeed); //slowest car in lanes

int BestLane(const vector<vector<double> > leftLane, const vector<vector<double> > middleLane, const vector<vector<double> > rightLane, double desiredSpeed, double currentSpeed, int currentLane, double myCars);

double CongestionCost(const vector<vector<double> > lane, double myCars);

int BestLaneDecider(double leftLaneCost, double middleLaneCost, double rightLaneCost, int currentLane);

bool CanIMerge(const vector<vector<double> > aheadCars, const vector<vector<double> > behindCars, double myCars);

//double PredictCars()

//add congestion and openness cost functions
//don't slow during lane change

#endif
