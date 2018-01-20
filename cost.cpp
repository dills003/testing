//Taken from quiz

#include "cost.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>
#include <vector>
#include <iostream>

using namespace std;

//Cars of Interest come in as speed, s, and then d
const double MERGE_BUFFER_AHEAD = 15.0; //makes sure we have space behind and infront of merge spot //30 and 20worked
const double MERGE_BUFFER_BEHIND = 25.0;
const double SPEED_COST_MULT = 1.0; //the overall cost of the lane speed
const double CONGESTION_COST_MULT = 500.0; //the cost of how close a car is in a merge lane //1000 worked


int BestLane(const vector<vector<double> > leftLane, const vector<vector<double> > middleLane, const vector<vector<double> > rightLane, double desiredSpeed, double currentSpeed, int currentLane, double myCars)
{
	int bestLane = 99;

	double leftLaneSpeed = LaneSpeed(leftLane, desiredSpeed);
	double middleLaneSpeed = LaneSpeed(middleLane, desiredSpeed);
	double rightLaneSpeed = LaneSpeed(rightLane, desiredSpeed);

	double leftLaneSpeedCost = (desiredSpeed - leftLaneSpeed) * SPEED_COST_MULT;
	double middleLaneSpeedCost = (desiredSpeed - middleLaneSpeed) * SPEED_COST_MULT;
	double rightLaneSpeedCost = (desiredSpeed - rightLaneSpeed) * SPEED_COST_MULT;

	double leftLaneCongestionCost = CongestionCost(leftLane, myCars) * CONGESTION_COST_MULT;
	double middleLaneCongestionCost = CongestionCost(middleLane, myCars) * CONGESTION_COST_MULT;
	double rightLaneCongestionCost = CongestionCost(rightLane, myCars) * CONGESTION_COST_MULT;

	double leftLaneCost = leftLaneSpeedCost + leftLaneCongestionCost;
	double middleLaneCost = middleLaneSpeedCost + middleLaneCongestionCost;
	double rightLaneCost = rightLaneSpeedCost + rightLaneCongestionCost;
	
	/*cout << "left speed cost: " << leftLaneSpeedCost << "	middle speed cost: " << middleLaneSpeedCost << "	right speed cost: " << rightLaneSpeedCost << endl;
	cout << "left congest cost: " << leftLaneCongestionCost << "	middle congest cost: " << middleLaneCongestionCost << "	right congest cost: " << rightLaneCongestionCost << endl;
	cout << "left cost: " << leftLaneCost << "	middle cost: " << middleLaneCost << "	right cost: " << rightLaneCost << endl;
	*/
	//pick the best lane
	bestLane = BestLaneDecider(leftLaneCost, middleLaneCost, rightLaneCost, currentLane);

	return bestLane;

}


double LaneSpeed(const vector<vector<double> > lane, double desiredSpeed) //slowest car in lanes
{
	double laneSpeed = desiredSpeed; 

	if (lane.size() == 0) //if the lane is empty, that is awesome
	{
		laneSpeed = desiredSpeed;
	}
	else
	{
		for (int i = 0; i < lane.size(); i++)
		{
			if (lane[i][0] < laneSpeed)
			{
				laneSpeed = lane[i][0];
			}

		}
	}

	return laneSpeed;

}


int BestLaneDecider(double leftLaneCost, double middleLaneCost, double rightLaneCost, int currentLane)
{
	int bestLane = 99;
	double currentLaneCost = 0.0;

	//Find the current lane's cost, keep lane if tie
	if (currentLane == 0)
	{
		currentLaneCost = leftLaneCost;
	}
	else if (currentLane == 1)
	{
		currentLaneCost = middleLaneCost;
	}
	else
		currentLaneCost = rightLaneCost;

	//I want it to stay put if we have a tie and move over to the next best lane not two if we have a tie
	if (leftLaneCost <= middleLaneCost && leftLaneCost <= rightLaneCost && leftLaneCost != currentLaneCost)
	{
		bestLane = 0;
	}
	else if (middleLaneCost < leftLaneCost && middleLaneCost <= rightLaneCost && middleLaneCost != currentLaneCost)
	{
		bestLane = 1;
	}
	else if (rightLaneCost < leftLaneCost && rightLaneCost < middleLaneCost && rightLaneCost != currentLaneCost)
	{
		bestLane = 2;
	}
	else
		bestLane = currentLane; //if we have a tie from current lane, we just stay put


	return bestLane;

}


bool CanIMerge(const vector<vector<double> > aheadCars, const vector<vector<double> > behindCars, double myCars)
{
	bool canIMergeFront = false;
	bool canIMergeBack = false;
	bool canIMerge = false;
	double temp_s = 0.0;
	double mindelta_s = MERGE_BUFFER_AHEAD;

	//loop through the vectors ahead and behind to see if any cars in buffer zone; add prediction stuff before this future
	if (aheadCars.size() > 0)
	{
		for (int i = 0; i < aheadCars.size(); i++)
		{
			temp_s = aheadCars[i][1];
			temp_s -= myCars;
			if (temp_s < MERGE_BUFFER_AHEAD) //if any car is in the bubble
			{
				mindelta_s = temp_s;
			}
			//cout << "CARS ahead: " << aheadCars[i][1] << "	MineS: " << myCars << " Size: " << aheadCars.size() << " temp_s: " << endl;
		}
		if (mindelta_s < MERGE_BUFFER_AHEAD)
		{
			canIMergeFront = false;
		}
		else
			canIMergeFront = true;
	}
	else
		canIMergeFront = true;
	

	mindelta_s = MERGE_BUFFER_BEHIND;

	if (behindCars.size() > 0)
	{
		for (int i = 0; i < behindCars.size(); i++)
		{
			temp_s = behindCars[i][1];
			temp_s = myCars - temp_s;
			if (temp_s < MERGE_BUFFER_BEHIND) //if any car is in the bubble
			{
				mindelta_s = temp_s;
			}
			//cout << "CARS behind: " << behindCars[i][1] << "	MineS: " << myCars << " Size: " << behindCars.size() << " temp_s: " << endl;
		}
		if (mindelta_s < MERGE_BUFFER_BEHIND)
		{
			canIMergeBack = false;
		}
		else
			canIMergeBack = true;
	}
	else
		canIMergeBack = true;


	if (canIMergeFront && canIMergeBack)
	{
		canIMerge = true;
	}

	return canIMerge;
	
}

double CongestionCost(const vector<vector<double> > lane, double myCars) //basically to see what lane is most open from my point
{
	double congestionCost = 0.0;
	double temp_s = 0.0;
	double mindelta_s = 10000; //bigger than ever...this whole project has turned into a mess

	//loop through the vectors ahead and see what sort of gap we have
	if (lane.size() > 0)
	{
		for (int i = 0; i < lane.size(); i++)
		{
			temp_s = lane[i][1];
			temp_s -= myCars;
			if (temp_s < mindelta_s) //if new car closer than old car
			{
				mindelta_s = temp_s;
			}
			
		}
		congestionCost = 1.0 / mindelta_s; //punish smaller gaps
	}
	else
		congestionCost = 0.0; //nothing in the lane, we have no cost

	return congestionCost;
}