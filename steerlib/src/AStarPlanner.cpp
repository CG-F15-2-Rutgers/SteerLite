#include "..\include\planning\AStarPlanner.h"
#include "..\include\planning\AStarPlanner.h"
#include "..\include\planning\AStarPlanner.h"
#include "..\include\planning\AStarPlanner.h"
//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))
#define VECTOR_ERROR -1

namespace SteerLib
{
	AStarPlanner::AStarPlanner() {}

	AStarPlanner::~AStarPlanner() {}

	bool AStarPlanner::canBeTraversed(int id)
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x - OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z - OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i <= x_range_max; i += GRID_STEP)
		{
			for (int j = z_range_min; j <= z_range_max; j += GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				traversal_cost += gSpatialDatabase->getTraversalCost(index);

			}
		}

		if (traversal_cost > COLLISION_COST)
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}









	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		double infinity = std::numeric_limits<double>::infinity();

		int startIndex = _gSpatialDatabase->getCellIndexFromLocation(start);
		start = getPointFromGridIndex(startIndex);

		int goalIndex = _gSpatialDatabase->getCellIndexFromLocation(goal);
		goal = getPointFromGridIndex(goalIndex);

		AStarPlannerNode startNode = AStarPlannerNode(start, 0, heuristic(start, goal), nullptr);
		AStarPlannerNode goalNode = AStarPlannerNode(goal, infinity, infinity, nullptr);

		std::cout << "Start Node Point :" << startNode.point << std::endl;

		std::vector<AStarPlannerNode> open;					// Holds node that have been evaluated
		std::vector<AStarPlannerNode> closed;				// Holds nodes that need to be evaluated

		open.push_back(startNode);

		std::vector<AStarPlannerNode> neighbors;
		AStarPlannerNode currentNode(Util::Point(0, 0, 0), 0, 0, nullptr);

		while (!open.empty()) {
			int index;
			neighbors.clear();
			currentNode = findNodeToEvaluate(open, index);
			if (currentNode == goalNode) {
				std::cout << "FOUND THE GOAL " << currentNode.point << std::endl;

				reconstructPath(currentNode, agent_path);
				return true;
			}

			closed.push_back(currentNode);
			open.erase(open.begin() + index);


			neighbors = findNeighbors(currentNode, goalNode);
			std::cout << "PARENT NODE: " << currentNode.point << " Fvalue: " << currentNode.f << std::endl;

			for (std::vector<AStarPlannerNode>::iterator iter = neighbors.begin(); iter != neighbors.end(); ++iter) {

				AStarPlannerNode neighbor = (*iter);

				if (testForNodeInVector(closed, neighbor)) {
					int offset = findNodeInVector(closed, neighbor);
					//std::cout << "IN CLOSED Neighbor: " << neighbor.point << " What Neighbors Parent Actually Is" << (closed.at(offset).parent)->point << std::endl;
					continue;
				}

				double tentative_g_score = currentNode.g + findCost(gSpatialDatabase->getCellIndexFromLocation(currentNode.point), gSpatialDatabase->getCellIndexFromLocation(neighbor.point));

				int neighborOffset = findNodeInVector(open, neighbor);

				if (neighborOffset != -1) {	//item has bee found in node

					if (tentative_g_score < neighbor.g)
					{

						open.at(neighborOffset).g = tentative_g_score;
						open.at(neighborOffset).parent = &currentNode;
						open.at(neighborOffset).f = open.at(neighborOffset).g + heuristic(neighbor.point, goalNode.point);
					}

				}
				else {
					open.push_back(neighbor);
				}

			}
		}
		return false;
	}









	// This is used as a heuristic for A*
	double AStarPlanner::heuristic(Util::Point CurrentNode, Util::Point Goal)
	{
		// returns |x_current - x_goal| + |z_current + z_goal| which is manhattan distance
		return (static_cast<double>(abs(CurrentNode.x - Goal.x) + abs(CurrentNode.z - Goal.z)));

		//Euclidean
		//return (static_cast<double>(sqrt(pow(CurrentNode.x - Goal.x, 2) + pow(CurrentNode.z - Goal.z, 2))));
	}







	// Returns distance between nodes
	double AStarPlanner::findCost(int urrentID, int neighborID)
	{
		//add support for diagonal nodes
		return 1.0;
	}






	// Finds Neighbors and creates planner nodes for them before returning a vector full of said neighbors
	std::vector<AStarPlannerNode> AStarPlanner::findNeighbors(AStarPlannerNode currentNode, AStarPlannerNode goal)
	{
		std::vector<AStarPlannerNode> neighborVector;

		int current_id = gSpatialDatabase->getCellIndexFromLocation(currentNode.point);

		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);


		// Finding the bounds

		int x_range_min, x_range_max, z_range_min, z_range_max;
		x_range_min = MAX(x - OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z - OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());

		// This loop iterates over defined range of nodes
		for (int i = x_range_min; i <= x_range_max; ++i) {
			for (int j = z_range_min; j <= z_range_max; ++j) {
				if (!((i == x) && (j == z)) && canBeTraversed(gSpatialDatabase->getCellIndexFromGridCoords(i, j))) { // Checks that we do not reinclude our current node and that all nodes we add can be traversed

					int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
					Util::Point neighborPoint;
					gSpatialDatabase->getLocationFromIndex(index, neighborPoint);	// find point where node will be

					double g = currentNode.g + findCost(current_id, gSpatialDatabase->getCellIndexFromGridCoords(i, j));				// finds g of node
					double f = g + heuristic(neighborPoint, goal.point);			// finds f of node

					AStarPlannerNode neighbor = AStarPlannerNode(neighborPoint, g, f, &currentNode);		// instantiate and initialize node

					neighborVector.push_back(neighbor);

				}
			}
		}

		return neighborVector;
	}

	int AStarPlanner::findNodeInVector(std::vector<AStarPlannerNode> open, AStarPlannerNode target)
	{
		int i = 0;
		for (std::vector<AStarPlannerNode>::iterator it = open.begin(); it != open.end(); ++it) {
			if (target == (*it)) {
				return i;
			}
			++i;
		}
		return -1;
	}




	bool AStarPlanner::testForNodeInVector(std::vector<AStarPlannerNode> closed, AStarPlannerNode target)
	{
		for (std::vector<AStarPlannerNode>::iterator it = closed.begin(); it != closed.end(); ++it) {
			if ((*it).parent != nullptr)std::cout << "Neighbor: " << (*it).point << " What Neighbors Parent Actually Is" << ((*it).parent)->point << std::endl;
			if (target == (*it)) {
				return true;
			}
		}
		return false;
	}



	AStarPlannerNode AStarPlanner::findNodeToEvaluate(std::vector<AStarPlannerNode> open, int& index)
	{
		index = -1;				// In case it somehow fails
		int i = 0;
		AStarPlannerNode temp = AStarPlannerNode(Util::Point(0, 0, 0), HUGE_VAL, HUGE_VAL, nullptr);
		for (std::vector<AStarPlannerNode>::iterator it = open.begin(); it != open.end(); ++it) {
			if (temp > (*it)) {
				index = i;
				temp = (*it);
			}
			if (temp.f == (*it).f) {
				if (temp.g < (*it).g) {

					index = i;
					temp = (*it);
				}
			}
			++i;
		}
		return temp;
	}








	void AStarPlanner::reconstructPath(AStarPlannerNode& goal, std::vector<Util::Point>& agent_path)
	{
		AStarPlannerNode* path = &goal;
		agent_path.insert(agent_path.begin(), path->point);
		while (1)
		{
			AStarPlannerNode* temp = path->parent;
			if (temp->parent == nullptr) { break; }
			agent_path.insert(agent_path.begin(), temp->point);
			path = temp;
		}
	}

}