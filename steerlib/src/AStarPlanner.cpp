//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <array>
#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include <limits>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

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


	// This operator overload is necessary for std::find.
	bool operator==(const SearchNodePtr& a, const SearchNodePtr& b) {
		return a->index() == b->index();
	}

	// This operator overload is necessary for std::min_element.
	bool operator<(const SearchNodePtr& a, const SearchNodePtr& b) {
		if (a->f() == b->f()) {
			return a->g() > b->g();
		}
		else {
			return a->f() < b->f();
		}
	}

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		int startIndex = gSpatialDatabase->getCellIndexFromLocation(start);
		SearchNodePtr startNode(new SearchNode(startIndex, 0, 0));

		int goalIndex = gSpatialDatabase->getCellIndexFromLocation(goal);

		std::vector<SearchNodePtr> openSet;
		std::vector<SearchNodePtr> closedSet;
		SearchNodePtr goalNode(nullptr);
		openSet.push_back(startNode);

		while (!openSet.empty()) {
			// Get the node with the minimum f, breaking ties on g. 
			std::vector<SearchNodePtr>::iterator minIter = std::min_element(openSet.begin(), openSet.end());
			SearchNodePtr minNode = *minIter;
			openSet.erase(minIter);

			// If we're visiting the goal, we're finished.
			if (minNode->index() == goalIndex) {
				goalNode = minNode;
				break;
			}

			// Expand this node.
			std::vector<SearchNodePtr> expandedList = _expand(minNode, goal);
			for (SearchNodePtr& expandedNode : expandedList) {
				// If this cell is already in the open set, check if this is a cheaper path.
				std::vector<SearchNodePtr>::iterator iter = std::find(openSet.begin(), openSet.end(), expandedNode);
				if (iter != openSet.end()) {
					if (expandedNode->g() < (*iter)->g()) {
						(*iter)->g(expandedNode->g());
						(*iter)->prev(minNode);
					}
				}
				else {
					if (std::find(closedSet.begin(), closedSet.end(), expandedNode) == closedSet.end()) {
						expandedNode->prev(minNode);
						openSet.push_back(expandedNode);
					}
				}
			}
			closedSet.push_back(minNode);
		}

		// Check if a path to the goal was not found.
		if (goalNode == nullptr) {
			return false;
		}

		// Go back from goal node to start.
		SearchNodePtr ptr = goalNode;
		while (ptr != nullptr) {
			Util::Point point;
			gSpatialDatabase->getLocationFromIndex(ptr->index(), point);
			agent_path.insert(agent_path.begin(), point);
			ptr = ptr->prev();
		}

		return true;
	}

	// Helper method which attempts to add a SearchNode to the output vector if it is traversable.
	void AStarPlanner::_tryToAdd(unsigned int x, unsigned int z, const SearchNodePtr& from, float cost, Util::Point goal, std::vector<SearchNodePtr>& out) {
		int index = gSpatialDatabase->getCellIndexFromGridCoords(x, z);
		if (!canBeTraversed(index)) return;
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(index, p);
		float h = distanceBetween(p, goal);
		//float h = distanceSquaredBetween(p, goal);
		out.push_back(SearchNodePtr(new SearchNode(index, from->g() + cost, h)));
	}

	// Returns a list of neighboring traversable cells.
	std::vector<SearchNodePtr> AStarPlanner::_expand(const SearchNodePtr& node, const Util::Point goal) {
		unsigned int x, z;
		int index;
		std::vector<SearchNodePtr> out;
		gSpatialDatabase->getGridCoordinatesFromIndex(node->index(), x, z);

		// Try to add the four cardinal directions.
		_tryToAdd(x - 1, z, node, 1, goal, out);
		_tryToAdd(x + 1, z, node, 1, goal, out);
		_tryToAdd(x, z - 1, node, 1, goal, out);
		_tryToAdd(x, z + 1, node, 1, goal, out);

		// Try the diagonals, with cost approximately sqrt(2).
		// NOTE: I realized that for part 1, the diagonal costs should be
		// equal to the regular costs. Therefore, for part 3, we only need to increase
		// the costs below.
		_tryToAdd(x - 1, z - 1, node, 1, goal, out);
		_tryToAdd(x - 1, z + 1, node, 1, goal, out);
		_tryToAdd(x + 1, z - 1, node, 1, goal, out);
		_tryToAdd(x + 1, z + 1, node, 1, goal, out);

		return std::move(out);
	}
}