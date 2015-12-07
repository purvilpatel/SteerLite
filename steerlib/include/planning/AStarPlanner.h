//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#ifndef __STEERLIB_A_STAR_PLANNER_H__
#define __STEERLIB_A_STAR_PLANNER_H__


#include <vector>
#include <stack>
#include <set>
#include <iostream>
#include <map>
#include "SteerLib.h"

namespace SteerLib
{
	class SearchNode;
	typedef std::shared_ptr<SearchNode> SearchNodePtr;
	class SearchNode {
	public:
		SearchNode(unsigned int index, float g, float h) : _index(index), _g(g), _h(h), _previous(nullptr) {}
		float g() const { return _g; }
		void g(float val) {
			_g = val;
		}
		float h() const { return _h; }
		float f() const { return _g + _h; }
		float index() const { return _index; }
		void prev(SearchNodePtr node) {
			this->_previous = node;
		}
		SearchNodePtr prev() {
			return this->_previous;
		}
		void printDebug() const {
			std::cout << "node.index = " << _index << " node.f = " << f()
				<< " node.g = " << _g << " node.h = " << _h << std::endl;
		}
	private:
		SearchNodePtr _previous;
		unsigned int _index;
		// The exact cost of reaching this node from the start.
		float _g;
		// The estimated cost of reaching the goal from this node.
		float _h;
		bool _expanded = false;
		bool _visited = false;
	};

	/*
	@function The AStarPlannerNode class gives a suggested container to build your search tree nodes.
	@attributes
	f : the f value of the node
	g : the cost from the start, for the node
	point : the point in (x,0,z) space that corresponds to the current node
	parent : the pointer to the parent AStarPlannerNode, so that retracing the path is possible.
	@operators
	The greater than, less than and equals operator have been overloaded. This means that objects of this class can be used with these operators. Change the functionality of the operators depending upon your implementation

	*/
	class STEERLIB_API AStarPlannerNode {
	public:
		double f;
		double g;
		Util::Point point;
		AStarPlannerNode* parent;
		AStarPlannerNode(Util::Point _point, double _g, double _f, AStarPlannerNode* _parent)
		{
			f = _f;
			point = _point;
			g = _g;
			parent = _parent;
		}
		bool operator<(AStarPlannerNode other) const
		{
			return this->f < other.f;
		}
		bool operator>(AStarPlannerNode other) const
		{
			return this->f > other.f;
		}
		bool operator==(AStarPlannerNode other) const
		{
			return ((this->point.x == other.point.x) && (this->point.z == other.point.z));
		}

	};



	class STEERLIB_API AStarPlanner {
	public:
		AStarPlanner();
		~AStarPlanner();
		// NOTE: There are four indices that need to be disambiguated
		// -- Util::Points in 3D space(with Y=0)
		// -- (double X, double Z) Points with the X and Z coordinates of the actual points
		// -- (int X_GRID, int Z_GRID) Points with the row and column coordinates of the GridDatabase2D. The Grid database can start from any physical point(say -100,-100). So X_GRID and X need not match
		// -- int GridIndex  is the index of the GRID data structure. This is an unique id mapping to every cell.
		// When navigating the space or the Grid, do not mix the above up

		/*
		@function canBeTraversed checkes for a OBSTACLE_CLEARANCE area around the node index id for the presence of obstacles.
		The function finds the grid coordinates for the cell index  as (X_GRID, Z_GRID)
		and checks cells in bounding box area
		[[X_GRID-OBSTACLE_CLEARANCE, X_GRID+OBSTACLE_CLEARANCE],
		[Z_GRID-OBSTACLE_CLEARANCE, Z_GRID+OBSTACLE_CLEARANCE]]
		This function also contains the griddatabase call that gets traversal costs.
		*/
		bool canBeTraversed(int id);
		/*
		@function getPointFromGridIndex accepts the grid index as input and returns an Util::Point corresponding to the center of that cell.
		*/
		Util::Point getPointFromGridIndex(int id);

		/*
		@function computePath
		DO NOT CHANGE THE DEFINITION OF THIS FUNCTION
		This function executes an A* query
		@parameters
		agent_path : The solution path that is populated by the A* search
		start : The start point
		goal : The goal point
		_gSpatialDatabase : The pointer to the GridDatabase2D from the agent
		append_to_path : An optional argument to append to agent_path instead of overwriting it.
		*/

		bool computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path = false);
	private:
		void _tryToAdd(unsigned int x, unsigned int z, const SearchNodePtr& from, float cost, Util::Point goal, std::vector<SearchNodePtr>& out);
		std::vector<SearchNodePtr> _expand(const SearchNodePtr& node, Util::Point goal);
		SteerLib::GridDatabase2D * gSpatialDatabase;
	};


}


#endif
