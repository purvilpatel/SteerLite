//
// Copyright (c) 2015 Mahyar Khayatkhoei
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include <algorithm>
#include <vector>
#include <set>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as timeStep size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
	int index = 1;
	int vectorSize = controlPoints.size();
	Point newPosition, oldPosition;
	oldPosition = controlPoints.at(0).position;

	for (float timeSamples = controlPoints.at(0).time; timeSamples <= controlPoints.at(vectorSize - 1).time; timeSamples += (float)window)
	{
		if (type == hermiteCurve)
		{
			newPosition = useHermiteCurve(index, timeSamples);
		}
		else if (type == catmullCurve)
		{
			newPosition = useCatmullCurve(index, timeSamples);
		}
		Util::DrawLib::drawLine(oldPosition, newPosition, curveColor);
		oldPosition = newPosition;
		if (timeSamples >= controlPoints.at(index).time)
			index++;
	}
#ifdef ENABLE_GUI
	return;
#endif
}

bool MyPointSortPredicate(const CurvePoint& point1, const CurvePoint& point2)
{
	return point1.time < point2.time;
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	// Sort the vector using predicate and std::sort
	std::sort(controlPoints.begin(), controlPoints.end(), MyPointSortPredicate);

	for (int i = controlPoints.size() - 1; i > 0; i--) {
		if (controlPoints.at(i).time == controlPoints.at(i - 1).time)
		{
			controlPoints.erase(controlPoints.begin() + i);
		}
	}
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	if (type == hermiteCurve)
	{
		if (controlPoints.size() >= 2)
			return true;
		else
			return false;
	}
	else if (type == catmullCurve)
	{
		if (controlPoints.size() >= 3)
			return true;
		else
			return false;
	}
	return false;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{

	for (int i = 0; i < controlPoints.size(); i++)
	{
		CurvePoint CurrentPoint = controlPoints.at(i);

		if (CurrentPoint.time > time)
		{
			nextPoint = i;
			break;
		}
	}

	return nextPoint < controlPoints.size() ? true : false;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	CurvePoint nextCurvePoint = controlPoints.at(nextPoint);
	CurvePoint previousCurvePoint = controlPoints.at(nextPoint - 1);
	float timeStep = (time - previousCurvePoint.time) / (nextCurvePoint.time - previousCurvePoint.time);
	float timeDiff = (nextCurvePoint.time - previousCurvePoint.time);

	// Calculate position at t = time on Hermite curve

	float h1 = 2 * pow(timeStep, 3) - 3 * pow(timeStep, 2) + 1;       // calculate basis function 1
	float h2 = -2 * pow(timeStep, 3) + 3 * pow(timeStep, 2);          // calculate basis function 2
	float h3 = pow(timeStep, 3) - 2 * pow(timeStep, 2) + timeStep;    // calculate basis function 3
	float h4 = pow(timeStep, 3) - pow(timeStep, 2);				      // calculate basis function 4
	h3  *= timeDiff;
	h4  *= timeDiff;

	newPosition = previousCurvePoint.position;
	newPosition.x = h1  *  previousCurvePoint.position.x +
		h2  *  nextCurvePoint.position.x +
		h3  *  previousCurvePoint.tangent.x +
		h4  *    nextCurvePoint.tangent.x;

	newPosition.y = h1  *  previousCurvePoint.position.y +
		h2  *  nextCurvePoint.position.y +
		h3  *   previousCurvePoint.tangent.y +
		h4  *    nextCurvePoint.tangent.y;

	newPosition.z = h1  *  previousCurvePoint.position.z +
		h2  *  nextCurvePoint.position.z +
		h3  *   previousCurvePoint.tangent.z +
		h4  *    nextCurvePoint.tangent.z;

	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	CurvePoint nextCurvePoint = controlPoints.at(nextPoint);
	CurvePoint previousCurvePoint = controlPoints.at(nextPoint - 1);
	float timeStep = (time - previousCurvePoint.time) / (nextCurvePoint.time - previousCurvePoint.time);
	float timeDiff = (nextCurvePoint.time - previousCurvePoint.time);

	float tangentOneX, tangentOneY, tangentOneZ, tangentTwoX, tangentTwoY, tangentTwoZ;

	//Case 1: Starting endpoint
	if (nextPoint == 1)
	{
		tangentOneX = ((controlPoints.at(2).time - controlPoints.at(0).time) / 
			(controlPoints.at(2).time - controlPoints.at(1).time)) *
			((controlPoints.at(1).position.x - controlPoints.at(0).position.x) / 
			(controlPoints.at(1).time - controlPoints.at(0).time)) - 
			((controlPoints.at(1).time - controlPoints.at(0).time) /
			(controlPoints.at(2).time - controlPoints.at(1).time)) * 
			((controlPoints.at(2).position.x - controlPoints.at(0).position.x) / 
			(controlPoints.at(2).time - controlPoints.at(0).time));
			
		tangentOneY = ((controlPoints.at(2).time - controlPoints.at(0).time) / 
			(controlPoints.at(2).time - controlPoints.at(1).time)) * 
			((controlPoints.at(1).position.y - controlPoints.at(0).position.y) /
			(controlPoints.at(1).time - controlPoints.at(0).time)) - 
			((controlPoints.at(1).time - controlPoints.at(0).time) / 
			(controlPoints.at(2).time - controlPoints.at(1).time)) *
			((controlPoints.at(2).position.y - controlPoints.at(0).position.y) / 
			(controlPoints.at(2).time - controlPoints.at(0).time));
			
		tangentOneZ = ((controlPoints.at(2).time - controlPoints.at(0).time) /
			(controlPoints.at(2).time - controlPoints.at(1).time)) * 
			((controlPoints.at(1).position.z - controlPoints.at(0).position.z) /
			(controlPoints.at(1).time - controlPoints.at(0).time)) - 
			((controlPoints.at(1).time - controlPoints.at(0).time) /
			(controlPoints.at(2).time - controlPoints.at(1).time)) * 
			((controlPoints.at(2).position.z - controlPoints.at(0).position.z) /
			(controlPoints.at(2).time - controlPoints.at(0).time));
			
		tangentTwoX = ((controlPoints.at(1).time - controlPoints.at(0).time) / 
			(controlPoints.at(2).time - controlPoints.at(0).time)) *
			((controlPoints.at(2).position.x - controlPoints.at(1).position.x) /
			(controlPoints.at(2).time - controlPoints.at(1).time)) + 
			((controlPoints.at(2).time - controlPoints.at(1).time) /
			(controlPoints.at(2).time - controlPoints.at(0).time)) * 
			((controlPoints.at(1).position.x - controlPoints.at(0).position.x) / 
			(controlPoints.at(1).time - controlPoints.at(0).time));
			
		tangentTwoY = ((controlPoints.at(1).time - controlPoints.at(0).time) /
			(controlPoints.at(2).time - controlPoints.at(0).time)) *
			((controlPoints.at(2).position.y - controlPoints.at(1).position.y) / 
			(controlPoints.at(2).time - controlPoints.at(1).time)) + 
			((controlPoints.at(2).time - controlPoints.at(1).time) / 
			(controlPoints.at(2).time - controlPoints.at(0).time)) * 
			((controlPoints.at(1).position.y - controlPoints.at(0).position.y) / 
			(controlPoints.at(1).time - controlPoints.at(0).time));
			
		tangentTwoZ = ((controlPoints.at(1).time - controlPoints.at(0).time) /
			(controlPoints.at(2).time - controlPoints.at(0).time)) * 
			((controlPoints.at(2).position.z - controlPoints.at(1).position.z) / 
			(controlPoints.at(2).time - controlPoints.at(1).time)) + 
			((controlPoints.at(2).time - controlPoints.at(1).time) / 
			(controlPoints.at(2).time - controlPoints.at(0).time)) *
			((controlPoints.at(1).position.z - controlPoints.at(0).position.z) /
			(controlPoints.at(1).time - controlPoints.at(0).time));
	}

	//Case 2: Ending endpoint
	else if (nextPoint == controlPoints.size() - 1)
	{
		tangentOneX = ((controlPoints.at(nextPoint - 1).time - controlPoints.at(nextPoint - 2).time) /
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 2).time)) * 
			((controlPoints.at(nextPoint).position.x - controlPoints.at(nextPoint - 1).position.x) /
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time)) + 
			((controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time) /
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 2).time)) *
			((controlPoints.at(nextPoint - 1).position.x - controlPoints.at(nextPoint - 2).position.x) / 
			(controlPoints.at(nextPoint - 1).time - controlPoints.at(nextPoint - 2).time));
	
		tangentOneY = ((controlPoints.at(nextPoint - 1).time - controlPoints.at(nextPoint - 2).time) / 
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 2).time)) * 
			((controlPoints.at(nextPoint).position.y - controlPoints.at(nextPoint - 1).position.y) /
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time)) +
			((controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time) / 
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 2).time)) * 
			((controlPoints.at(nextPoint - 1).position.y - controlPoints.at(nextPoint - 2).position.y) / 
			(controlPoints.at(nextPoint - 1).time - controlPoints.at(nextPoint - 2).time));
		
		tangentOneZ = ((controlPoints.at(nextPoint - 1).time - controlPoints.at(nextPoint - 2).time) /
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 2).time)) * 
			((controlPoints.at(nextPoint).position.z - controlPoints.at(nextPoint - 1).position.z) /
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time)) + 
			((controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time) / 
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 2).time)) * 
			((controlPoints.at(nextPoint - 1).position.z - controlPoints.at(nextPoint - 2).position.z) / 
			(controlPoints.at(nextPoint - 1).time - controlPoints.at(nextPoint - 2).time));
			
			
		tangentTwoX = ((controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 2).time) / 
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time)) * 
			((controlPoints.at(nextPoint - 1).position.x - controlPoints.at(nextPoint - 2).position.x) /
			(controlPoints.at(nextPoint - 1).time - controlPoints.at(nextPoint - 2).time)) -
			((controlPoints.at(nextPoint - 1).time - controlPoints.at(nextPoint - 2).time) / 
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time)) * 
			((controlPoints.at(nextPoint).position.x - controlPoints.at(nextPoint - 2).position.x) /
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 2).time));
			
		tangentTwoY = ((controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 2).time) /
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time)) * 
			((controlPoints.at(nextPoint - 1).position.y - controlPoints.at(nextPoint - 2).position.y) /
			(controlPoints.at(nextPoint - 1).time - controlPoints.at(nextPoint - 2).time)) - 
			((controlPoints.at(nextPoint - 1).time - controlPoints.at(nextPoint - 2).time) / 
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time)) * 
			((controlPoints.at(nextPoint).position.y - controlPoints.at(nextPoint - 2).position.y) / 
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 2).time));
			
		tangentTwoZ = ((controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 2).time) /
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time)) *
			((controlPoints.at(nextPoint - 1).position.z - controlPoints.at(nextPoint - 2).position.z) / 
			(controlPoints.at(nextPoint - 1).time - controlPoints.at(nextPoint - 2).time)) - 
			((controlPoints.at(nextPoint - 1).time - controlPoints.at(nextPoint - 2).time) /
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time)) * 
			((controlPoints.at(nextPoint).position.z - controlPoints.at(nextPoint - 2).position.z) / 
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 2).time));
	}

	//Case 3: General middle point
	else
	{
		tangentOneX = ((controlPoints.at(nextPoint - 1).time - controlPoints.at(nextPoint - 2).time) / 
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 2).time)) *
			((controlPoints.at(nextPoint).position.x - controlPoints.at(nextPoint - 1).position.x) / 
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time)) +
			((controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time) / 
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 2).time)) *
			((controlPoints.at(nextPoint - 1).position.x - controlPoints.at(nextPoint - 2).position.x) /
			(controlPoints.at(nextPoint - 1).time - controlPoints.at(nextPoint - 2).time));
		tangentOneY = ((controlPoints.at(nextPoint - 1).time - controlPoints.at(nextPoint - 2).time) /
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 2).time)) * 
			((controlPoints.at(nextPoint).position.y - controlPoints.at(nextPoint - 1).position.y) / 
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time)) + 
			((controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time) /
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 2).time)) * 
			((controlPoints.at(nextPoint - 1).position.y - controlPoints.at(nextPoint - 2).position.y) /
			(controlPoints.at(nextPoint - 1).time - controlPoints.at(nextPoint - 2).time));
		tangentOneZ = ((controlPoints.at(nextPoint - 1).time - controlPoints.at(nextPoint - 2).time) /
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 2).time)) *
			((controlPoints.at(nextPoint).position.z - controlPoints.at(nextPoint - 1).position.z) / 
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time)) + 
			((controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time) / 
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 2).time)) *
			((controlPoints.at(nextPoint - 1).position.z - controlPoints.at(nextPoint - 2).position.z) / 
			(controlPoints.at(nextPoint - 1).time - controlPoints.at(nextPoint - 2).time));
		tangentTwoX = ((controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time) /
			(controlPoints.at(nextPoint + 1).time - controlPoints.at(nextPoint - 1).time)) *
			((controlPoints.at(nextPoint + 1).position.x - controlPoints.at(nextPoint).position.x) /
			(controlPoints.at(nextPoint + 1).time - controlPoints.at(nextPoint).time)) + 
			((controlPoints.at(nextPoint + 1).time - controlPoints.at(nextPoint).time) / 
			(controlPoints.at(nextPoint + 1).time - controlPoints.at(nextPoint - 1).time)) * 
			((controlPoints.at(nextPoint).position.x - controlPoints.at(nextPoint - 1).position.x) /
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time));
		tangentTwoY = ((controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time) /
			(controlPoints.at(nextPoint + 1).time - controlPoints.at(nextPoint - 1).time)) * 
			((controlPoints.at(nextPoint + 1).position.y - controlPoints.at(nextPoint).position.y) / 
			(controlPoints.at(nextPoint + 1).time - controlPoints.at(nextPoint).time)) +
			((controlPoints.at(nextPoint + 1).time - controlPoints.at(nextPoint).time) / 
			(controlPoints.at(nextPoint + 1).time - controlPoints.at(nextPoint - 1).time)) *
			((controlPoints.at(nextPoint).position.y - controlPoints.at(nextPoint - 1).position.y) / 
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time));
		tangentTwoZ = ((controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time) /
			(controlPoints.at(nextPoint + 1).time - controlPoints.at(nextPoint - 1).time)) * 
			((controlPoints.at(nextPoint + 1).position.z - controlPoints.at(nextPoint).position.z) / 
			(controlPoints.at(nextPoint + 1).time - controlPoints.at(nextPoint).time)) + 
			((controlPoints.at(nextPoint + 1).time - controlPoints.at(nextPoint).time) /
			(controlPoints.at(nextPoint + 1).time - controlPoints.at(nextPoint - 1).time)) * 
			((controlPoints.at(nextPoint).position.z - controlPoints.at(nextPoint - 1).position.z) / 
			(controlPoints.at(nextPoint).time - controlPoints.at(nextPoint - 1).time));
	}

	// Calculate position at t = time on Catmull-Rom curve
	newPosition.x = (1 + 2 * timeStep) * (1 - timeStep) * (1 - timeStep) * controlPoints.at(nextPoint - 1).position.x +
		timeStep * (1 - timeStep) * (1 - timeStep) * timeDiff * tangentOneX +
		timeStep * timeStep * (3 - 2 * timeStep) * controlPoints.at(nextPoint).position.x +
		timeStep * timeStep * (timeStep - 1) * timeDiff * tangentTwoX;

	newPosition.y = (1 + 2 * timeStep) * (1 - timeStep) * (1 - timeStep) * controlPoints.at(nextPoint - 1).position.y +
		timeStep * (1 - timeStep) * (1 - timeStep) * timeDiff * tangentOneY +
		timeStep * timeStep * (3 - 2 * timeStep) * controlPoints.at(nextPoint).position.y +
		timeStep * timeStep * (timeStep - 1) * timeDiff * tangentTwoY;

	newPosition.z = (1 + 2 * timeStep) * (1 - timeStep) * (1 - timeStep) * controlPoints.at(nextPoint - 1).position.z +
		timeStep * (1 - timeStep) * (1 - timeStep) * timeDiff * tangentOneZ +
		timeStep * timeStep * (3 - 2 * timeStep) * controlPoints.at(nextPoint).position.z +
		timeStep * timeStep * (timeStep - 1) * timeDiff * tangentTwoZ;

	// Return result
	return newPosition;
}