//
// Copyright (c) 2015 Mahyar Khayatkhoei
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include <algorithm>
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
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

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	// Robustness: make sure there is at least two control point: start and end points
	if(!checkRobust()){
		return;
	}

	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	Point nextPoint;
	Point lastPoint = controlPoints[0].position;

	float time = 0.0;

	DrawLib::glColor(curveColor);
	while(calculatePoint(nextPoint,time)){
		DrawLib::drawLine(lastPoint,nextPoint);
		lastPoint = nextPoint;
		time = time + 0.01*(float)window;
	}

	return;

#endif
}

bool controlPointComparitor (CurvePoint p1, CurvePoint p2) {
	return (p1.time<p2.time);
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	//superclass: CurvePoint - Point, Vector, float time
	//subclass: Point - float x, y, z
	//subclass: Vector - float x, y, z

	// Use the controlPointComparitor function to sort the vector<controlPoints>
	// The controlPointComparitor function sorts it by time
	std::sort(controlPoints.begin(),controlPoints.end(),controlPointComparitor);

	return;

}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;
	float normalTime, intervalTime;

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
// Check for size > 2
bool Curve::checkRobust()
{
	if(controlPoints.size() > 2){
		return true;
	}

	return false;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
// Calculate the next index of the control point (nextPoint) (i.e. if the next point is at t=40 and time=30, return nextPoint as the index
// of the point that's at t=40 (check for case when time is AT a control point)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	// If the controlPoint time is greater than the current time, this is the next point
	for(int i = 0;i < controlPoints.size();i++){
		if(controlPoints[i].time > time){
			nextPoint = i;
			return true;
		}
	}

	return false;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point P1, P2, R1, R2;

	P1 = (controlPoints[nextPoint-1]).position; // Point P1
	P2 = (controlPoints[nextPoint]).position; // Point P2

	// If nextPoint is the second control point (it can't ever be the first), set P0 = P1 (phantom control point)
	if(nextPoint == 1){
		float R1x = (controlPoints[nextPoint]).position.x - (controlPoints[nextPoint-1]).position.x; // Tangent @ P1 = R1 = P2 - P1
		float R1y = (controlPoints[nextPoint]).position.y - (controlPoints[nextPoint-1]).position.y; // Tangent @ P1 = R1 = P2 - P1
		float R1z = (controlPoints[nextPoint]).position.z - (controlPoints[nextPoint-1]).position.z; // Tangent @ P1 = R1 = P2 - P1
		R1 = Point(R1x,R1y,R1z);
	} else {
		float R1x = (controlPoints[nextPoint]).position.x - (controlPoints[nextPoint-2]).position.x; // Tangent @ P1 = R1 = P2 - P0
		float R1y = (controlPoints[nextPoint]).position.y - (controlPoints[nextPoint-2]).position.y; // Tangent @ P1 = R1 = P2 - P0
		float R1z = (controlPoints[nextPoint]).position.z - (controlPoints[nextPoint-2]).position.z; // Tangent @ P1 = R1 = P2 - P0
		R1 = Point(R1x,R1y,R1z);
	}
	// If nextPoint is the end point, set P3 = P2 (phantom control point)
	if(nextPoint == (controlPoints.size()-1)){
		float R2x = (controlPoints[nextPoint]).position.x - (controlPoints[nextPoint-1]).position.x; // Tangent @ P2 = R2 = P2 - P1
		float R2y = (controlPoints[nextPoint]).position.y - (controlPoints[nextPoint-1]).position.y; // Tangent @ P2 = R2 = P2 - P1
		float R2z = (controlPoints[nextPoint]).position.z - (controlPoints[nextPoint-1]).position.z; // Tangent @ P2 = R2 = P2 - P1
		R2 = Point(R2x,R2y,R2z);
	} else {
		float R2x = (controlPoints[nextPoint+1]).position.x - (controlPoints[nextPoint-1]).position.x; // Tangent @ P2 = R2 = P3 - P1
		float R2y = (controlPoints[nextPoint+1]).position.y - (controlPoints[nextPoint-1]).position.y; // Tangent @ P2 = R2 = P3 - P1
		float R2z = (controlPoints[nextPoint+1]).position.z - (controlPoints[nextPoint-1]).position.z; // Tangent @ P2 = R2 = P3 - P1
		R2 = Point(R2x,R2y,R2z);
	}

	float scaledTime = (time-controlPoints[nextPoint-1].time)/(controlPoints[nextPoint].time-controlPoints[nextPoint-1].time);

	float H0 = 2*(pow(scaledTime,3)) - 3*(pow(scaledTime,2)) + 1;
 	float H1 = -2*(pow(scaledTime,3)) + 3*(pow(scaledTime,2));
 	float H2 = pow(scaledTime,3) - 2*(pow(scaledTime,2)) + scaledTime;
 	float H3 = pow(scaledTime,3) - pow(scaledTime,2);

	float x = H0*P1.x + H1*P2.x + H2*R1.x + H3*R2.x;
	float y = H0*P1.y + H1*P2.y + H2*R1.y + H3*R2.y;
	float z = H0*P1.z + H1*P2.z + H2*R1.z + H3*R2.z;

	return Point(x,y,z);

}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	/* Theory:
	
	P0 = Control point before t = 0
	P1 = Control point at t = 0
	P2 = Control point at t = 1
	P3 = Control point after t = 1
	scaledTime = Time must be scaled from 0 to 1 by making it a fraction with the nextPoint's arrival time. The
		last point must be subtracted from the top and bottom to allow it to connect the segments properly
	x, y, z = Values generated from the Catmull-Rom Cubic Spline equation

	*/

	Point P0, P1, P2, P3;

	// If nextPoint is the second control point (it can't ever be the first), set P0 and P1 to that point (phantom control point)
	if(nextPoint == 1){
		P0 = (controlPoints[nextPoint-1]).position;
		P1 = P0;
	} else {
		P0 = (controlPoints[nextPoint-2]).position;
		P1 = (controlPoints[nextPoint-1]).position;
	}
	// If nextPoint is the end point, set P2 and P3 to that point (phantom control point)
	if(nextPoint == (controlPoints.size()-1)){
		P2 = (controlPoints[nextPoint]).position;
		P3 = P2;
	} else {
		P2 = (controlPoints[nextPoint]).position;
		P3 = (controlPoints[nextPoint+1]).position;
	}

	// Scale the time from 0 to 1
	float scaledTime = (time-controlPoints[nextPoint-1].time)/(controlPoints[nextPoint].time-controlPoints[nextPoint-1].time);

	// Calculate new x value
	float x = 0.5*((2*P1.x)+(-P0.x+P2.x)*scaledTime+(2*P0.x-5*P1.x+4*P2.x-P3.x)*pow(scaledTime,2)+(-P0.x+3*P1.x-3*P2.x+P3.x)*pow(scaledTime,3));
	// Calculate new y value
	float y = 0.5*((2*P1.y)+(-P0.y+P2.y)*scaledTime+(2*P0.y-5*P1.y+4*P2.y-P3.y)*pow(scaledTime,2)+(-P0.y+3*P1.y-3*P2.y+P3.y)*pow(scaledTime,3));
	// Calculate new z value
	float z = 0.5*((2*P1.z)+(-P0.z+P2.z)*scaledTime+(2*P0.z-5*P1.z+4*P2.z-P3.z)*pow(scaledTime,2)+(-P0.z+3*P1.z-3*P2.z+P3.z)*pow(scaledTime,3));

	//Print x, y, z values (good for matlab plotting to visualize the curve)
	//std::cout << x << "," << y << "," << z << "," << std::endl;

	// Compile new x, y and z values into a point and return
	return Point(x,y,z);
}

