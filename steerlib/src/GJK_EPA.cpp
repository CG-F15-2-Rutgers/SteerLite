/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> simplex;
	bool is_colliding = GJK(_shapeA, _shapeB, simplex);
	     if (is_colliding == true)
	{
		return true;
	
	}
	else
	{
		return false;
	}
}

// Gets some vector d and finds the furthest point of shape along that direction using projection method
Util::Point SteerLib::GJK_EPA::getFurthestPointinDirection(const std::vector<Util::Vector>& _shape, const Util::Vector & d)
{
	Util::Point furthestPoint(0, 0, 0);
	float greatestProjectionMagnitude = 0;

	// Finds Furthest Point By iterating through the list
	for (std::vector<Util::Vector>::const_iterator it = _shape.begin(); it != _shape.end(); ++it)
	{
		//compute the dot product, whhich gets the projection of the shape vector onto 
		float magnitude = (*it * d);
		if (magnitude > greatestProjectionMagnitude)
		{
			greatestProjectionMagnitude = magnitude;
			furthestPoint = Util::Point();
		}
	}
	return furthestPoint;
}


// Finds and returns point that will be used to create the new simplex
 Util::Vector SteerLib::GJK_EPA::Support(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, const Util::Vector & d)
{
	Util::Point a_point = getFurthestPointinDirection(_shapeA, d);
	Util::Vector d_negated = -1.0f * d;
	Util::Point b_point = getFurthestPointinDirection(_shapeB, d_negated );
	Util::Vector minkowski_diff_point = a_point - b_point;
	return minkowski_diff_point;
}

 bool SteerLib::GJK_EPA::hasOrigin(std::vector<Util::Vector>& simplex, Util::Vector & d)
 {
	 Util::Vector A = simplex[simplex.size() - 1];
	 Util::Vector A0 = -A; //-A = {0,0,0} - A Going from A TO 0
	 if (simplex.size() == 3)	// we deal with situation of choosing best line and finding it's normal 
	 { // We know that the new point A is past origin so we can check AB and AC to see if their normal away from the inside of the simplex is in the direction of origin
	   // Order We call B and C doesn't matter]

		Util::Vector B = simplex[0];
		Util::Vector C = simplex[1];

		Util::Vector AB = B - A;
		Util::Vector AC = C - A;

		Util::Vector abNorm = getNormal(AC, AB, AB);
		if (abNorm * A0 > 0) // Origin is out of the simplex and the normal points to it from AB
		{
			simplex.erase(simplex.begin() + 1); // Remove C
			d = abNorm;
		} else 
		{
			Util::Vector acNorm = getNormal(AB, AC, AC); // Origin is out of the simplex and the normal points to it from AC
			if (acNorm * A0 > 0) 
			{
				simplex.erase(simplex.begin()); // Remove B
				d = acNorm;
			} else
			{
				// We know from prior steps that it is not outside 
				return true;
			}
		}
	 } else // This is a line segment now
	 {
		 Util::Vector B = simplex[0];

		 Util::Vector AB = B - A;

		 Util::Vector abNorm = getNormal(AB,A0,AB);

		 d = abNorm;
	 }
	 return false;
 }

 bool SteerLib::GJK_EPA::GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& simplex)
{
	Util::Vector d(1, 0, 0);
	simplex.push_back(Support(_shapeA, _shapeB, d));

	// Negates the vector
	d = -d;

	while (true)
	{
		simplex.push_back(Support(_shapeA, _shapeB, d));
		//simplex.add

		if ((simplex[simplex.size()-1]*d <= 0.0f)) // vector did not pass the origin, it is impossible for the origin to be in the mink. diff.
		{
			return false;
		} else {
			if (hasOrigin(simplex,d))
			{
				return true;
			}
		}
	}
}

 Util::Vector SteerLib::GJK_EPA::getNormal(const Util::Vector& A, const Util::Vector& B, const Util::Vector& C)
 {
	 return B*(C*A) - A * (C * B); //ALMOST COMPLETE CHANCE I messed up
 }
