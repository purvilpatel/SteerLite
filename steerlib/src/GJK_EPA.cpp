#include "obstacles/GJK_EPA.h"
#include "util/Geometry.h"

SteerLib::GJK_EPA::GJK_EPA()
{}

Util::Vector SteerLib::GJK_EPA::tripleProduct(Util::Vector a, Util::Vector b, Util::Vector c)
{
	return Util::normalize(b*Util::dot(c, a) - a*Util::dot(c, b));
}

bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> _simplex;
	if (GJK(_shapeA, _shapeB, _simplex))
	{
		SteerLib::GJK_EPA::EPA(return_penetration_depth, return_penetration_vector, _shapeA, _shapeB, _simplex);
		return true;
	}
	else
	{
		return false;
	}
}

void SteerLib::GJK_EPA::EPA(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const  std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector> simplex)
{
	bool isDone = false;
	int i = 0;
	while (!isDone)
	{
		i++;
		SteerLib::GJK_EPA::Edge e = SteerLib::GJK_EPA::findClosestEdge(simplex);

		Util::Vector p = SteerLib::GJK_EPA::support(_shapeA, _shapeB, e.normal);
		float d = Util::dot(p, e.normal);
		if (fabs(d - e.distance) < 0.01f)
		{
			return_penetration_vector = e.normal;
			return_penetration_depth = d;
			isDone = true;
		}
		else
		{
			simplex.insert(simplex.begin() + e.index, p);
		}
	}
}

SteerLib::GJK_EPA::Edge SteerLib::GJK_EPA::findClosestEdge(std::vector<Util::Vector> simplex)
{
	SteerLib::GJK_EPA::Edge closedEdge;
	closedEdge.distance = FLT_MAX;

	for (int i = 0; i < simplex.size(); i++)
	{
		int j = i + 1;
		if (j == simplex.size())
			j = 0;

		Util::Vector a = simplex.at(i);
		Util::Vector b = simplex.at(j);
		Util::Vector e = b - a;
		Util::Vector n = e;
		n.x = -e.z;
		n.y = 0.0f;
		n.z = e.x;

		float d = Util::dot(n, -a);
		if (d >= 0.0f)
		{
			n.x = -n.x;
			n.y = -n.y;
			n.z = -n.z;
		}

		n = n*(1 / sqrt(Util::dot(n, n)));

		d = Util::dot(a, n);

		if (d < closedEdge.distance)
		{
			closedEdge.distance = d;
			closedEdge.normal = n;
			closedEdge.index = j;
		}
	}
	return closedEdge;
}

Util::Vector SteerLib::GJK_EPA::getFarthestPointInDirection(std::vector<Util::Vector> _shape, Util::Vector direction)
{
	float max = Util::dot(_shape.at(0), direction);
	int maxIndex = 0;
	for (int i = 1; i < _shape.size(); i++)
	{
		if ((Util::dot(_shape.at(i), direction)) > max)
		{
			maxIndex = i;
			max = Util::dot(_shape.at(i), direction);
		}
	}
	return _shape.at(maxIndex);
}

Util::Vector SteerLib::GJK_EPA::support(std::vector<Util::Vector> _shapeA, std::vector<Util::Vector> _shapeB, Util::Vector direction)
{
	Util::Vector point1 = SteerLib::GJK_EPA::getFarthestPointInDirection(_shapeA, direction);
	direction.x = -direction.x;
	direction.y = -direction.y;
	direction.z = -direction.z;
	Util::Vector point2 = SteerLib::GJK_EPA::getFarthestPointInDirection(_shapeB, direction);
	Util::Vector point3 = point1 - point2;

	return point3;
}

bool SteerLib::GJK_EPA::GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& _simplex)
{
	Util::Vector direction(1, 0, 1);
	std::vector<Util::Vector> simplex;
	simplex.push_back(SteerLib::GJK_EPA::support(_shapeA, _shapeB, direction));
	direction.x = -direction.x;
	direction.y = -direction.y;
	direction.z = -direction.z;

	int j = 0;
	while (true)
	{
		j++;
		simplex.push_back(SteerLib::GJK_EPA::support(_shapeA, _shapeB, direction));
		if (Util::dot(simplex.back(), direction) < 0.0f)
		{
			return false;
		}
		else
		{			
			if (SteerLib::GJK_EPA::doContainsOrigin(simplex, direction))
			{
				_simplex = simplex;
				return true;
			}
		}

	}

	return false;
}


bool  SteerLib::GJK_EPA::doContainsOrigin(std::vector<Util::Vector>& simplex, Util::Vector& direction)
{
	Util::Vector a = simplex.back();
	Util::Vector ao = a;
	ao.x = -ao.x;
	ao.y = -ao.y;
	ao.z = -ao.z;

	if (simplex.size() == 3)
	{
		Util::Vector c = simplex.at(1);
		Util::Vector b = simplex.at(0);
		Util::Vector ac = c - a;
		Util::Vector ab = b - a;
		Util::Vector abPerp = ab;
		abPerp.x = -ab.z;
		abPerp.y = 0;
		abPerp.z = ab.x;
		
		if (Util::dot(abPerp, c) >= 0.0f)
		{
			abPerp.x = -abPerp.x;
			abPerp.y = -abPerp.y;
			abPerp.z = -abPerp.z;
		}

		if (Util::dot(abPerp, ao) > 0.0f)
		{
			simplex.erase(std::find(simplex.begin(), simplex.end(), c)); 
			direction = abPerp; 
		}
		else
		{			
			Util::Vector acPerp = ac;
			acPerp.x = -ac.z;
			acPerp.y = 0;
			acPerp.z = ac.x;
			
			if (Util::dot(acPerp, b) >= 0.0f)
			{
				acPerp.x = -acPerp.x;
				acPerp.y = -acPerp.y;
				acPerp.z = -acPerp.z;
			}

			if (Util::dot(acPerp, ao) <= 0.0f) 
			{
				return true;
			}

			simplex.erase(std::find(simplex.begin(), simplex.end(), b));
			direction = acPerp;
		}
	}
	else if (simplex.size() == 2)
	{
	
		Util::Vector b = simplex.at(0);
		
		Util::Vector ab = b - a;
		
		Util::Vector abPerp = ab;
		abPerp.x = -ab.z;
		abPerp.y = 0.0f;
		abPerp.z = ab.x;

		if (Util::dot(abPerp, ao) < 0.0f) {
			abPerp.x = -abPerp.x;
			abPerp.y = -abPerp.y;
			abPerp.z = -abPerp.z;
		}
		direction = abPerp;
	}
	return false;
}