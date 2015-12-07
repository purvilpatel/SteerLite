#ifndef GJK_EPA_H_
#define GJK_EPA_H_


#include "util/Geometry.h"
#include "SteerLib.h"

#include <vector>


namespace SteerLib
{

	class STEERLIB_API GJK_EPA
	{
	public:
		static struct Edge {                   
			float distance;
			int index;
			Util::Vector normal;
		};

		GJK_EPA();

		static Util::Vector tripleProduct(Util::Vector a, Util::Vector b, Util::Vector c);

		static Util::Vector getFarthestPointInDirection(const std::vector<Util::Vector> _shape, Util::Vector direction);

		static Util::Vector support(const std::vector<Util::Vector> _shapeA, const std::vector<Util::Vector> _shapeB, Util::Vector direction);

		static SteerLib::GJK_EPA::Edge findClosestEdge(std::vector<Util::Vector> simplex);

		static bool doContainsOrigin(std::vector<Util::Vector>& simplex, Util::Vector& direction);

		static bool GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& _simplex);

		static void EPA(float & return_penetration_depth, Util::Vector & return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector> simplex);

		static Util::Vector ORIGIN();

		static bool intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB);
	private:

	}; // class GJK_EPA

} // namespace SteerLib


#endif /* GJK_EPA_H_ */
