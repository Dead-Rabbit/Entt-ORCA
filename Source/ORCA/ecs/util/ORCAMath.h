#pragma once
#include <complex>
#include <vector>

#include "ORCA/ecs/component/Components.h"

namespace ecs
{
	class ORCAMath
	{
	public:
		static bool LinearProgram1(const std::vector<NavigationLine> &lines, size_t lineNo, float radius, const FVector &optVelocity,
			bool directionOpt, FVector &result);
		static size_t LinearProgram2(const std::vector<NavigationLine> &lines, float radius, const FVector &optVelocity,
			bool directionOpt, FVector &result);
		static void LinearProgram3(const std::vector<NavigationLine> &lines, size_t numObstLines, size_t beginLine, float radius, FVector &result);

		static float Sqr(float a) { return a * a; }

		static float Det2D(const FVector vector1, const FVector vector2)
		{
			return vector1.X * vector2.Y - vector1.Y * vector2.X;
		}
		
		static float LeftOf(const FVector &a, const FVector &b, const FVector &c)
		{
			return Det2D(a - c, b - a);
		}

		static float AbsSq2D(const FVector &vector)
		{
			const auto tempVec = FVector(vector.X, vector.Y, 0);
			return tempVec | tempVec;
		}
		
		static FVector Normalize(const FVector &vector)
		{
			return vector / vector.Size();
		}
		
		static float DistSqPointLineSegment(const FVector &a, const FVector &b, const FVector &c)
		{
			const float r = ((c - a) | (b - a)) / AbsSq2D(b - a);

			if (r < 0.0f) {
				return AbsSq2D(c - a);
			}
			
			if (r > 1.0f)
				return AbsSq2D(c - b);
			
			return AbsSq2D(c - (a + r * (b - a)));
		}

		static float RVO_EPSILON;
		static size_t RVO_ERROR;
	};
}
