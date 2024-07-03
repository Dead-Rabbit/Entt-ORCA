#include "ORCAMath.h"

namespace ecs
{
	float ORCAMath::RVO_EPSILON = 0.00001f;
	size_t ORCAMath::RVO_ERROR = std::numeric_limits<size_t>::max();

	bool ORCAMath::LinearProgram1(const std::vector<NavigationLine>& lines, size_t lineNo, float radius,
		const FVector& optVelocity, bool directionOpt, FVector& result)
	{
		const float dotProduct = lines[lineNo].point | lines[lineNo].direction;
		const float discriminant = dotProduct * dotProduct + radius * radius - FVector::DotProduct(lines[lineNo].point, lines[lineNo].point);

		if (discriminant < 0.0f) {
			/* Max speed circle fully invalidates line lineNo. */
			return false;
		}

		const float sqrtDiscriminant = sqrt(discriminant);
		float tLeft = -dotProduct - sqrtDiscriminant;
		float tRight = -dotProduct + sqrtDiscriminant;

		for (size_t i = 0; i < lineNo; ++i) {
			const float denominator = Det2D(lines[lineNo].direction, lines[i].direction);
			const float numerator = Det2D(lines[i].direction, lines[lineNo].point - lines[i].point);

			if (fabs(denominator) <= RVO_EPSILON) {
				/* Lines lineNo and i are (almost) parallel. */
				if (numerator < 0.0f) {
					return false;
				}
				else {
					continue;
				}
			}

			const float t = numerator / denominator;

			if (denominator >= 0.0f) {
				/* Line i bounds line lineNo on the right. */
				tRight = std::min(tRight, t);
			}
			else {
				/* Line i bounds line lineNo on the left. */
				tLeft = std::max(tLeft, t);
			}

			if (tLeft > tRight) {
				return false;
			}
		}

		if (directionOpt) {
			/* Optimize direction. */
			if ((optVelocity | lines[lineNo].direction) > 0.0f) {
				/* Take right extreme. */
				result = lines[lineNo].point + tRight * lines[lineNo].direction;
			}
			else {
				/* Take left extreme. */
				result = lines[lineNo].point + tLeft * lines[lineNo].direction;
			}
		}
		else {
			/* Optimize closest point. */
			const float t = lines[lineNo].direction | optVelocity - lines[lineNo].point;

			if (t < tLeft) {
				result = lines[lineNo].point + tLeft * lines[lineNo].direction;
			}
			else if (t > tRight) {
				result = lines[lineNo].point + tRight * lines[lineNo].direction;
			}
			else {
				result = lines[lineNo].point + t * lines[lineNo].direction;
			}
		}

		return true;
	}

	size_t ORCAMath::LinearProgram2(const std::vector<NavigationLine>& lines, float radius, const FVector& optVelocity,
	                                    bool directionOpt, FVector& result)
	{
		if (directionOpt) {
			/*
			 * Optimize direction. Note that the optimization velocity is of unit
			 * length in this case.
			 */
			result = optVelocity * radius;
		}
		else if (optVelocity.SizeSquared() > radius * radius) {
			/* Optimize closest point and outside circle. */
			FVector normalVelocity = optVelocity;
			normalVelocity.Normalize();
			result = normalVelocity * radius;
		}
		else {
			/* Optimize closest point and inside circle. */
			result = optVelocity;
		}

		for (size_t i = 0; i < lines.size(); ++i) {
			if (Det2D(lines[i].direction, lines[i].point - result) > 0.0f) {
				/* Result does not satisfy constraint i. Compute new optimal result. */
				const FVector tempResult = result;

				if (!LinearProgram1(lines, i, radius, optVelocity, directionOpt, result)) {
					result = tempResult;
					return i;
				}
			}
		}

		return lines.size();
	}

	void ORCAMath::LinearProgram3(const std::vector<NavigationLine>& lines, size_t numObstLines, size_t beginLine,
		float radius, FVector& result)
	{
		float distance = 0.0f;

		for (size_t i = beginLine; i < lines.size(); ++i) {
			if (Det2D(lines[i].direction, lines[i].point - result) > distance) {
				
				/* Result does not satisfy constraint of line i. */
				std::vector<NavigationLine> projLines(lines.begin(), lines.begin() + static_cast<ptrdiff_t>(numObstLines));

				for (size_t j = numObstLines; j < i; ++j) {
					NavigationLine line;

					float determinant = Det2D(lines[i].direction, lines[j].direction);

					if (fabs(determinant) <= RVO_EPSILON) {
						/* Line i and line j are parallel. */
						if ((lines[i].direction | lines[j].direction) > 0.0f) {
							/* Line i and line j point in the same direction. */
							continue;
						}
						else {
							/* Line i and line j point in opposite direction. */
							line.point = 0.5f * (lines[i].point + lines[j].point);
						}
					}
					else {
						line.point = lines[i].point + (Det2D(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
					}

					line.direction = lines[j].direction - lines[i].direction;
					line.direction.Normalize();
					projLines.push_back(line);
				}

				const FVector tempResult = result;

				if (LinearProgram2(projLines, radius, FVector(-lines[i].direction.Y, lines[i].direction.X, 0), true, result) < projLines.size()) {
					/* This should in principle not happen.  The result is by definition
					 * already in the feasible region of this linear program. If it fails,
					 * it is due to small floating point error, and the current result is
					 * kept.
					 */
					result = tempResult;
				}

				distance = Det2D(lines[i].direction, lines[i].point - result);
			}
		}
	}
}
