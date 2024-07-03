#include "SysUnitNavigation.h"

#include "DrawDebugHelpers.h"
#include "../component/Components.h"
#include "ORCA/ORCAEntry.h"
#include "ORCA/ecs/ECSWorld.h"
#include "ORCA/ecs/util/ORCAMath.h"

void ecs::SysUnitNavigation::OnInit()
{
	ECSSystem::OnInit();
}

void ecs::SysUnitNavigation::OnStart()
{
	ECSSystem::OnStart();
	
	auto reg = _world->GetReg();
}

void ecs::SysUnitNavigation::OnUpdate(float dt)
{
	auto reg = _world->GetReg();
	if (reg == nullptr)
		return;

	auto view = reg->view<ComNavigationAgent, ComTransform>();

	// get neighbor
	kdTree_->BuildAgentTree(reg);
	kdTree_->BuildObstacleTree();
	// float radius = 3000.0f;
	// set prefered velocities
	for (const auto unit : view)
	{
		auto & comTrans = reg->get<ComTransform>(unit);
		auto & comAgent = reg->get<ComNavigationAgent>(unit);
		FVector targetVector = comAgent.targetPos - comTrans.location;
		
		if (targetVector.SizeSquared() > 1.0f) {
			targetVector.Normalize();
			targetVector *= comAgent.maxSpeed;
		}

		/*
		 * Perturb a little to avoid deadlocks due to perfect symmetry.
		 */
		float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
		float dist = std::rand() * 0.0001f / RAND_MAX;
		comAgent.prefVelocity = targetVector + dist * FVector(std::cos(angle), std::sin(angle), 0);

		// get obs neighbors
		comAgent.obsNeis.clear();
		std::vector<std::pair<float, ECSObstacle*>> obsNei;
		float rangeSq = ORCAMath::Sqr(comAgent.timeHorizonObst * comAgent.maxSpeed + comAgent.radius);
		kdTree_->ComputeObstacleNeighbors(comTrans.location, rangeSq, obsNei);
		for (auto obs : obsNei)
		{
			comAgent.obsNeis.push_back(obs);
		}
		
		// get agent neighbors
		comAgent.neis.clear();
		std::vector<uint32_t> neis;
		kdTree_->ComputeAgentNeighbors(comTrans.location, comAgent.radiusSeq, &neis);
		
		for (auto u : neis)
		{
			if (u == unit)
				continue;
		
			const ComNavigationAgent comOtherAgent = view.get<ComNavigationAgent>(u);
			if (comAgent.mass > comOtherAgent.mass)
				continue;
			
			// const auto otherTrans = view.get<ComTransform>(u);
			// auto RLoc = comTrans.location - otherTrans.location;
			// auto RVel = comOtherAgent.velocity - comAgent.velocity;
			// //先假设预警时间是1s；
			// float disChangebyVel = FVector::DotProduct(RLoc.GetUnsafeNormal2D(), RVel) * 10.0f;
			// float comR = comAgent.radius + comOtherAgent.radius;
			//
			// if (RLoc.SizeSquared() > (comR + disChangebyVel) * (comR + disChangebyVel))
			// 	continue;

			comAgent.neis.push_back(u);
		}
	}

	/* Search For The Best New Velocity */
	for (const auto entity : view)
	{
		auto & comTrans = view.get<ComTransform>(entity);
		auto & comAgent = view.get<ComNavigationAgent>(entity);
		const float radius_ = comAgent.radius;
		const float radiusSq = radius_ * radius_;
		const auto position_ = comTrans.location;
		const auto velocity_ = comAgent.velocity;

		comAgent.orcaLines_.clear();

		// TODO 待加入针对静态物体的避障算法
		
		const float invTimeHorizonObst = 1.0f / comAgent.timeHorizonObst;

		/* Create obstacle ORCA lines. */
		for (size_t i = 0; i < comAgent.obsNeis.size(); ++i) {

			const ECSObstacle *obstacle1 = comAgent.obsNeis[i].second;
			const ECSObstacle *obstacle2 = obstacle1->nextObstacle_;

			const FVector relativePosition1 = obstacle1->point_ - position_;
			const FVector relativePosition2 = obstacle2->point_ - position_;

			/*
			 * Check if velocity obstacle of obstacle is already taken care of by
			 * previously constructed obstacle ORCA lines.
			 */
			bool alreadyCovered = false;

			for (size_t j = 0; j < comAgent.orcaLines_.size(); ++j) {
				if (ORCAMath::Det2D(invTimeHorizonObst * relativePosition1 - comAgent.orcaLines_[j].point, comAgent.orcaLines_[j].direction) - invTimeHorizonObst * comAgent.radius >= -ORCAMath::RVO_EPSILON
					&& ORCAMath::Det2D(invTimeHorizonObst * relativePosition2 - comAgent.orcaLines_[j].point, comAgent.orcaLines_[j].direction) - invTimeHorizonObst * comAgent.radius >=  -ORCAMath::RVO_EPSILON) {
					alreadyCovered = true;
					break;
				}
			}

			if (alreadyCovered) {
				continue;
			}

			/* Not yet covered. Check for collisions. */

			const float distSq1 = ORCAMath::AbsSq2D(relativePosition1);
			const float distSq2 = ORCAMath::AbsSq2D(relativePosition2);

			const FVector obstacleVector = obstacle2->point_ - obstacle1->point_;
			const float s = (-relativePosition1 | obstacleVector) / ORCAMath::AbsSq2D(obstacleVector);
			const float distSqLine = ORCAMath::AbsSq2D(-relativePosition1 - s * obstacleVector);

			NavigationLine line;

			if (s < 0.0f && distSq1 <= radiusSq) {
				/* Collision with left vertex. Ignore if non-convex. */
				if (obstacle1->isConvex_) {
					line.point = FVector(0.0f, 0.0f, 0.0f);
					line.direction = ORCAMath::Normalize(FVector(-relativePosition1.Y, relativePosition1.X, 0));
					comAgent.orcaLines_.push_back(line);
				}

				continue;
			}
			else if (s > 1.0f && distSq2 <= radiusSq) {
				/* Collision with right vertex. Ignore if non-convex
				 * or if it will be taken care of by neighoring obstace */
				if (obstacle2->isConvex_ && ORCAMath::Det2D(relativePosition2, obstacle2->unitDir_) >= 0.0f) {
					line.point = FVector(0.0f, 0.0f, 0.0f);
					line.direction = ORCAMath::Normalize(FVector(-relativePosition2.Y, relativePosition2.X, 0.0f));
					comAgent.orcaLines_.push_back(line);
				}

				continue;
			}
			else if (s >= 0.0f && s <= 1.0f && distSqLine <= radiusSq) {
				/* Collision with obstacle segment. */
				line.point = FVector(0.0f, 0.0f, 0.0f);
				line.direction = -obstacle1->unitDir_;
				comAgent.orcaLines_.push_back(line);
				continue;
			}

			/*
			 * No collision.
			 * Compute legs. When obliquely viewed, both legs can come from a single
			 * vertex. Legs extend cut-off line when nonconvex vertex.
			 */

			FVector leftLegDirection, rightLegDirection;

			if (s < 0.0f && distSqLine <= radiusSq) {
				/*
				 * ECSObstacle viewed obliquely so that left vertex
				 * defines velocity obstacle.
				 */
				if (!obstacle1->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}

				obstacle2 = obstacle1;

				const float leg1 = std::sqrt(distSq1 - radiusSq);
				leftLegDirection = FVector(relativePosition1.X * leg1 - relativePosition1.Y * radius_, relativePosition1.X * radius_ + relativePosition1.Y * leg1, 0) / distSq1;
				rightLegDirection = FVector(relativePosition1.X * leg1 + relativePosition1.Y * radius_, -relativePosition1.X * radius_ + relativePosition1.Y * leg1, 0) / distSq1;
			}
			else if (s > 1.0f && distSqLine <= radiusSq) {
				/*
				 * ECSObstacle viewed obliquely so that
				 * right vertex defines velocity obstacle.
				 */
				if (!obstacle2->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}

				obstacle1 = obstacle2;

				const float leg2 = std::sqrt(distSq2 - radiusSq);
				leftLegDirection = FVector(relativePosition2.X * leg2 - relativePosition2.Y * radius_, relativePosition2.X * radius_ + relativePosition2.Y * leg2, 0) / distSq2;
				rightLegDirection = FVector(relativePosition2.X * leg2 + relativePosition2.Y * radius_, -relativePosition2.X * radius_ + relativePosition2.Y * leg2, 0) / distSq2;
			}
			else {
				/* Usual situation. */
				if (obstacle1->isConvex_) {
					const float leg1 = std::sqrt(distSq1 - radiusSq);
					leftLegDirection = FVector(relativePosition1.X * leg1 - relativePosition1.Y * radius_, relativePosition1.X * radius_ + relativePosition1.Y * leg1, 0) / distSq1;
				}
				else {
					/* Left vertex non-convex; left leg extends cut-off line. */
					leftLegDirection = -obstacle1->unitDir_;
				}

				if (obstacle2->isConvex_) {
					const float leg2 = std::sqrt(distSq2 - radiusSq);
					rightLegDirection = FVector(relativePosition2.X * leg2 + relativePosition2.Y * radius_, -relativePosition2.X * radius_ + relativePosition2.Y * leg2, 0) / distSq2;
				}
				else {
					/* Right vertex non-convex; right leg extends cut-off line. */
					rightLegDirection = obstacle1->unitDir_;
				}
			}

			/*
			 * Legs can never point into neighboring edge when convex vertex,
			 * take cutoff-line of neighboring edge instead. If velocity projected on
			 * "foreign" leg, no constraint is added.
			 */

			const ECSObstacle *const leftNeighbor = obstacle1->prevObstacle_;

			bool isLeftLegForeign = false;
			bool isRightLegForeign = false;

			if (obstacle1->isConvex_ && ORCAMath::Det2D(leftLegDirection, -leftNeighbor->unitDir_) >= 0.0f) {
				/* Left leg points into obstacle. */
				leftLegDirection = -leftNeighbor->unitDir_;
				isLeftLegForeign = true;
			}

			if (obstacle2->isConvex_ && ORCAMath::Det2D(rightLegDirection, obstacle2->unitDir_) <= 0.0f) {
				/* Right leg points into obstacle. */
				rightLegDirection = obstacle2->unitDir_;
				isRightLegForeign = true;
			}

			/* Compute cut-off centers. */
			const FVector leftCutoff = invTimeHorizonObst * (obstacle1->point_ - position_);
			const FVector rightCutoff = invTimeHorizonObst * (obstacle2->point_ - position_);
			const FVector cutoffVec = rightCutoff - leftCutoff;

			/* Project current velocity on velocity obstacle. */

			/* Check if current velocity is projected on cutoff circles. */
			const float t = (obstacle1 == obstacle2 ? 0.5f : ((velocity_ - leftCutoff) | cutoffVec) / ORCAMath::AbsSq2D(cutoffVec));
			const float tLeft = ((velocity_ - leftCutoff) | leftLegDirection);
			const float tRight = ((velocity_ - rightCutoff) | rightLegDirection);

			if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f)) {
				/* Project on left cut-off circle. */
				const FVector unitW = ORCAMath::Normalize(velocity_ - leftCutoff);

				line.direction = FVector(unitW.Y, -unitW.X, 0);
				line.point = leftCutoff + radius_ * invTimeHorizonObst * unitW;
				comAgent.orcaLines_.push_back(line);
				continue;
			}
			else if (t > 1.0f && tRight < 0.0f) {
				/* Project on right cut-off circle. */
				const FVector unitW = ORCAMath::Normalize(velocity_ - rightCutoff);

				line.direction = FVector(unitW.Y, -unitW.X, 0);
				line.point = rightCutoff + radius_ * invTimeHorizonObst * unitW;
				comAgent.orcaLines_.push_back(line);
				continue;
			}

			/*
			 * Project on left leg, right leg, or cut-off line, whichever is closest
			 * to velocity.
			 */
			const float distSqCutoff = ((t < 0.0f || t > 1.0f || obstacle1 == obstacle2) ? std::numeric_limits<float>::infinity() : ORCAMath::AbsSq2D(velocity_ - (leftCutoff + t * cutoffVec)));
			const float distSqLeft = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity() : ORCAMath::AbsSq2D(velocity_ - (leftCutoff + tLeft * leftLegDirection)));
			const float distSqRight = ((tRight < 0.0f) ? std::numeric_limits<float>::infinity() : ORCAMath::AbsSq2D(velocity_ - (rightCutoff + tRight * rightLegDirection)));

			if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
				/* Project on cut-off line. */
				line.direction = -obstacle1->unitDir_;
				line.point = leftCutoff + radius_ * invTimeHorizonObst * FVector(-line.direction.Y, line.direction.X, 0);
				comAgent.orcaLines_.push_back(line);
				continue;
			}
			else if (distSqLeft <= distSqRight) {
				/* Project on left leg. */
				if (isLeftLegForeign) {
					continue;
				}

				line.direction = leftLegDirection;
				line.point = leftCutoff + radius_ * invTimeHorizonObst * FVector(-line.direction.Y, line.direction.X, 0);
				comAgent.orcaLines_.push_back(line);
				continue;
			}
			else {
				/* Project on right leg. */
				if (isRightLegForeign) {
					continue;
				}

				line.direction = -rightLegDirection;
				line.point = rightCutoff + radius_ * invTimeHorizonObst * FVector(-line.direction.Y, line.direction.X, 0);
				comAgent.orcaLines_.push_back(line);
				continue;
			}
		}

		if (comAgent.debugShow) {
			auto renderWorld = AORCAEntry::get();
			for (auto line : comAgent.orcaLines_) {
				DrawDebugLine(renderWorld->GetWorld(), position_ + line.point - line.direction * 800 + FVector(0, 0, 10),
					position_ + line.point + line.direction * 800 + FVector(0, 0, 10),
					FColor::Red, false, -1, 0, 8.0f);
			}
		}

		const size_t numObstLines = comAgent.orcaLines_.size();
		const float invTimeHorizon = 1.0f / comAgent.timeHorizon;
		
		for (size_t i = 0; i < comAgent.neis.size(); ++i) {
			uint32_t otherEntity = comAgent.neis[i];
			const auto comOtherAgent = view.get<ComNavigationAgent>(otherEntity);
			const auto comOtherTrans = view.get<ComTransform>(otherEntity);

			const FVector relativePosition = comOtherTrans.location - comTrans.location;
			const FVector relativeVelocity = comAgent.velocity - comOtherAgent.velocity;
			const float distSq = relativePosition.SizeSquared();
			const float combinedRadius = radius_ + comOtherAgent.radius;
			const float combinedRadiusSq = combinedRadius * combinedRadius;

			NavigationLine line;
			FVector u;

			if (distSq > combinedRadiusSq) {
				/* No collision. */
				const FVector w = relativeVelocity - invTimeHorizon * relativePosition;
				/* Vector from cutoff center to relative velocity. */
				const float wLengthSq = w.SizeSquared();

				const float dotProduct1 = w | relativePosition;

				if (dotProduct1 < 0.0f && dotProduct1 * dotProduct1 > combinedRadiusSq * wLengthSq) {
					/* Project on cut-off circle. */
					const float wLength = std::sqrt(wLengthSq);
					const FVector unitW = w / wLength;

					line.direction = FVector(unitW.Y, -unitW.X, 0);
					u = (combinedRadius * invTimeHorizon - wLength) * unitW;
				}
				else {
					/* Project on legs. */
					const float leg = std::sqrt(distSq - combinedRadiusSq);

					if (ORCAMath::Det2D(relativePosition, w) > 0.0f) {
						/* Project on left leg. */
						line.direction = FVector(relativePosition.X * leg - relativePosition.Y * combinedRadius,
							relativePosition.X * combinedRadius + relativePosition.Y * leg, 0) / distSq;
					}
					else {
						/* Project on right leg. */
						line.direction = -FVector(relativePosition.X * leg + relativePosition.Y * combinedRadius,
							-relativePosition.X * combinedRadius + relativePosition.Y * leg, 0) / distSq;
					}

					const float dotProduct2 = relativeVelocity | line.direction;

					u = dotProduct2 * line.direction - relativeVelocity;
				}
			}
			else {
				/* Collision. Project on cut-off circle of time timeStep. */
				const float invTimeStep = 1.0f / timeStep_;

				/* Vector from cutoff center to relative velocity. */
				const FVector w = relativeVelocity - invTimeStep * relativePosition;

				const float wLength = w.Size();
				const FVector unitW = w / wLength;

				line.direction = FVector(unitW.Y, -unitW.X, 0);
				u = (combinedRadius * invTimeStep - wLength) * unitW;
			}

			line.point = comAgent.velocity + 0.5f * u;
			comAgent.orcaLines_.push_back(line);
		}

		size_t lineFail = ORCAMath::LinearProgram2(comAgent.orcaLines_, comAgent.maxSpeed,
			comAgent.prefVelocity, false, comAgent.newVelocity);

		if (lineFail < comAgent.orcaLines_.size()) {
			ORCAMath::LinearProgram3(comAgent.orcaLines_, numObstLines, lineFail, comAgent.maxSpeed, comAgent.newVelocity);
		}
	}

	// 生效
	for (const auto entity : view)
	{
		auto & comTrans = reg->get<ComTransform>(entity);
		auto & comAgent = reg->get<ComNavigationAgent>(entity);

		comAgent.velocity = comAgent.newVelocity;
		comTrans.location += comAgent.velocity * timeStep_;

		if (false)
		{
			// 修改对应 agent actor
			AActor * agentActor = comAgent.actor;
			// 检查是否到站
			if (!comAgent.reached && (comTrans.location - comAgent.targetPos).SizeSquared() < 10 * 10) {
				comAgent.reached = true;

				if (agentActor)
				{
					USkeletalMeshComponent * skeletalCom = agentActor->FindComponentByClass<USkeletalMeshComponent>();
					if (skeletalCom)
					{
						UAnimSequence *AnimSequence = LoadObject<UAnimSequence>(0, TEXT("AnimSequence'/Game/ORCA/Meshes/Charactors/Motions/Zombie_Idle.Zombie_Idle'"));
						skeletalCom->PlayAnimation(AnimSequence, true);
					}
				}
			} else if (comAgent.reached && (comTrans.location - comAgent.targetPos).SizeSquared() > 10 * 10)
			{
				comAgent.reached = false;
				if (agentActor)
				{
					USkeletalMeshComponent * skeletalCom = agentActor->FindComponentByClass<USkeletalMeshComponent>();
					if (skeletalCom)
					{
						UAnimSequence *AnimSequence = LoadObject<UAnimSequence>(0, TEXT("AnimSequence'/Game/ORCA/Meshes/Charactors/Motions/Zombie_Walking.Zombie_Walking'"));
						skeletalCom->PlayAnimation(AnimSequence, true);
					}
				}
			}

			if (agentActor)
			{
				const auto prePos = agentActor->GetActorLocation();
				const auto pos = comTrans.location;
				const auto curPos = FVector(pos.X, pos.Y, pos.Z);
				if ((curPos - prePos).SizeSquared() > 1)
				{
					agentActor->SetActorLocation(curPos);
				
					auto dir = curPos - prePos;
					if (dir.SizeSquared() > 0.1)
					{
						agentActor->SetActorRotation(FQuat::Slerp(agentActor->GetActorRotation().Quaternion(),
						dir.ToOrientationQuat(), 0.03f));
					}
				}
			}
		}
	}
}

void ecs::SysUnitNavigation::OnDestroy()
{
	ECSSystem::OnDestroy();
}
