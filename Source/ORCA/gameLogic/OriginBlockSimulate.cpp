#include "OriginBlockSimulate.h"

#include "Animation/AnimSingleNodeInstance.h"

namespace RVO
{
	void OriginBlockSimulate::Start(AActor * entryPoint)
	{
		/* Create a new simulator instance. */
		sim = new RVOSimulator(entryPoint);
		setupScenario();
	}

	void OriginBlockSimulate::Update()
	{
		UpdateVisualization();
		setPreferredVelocities();
		reachedGoal();
		sim->doStep();
	}

	void OriginBlockSimulate::UpdateVisualization()
	{
		for (size_t i = 0; i < sim->getNumAgents(); ++i) {
			sim->SetAgentActorLocation(i);
		}
	}

	void OriginBlockSimulate::setupScenario()
	{
		/* Specify the global time step of the simulation. */
		sim->setTimeStep(0.25f);

		/* Specify the default parameters for agents that are subsequently added. */
		sim->setAgentDefaults(200.0f, 10, 5.0f, 5.0f, 22.0f, 16.0f);

		/*
		 * Add agents, specifying their start position, and store their goals on the
		 * opposite side of the environment.
		 */
		for (size_t i = 0; i < 1; ++i) {
			for (size_t j = 0; j < 1; ++j) {
				sim->addAgent(RVO::Vector2(2000 + i * 100.0f, 2000 + j * 100.0f));
				goals.push_back(RVO::Vector2(0, 0));

				// sim->addAgent(RVO::Vector2(-550.0f - i * 100.0f,  550.0f + j * 100.0f));
				// goals.push_back(RVO::Vector2(750.0f- i * 100.0f, -750.0f+ j * 100.0f));
				//
				// sim->addAgent(RVO::Vector2(550.0f + i * 100.0f, -550.0f - j * 100.0f));
				// goals.push_back(RVO::Vector2(-750.0f+ i * 100.0f, 750.0f- j * 100.0f));
				//
				// sim->addAgent(RVO::Vector2(-550.0f - i * 100.0f, -550.0f - j * 100.0f));
				// goals.push_back(RVO::Vector2(750.0f- i * 100.0f, 750.0f- j * 100.0f));
				
				reached.push_back(false);
				// reached.push_back(false);
				// reached.push_back(false);
				// reached.push_back(false);
			}
		}

		/*
		 * Add (polygonal) obstacles, specifying their vertices in counterclockwise
		 * order.
		 */
		// std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;
		//
		// obstacle1.push_back(RVO::Vector2(-100.0f, 400.0f));
		// obstacle1.push_back(RVO::Vector2(-400.0f, 400.0f));
		// obstacle1.push_back(RVO::Vector2(-400.0f, 100.0f));
		// obstacle1.push_back(RVO::Vector2(-100.0f, 100.0f));
		//
		// obstacle2.push_back(RVO::Vector2(100.0f, 400.0f));
		// obstacle2.push_back(RVO::Vector2(100.0f, 100.0f));
		// obstacle2.push_back(RVO::Vector2(400.0f, 100.0f));
		// obstacle2.push_back(RVO::Vector2(400.0f, 400.0f));
		//
		// obstacle3.push_back(RVO::Vector2(100.0f, -400.0f));
		// obstacle3.push_back(RVO::Vector2(400.0f, -400.0f));
		// obstacle3.push_back(RVO::Vector2(400.0f, -100.0f));
		// obstacle3.push_back(RVO::Vector2(100.0f, -100.0f));
		//
		// obstacle4.push_back(RVO::Vector2(-100.0f, -400.0f));
		// obstacle4.push_back(RVO::Vector2(-100.0f, -100.0f));
		// obstacle4.push_back(RVO::Vector2(-400.0f, -100.0f));
		// obstacle4.push_back(RVO::Vector2(-400.0f, -400.0f));
		//
		// sim->addObstacle(obstacle1);
		// sim->addObstacle(obstacle2);
		// sim->addObstacle(obstacle3);
		// sim->addObstacle(obstacle4);
		std::vector<RVO::Vector2> obstacle;
		obstacle.push_back(RVO::Vector2(800.0f, 1200.0f));
		obstacle.push_back(RVO::Vector2(800.0f, 800.0f));
		obstacle.push_back(RVO::Vector2(1200.0f, 800.0f));
		obstacle.push_back(RVO::Vector2(1200.0f, 1200.0f));
		sim->addObstacle(obstacle);
		/* Process the obstacles so that they are accounted for in the simulation. */
		sim->processObstacles();
	}

	void OriginBlockSimulate::setPreferredVelocities()
	{
		/*
		 * Set the preferred velocity to be a vector of unit magnitude (speed) in the
		 * direction of the goal.
		 */
		#ifdef _OPENMP
	#pragma omp parallel for
	#endif
		for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
			Vector2 goalVector = goals[i] - sim->getAgentPosition(i);

			if (absSq(goalVector) > 1.0f) {
				goalVector = normalize(goalVector) * 4.3;
				// goalVector = normalize(goalVector) * 43;
			}

			sim->setAgentPrefVelocity(i, goalVector);

			/*
			 * Perturb a little to avoid deadlocks due to perfect symmetry.
			 */
			float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
			float dist = std::rand() * 0.0001f / RAND_MAX;

			sim->setAgentPrefVelocity(i, sim->getAgentPrefVelocity(i) +
									  dist * Vector2(std::cos(angle), std::sin(angle)));
		}
	}
	
	void OriginBlockSimulate::reachedGoal()
	{
		for (size_t i = 0; i < sim->getNumAgents(); ++i) {
			if (!reached[i] && absSq(sim->getAgentPosition(i) - goals[i]) < 10 * 10) {
				reached[i] = true;
				
				USkeletalMeshComponent * skeletalCom = sim->agentActors[i]->FindComponentByClass<USkeletalMeshComponent>();
				if (skeletalCom)
				{
					// skeletalCom->SetPlayRate()
					UAnimSequence *AnimSequence = LoadObject<UAnimSequence>(0, TEXT("AnimSequence'/Game/ORCA/Meshes/Charactors/Motions/Zombie_Idle.Zombie_Idle'"));
					skeletalCom->PlayAnimation(AnimSequence, true);
					
					FVector BlendParams(50.0f, 0.0f, 0.0f);  
					skeletalCom->GetSingleNodeInstance()->SetBlendSpaceInput(BlendParams);
				}
			}
		}
	}
}
