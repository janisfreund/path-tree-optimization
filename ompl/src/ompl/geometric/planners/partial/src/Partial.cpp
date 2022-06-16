//
// Created by janis on 05.04.22.
//

#include "ompl/geometric/planners/partial/Partial.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

ompl::geometric::Partial::Partial(const base::SpaceInformationPtr &si) : base::Planner(si, "partial")
{
    // constructor
}

ompl::geometric::Partial::~Partial()
{
    // freeMemory();
}

ompl::base::PlannerStatus ompl::geometric::Partial::solve(const ompl::base::PlannerTerminationCondition &ptc) {
    // make sure the planner is configured correctly; ompl::base::Planner::checkValidity
    // ensures that there is at least one input state and a ompl::base::Goal object specified
    std::cout << "Started solving process using planner Partial.\n";

    checkValidity();

    // get a handle to the Goal from the ompl::base::ProblemDefinition member, pdef_
    base::Goal *goal = pdef_->getGoal().get();
    // get sampleable region from goal
    auto *goal_s = dynamic_cast<ompl::base::GoalSampleableRegion *>(goal);

    // Ensure that we have a state sampler
    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    int numWorldStates = si_->getWorld()->getNumWorldStates();

    // get input states with PlannerInputStates helper, pis_
    while (const base::State *st = pis_.nextStart()) {
    // st will contain a start state.  Typically this state will
    // be cloned here and inserted into the Planner's data structure.
        // si_->copyState() // create suitable data structure
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        for (int i = 0; i < numWorldStates; i++) {
            nn_.at(i)->add(motion);
        }
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    // if needed, sample states from the goal region (and wait until a state is sampled)
    // const base::State *st = pis_.nextGoal(ptc);

    Motion *solution[numWorldStates];
    for (int i = 0; i < numWorldStates; i++) {
        solution[i] = nullptr;
    }
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;

    // periodically check if ptc() returns true.
    // if it does, terminate planning.
    while (!ptc()) {
        // Start planning here.
        std::cout << "New state sampled.\n";
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        // Sample world
        // int worldIdx = rand() % si_->getWorld().getNumWorldStates();

        ompl::base::World* world = si_->getWorld();

        bool flag = false;
        /// iterate worlds
        for (int worldIdx = 0; worldIdx < numWorldStates; worldIdx++) {
            if (solution[worldIdx] != nullptr) {
                continue;
            }

            std::cout << "New world set." << std::endl;
            // set world state
            world->setState(worldIdx);

            // find closest state in the tree
            Motion *nmotion = nn_.at(worldIdx)->nearest(rmotion);
            base::State *dstate = rstate;

            // check if motion is valid
            if (si_->checkMotionWorlds(nmotion->state, dstate)) {
                // add motion to nn
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, dstate);
                motion->parent = nmotion;
                nn_.at(worldIdx)->add(motion);

                const auto* state3D =
                        motion->state->as<ompl::base::RealVectorStateSpace::StateType>();
                const auto* parent3D =
                        motion->parent->state->as<ompl::base::RealVectorStateSpace::StateType>();

                nmotion = motion;

                std::cout << "New motion added to world " << worldIdx << " from state [" << parent3D->values[0] << ", "
                    << parent3D->values[1] << ", " << parent3D->values[2] << "] to state [" << state3D->values[0] << ", "
                    << state3D->values[1] << ", " << state3D->values[2] << "]" << std::endl;
            }

            // check if solution found
            double dist = 0.0;
            bool sat = goal->isSatisfied(nmotion->state, &dist);
            if (sat) {
                solution[worldIdx] = nmotion;
                bool allWorldsSolved = true;
                for (Motion *s : solution) {
                    if (s == nullptr) {
                        allWorldsSolved = false;
                        break;
                    }
                }
                if (allWorldsSolved) {
                    flag = true;
                    break;
                }
            }
        }

        if (flag) {
            break;
        }

        // call routines from SpaceInformation (si_) as needed. i.e.,
        // si_->allocStateSampler() for sampling,
        // si_->checkMotion(state1, state2) for state validity, etc...

            // si_->checkMotion()

        // use the Goal pointer to evaluate whether a sampled state satisfies the goal requirements

        // use log macros for informative messaging, i.e., logInfo("Planner found a solution!");
    }


    // When a solution path is computed, save it here
    std::vector<int> worldsSolved;
    std::vector<int> worldsUnsolved;
    int idx = 0;
    for (Motion *s : solution) {
        if (s != nullptr) {
            worldsSolved.push_back(idx);
            idx++;
        } else {
            worldsUnsolved.push_back(idx);
        }
    }
    if (!worldsSolved.empty()) {
        std::cout << std::endl << std::endl;
        if (!worldsUnsolved.empty()) {
            std::cout << "No solutions found for worlds ";
            for (int i: worldsUnsolved) {
                std::cout << i << " ";
            }
            std::cout << std::endl;
        }
        /* construct the solution path */
        for (int i : worldsSolved) {
            std::vector<Motion *> mpath;
            while (solution[i] != nullptr)
            {
                mpath.push_back(solution[i]);
                solution[i] = solution[i]->parent;
            }

            /* set the solution path */
            auto path(std::make_shared<PathGeometric>(si_));
            for (int i = mpath.size() - 1; i >= 0; --i)
                path->append(mpath[i]->state);

            std::cout << "Found solution for world " << i << ":" << std::endl;
            path->print(std::cout);
            pdef_->addSolutionPath(path);
        }
    }

    // clear memory
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    if (worldsSolved.empty()) {
        return base::PlannerStatus::TIMEOUT;
    }
    // Return a value from the PlannerStatus enumeration.
    // See ompl::base::PlannerStatus for the possible return values; base::PlannerStatus::EXACT_SOLUTION
    return base::PlannerStatus::EXACT_SOLUTION;
}
