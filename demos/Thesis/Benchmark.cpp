//
// Created by janis on 04.01.23.
//

#include "ompl/tools/benchmark/Benchmark.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include "ompl/geometric/planners/partial/Partial.h"
#include "ompl/base/World.h"

const int NUM_OBJECTS = 4;
const double SOLVE_TIME = 30;

namespace ob = ompl::base;
namespace og = ompl::geometric;

class RigidBodyPlanning {
public:

    static bool isStateValid(const ob::State *state, ob::World world) {
        const auto *state3D =
                state->as<ob::RealVectorStateSpace::StateType>();
        double x = state3D->values[0];
        double y = state3D->values[1];
        double z = state3D->values[2];

        // check validity of state defined by pos & rot
        // no self-collision checking necessary for toy example
        // no bounds checking necessary because no states outside the bounds can be sampled

        // check collision with environment, in this toy example only check collision with POObjects
        std::vector<ob::ObjectState> worldState = world.getState();

        int i = 0;
        for (ob::ObjectState objectState: worldState) {
            if (objectState == ob::ObjectState(1)) {
                // POObject exists -> check collision
                std::vector<double> pos = POObjects.at(i).first;
                std::vector<double> len = POObjects.at(i).second;
                if ((pos.at(0) - (len.at(0) / 2) <= x && x <= pos.at(0) + (len.at(0) / 2)) ||
                    (pos.at(1) - (len.at(1) / 2) <= y && y <= pos.at(1) + (len.at(1) / 2)) ||
                    (pos.at(2) - (len.at(2) / 2) <= z && z <= pos.at(2) + (len.at(2) / 2))) {
//                    std::cout << x << ", " << y << ", " << z << ": Not valid in world ";
//                    for (int n = 0; n < world.getNumObjects(); n++) {
//                        std::cout << world.getState().at(n);
//                    }
//                    std::cout << std::endl;
                    return false;
                }
            }
            i++;
        }
//        std::cout << x << ", " << y << ", " << z << ": Valid in world ";
//        for (int i = 0; i < world.getNumObjects(); i++) {
//            std::cout << world.getState().at(i);
//        }
//        std::cout << std::endl;
        return true;
    }

    static std::vector<int> targetFound(const ob::State *state) {
        int p = rand() % 100;
        if (p > 75) {
            int r = rand() % NUM_OBJECTS;
            return std::vector<int>{r};
        }
        return std::vector<int>{};
    }

    static ob::StateSamplerPtr allocCameraStateSampler(const ob::StateSpace *space)
    {
        return std::make_shared<ob::CameraStateSampler>(space);
    }


    void plan() {
        auto space(std::make_shared<ob::RealVectorBeliefStateSpace>());

        space->addDimension();
        space->addDimension();
        space->addDimension();
        space->addDimension();
        space->addDimension();
//        space->addDimension();
//        space->addDimension();
//        space->addDimension();
//        space->addDimension();
//        space->addDimension();

        // set the bounds for the R^3 part of SE(3)
        ob::RealVectorBounds bounds(5);
        bounds.setLow(-M_PI);
        bounds.setHigh(M_PI);

        space->setBounds(bounds);

        space->setStateSamplerAllocator(allocCameraStateSampler);

        ss_ = new ompl::geometric::SimpleSetup(space);

        // construct an instance of  space information from this state space
        auto si = ss_->getSpaceInformation();
        // auto si(std::make_shared<ob::SpaceInformation>(space));

        // set state validity checking for this space
        //si->setStateValidityChecker(isStateValid);

        si->initWorld(NUM_OBJECTS, true);

//        si->setStateValidityAndTargetChecker(isStateValid, targetFound, *si->getWorld());
        ss_->setStateValidityAndTargetChecker(isStateValid, targetFound, *si->getWorld());

        // create POObjects based on NUM_OBJECTS
        for (int i = 0; i < NUM_OBJECTS; i++) {
            std::pair<std::vector<double>, std::vector<double>> obj = std::make_pair(std::vector<double>({0, 0, 0}), std::vector<double>({0.02, 0.02, 0.02}));
            POObjects.push_back(obj);
        }

        // create a random start state
        ob::ScopedState<> start(space);
//    start.random();
        start->as<ob::RealVectorBeliefStateSpace::StateType>()->values[0] = 0;
        start->as<ob::RealVectorBeliefStateSpace::StateType>()->values[1] = 0;
        start->as<ob::RealVectorBeliefStateSpace::StateType>()->values[2] = 0;
        start->as<ob::RealVectorBeliefStateSpace::StateType>()->values[3] = 0;
        start->as<ob::RealVectorBeliefStateSpace::StateType>()->values[4] = 0;
//        start->as<ob::RealVectorBeliefStateSpace::StateType>()->values[5] = 0;
//        start->as<ob::RealVectorBeliefStateSpace::StateType>()->values[6] = 0;
//        start->as<ob::RealVectorBeliefStateSpace::StateType>()->values[7] = 0;
//        start->as<ob::RealVectorBeliefStateSpace::StateType>()->values[8] = 0;
//        start->as<ob::RealVectorBeliefStateSpace::StateType>()->values[9] = 0;
        std::cout << "START: " << start << std::endl;

        // create a random goal state
        ob::ScopedState<> goal(space);
//    goal.random();
        goal->as<ob::RealVectorBeliefStateSpace::StateType>()->values[0] = 0.8;
        goal->as<ob::RealVectorBeliefStateSpace::StateType>()->values[1] = 0.8;
        goal->as<ob::RealVectorBeliefStateSpace::StateType>()->values[2] = 0.8;
        goal->as<ob::RealVectorBeliefStateSpace::StateType>()->values[3] = 0.8;
        goal->as<ob::RealVectorBeliefStateSpace::StateType>()->values[4] = 0.8;
//        goal->as<ob::RealVectorBeliefStateSpace::StateType>()->values[5] = 0.8;
//        goal->as<ob::RealVectorBeliefStateSpace::StateType>()->values[6] = 0.8;
//        goal->as<ob::RealVectorBeliefStateSpace::StateType>()->values[7] = 0.8;
//        goal->as<ob::RealVectorBeliefStateSpace::StateType>()->values[8] = 0.8;
//        goal->as<ob::RealVectorBeliefStateSpace::StateType>()->values[9] = 0.8;
        std::cout << "GOAL: " << goal << std::endl;

        // create a problem instance
//        auto pdef(std::make_shared<ob::ProblemDefinition>(si));

        // set the start and goal states
//        pdef->setStartAndGoalStates(start, goal);

        ss_->setStartAndGoalStates(start, goal);

        // set multiple goal states
//        pdef->addGoalState(std::vector<double>{2.844, 1.124, 1.455, 0.595, 1.422});
//        pdef->addGoalState(std::vector<double>{-3.042, 1.124, 1.455, 0.595, 1.422});
//        pdef->addGoalState(std::vector<double>{-3.042, -3.142, -1.587, 0.595, 1.422});
//        pdef->addGoalState(std::vector<double>{3.009, -3.142, -1.587, 0.595, 1.422});

        ss_->addGoalState(std::vector<double>{2.844, 1.124, 1.455, 0.595, 1.422});
        ss_->addGoalState(std::vector<double>{-3.042, 1.124, 1.455, 0.595, 1.422});
        ss_->addGoalState(std::vector<double>{-3.042, -3.142, -1.587, 0.595, 1.422});
        ss_->addGoalState(std::vector<double>{3.009, -3.142, -1.587, 0.595, 1.422});

//        // create a planner for the defined space
//        auto planner(std::make_shared<og::Partial>(si));
//
//        // set the problem we are trying to solve for the planner
//        planner->setProblemDefinition(pdef);
//
//        // perform setup steps for the planner
//        planner->setup();
//
//
//        // print the settings for this space
//        si->printSettings(std::cout);
//
//        // print the problem settings
//        pdef->print(std::cout);
    }

    void benchmark() {
        // First we create a benchmark class:
        ompl::tools::Benchmark b(*ss_, "my experiment");

        // We add the planners to evaluate.
        b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::Partial(ss_->getSpaceInformation())));

        // etc.
        // Now we can benchmark: 5 second time limit for each plan computation,
        // 100 MB maximum memory usage per plan computation, 50 runs for each planner
        // and true means that a text-mode progress bar should be displayed while
        // computation is running.
        ompl::tools::Benchmark::Request req;
        req.maxTime = 5.0;
        req.maxMem = 100.0;
        req.runCount = 5;
        req.displayProgress = true;
        b.benchmark(req);

        // This will generate a file of the form ompl_host_time.log
        b.saveResultsToFile();
    }

private:
    ompl::geometric::SimpleSetup *ss_;
    static inline std::vector<std::pair<std::vector<double>, std::vector<double>>> POObjects;
};

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    static RigidBodyPlanning rbp;

    rbp.plan();

    rbp.benchmark();

    return 0;
}