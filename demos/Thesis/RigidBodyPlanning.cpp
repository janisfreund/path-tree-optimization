/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include "ompl/geometric/planners/partial/Partial.h"
#include "ompl/base/World.h"

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
                    std::cout << x << ", " << y << ", " << z << ": Not valid in world ";
                    for (int n = 0; n < world.getNumObjects(); n++) {
                        std::cout << world.getState().at(n);
                    }
                    std::cout << std::endl;
                    return false;
                }
            }
            i++;
        }
        std::cout << x << ", " << y << ", " << z << ": Valid in world ";
        for (int i = 0; i < world.getNumObjects(); i++) {
            std::cout << world.getState().at(i);
        }
        std::cout << std::endl;
        return true;
    }

    static std::vector<int> targetFound(const ob::State *state) {
        return std::vector<int>{};
    }

    void plan() {
        // construct the state space we are planning in
        auto space(std::make_shared<ob::RealVectorBeliefStateSpace>());

        space->addDimension();
        space->addDimension();
        space->addDimension();

        // set the bounds for the R^3 part of SE(3)
        ob::RealVectorBounds bounds(3);
        bounds.setLow(-1);
        bounds.setHigh(1);

        space->setBounds(bounds);

        // construct an instance of  space information from this state space
        auto si(std::make_shared<ob::SpaceInformation>(space));

        // set state validity checking for this space
        //si->setStateValidityChecker(isStateValid);

        si->setStateValidityAndTargetChecker(isStateValid, targetFound);

        si->initWorld(1);

        // create a random start state
        ob::ScopedState<> start(space);
//    start.random();
        start->as<ob::RealVectorBeliefStateSpace::StateType>()->values[0] = -0.8;
        start->as<ob::RealVectorBeliefStateSpace::StateType>()->values[1] = -0.8;
        start->as<ob::RealVectorBeliefStateSpace::StateType>()->values[2] = -0.8;
        std::cout << "START: " << start << std::endl;

        // create a random goal state
        ob::ScopedState<> goal(space);
//    goal.random();
        goal->as<ob::RealVectorBeliefStateSpace::StateType>()->values[0] = 0.8;
        goal->as<ob::RealVectorBeliefStateSpace::StateType>()->values[1] = 0.8;
        goal->as<ob::RealVectorBeliefStateSpace::StateType>()->values[2] = 0.8;
        std::cout << "GOAL: " << goal << std::endl;

        // create a problem instance
        auto pdef(std::make_shared<ob::ProblemDefinition>(si));

        // set the start and goal states
        pdef->setStartAndGoalStates(start, goal);

        // create a planner for the defined space
        auto planner(std::make_shared<og::Partial>(si));
//    auto planner(std::make_shared<og::RRTConnect>(si));

        // set the problem we are trying to solve for the planner
        planner->setProblemDefinition(pdef);

        // perform setup steps for the planner
        planner->setup();


        // print the settings for this space
        si->printSettings(std::cout);

        // print the problem settings
        pdef->print(std::cout);

        // attempt to solve the problem within one second of planning time
        ob::PlannerStatus solved = planner->ob::Planner::solve(5.0);

        if (solved) {
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path
            ob::PathPtr path = pdef->getSolutionPath();
            std::cout << "Found solution:" << std::endl;

            // print the path to screen
            path->print(std::cout);
        } else
            std::cout << "No solution found" << std::endl;
    }

private:
    static inline std::vector<std::pair<std::vector<double>, std::vector<double>>> POObjects = {{{0, 0, 0}, {0.02, 0.02, 0.02}}};
};

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    static RigidBodyPlanning rbp;

    rbp.plan();

    return 0;
}