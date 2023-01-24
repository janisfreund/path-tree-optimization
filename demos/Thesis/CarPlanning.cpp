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

/* Author: Mark Moll */

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <boost/program_options.hpp>
#include "ompl/geometric/planners/partial/Partial.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;

// The easy problem is the standard narrow passage problem: two big open
// spaces connected by a narrow passage. The hard problem is essentially
// one long narrow passage with the robot facing towards the long walls
// in both the start and goal configurations.

bool isStateValidEasy(const ob::SpaceInformation *si, const ob::State *state)
{
    const auto *s = state->as<ob::SE2StateSpace::StateType>();
    double x=s->getX(), y=s->getY();
    return si->satisfiesBounds(s) && (x<5 || x>13 || (y>8.5 && y<9.5));
}

bool isStateValidHard(const ob::SpaceInformation *si, const ob::State *state)
{
    return si->satisfiesBounds(state);
}

static bool isStateValidWorld(const ob::State *state, ob::World *world) {
    return true;
}

static std::vector<int> targetFound(const ob::State *state) {
    int p = rand() % 100;
    if (p > 75) {
        int r = rand() % 4;
        return std::vector<int>{r};
    }
    return std::vector<int>{};
}

void plan(const ob::StateSpacePtr& space, bool easy)
{
    ob::ScopedState<> start(space), goal(space);
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    if (easy)
        bounds.setHigh(18);
    else
    {
        bounds.high[0] = 6;
        bounds.high[1] = .6;
    }
    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    const ob::SpaceInformation *si = ss.getSpaceInformation().get();
//    auto isStateValid = easy ? isStateValidEasy : isStateValidHard;
//    ss.setStateValidityChecker([isStateValid, si](const ob::State *state)
//                               {
//                                   return isStateValid(si, state);
//                               });

    auto si_ = ss.getSpaceInformation();

    // 4 parking spaces
    si_->initWorld(4, true);

    ss.setStateValidityAndTargetChecker(isStateValidWorld, targetFound, si_->getWorld());

    // set the start and goal states
    if (easy)
    {
        start[0] = start[1] = 1.; start[2] = 0.;
        goal[0] = goal[1] = 17; goal[2] = -.99*boost::math::constants::pi<double>();
    }
    else
    {
        start[0] = start[1] = .5; start[2] = .5*boost::math::constants::pi<double>();;
        goal[0] = 5.5; goal[1] = .5; goal[2] = .5*boost::math::constants::pi<double>();
    }
    ss.setStartAndGoalStates(start, goal);

    ss.addGoalState(std::vector<double>{4, 2, 0});
    ss.addGoalState(std::vector<double>{6, 2, 0});
    ss.addGoalState(std::vector<double>{8, 2, 0});
    ss.addGoalState(std::vector<double>{10, 2, 0});

    // this call is optional, but we put it in to get more output information
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
    auto planner(std::make_shared<og::Partial>(ss.getSpaceInformation()));
    planner->setProblemDefinition(ss.getProblemDefinition());
    planner->setup();
    ss.setPlanner(planner);
    ss.setup();
    ss.print();

    // attempt to solve the problem within 30 seconds of planning time
    ob::PlannerStatus solved = ss.solve(30.0);

    if (solved)
    {
        std::vector<double> reals;

        std::cout << "Found solution:" << std::endl;
        ss.simplifySolution();
        og::PathGeometric path = ss.getSolutionPath();
        path.interpolate(1000);
        path.printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

void printTrajectory(const ob::StateSpacePtr& space, const std::vector<double>& pt)
{
    if (pt.size()!=3) throw ompl::Exception("3 arguments required for trajectory option");
    const unsigned int num_pts = 50;
    ob::ScopedState<> from(space), to(space), s(space);
    std::vector<double> reals;

    from[0] = from[1] = from[2] = 0.;

    to[0] = pt[0];
    to[1] = pt[1];
    to[2] = pt[2];

    std::cout << "distance: " << space->distance(from(), to()) << "\npath:\n";
    for (unsigned int i=0; i<=num_pts; ++i)
    {
        space->interpolate(from(), to(), (double)i/num_pts, s());
        reals = s.reals();
        std::cout << "path " << reals[0] << ' ' << reals[1] << ' ' << reals[2] << ' ' << std::endl;
    }
}

void printDistanceGrid(const ob::StateSpacePtr& space)
{
    // print the distance for (x,y,theta) for all points in a 3D grid in SE(2)
    // over [-5,5) x [-5, 5) x [-pi,pi).
    //
    // The output should be redirected to a file, say, distance.txt. This
    // can then be read and plotted in Matlab like so:
    //     x = reshape(load('distance.txt'),200,200,200);
    //     for i=1:200,
    //         contourf(squeeze(x(i,:,:)),30);
    //         axis equal; axis tight; colorbar; pause;
    //     end;
    const unsigned int num_pts = 200;
    ob::ScopedState<> from(space), to(space);
    from[0] = from[1] = from[2] = 0.;

    for (unsigned int i=0; i<num_pts; ++i)
        for (unsigned int j=0; j<num_pts; ++j)
            for (unsigned int k=0; k<num_pts; ++k)
            {
                to[0] = 5. * (2. * (double)i/num_pts - 1.);
                to[1] = 5. * (2. * (double)j/num_pts - 1.);
                to[2] = boost::math::constants::pi<double>() * (2. * (double)k/num_pts - 1.);
                std::cout << space->distance(from(), to()) << '\n';
            }

}

int main(int argc, char* argv[])
{
    try
    {
        ob::StateSpacePtr space(std::make_shared<ob::ReedsSheppStateSpace>());

        plan(space, true);
    }
    catch(std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    catch(...) {
        std::cerr << "Exception of unknown type!\n";
    }

    return 0;
}
