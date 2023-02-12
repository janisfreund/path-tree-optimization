//
// Created by janis on 05.04.22.
//

#include <boost/graph/copy.hpp>
#include "ompl/geometric/planners/partial/Partial.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/geometric/PathSimplifier.h"

ompl::geometric::Partial::Partial(const base::SpaceInformationPtr &si) : base::Planner(si, "partial")
{
    // constructor
}

ompl::geometric::Partial::~Partial()
{
    freeMemory();
}

ompl::base::PlannerStatus ompl::geometric::Partial::solve(const ompl::base::PlannerTerminationCondition &ptc) {
    std::cout << "Started solving process using planner Partial.\n";

    // print graphs etc.
    bool extendedOutput = false;
    // stop sampling if there is a solution for every world
    bool terminateIfSolutionFound = false;

    // get the world and number of different possible world states from space information
    ompl::base::World *world = si_->getWorld();
    int numWorldStates = world->getNumWorldStates();

    // vector for storing motions that lead to final states
    for (int i = 0; i < numWorldStates; i++) {
        lastGoalMotion_.push_back(nullptr);
    }

    // define final states for each world
    std::vector<base::State *> goalStates = pdef_->getGoalStates();

    // define variables for benchmarking time needed by specific parts of the planner and distances of the planned paths
    std::vector<double> distancesDirect;
    std::vector<double> distancesPlanned(static_cast<int>(goalStates.size()));
    double timeTotal = 0;
    double timeSampling = 0;
    double timeCheckMotion = 0;
    double timeBeliefGraph = 0;
    double timePolicyExtraction = 0;
    double timeOptimalPathTree = 0;

    std::chrono::steady_clock::time_point t_total_start = std::chrono::steady_clock::now();

    // make sure the planner is configured correctly
    checkValidity();

    // ensure that we have a state sampler
    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    // set seed if there is a seed set in the problem definition
    if (pdef_->getSeed() != -1) {
        int seed = pdef_->getSeed();
        rng_.setLocalSeed(0);
        sampler_->setSeed(seed);
    }
    else {
        rng_.setLocalSeed(0);
    }

    // define colors for visualizing different beliefs in graphs
    std::vector<std::string> colors = {"aquamarine", "blue", "coral", "cyan", "darkred", "gold", "lime", "webpurple"};

    // create random graph
    Graph randomGraph;
    std::vector<VertexTrait> randomGraphVertices;
    std::vector<EdgeTrait> randomGraphEdges;
    std::vector<int> randomGraphFinalStates;

    // get input states with PlannerInputStates helper, pis_
    while (const base::State *st = pis_.nextStart()) {
        // compute distancesDirect from start state to all possible goal states
        for (base::State *gState : goalStates) {
            distancesDirect.push_back(si_->getStateSpace()->distanceBase(gState, st, 2));
        }
        // fill nn structure with starting state for all possible worlds
        for (int i = 0; i < numWorldStates; i++) {
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, st);
            std::vector<float> initBelief;
            int numObjects = world->getNumObjects();
            for (int i = 0; i < numObjects; i++) {
                initBelief.push_back(1 / numObjects);
            }
            motion->nodeIdx = 0;
            motion->isGoal = 0;
            nn_.at(i)->add(motion);

            // add start node to random graph once
            if (i == 0) {
                VertexTrait v = add_vertex(randomGraph);
                randomGraph[v].state = motion->state;
                randomGraph[v].observableObjects = si_->targetFound(motion->state);
                randomGraph[v].fontcolor = "red";
                double x = static_cast<const base::RealVectorStateSpace::StateType *>(randomGraph[v].state)->values[0] * 10;
                double y = (static_cast<const base::RealVectorStateSpace::StateType *>(randomGraph[v].state)->values[1]) * 10;
                std::string pos_str = std::to_string(x) + ", " + std::to_string(y) + "!";
                randomGraph[v].pos = pos_str;
                randomGraphVertices.push_back(v);
            }
        }
    }

    // initialize solution motions for all world states with nullptr
    Motion *solution[numWorldStates];
    for (int i = 0; i < numWorldStates; i++) {
        solution[i] = nullptr;
    }

    // create new Motion objects to hold a new random state
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;

    // max distance to goal for goal states
    double dist = 0.0;

    int randomGraphIdx = 1;
    int sampledIdx = 1;

    // periodically check if ptc() returns true / the max number of iterations is reached
    std::chrono::steady_clock::time_point t_sampling_start = std::chrono::steady_clock::now();
    int numIterations = pdef_->getIterations();
    bool iterationTermination = false;
    if (numIterations > 0) {
        iterationTermination = true;
    }
    while (!(ptc() && !iterationTermination) && (numIterations > 0 || !iterationTermination)) {
        numIterations--;
        // sample world
        int sampledWorldIdx;
        if (sampledIdx <= world->getNumWorldStates()) {
            sampledWorldIdx = randomGraphIdx - 1;
        }
        else {
            sampledWorldIdx = rng_.uniformInt(0, world->getNumWorldStates() - 1);
        }
        // set world to the sampled idx
        world->setState(sampledWorldIdx);

        // set goal state if there are multiple goal states
        int finStateIdx = sampledWorldIdx;
        if (static_cast<int>(goalStates.size()) != 0) {
            if (static_cast<int>(goalStates.size()) == numWorldStates) {
                // set goal to goal state corresponding to the sampled world
                pdef_->setGoalState(goalStates[sampledWorldIdx], std::numeric_limits<double>::epsilon());
            } else {
                // set random goal
                int ridx = rand() % static_cast<int>(goalStates.size());
                pdef_->setGoalState(goalStates[ridx], std::numeric_limits<double>::epsilon());
                finStateIdx = ridx;
            }
        }

        // get a handle to the Goal from the ompl::base::ProblemDefinition member, pdef_
        base::Goal *goal = pdef_->getGoal().get();
        // get sampleable region from goal
        auto *goal_s = dynamic_cast<ompl::base::GoalSampleableRegion *>(goal);

        // sample a new state
        double rValue = rng_.uniform01();
        // sample state at base starting position, looking towards the goal
        if (sampledIdx <= world->getNumWorldStates()) {
            sampler_->sampleGoodCameraPositionNear(rstate, pdef_->getStartState(0)->as<base::RealVectorStateSpace::StateType>()->values[0], pdef_->getStartState(0)->as<base::RealVectorStateSpace::StateType>()->values[1]);
            sampledIdx++;
        }
        // sample the goal state
        else if ((goal_s != nullptr) && rValue < goalBias_ && goal_s->canSample()) {
            goal_s->sampleGoal(rstate);
        }
        // sample uniformly
        else if (rValue < (goalBias_ + 0.2)) {
            sampler_->sampleUniform(rstate);
        }
        // sample uniformly, looking towards the goal
        else {
            sampler_->sampleGoodCameraPosition(rstate);
        }

        // check if sampled state is valid in sampled world
        if (!si_->isValid(rstate, world)) {
            continue;
        }

        // add new node to random graph
        base::State *dstate = rstate;
        VertexTrait v = add_vertex(randomGraph);
        randomGraph[v].state = si_->allocState();
        si_->copyState(randomGraph[v].state, dstate);

        // store state base position for creating graphs
        double x = static_cast<const base::RealVectorStateSpace::StateType *>(randomGraph[v].state)->values[0] * 10;
        double y = (static_cast<const base::RealVectorStateSpace::StateType *>(randomGraph[v].state)->values[1]) * 10;
        std::string pos_str = std::to_string(x) + ", " + std::to_string(y) + "!";
        randomGraph[v].pos = pos_str;

        if (extendedOutput) {std::cout << "Sampled state: " << pos_str << std::endl;}

        // TODO currently using camera image taken in a world in which all objects are present
        world->setState(world->getNumWorldStates() - 1);

        // check which objects are observable from sampled state
        randomGraph[v].observableObjects = si_->targetFound(dstate);

        // store sampled state in random graph
        randomGraphVertices.push_back(v);

        // terminate if solution for all worlds is found
        bool flag = false;

        std::set<int> parentNodes;

        // iterate worlds
        for (int worldIdx = 0; worldIdx < numWorldStates; worldIdx++) {
            // set world state
            world->setState(worldIdx);

            // find the states in the tree within radius
            std::vector<Motion*> nmotionVec;
            // TODO no fixed radius
            nn_.at(worldIdx)->nearestR(rmotion, 0.5, nmotionVec);

            // add nearest state if there is no state within the specified radius
            if (nmotionVec.empty()) {
                nmotionVec.push_back(nn_.at(worldIdx)->nearest(rmotion));
            }

            // iterate over all nearest motions
            bool motionAdded = false;
            for (Motion *nmotion : nmotionVec) {
                // check if motion is valid
                std::chrono::steady_clock::time_point t_checkM_start = std::chrono::steady_clock::now();
                bool validMotion = si_->checkMotionWorlds(nmotion->state, dstate);
                std::chrono::steady_clock::time_point t_checkM_end = std::chrono::steady_clock::now();
                timeCheckMotion += (std::chrono::duration_cast<std::chrono::milliseconds>(t_checkM_end - t_checkM_start).count()) / 1000.0;
                if (validMotion) {
                    // add motion to nn
                    auto *motion = new Motion(si_);
                    si_->copyState(motion->state, dstate);
                    motion->parent = nmotion;
                    motion->nodeIdx = randomGraphIdx;
                    motion->isGoal = goal->isSatisfied(dstate, &dist);
                    // only add motion with same state once to nn
                    if (!motionAdded) {
                        nn_.at(worldIdx)->add(motion);
                        motionAdded = true;
                    }

                    // don't connect if both states are goal states
                    if (!(motion->isGoal && motion->parent->isGoal)) {
                        // add new edge to random graph if it does not yet exist
                        if (parentNodes.find(motion->parent->nodeIdx) == parentNodes.end()) {
                            std::pair<EdgeTrait, bool> p = add_edge(randomGraphVertices.at(motion->nodeIdx),
                                                                    randomGraphVertices.at(motion->parent->nodeIdx),
                                                                    randomGraph);
                            EdgeTrait e = p.first;
                            randomGraph[e].worldValidities.push_back(worldIdx);
                            randomGraphEdges.push_back(e);
                            parentNodes.insert(motion->parent->nodeIdx);
                            std::vector<int> vals = randomGraph[e].worldValidities;

                        }
                        // push back valid world idx to random graph edge
                        else {
                            int edgeIdx = 0;
                            for (; edgeIdx < static_cast<int>(randomGraphEdges.size()); edgeIdx++) {
                                if ((randomGraphEdges.at(edgeIdx).m_source == motion->nodeIdx &&
                                     randomGraphEdges.at(edgeIdx).m_target == motion->parent->nodeIdx) ||
                                    (randomGraphEdges.at(edgeIdx).m_source == motion->parent->nodeIdx &&
                                     randomGraphEdges.at(edgeIdx).m_target == motion->nodeIdx)) {
                                    break;
                                }
                            }
                            randomGraph[randomGraphEdges.at(edgeIdx)].worldValidities.push_back(worldIdx);
                            std::vector<int> vals = randomGraph[randomGraphEdges.at(edgeIdx)].worldValidities;
                        }
                    }

                    nmotion = motion;
                }

                // check if sampled state is final in world
                bool sat = goal->isSatisfied(dstate, &dist);
                if (sat) {
                    solution[worldIdx] = nmotion;
                    lastGoalMotion_[worldIdx] = solution[worldIdx];
                }
            }
        }

        // check if solution is found
        bool sat = goal->isSatisfied(dstate, &dist);
        if (sat) {
            // set vertex of random graph to final
            randomGraph[randomGraphIdx].fontcolor = "blue";
            randomGraph[randomGraphIdx].finalSateIdx = finStateIdx;
            randomGraphFinalStates.push_back(randomGraphIdx);
            flag = true;
        }

        // terminate if solution is found and terminateIfSolutionFound is set
        if (flag && terminateIfSolutionFound) {
            break;
        }

        randomGraphIdx++;
    }

    std::chrono::steady_clock::time_point t_sampling_end = std::chrono::steady_clock::now();
    timeSampling = (std::chrono::duration_cast<std::chrono::milliseconds>(t_sampling_end - t_sampling_start).count()) / 1000.0;
    // save random graph
    if (extendedOutput) {
        saveGraph(randomGraph, "random", false, true);
    }

    std::chrono::steady_clock::time_point t_belief_start = std::chrono::steady_clock::now();

    // create belief graph
    std::vector<base::BeliefState> beliefStates = world->getAllBeliefStates();
    Graph beliefGraph;
    std::vector<VertexTrait> beliefGraphVertices[static_cast<int>(beliefStates.size())];
    std::vector<VertexTrait> allBeliefGraphVertices;

    // create belief graphs containing only vertices of one belief
    Graph singleBeliefGraph[static_cast<int>(world->getAllBeliefStates().size())];
    std::vector<EdgeTrait> singleBeliefGraphEdges[static_cast<int>(world->getAllBeliefStates().size())];

    // compute which nodes should be added to different beliefs
    bool activeNodes[static_cast<int>(world->getAllBeliefStates().size())][static_cast<int>(randomGraphVertices.size())];
    // beliefs x random graph vertices, init with false
    for (int i = 0; i < static_cast<int>(world->getAllBeliefStates().size()); i++) {
        for (int n = 0; n < static_cast<int>(randomGraphVertices.size()); n++) {
            activeNodes[i][n] = false;
        }
    }
    for (EdgeTrait re : randomGraphEdges) {
        std::pair<std::vector<int>, std::vector<base::BeliefState>> bp = world->getCompatibleBeliefs(
                randomGraph[re].worldValidities);
        std::vector<int> beliefIdx = bp.first;
        for (int idx: beliefIdx) {
            activeNodes[idx][re.m_source] = true;
            activeNodes[idx][re.m_target] = true;
        }
    }

    // for mapping from random graph to belief graph
    std::map<int, int> graphMap[static_cast<int>(world->getAllBeliefStates().size())];
    // for mapping from belief graph to random graph
    std::map<int, int> graphMapReverse[static_cast<int>(world->getAllBeliefStates().size())];
    // for mapping from random graph to the corresponding belief graph that contains only one belief
    std::map<int, int> singleGraphMap[static_cast<int>(world->getAllBeliefStates().size())];

    // add nodes for all possible beliefs
    int nodesCount = 0;
    for (VertexTrait node : randomGraphVertices) {
        int idx = 0;
        for (base::BeliefState b : beliefStates) {
            if (activeNodes[idx][node]) {
                VertexTrait v = add_vertex(beliefGraph);
                beliefGraph[v].state = randomGraph[node].state;
                beliefGraph[v].observableObjects = randomGraph[node].observableObjects;
                beliefGraph[v].fontcolor = randomGraph[node].fontcolor;
                beliefGraph[v].finalSateIdx = randomGraph[node].finalSateIdx;
                beliefGraph[v].beliefState = b;

                // check if final random graph node is also final in belief
                if (beliefGraph[v].fontcolor == "blue") {
                    if (pdef_->getMode() == 1) {
                        if (!(fabs(beliefGraph[v].beliefState.at(beliefGraph[v].finalSateIdx) - 1) < 1e-3)) {
                            beliefGraph[v].fontcolor = "";
                        }
                    }
                }
                beliefGraph[v].color = colors[idx % static_cast<int>(colors.size())];
                beliefGraph[v].pos = randomGraph[node].pos;
                beliefGraph[v].label = std::to_string(nodesCount);
                beliefGraphVertices[idx].push_back(v);
                allBeliefGraphVertices.push_back(v);
                graphMap[idx][node] = v;
                graphMapReverse[idx][v] = node;
                completeGraphMap[v] = node;

                VertexTrait a = add_vertex(singleBeliefGraph[idx]);
                singleBeliefGraph[idx][a].state = randomGraph[node].state;
                singleBeliefGraph[idx][a].observableObjects = randomGraph[node].observableObjects;
                singleBeliefGraph[idx][a].fontcolor = randomGraph[node].fontcolor;
                singleBeliefGraph[idx][a].finalSateIdx = randomGraph[node].finalSateIdx;
                singleBeliefGraph[idx][a].color = colors[idx % static_cast<int>(colors.size())];
                singleBeliefGraph[idx][a].beliefState = b;
                singleBeliefGraph[idx][a].pos = randomGraph[node].pos;
                singleBeliefGraph[idx][a].label = std::to_string(nodesCount);

                // check if final random graph node is also final in belief
                if (singleBeliefGraph[idx][a].fontcolor == "blue") {
                    if (!fabs(singleBeliefGraph[idx][a].beliefState.at(singleBeliefGraph[idx][a].finalSateIdx) - 1) < 1e-3) {
                        singleBeliefGraph[idx][a].fontcolor = "";
                    }
                }

                singleGraphMap[idx][node] = a;
                nodesCount++;
            }

            idx++;
        }
    }

    // connect nodes within same belief
    for (EdgeTrait re : randomGraphEdges) {
        std::pair<std::vector<int>, std::vector<base::BeliefState>> bp = world->getCompatibleBeliefs(
                randomGraph[re].worldValidities);
        std::vector<int> beliefIdx = bp.first;
        for (int idx: beliefIdx) {
            std::pair<EdgeTrait, bool> p = add_edge(graphMap[idx].find(re.m_source)->second,
                                                    graphMap[idx].find(re.m_target)->second, beliefGraph);
            EdgeTrait e = p.first;
            beliefGraph[e].isWorldConnection = false;

            std::pair<EdgeTrait, bool> a = add_edge(singleGraphMap[idx].find(re.m_source)->second,
                                                    singleGraphMap[idx].find(re.m_target)->second, singleBeliefGraph[idx]);
            EdgeTrait b = a.first;
            beliefGraph[b].isWorldConnection = false;
            singleBeliefGraphEdges[idx].push_back(b);
        }
    }

    // print individual belief graphs
    if (extendedOutput) {
        int bIdx = 0;
        for (Graph g: singleBeliefGraph) {
            std::stringstream name;
            name << "[";
            for (float f: beliefStates.at(bIdx)) {
                name << f << ",";
            }
            name << "]";
            saveGraph(g, name.str(), true, true);
            bIdx++;
        }
    }

    // create transitions between beliefs due to observations
    for (int beliefIdx = 0; beliefIdx < sizeof(beliefGraphVertices) / sizeof(beliefGraphVertices[0]); beliefIdx++) {
        for (int nodeIdx = 0; nodeIdx < static_cast<int>(beliefGraphVertices[beliefIdx].size()); nodeIdx++) {
            VertexTrait v = beliefGraphVertices[beliefIdx].at(nodeIdx);
            for (int objectIdx : beliefGraph[v].observableObjects) {
                std::vector<base::BeliefState> newBeliefs = world->observe(beliefGraph[v].beliefState, objectIdx);
                // add edges if beliefs change
                if (static_cast<int>(newBeliefs.size()) != 1) {
                    for (base::BeliefState belief : newBeliefs) {
                        int newBeliefIdx = world->getBeliefIdx(belief);
                        std::pair<EdgeTrait , bool> p = add_edge(v, graphMap[newBeliefIdx].find(graphMapReverse[beliefIdx].find(v)->second)->second, beliefGraph);
                        EdgeTrait e = p.first;
                        beliefGraph[e].isWorldConnection = true;
                        beliefGraph[e].color = "red";
                        beliefGraph[v].beliefChildren.push_back(graphMap[newBeliefIdx].find(graphMapReverse[beliefIdx].find(v)->second)->second);
                    }
                }
            }
        }
    }

    std::chrono::steady_clock::time_point t_belief_end = std::chrono::steady_clock::now();
    timeBeliefGraph = (std::chrono::duration_cast<std::chrono::milliseconds>(t_belief_end - t_belief_start).count()) / 1000.0;
    // save belief graph
    if (extendedOutput) {
        saveGraph(beliefGraph, "belief", true, true);
        saveGraph(beliefGraph, "belief_no_pos", true, false);
    }

    std::chrono::steady_clock::time_point t_policy_start = std::chrono::steady_clock::now();

    // policy extraction
    std::priority_queue<std::pair<double, VertexTrait>, std::vector<std::pair<double, VertexTrait>>, std::greater<std::pair<double, VertexTrait>>> pq;
    // init priority queue
    for (VertexTrait v : allBeliefGraphVertices) {
        // goal states have 0 costs
        if(beliefGraph[v].fontcolor == "blue") {
            costs.push_back(0);
            pq.push(std::make_pair(0, v));
        }
        // all other states have inf costs
        else {
            costs.push_back(std::numeric_limits<double>::infinity());
        }
    }

    while (!pq.empty())
    {
        // take vertex with lowest cost from priority queue
        VertexTrait v = pq.top().second;
        pq.pop();
        Graph::adjacency_iterator it, end;
        std::tie(it, end) = boost::adjacent_vertices(v, beliefGraph);

        // iterate over all adjacent vertices
        for (; it != end; it++) {
            VertexTrait parent = it.dereference();
            EdgeTrait e = boost::edge(v, parent, beliefGraph).first;
            double newCost;
            // if edge is action edge -> Bellman update
            if (!beliefGraph[e].isWorldConnection) {
                if (std::isinf(costs[v])) {
                    newCost = std::numeric_limits<double>::infinity();
                }
                else {
                    double distance = si_->getStateSpace()->distanceBase(beliefGraph[v].state, beliefGraph[parent].state, 2);
                    newCost = distance + costs[v];
                }
            }
            // if edge is connection between beliefs
            else {
                std::vector<int> children = beliefGraph[parent].beliefChildren;
                if (children.empty()) {
                    newCost = std::numeric_limits<double>::infinity();
                }
                else {
                    // newCost = sum of branching probabilities * costs of children
                    newCost = 0;
                    double infCount = 0;
                    for (int nodeIdx : children) {
                        if (costs[nodeIdx] == std::numeric_limits<double>::infinity()) {
                            infCount += world->calcBranchingProbabilitiy(beliefGraph[parent].beliefState,
                                                                         beliefGraph[nodeIdx].beliefState);
                        }
                        else {
                            newCost += world->calcBranchingProbabilitiy(beliefGraph[parent].beliefState,
                                                                        beliefGraph[nodeIdx].beliefState) *
                                       costs[nodeIdx];
                        }
                    }
                    if (newCost == 0) {
                        newCost = std::numeric_limits<double>::infinity();
                    } else {
                        if (infCount != 0) {
                            // if mode==2: vertex is not inf if one of its belief children is not inf
                            if (pdef_->getMode() == 2) {
                                double multiplier = 1 / (1 - infCount);
                                newCost *= multiplier;
                            } else {
                                newCost = std::numeric_limits<double>::infinity();
                            }
                        }
                    }
                }
            }
            // update cost if it improved
            if (newCost < costs[parent]) {
                costs[parent] = newCost;
                pq.push(std::make_pair(newCost, parent));
            }
        }
    }

    std::chrono::steady_clock::time_point t_policy_end = std::chrono::steady_clock::now();
    timePolicyExtraction = (std::chrono::duration_cast<std::chrono::milliseconds>(t_policy_end - t_policy_start).count()) / 1000.0;

    pdef_->setSolutionCost(costs[0]);

    // print costs
    if (extendedOutput) {
        std::cout << "Costs: " << std::endl;
        int n = 0;
        for (double c: costs) {
            std::cout << n << ": " << c << " (" << world->getBeliefIdx(beliefGraph[n].beliefState) << ")" << std::endl;
            n++;
        }
    }

    std::chrono::steady_clock::time_point t_pathTree_start = std::chrono::steady_clock::now();

    // create optimal path tree
    VertexTrait currVertex = 0;
    VertexTrait v = add_vertex(pathTree);
    pathTree[v].state = beliefGraph[currVertex].state;
    pathTree[v].fontcolor = beliefGraph[currVertex].fontcolor;
    pathTree[v].finalSateIdx = beliefGraph[currVertex].finalSateIdx;
    pathTree[v].color = beliefGraph[currVertex].color;
    pathTree[v].beliefState = beliefGraph[currVertex].beliefState;
    pathTree[v].label = beliefGraph[currVertex].label;
    pathTree[v].observableObjects = beliefGraph[currVertex].observableObjects;
    pathTree[v].pos = beliefGraph[currVertex].pos;

    // create a debug graph for visualizing the path tree including the costs of vertices and edge + all adjacent vertices
    VertexTrait d = add_vertex(debugGraph);
    debugGraph[d].state = beliefGraph[currVertex].state;
    debugGraph[d].fontcolor = beliefGraph[currVertex].fontcolor;
    debugGraph[d].color = beliefGraph[currVertex].color;
    debugGraph[d].label = std::to_string(currVertex) + "\n" + std::to_string(completeGraphMap.find(currVertex)->second) + "\n" + std::to_string(std::round(costs[currVertex] * 100) / 100).substr(0, 4);
    debugGraph[d].pos = beliefGraph[currVertex].pos;

    // add all adjacent vertices to debug_graph
    Graph::adjacency_iterator it_, end_;
    std::tie(it_, end_) = boost::adjacent_vertices(currVertex, beliefGraph);
    for (; it_ != end_; it_++) {
        VertexTrait d_n = add_vertex(debugGraph);
        debugGraph[d_n].state = beliefGraph[it_.dereference()].state;
        debugGraph[d_n].fontcolor = beliefGraph[it_.dereference()].fontcolor;
        debugGraph[d_n].color = beliefGraph[it_.dereference()].color;
        debugGraph[d_n].label = std::to_string(it_.dereference()) + "\n" + std::to_string(completeGraphMap.find(it_.dereference())->second) + "\n" + std::to_string(std::round(costs[it_.dereference()] * 100) / 100).substr(0, 4);
        debugGraph[d_n].pos = beliefGraph[it_.dereference()].pos;

        double dis = si_->getStateSpace()->distanceBase(debugGraph[d].state, debugGraph[d_n].state, 2);
        if (boost::edge(currVertex, it_.dereference(), beliefGraph).second && beliefGraph[boost::edge(currVertex, it_.dereference(), beliefGraph).first].isWorldConnection) {
            if (std::find(beliefGraph[currVertex].beliefChildren.begin(), beliefGraph[currVertex].beliefChildren.end(), it_.dereference()) != beliefGraph[currVertex].beliefChildren.end()) {
                std::pair<EdgeTraitD, bool> d_p_n = add_edge(d, d_n, debugGraph);
                EdgeTraitD d_e_n = d_p_n.first;
                debugGraph[d_e_n].color = "red";
                debugGraph[d_e_n].label = std::to_string(std::round(dis * 100) / 100).substr(0, 4);
            } else {
                std::pair<EdgeTraitD, bool> d_p_n = add_edge(d_n, d, debugGraph);
                EdgeTraitD d_e_n = d_p_n.first;
                debugGraph[d_e_n].color = "red";
                debugGraph[d_e_n].label = std::to_string(std::round(dis * 100) / 100).substr(0, 4);
            }

        }
        else {
            std::pair<EdgeTraitD, bool> d_p_n = add_edge(d, d_n, debugGraph);
            EdgeTraitD d_e_n = d_p_n.first;
            debugGraph[d_e_n].label = std::to_string(std::round(dis * 100) / 100).substr(0, 4);
        }
    }

    // recursively construct the path tree
    constructPathTree(beliefGraph, costs, v, currVertex, std::set<VertexTrait>{}, d);

    // visualizing costs of vertices and edges of one vertex with all its surrounding vertices
    if (false) {
        GraphD tmpGraph;
        VertexTrait vertex_num = 126;
        VertexTrait d = add_vertex(tmpGraph);
        tmpGraph[d].state = beliefGraph[vertex_num].state;
        tmpGraph[d].fontcolor = "red";
        tmpGraph[d].color = "red";
        tmpGraph[d].label = std::to_string(vertex_num) + "\n" + std::to_string(completeGraphMap.find(vertex_num)->second) + "\n" + std::to_string(std::round(costs[vertex_num] * 100) / 100).substr(0, 4);
        tmpGraph[d].pos = beliefGraph[vertex_num].pos;
        Graph::adjacency_iterator a_it, a_end;
        std::tie(a_it, a_end) = boost::adjacent_vertices(vertex_num, beliefGraph);
        for (; a_it != a_end; a_it++) {
            VertexTrait d_n = add_vertex(tmpGraph);
            tmpGraph[d_n].state = beliefGraph[a_it.dereference()].state;
            tmpGraph[d_n].fontcolor = beliefGraph[a_it.dereference()].fontcolor;
            tmpGraph[d_n].color = beliefGraph[a_it.dereference()].color;
            tmpGraph[d_n].label = std::to_string(a_it.dereference()) + "\n" + std::to_string(completeGraphMap.find(a_it.dereference())->second) + "\n" + std::to_string(std::round(costs[a_it.dereference()] * 100) / 100).substr(0, 4);
            tmpGraph[d_n].pos = beliefGraph[a_it.dereference()].pos;

            double dis = si_->getStateSpace()->distanceBase(tmpGraph[d].state, tmpGraph[d_n].state, 2);
            if (boost::edge(vertex_num, a_it.dereference(), beliefGraph).second && beliefGraph[boost::edge(vertex_num, a_it.dereference(), beliefGraph).first].isWorldConnection) {
                if (std::find(beliefGraph[vertex_num].beliefChildren.begin(), beliefGraph[vertex_num].beliefChildren.end(), a_it.dereference()) != beliefGraph[vertex_num].beliefChildren.end()) {
                    std::pair<EdgeTraitD, bool> d_p_n = add_edge(d, d_n, tmpGraph);
                    EdgeTraitD d_e_n = d_p_n.first;
                    tmpGraph[d_e_n].color = "red";
                    tmpGraph[d_e_n].label = std::to_string(std::round(dis * 100) / 100).substr(0, 4);
                } else {
                    std::pair<EdgeTraitD, bool> d_p_n = add_edge(d_n, d, tmpGraph);
                    EdgeTraitD d_e_n = d_p_n.first;
                    tmpGraph[d_e_n].color = "red";
                    tmpGraph[d_e_n].label = std::to_string(std::round(dis * 100) / 100).substr(0, 4);
                }

            }
            else {
                std::pair<EdgeTraitD, bool> d_p_n = add_edge(d, d_n, tmpGraph);
                EdgeTraitD d_e_n = d_p_n.first;
                tmpGraph[d_e_n].label = std::to_string(std::round(dis * 100) / 100).substr(0, 4);
            }
        }
        saveGraph(tmpGraph, "188", true, false);
    }

    std::chrono::steady_clock::time_point t_pathTree_end = std::chrono::steady_clock::now();
    timeOptimalPathTree = (std::chrono::duration_cast<std::chrono::milliseconds>(t_pathTree_end - t_pathTree_start).count()) / 1000.0;

    // save debug graph and path tree
    if (extendedOutput) {
        saveGraph(pathTree, "path", true, true);
        saveGraph(debugGraph, "debug", true, false);
        saveGraph(pathTree, "path_no_pos", true, false);
        saveGraph(debugGraph, "debug_pos", true, true);
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
    }

    // save belief graph including costs
    if (extendedOutput) {
        for (VertexTrait vt: boost::make_iterator_range(vertices(beliefGraph))) {
            beliefGraph[vt].label =
                    std::to_string(vt) + "\n" + std::to_string(std::round(costs[vt] * 100) / 100).substr(0, 4);
        }
        for (EdgeTrait et: boost::make_iterator_range(edges(beliefGraph))) {
            beliefGraph[et].label = std::to_string(std::round(
                    si_->getStateSpace()->distanceBase(beliefGraph[et.m_source].state, beliefGraph[et.m_target].state,
                                                       2) * 100) / 100).substr(0, 4);
        }
        saveGraph(beliefGraph, "costs", true, false);
    }

    // return optimal path tree
    std::vector<std::vector<float>> pWorlds;

    // print worlds
    if (extendedOutput) {
        std::cout << "Worlds: ";
        int c = 0;
        for (std::vector<base::ObjectState> w: world->getWorldStates()) {
            std::cout << c << ": ";
            world->printStateFromInt(world->getStateIntFromObjectState(w));
            std::cout << "\n";
            c++;
        }
    }
    // if state 0 has infinite costs, no solution has been found
    if (costs[0] == std::numeric_limits<double>::infinity()) {
        return base::PlannerStatus::TIMEOUT;
    }
    // return solution path
    // iterate over all final states of the path tree
    for (VertexTraitD v : pathTreeFinalStates) {
        pWorlds.push_back(world->beliefToWorld(pathTree[v].beliefState));
        pdef_->addSolutionIdx(world->getBeliefIdx(pathTree[v].beliefState));
        double dis = 0;
        std::vector<int> observationIdx;
        int planIdx = pathTree[v].finalSateIdx;
        if (extendedOutput) {
            const auto *state3D =
                    pathTree[v].state->as<ompl::base::RealVectorStateSpace::StateType>();
            std::cout << "\nGoal satisfied with state " << v << ": [" << state3D->values[0] << ", "
                      << state3D->values[1]
                      << ", " << state3D->values[2] << "]" << " which is final state " << pathTree[v].finalSateIdx
                      << std::endl;
            std::cout << "Solution path in reversed order (form goal to start):" << std::endl;
        }
        auto path(std::make_shared<PathGeometric>(si_));
        std::vector<base::State*> pathR;
        VertexTraitD currNode = v;
        int c = 0;
        std::vector<base::BeliefState> stateBeliefs;
        // add next vertex of the path tree to the path until path to the start state is found
        while (currNode != 0) {
            if (extendedOutput) {
                std::cout << "Append to path: State " << pathTree[currNode].label << ": ";
                getSpaceInformation()->getStateSpace()->printState(pathTree[currNode].state, std::cout);
                std::cout << " with belief state ";
                world->printBelief(pathTree[currNode].beliefState);
                std::cout << std::endl;
            }
            pathR.push_back(pathTree[currNode].state);
            stateBeliefs.push_back(pathTree[currNode].beliefState);
            c++;
            GraphD::in_edge_iterator it, end;
            std::tie(it, end) = boost::in_edges(currNode, pathTree);
            for (; it != end; it++) {
                VertexTraitD nextNode = it->m_source;
                EdgeTraitD e = it.dereference();
                if (pathTree[e].color == "red") {
                    observationIdx.push_back(c);
                    pdef_->addObservationPoint(std::make_pair(pathTree[nextNode].state, pathTree[nextNode].observableObjects));
                }
                dis += si_->getStateSpace()->distanceBase(pathTree[currNode].state, pathTree[nextNode].state, 2);
                currNode = nextNode;
            }
        }
        // add start state to path
        pathR.push_back(pathTree[currNode].state);
        stateBeliefs.push_back(pathTree[currNode].beliefState);
        if (extendedOutput) {
            std::cout << "Append to path: State " << pathTree[currNode].label << ": ";
            getSpaceInformation()->getStateSpace()->printState(pathTree[currNode].state, std::cout);
            std::cout << " with belief state ";
            world->printBelief(pathTree[currNode].beliefState);
            std::cout << std::endl;
        }

        if (static_cast<int>(distancesPlanned.size()) > planIdx) {
            distancesPlanned.at(planIdx) = dis;
        }

        // split path into segment between observation points
        // allows path simplification for these segments
        int numSegments = static_cast<int>(observationIdx.size()) + 1;
        std::vector<std::shared_ptr<PathGeometric>> segments;
        if (numSegments > 1) {
            int i = static_cast<int>(pathR.size()) - 1;
            int c = 0;
            for (int segIdx = numSegments - 1; segIdx >= 0; segIdx--) {
                auto sPath(std::make_shared<PathGeometric>(si_));
                for (; i >= 0; ) {
                    sPath->append(pathR[i]);
                    if (segIdx > 0 && (static_cast<int>(pathR.size()) - 1 - observationIdx[segIdx - 1])  == c) {
                        std::vector<int> compatibleWorld;
                        for (float f : world->beliefToWorld(stateBeliefs[i+1])) {
                            if (fabs(f) < 1e-3) {
                                compatibleWorld.push_back(0);
                            } else {
                                compatibleWorld.push_back(1);
                            }
                        }
                        int worldIdx = world->getStateIdx(compatibleWorld);
                        pdef_->setGoalState(pathR[i], std::numeric_limits<double>::epsilon());
                        world->setState(worldIdx);
                        ompl::geometric::PathSimplifier psk = ompl::geometric::PathSimplifier(si_,
                                                                                              pdef_->getGoal(),
                                                                                              pdef_->getOptimizationObjective());
                        psk.shortcutPath(static_cast<ompl::geometric::PathGeometric &>(*sPath), sPath->getStateCount() * 50, sPath->getStateCount() * 50);

                        segments.push_back(sPath);
                        c++;
                        i--;
                        break;
                    }
                    // simplify the last segment
                    else if (i == 0) {
                        std::vector<int> compatibleWorld;
                        for (float f : world->beliefToWorld(pathTree[v].beliefState)) {
                            if (fabs(f) < 1e-3) {
                                compatibleWorld.push_back(0);
                            } else {
                                compatibleWorld.push_back(1);
                            }
                        }
                        int worldIdx = world->getStateIdx(compatibleWorld);
                        pdef_->setGoalState(pathR[i/*static_cast<int>(pathR.size()) - 1 - i*/], std::numeric_limits<double>::epsilon());
                        world->setState(worldIdx);
                        ompl::geometric::PathSimplifier psk = ompl::geometric::PathSimplifier(si_,
                                                                                              pdef_->getGoal(),
                                                                                              pdef_->getOptimizationObjective());
                        psk.shortcutPath(static_cast<ompl::geometric::PathGeometric &>(*sPath), 100, 100);

                        segments.push_back(sPath);
                    }
                    c++;
                    i--;
                }
            }

            // put segments together
            for (std::shared_ptr<PathGeometric> pathSeg : segments) {
                if (extendedOutput) {std::cout << "Segment: " << std::endl;}
                for (int i = 0; i < static_cast<int>(pathSeg->getStateCount()); i++) {
                    if (extendedOutput) {getSpaceInformation()->getStateSpace()->printState(pathSeg->getState(i), std::cout);}
                    path->append(pathSeg->getState(i));
                }
            }
        }
        else {
            for (int i = static_cast<int>(pathR.size()) - 1; i >= 0; i--) {
                path->append(pathR[i]);
            }
        }

        // store the non-simplified graph
        std::vector<base::State *> pathRaw;
        for (int i = static_cast<int>(pathR.size()) - 1; i >= 0; i--) {
            pathRaw.push_back(pathR[i]);
        }

        pdef_->addSolutionPath(path);
        pdef_->addRawSolution(pathRaw);
    }

    pdef_->setPWorlds(pWorlds);

    std::chrono::steady_clock::time_point t_total_end = std::chrono::steady_clock::now();
    timeTotal = (std::chrono::duration_cast<std::chrono::milliseconds>(t_total_end - t_total_start).count()) / 1000.0;

    std::cout << std::endl << std::endl;
    std::cout << "Number of sampled states: " << static_cast<int>(randomGraphVertices.size()) << std::endl;
    std::cout << "Number of beliefs: " << static_cast<int>(beliefStates.size()) << std::endl;
    std::cout << "Total time: " << timeTotal << "s" << std::endl;
    std::cout << "Sampling time: " << timeSampling << "s" << std::endl;
    std::cout << "Check motion time: " << timeCheckMotion << "s" << std::endl;
    std::cout << "Belief graph creation time: " << timeBeliefGraph << "s" << std::endl;
    std::cout << "Policy extraction time: " << timePolicyExtraction << "s" << std::endl;
    std::cout << "Optimal path tree creation time: " << timeOptimalPathTree << "s" << std::endl;
    std::cout << std::endl << std::endl;

    if (static_cast<int>(distancesDirect.size()) > 0) {std::cout << "Distances:" << std::endl;}
    int disIdx = 0;
    for (int i = 0; i < static_cast<int>(distancesDirect.size()); i++) {
        std::cout << "----- Goal State " << disIdx << " -----" << std::endl;
        std::cout << "Distance of planned path: " << distancesPlanned.at(i) << std::endl;
        std::cout << "Optimal distance without collision: " << distancesDirect.at(i) << std::endl;
        disIdx++;
    }
    std::cout << std::endl << std::endl;

    // clear memory
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    return base::PlannerStatus::EXACT_SOLUTION;
}

void ompl::geometric::Partial::constructPathTree(Graph beliefGraph, std::vector<double> costs, VertexTrait v, VertexTrait currVertex, std::set<VertexTrait> visited, VertexTrait d_v) {
    while (true) {
        Graph::adjacency_iterator it, end;
        std::tie(it, end) = boost::adjacent_vertices(currVertex, beliefGraph);
        VertexTrait bestVertex = 0;
        VertexTrait bestVertexObs = 0;
        bool isConn = !beliefGraph[currVertex].beliefChildren.empty();
        for (; it != end; it++) {
            VertexTrait der = it.dereference();
            if (visited.find(it.dereference()) == visited.end()) {
                if (beliefGraph[boost::edge(it.dereference(), currVertex, beliefGraph).first].isWorldConnection) {
                    if (isConn) {
                        VertexTraitD w = add_vertex(pathTree);
                        pathTree[w].state = beliefGraph[it.dereference()].state;
                        pathTree[w].fontcolor = beliefGraph[it.dereference()].fontcolor;
                        pathTree[w].finalSateIdx = beliefGraph[it.dereference()].finalSateIdx;
                        pathTree[w].color = beliefGraph[it.dereference()].color;
                        pathTree[w].beliefState = beliefGraph[it.dereference()].beliefState;
                        pathTree[w].label = beliefGraph[it.dereference()].label;
                        pathTree[w].observableObjects = beliefGraph[it.dereference()].observableObjects;
                        if (pathTree[w].fontcolor == "blue") {
                            pathTreeFinalStates.push_back(w);
                        }
                        pathTree[w].pos = beliefGraph[it.dereference()].pos;
                        std::pair<EdgeTraitD, bool> p = add_edge(v, w, pathTree);
                        EdgeTraitD e = p.first;
                        pathTree[e].color = "red";

                        VertexTrait d = add_vertex(debugGraph);
                        debugGraph[d].state = pathTree[w].state;
                        debugGraph[d].fontcolor = pathTree[w].fontcolor;
                        debugGraph[d].color = pathTree[w].color;
                        debugGraph[d].label = std::to_string(it.dereference()) + "\n" + std::to_string(completeGraphMap.find(it.dereference())->second) + "\n" + std::to_string(std::round(costs[it.dereference()] * 100) / 100).substr(0, 4);
                        debugGraph[d].pos = pathTree[w].pos;

                        std::pair<EdgeTraitD, bool> d_p = add_edge(d_v, d, debugGraph);
                        EdgeTraitD d_e = d_p.first;
                        debugGraph[d_e].color = "red";
                        debugGraph[d_e].label = std::to_string(std::round(si_->getStateSpace()->distanceBase(debugGraph[d_v].state, debugGraph[d].state, 2) * 100) / 100).substr(0, 4);

                        // add all adjacent vertices to debug_graph
                        Graph::adjacency_iterator it_, end_;
                        std::tie(it_, end_) = boost::adjacent_vertices(it.dereference(), beliefGraph);
                        for (; it_ != end_; it_++) {
                            VertexTrait d_n = add_vertex(debugGraph);
                            debugGraph[d_n].state = beliefGraph[it_.dereference()].state;
                            debugGraph[d_n].fontcolor = beliefGraph[it_.dereference()].fontcolor;
                            debugGraph[d_n].color = beliefGraph[it_.dereference()].color;
                            debugGraph[d_n].label = std::to_string(it_.dereference()) + "\n" + std::to_string(completeGraphMap.find(it_.dereference())->second) + "\n" + std::to_string(std::round(costs[it_.dereference()] * 100) / 100).substr(0, 4);
                            debugGraph[d_n].pos = beliefGraph[it_.dereference()].pos;

                            double dis = si_->getStateSpace()->distanceBase(debugGraph[d].state, debugGraph[d_n].state, 2);
                            if (boost::edge(currVertex, it_.dereference(), beliefGraph).second && beliefGraph[boost::edge(currVertex, it_.dereference(), beliefGraph).first].isWorldConnection) {
                                if (std::find(beliefGraph[currVertex].beliefChildren.begin(), beliefGraph[currVertex].beliefChildren.end(), it_.dereference()) != beliefGraph[currVertex].beliefChildren.end()) {
                                    std::pair<EdgeTraitD, bool> d_p_n = add_edge(d, d_n, debugGraph);
                                    EdgeTraitD d_e_n = d_p_n.first;
                                    debugGraph[d_e_n].color = "red";
                                    debugGraph[d_e_n].label = std::to_string(std::round(dis * 100) / 100).substr(0, 4);
                                } else {
                                    std::pair<EdgeTraitD, bool> d_p_n = add_edge(d_n, d, debugGraph);
                                    EdgeTraitD d_e_n = d_p_n.first;
                                    debugGraph[d_e_n].color = "red";
                                    debugGraph[d_e_n].label = std::to_string(std::round(dis * 100) / 100).substr(0, 4);
                                }

                            }
                            else {
                                std::pair<EdgeTraitD, bool> d_p_n = add_edge(d, d_n, debugGraph);
                                EdgeTraitD d_e_n = d_p_n.first;
                                debugGraph[d_e_n].label = std::to_string(std::round(dis * 100) / 100).substr(0, 4);
                            }
                        }

                        visited.insert(it.dereference());
                        if (pathTree[w].fontcolor != "blue") {
                            constructPathTree(beliefGraph, costs, w, it.dereference(), visited, d);
                        }
                    }
                } else {
                    if (bestVertex == 0 || ((costs[it.dereference()] + si_->getStateSpace()->distanceBase(beliefGraph[currVertex].state, beliefGraph[it.dereference()].state, 2)) <= (costs[bestVertex] + si_->getStateSpace()->distanceBase(beliefGraph[currVertex].state, beliefGraph[bestVertex].state, 2)))) {
                        if (si_->getStateSpace()->distanceBase(beliefGraph[currVertex].state,
                                                               beliefGraph[it.dereference()].state, 2) < 1e-2) {
                            double calculatedCosts = costs[bestVertex] +
                                                     si_->getStateSpace()->distanceBase(beliefGraph[currVertex].state,
                                                                                        beliefGraph[bestVertex].state,
                                                                                        2);
                            double actualCost = costs[currVertex];
                            if (!(actualCost - 0.01 < calculatedCosts && calculatedCosts < actualCost + 0.01) ||
                                bestVertex == 0) {
                                bestVertex = it.dereference();
                            }
                        } else {
                            bestVertex = it.dereference();
                        }
                    }
                }
            }
        }

        if (isConn || bestVertex == 0 || costs[bestVertex] == std::numeric_limits<double>::infinity()) {
            break;
        }

        VertexTraitD u = add_vertex(pathTree);
        pathTree[u].state = beliefGraph[bestVertex].state;
        pathTree[u].fontcolor = beliefGraph[bestVertex].fontcolor;
        pathTree[u].finalSateIdx = beliefGraph[bestVertex].finalSateIdx;
        pathTree[u].color = beliefGraph[bestVertex].color;
        pathTree[u].beliefState = beliefGraph[bestVertex].beliefState;
        pathTree[u].label = beliefGraph[bestVertex].label;
        pathTree[u].observableObjects = beliefGraph[bestVertex].observableObjects;
        if (pathTree[u].fontcolor == "blue") {
            pathTreeFinalStates.push_back(u);
        }
        pathTree[u].pos = beliefGraph[bestVertex].pos;
        std::pair<EdgeTraitD , bool> p = add_edge(v, u, pathTree);

        VertexTrait d = add_vertex(debugGraph);
        debugGraph[d].state = pathTree[u].state;
        debugGraph[d].fontcolor = pathTree[u].fontcolor;
        debugGraph[d].color = pathTree[u].color;
        debugGraph[d].label = std::to_string(bestVertex) + "\n" + std::to_string(completeGraphMap.find(bestVertex)->second) + "\n" + std::to_string(std::round(costs[bestVertex] * 100) / 100).substr(0, 4);
        debugGraph[d].pos = pathTree[u].pos;

        std::pair<EdgeTraitD, bool> d_p = add_edge(d_v, d, debugGraph);
        EdgeTraitD d_e = d_p.first;
        debugGraph[d_e].label = std::to_string(std::round(si_->getStateSpace()->distanceBase(debugGraph[d_v].state, debugGraph[d].state, 2) * 100) / 100).substr(0, 4);
        debugGraph[d_e].color = "blue";

        // add all adjacent vertices to debug_graph
        Graph::adjacency_iterator it_, end_;
        std::tie(it_, end_) = boost::adjacent_vertices(bestVertex, beliefGraph);
        for (; it_ != end_; it_++) {
            if (it_.dereference() != currVertex /*not correct*/ /*&& visited.find(it_.dereference()) == visited.end()*/) {
                VertexTrait d_n = add_vertex(debugGraph);
                debugGraph[d_n].state = beliefGraph[it_.dereference()].state;
                debugGraph[d_n].fontcolor = beliefGraph[it_.dereference()].fontcolor;
                debugGraph[d_n].color = beliefGraph[it_.dereference()].color;
                debugGraph[d_n].label = std::to_string(it_.dereference()) + "\n" + std::to_string(completeGraphMap.find(it_.dereference())->second) + "\n" + std::to_string(std::round(costs[it_.dereference()] * 100) / 100).substr(0, 4);
                debugGraph[d_n].pos = beliefGraph[it_.dereference()].pos;

                double dis = si_->getStateSpace()->distanceBase(debugGraph[d].state, debugGraph[d_n].state, 2);
                if (boost::edge(bestVertex, it_.dereference(), beliefGraph).second && beliefGraph[boost::edge(bestVertex, it_.dereference(), beliefGraph).first].isWorldConnection) {
                    if (std::find(beliefGraph[bestVertex].beliefChildren.begin(), beliefGraph[bestVertex].beliefChildren.end(), it_.dereference()) != beliefGraph[bestVertex].beliefChildren.end()) {
                        std::pair<EdgeTraitD, bool> d_p_n = add_edge(d, d_n, debugGraph);
                        EdgeTraitD d_e_n = d_p_n.first;
                        debugGraph[d_e_n].color = "red";
                        debugGraph[d_e_n].label = std::to_string(std::round(dis * 100) / 100).substr(0, 4);
                    } else {
                        std::pair<EdgeTraitD, bool> d_p_n = add_edge(d_n, d, debugGraph);
                        EdgeTraitD d_e_n = d_p_n.first;
                        debugGraph[d_e_n].color = "red";
                        debugGraph[d_e_n].label = std::to_string(std::round(dis * 100) / 100).substr(0, 4);
                    }

                }
                else {
                    std::pair<EdgeTraitD, bool> d_p_n = add_edge(d, d_n, debugGraph);
                    EdgeTraitD d_e_n = d_p_n.first;
                    debugGraph[d_e_n].label = std::to_string(std::round(dis * 100) / 100).substr(0, 4);
                }
            }
        }

        v = u;
        d_v = d;
        currVertex = bestVertex;
        visited.insert(currVertex);

        if (beliefGraph[bestVertex].fontcolor == "blue") {
            break;
        }
    }
}

void ompl::geometric::Partial::getPlannerData(base::PlannerData &data) const
{
    // add all edges and start state from nn structure (better belief graph?) to data
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    for (std::shared_ptr<NearestNeighbors<Motion *>> nn : nn_) {
        if (nn)
            nn->list(motions);
    }

    for (Motion *m : lastGoalMotion_) {
        if (m != nullptr)
            data.addGoalVertex(base::PlannerDataVertex(m->state));
    }

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}

void ompl::geometric::Partial::freeMemory()
{
    for (std::shared_ptr<NearestNeighbors<Motion *>> nn : nn_) {
        if (nn) {
            std::vector<Motion *> motions;
            nn->list(motions);
            for (auto &motion: motions) {
                if (motion->state != nullptr)
                    si_->freeState(motion->state);
                delete motion;
            }
        }
    }
    // TODO also delete graphs
}

// save graph as png
void ompl::geometric::Partial::saveGraph(Graph g, std::string name, bool useLabels, bool usePos) {
//    std::ofstream colored_dot_file(name + std::string(".dot"));
//    boost::dynamic_properties dp_no_pos;
//    dp_no_pos.property("node_id",   get(boost::vertex_index, g));
//    dp_no_pos.property("color", get(&EdgeStruct::color, g));
//    dp_no_pos.property("color", get(&VertexStruct::color, g));
//    dp_no_pos.property("fontcolor", get(&VertexStruct::fontcolor, g));
//    if (useLabels) {
//        dp_no_pos.property("label", get(&VertexStruct::label, g));
//        dp_no_pos.property("label", get(&EdgeStruct::label, g));
//    }
//    if (usePos) {
//        dp_no_pos.property("pos", get(&VertexStruct::pos, g));
//    }
//    dp_no_pos.property("splines", boost::make_constant_property<Graph*>(true));
//    dp_no_pos.property("overlap", boost::make_constant_property<Graph*>(false));
//    boost::write_graphviz_dp(colored_dot_file, g, dp_no_pos);
//    std::stringstream command;
//    command << "neato -T png " << name << ".dot -o " << name << ".png";
//    system(command.str().c_str());
    std::cout << "Graph " << name << " saved." << std::endl;
}

void ompl::geometric::Partial::saveGraph(GraphD g, std::string name, bool useLabels, bool usePos) {
//    std::ofstream colored_dot_file(name + std::string(".dot"));
//    boost::dynamic_properties dp_no_pos;
//    dp_no_pos.property("node_id",   get(boost::vertex_index, g));
//    dp_no_pos.property("color", get(&EdgeStruct::color, g));
//    dp_no_pos.property("color", get(&VertexStruct::color, g));
//    dp_no_pos.property("fontcolor", get(&VertexStruct::fontcolor, g));
//    if (useLabels) {
//        dp_no_pos.property("label", get(&VertexStruct::label, g));
//        dp_no_pos.property("label", get(&EdgeStruct::label, g));
//    }
//    if (usePos) {
//        dp_no_pos.property("pos", get(&VertexStruct::pos, g));
//    }
//    dp_no_pos.property("splines", boost::make_constant_property<GraphD*>(true));
//    dp_no_pos.property("overlap", boost::make_constant_property<GraphD*>(false));
//    boost::write_graphviz_dp(colored_dot_file, g, dp_no_pos);
//    std::stringstream command;
//    command << "neato -T png " << name << ".dot -o " << name << ".png";
//    system(command.str().c_str());
    std::cout << "Graph " << name << " saved." << std::endl;
}

