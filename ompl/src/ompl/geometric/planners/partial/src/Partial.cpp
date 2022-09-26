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

    bool extendedOutput = false;
    bool terminateIfSolutionFound = false;

    bool returnRandomGraph = false;
    bool returnBeliefGraph = false;
    bool returnPathTree = true;
    bool returnNN = false;

    double timeTotal = 0;
    double timeSampling = 0;
    double timeCheckMotion = 0;
    double timeBeliefGraph = 0;
    double timePolicyExtraction = 0;
    double timeOptimalPathTree = 0;

    std::chrono::steady_clock::time_point t_total_start = std::chrono::steady_clock::now();

    checkValidity();

    // get a handle to the Goal from the ompl::base::ProblemDefinition member, pdef_
    base::Goal *goal = pdef_->getGoal().get();
    // get sampleable region from goal
    auto *goal_s = dynamic_cast<ompl::base::GoalSampleableRegion *>(goal);

    // Ensure that we have a state sampler
    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    ompl::base::World *world = si_->getWorld();
    int numWorldStates = world->getNumWorldStates();

    std::vector<std::string> colors = {"aquamarine", "blue", "coral", "cyan", "darkred", "gold", "lime", "webpurple"};

    // create random graph
    Graph randomGraph;
    std::vector<VertexTrait> randomGraphVertices;
    std::vector<EdgeTrait> randomGraphEdges;
    std::vector<int> randomGraphFinalStates;

    // get input states with PlannerInputStates helper, pis_
    while (const base::State *st = pis_.nextStart()) {
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

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    Motion *solution[numWorldStates];
    for (int i = 0; i < numWorldStates; i++) {
        solution[i] = nullptr;
    }

    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;

    // max distance to goal for goal states
    double dist = 0.0;

    int randomGraphIdx = 1;

    // periodically check if ptc() returns true.
    // if it does, terminate planning.
    std::chrono::steady_clock::time_point t_sampling_start = std::chrono::steady_clock::now();
    while (!ptc()) {
        // Sample a new state uniformly or sample the goal
        std::cout << "New state sampled.\n";
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        // Sample world
        int sampledWorldIdx = rand() % world->getNumWorldStates();
        world->setState(sampledWorldIdx);

        // Check if sampled state is valid in sampled world
        if (!si_->isValid(rstate, *world)) {
            continue;
        }

        base::State *dstate = rstate;
        // add new node to random graph
        VertexTrait v = add_vertex(randomGraph);
        randomGraph[v].state = si_->allocState();
        si_->copyState(randomGraph[v].state, dstate);

        double x = static_cast<const base::RealVectorStateSpace::StateType *>(randomGraph[v].state)->values[0] * 10;
        double y = (static_cast<const base::RealVectorStateSpace::StateType *>(randomGraph[v].state)->values[1]) * 10;
        std::string pos_str = std::to_string(x) + ", " + std::to_string(y) + "!";
        randomGraph[v].pos = pos_str;

        // check which objects are visible from sampled state
        // TODO currently using camera image taken in a world in which all objects are present
        world->setState(world->getNumWorldStates() - 1);
        randomGraph[v].observableObjects = si_->targetFound(dstate);

        randomGraphVertices.push_back(v);

        // terminate if solution for all worlds is found
        bool flag = false;

        std::set<int> parentNodes;

        // iterate worlds
        for (int worldIdx = 0; worldIdx < numWorldStates; worldIdx++) {
            // set world state
            world->setState(worldIdx);
            std::cout << "New world set." << std::endl;

            // find the states in the tree within radius
            std::vector<Motion*> nmotionVec;
            nn_.at(worldIdx)->nearestR(rmotion, 3.0, nmotionVec);

            // buggy -> use only nearest
//            nn_.at(worldIdx)->nearestK(rmotion, 20, nmotionVec);

//            nmotionVec.push_back(nn_.at(worldIdx)->nearest(rmotion));

            if (nmotionVec.empty()) {
                nmotionVec.push_back(nn_.at(worldIdx)->nearest(rmotion));
            }

            std::cout << "#nearest: " << static_cast<int>(nmotionVec.size()) << std::endl;

            // iterate over all nearest motions
            for (Motion *nmotion : nmotionVec) {
                std::cout << "Nearest motion from " << randomGraphIdx << ": " << nmotion->nodeIdx << std::endl;
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
                    nn_.at(worldIdx)->add(motion);

                    // add new edge to random graph if it does not yet exist; otherwise push back valid world idx
                    if (parentNodes.find(motion->parent->nodeIdx) == parentNodes.end()) {
                        std::pair<EdgeTrait, bool> p = add_edge(randomGraphVertices.at(motion->nodeIdx),
                                                                randomGraphVertices.at(motion->parent->nodeIdx),
                                                                randomGraph);
                        EdgeTrait e = p.first;
                        randomGraph[e].worldValidities.push_back(worldIdx);
                        randomGraphEdges.push_back(e);
                        parentNodes.insert(motion->parent->nodeIdx);

                        std::cout << "New edge added to random graph from " << e.m_source << " to " << e.m_target << std::endl;
                        std::vector<int> vals = randomGraph[e].worldValidities;
                        std::cout << "Current world validities: ";
                        for (int val : vals) {
                            std::cout << val << ", ";
                        }
                        std::cout << std::endl << std::endl;

                    } else {
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

                        std::cout << "Random graph edge from " << randomGraphEdges.at(edgeIdx).m_source << " to " << randomGraphEdges.at(edgeIdx).m_target << " modified; " << worldIdx << " added" << std::endl;
                        std::vector<int> vals = randomGraph[randomGraphEdges.at(edgeIdx)].worldValidities;
                        std::cout << "Current world validities: ";
                        for (int val : vals) {
                            std::cout << val << ", ";
                        }
                        std::cout << std::endl << std::endl;
                    }

                    // print info
                    const auto *state3D =
                            motion->state->as<ompl::base::RealVectorStateSpace::StateType>();
                    const auto *parent3D =
                            motion->parent->state->as<ompl::base::RealVectorStateSpace::StateType>();

                    nmotion = motion;

//                    std::cout << "New motion added to world " << worldIdx << " from state [" << parent3D->values[0]
//                              << ", "
//                              << parent3D->values[1] << ", " << parent3D->values[2] << "] to state ["
//                              << state3D->values[0] << ", "
//                              << state3D->values[1] << ", " << state3D->values[2] << "]" << std::endl;
                }

                // check if solution is found
                bool sat = goal->isSatisfied(nmotion->state, &dist);
                if (sat) {
                    solution[worldIdx] = nmotion;
                    randomGraph[randomGraphIdx].fontcolor = "blue";
                    randomGraphFinalStates.push_back(randomGraphIdx);
                    bool allWorldsSolved = true;
                    for (Motion *s: solution) {
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
        }

        if (flag && terminateIfSolutionFound) {
            break;
        }

        randomGraphIdx++;
    }

    std::chrono::steady_clock::time_point t_sampling_end = std::chrono::steady_clock::now();
    timeSampling = (std::chrono::duration_cast<std::chrono::milliseconds>(t_sampling_end - t_sampling_start).count()) / 1000.0;
    saveGraph(randomGraph, "random", false);

    std::chrono::steady_clock::time_point t_belief_start = std::chrono::steady_clock::now();

    // create belief graph
    std::vector<base::BeliefState> beliefStates = world->getAllBeliefStates();
    Graph beliefGraph;
    std::vector<VertexTrait> beliefGraphVertices[static_cast<int>(beliefStates.size())];
    std::vector<VertexTrait> allBeliefGraphVertices;

    // for printing seperated graphs
    Graph singleBeliefGraph[static_cast<int>(world->getAllBeliefStates().size())];
    std::vector<EdgeTrait> singleBeliefGraphEdges[static_cast<int>(world->getAllBeliefStates().size())];

    // compute which nodes should be added to different beliefs
    bool activeNodes[static_cast<int>(world->getAllBeliefStates().size())][static_cast<int>(randomGraphVertices.size())];
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
    std::map<int, int> graphMapReverse[static_cast<int>(world->getAllBeliefStates().size())];
    std::map<int, int> singleGraphMap[static_cast<int>(world->getAllBeliefStates().size())];

    // add nodes for all possible beliefs
    int nodesCount = 0;
    for (VertexTrait node : randomGraphVertices) {
        // print info
        const auto* state3D =
                randomGraph[node].state->as<ompl::base::RealVectorStateSpace::StateType>();

        std::cout << "Added to RandomGraph: [" << state3D->values[0] << ", "
                  << state3D->values[1] << ", " << state3D->values[2] << "]" << std::endl;

        int idx = 0;
        for (base::BeliefState b : beliefStates) {
            if (activeNodes[idx][node]) {
                VertexTrait v = add_vertex(beliefGraph);
                beliefGraph[v].state = randomGraph[node].state;
                beliefGraph[v].observableObjects = randomGraph[node].observableObjects;
                beliefGraph[v].fontcolor = randomGraph[node].fontcolor;
                beliefGraph[v].color = colors[idx % static_cast<int>(colors.size())];
                beliefGraph[v].beliefState = b;
                beliefGraph[v].pos = randomGraph[node].pos;
                beliefGraph[v].label = std::to_string(nodesCount);
                beliefGraphVertices[idx].push_back(v);
                allBeliefGraphVertices.push_back(v);
                graphMap[idx][node] = v;
                graphMapReverse[idx][v] = node;

                VertexTrait a = add_vertex(singleBeliefGraph[idx]);
                singleBeliefGraph[idx][a].state = randomGraph[node].state;
                singleBeliefGraph[idx][a].observableObjects = randomGraph[node].observableObjects;
                singleBeliefGraph[idx][a].fontcolor = randomGraph[node].fontcolor;
                singleBeliefGraph[idx][a].color = colors[idx % static_cast<int>(colors.size())];
                singleBeliefGraph[idx][a].beliefState = b;
                singleBeliefGraph[idx][a].pos = randomGraph[node].pos;
                singleBeliefGraph[idx][a].label = std::to_string(nodesCount);
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

    // delete unconnected vertices NOT WORKING
//    int gIdx = 0;
//    for (Graph g : singleBeliefGraph) {
//        Graph::vertex_iterator n, nend;
//        for (boost::tie(n, nend) = vertices(g); n != nend; ++n) {
//            Graph::adjacency_iterator it, end;
//            std::tie(it, end) = boost::adjacent_vertices(*n, beliefGraph);
//            if (it == end) {
//                std::cout << "Vertex " << *n << " deleted!" << std::endl;
//                remove_vertex(*n, g);
//            }
//        }
//        singleBeliefGraph[gIdx] = g;
//    }

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
            saveGraph(g, name.str(), true);
            bIdx++;
        }
    }

    // create transitions between beliefs due to observations
    for (int beliefIdx = 0; beliefIdx < sizeof(beliefGraphVertices) / sizeof(beliefGraphVertices[0]); beliefIdx++) {
        //std::set<int> createdConnections;
        for (int nodeIdx = 0; nodeIdx < static_cast<int>(beliefGraphVertices[beliefIdx].size()); nodeIdx++) {
            VertexTrait v = beliefGraphVertices[beliefIdx].at(nodeIdx);
            // TODO handle multiple objects at once
            for (int objectIdx : beliefGraph[v].observableObjects) {
                std::vector<base::BeliefState> newBeliefs = world->observe(beliefGraph[v].beliefState, objectIdx);
                std::cout << "From belief " << world->getBeliefIdx(beliefGraph[v].beliefState) << " (";
                world->printBelief(beliefGraph[v].beliefState);
                std::cout << ") to belief ";
                // add edges if beliefs change
                if (static_cast<int>(newBeliefs.size()) != 1) {
                    for (base::BeliefState belief : newBeliefs) {
                        int newBeliefIdx = world->getBeliefIdx(belief);
                        std::cout << newBeliefIdx << " (";
                        world->printBelief(belief);
                        std::cout << ") ,";
                        if (static_cast<int>(beliefGraphVertices[newBeliefIdx].size()) > nodeIdx) {
                            //if (createdConnections.find(newBeliefIdx) == createdConnections.end()) {
                                // std::pair<EdgeTrait , bool> p = add_edge(v, beliefGraphVertices[newBeliefIdx].at(nodeIdx), beliefGraph);
                                std::pair<EdgeTrait , bool> p = add_edge(v, graphMap[newBeliefIdx].find(graphMapReverse[beliefIdx].find(v)->second)->second, beliefGraph);
                                EdgeTrait e = p.first;
                                beliefGraph[e].isWorldConnection = true;
                                beliefGraph[e].color = "red";
                                beliefGraph[beliefGraphVertices[beliefIdx].at(nodeIdx)].beliefChildren.push_back(beliefGraphVertices[newBeliefIdx].at(nodeIdx));
                                //createdConnections.insert(newBeliefIdx);
                            //}
                        }
                    }
                    std::cout << std::endl;
                }
            }
        }
    }

    std::chrono::steady_clock::time_point t_belief_end = std::chrono::steady_clock::now();
    timeBeliefGraph = (std::chrono::duration_cast<std::chrono::milliseconds>(t_belief_end - t_belief_start).count()) / 1000.0;
    saveGraph(beliefGraph, "belief", true);

    std::chrono::steady_clock::time_point t_policy_start = std::chrono::steady_clock::now();

    // policy extraction
    std::vector<double> costs;
    std::priority_queue<std::pair<double, VertexTrait>, std::vector<std::pair<double, VertexTrait>>, std::greater<std::pair<double, VertexTrait>>> pq;
    // init priority queue
    for (VertexTrait v : allBeliefGraphVertices) {
        // goal states have 0 costs
        // check if final belief
        bool final = false;
        for (int i : beliefGraph[v].beliefState) {
            if (i == 1) {
                final = true;
            }
        }
        if(beliefGraph[v].fontcolor == "blue" /*&& final*/) {
            costs.push_back(0);
            pq.push(std::make_pair(0, v));
            std::cout << "New goal state added: " << std::endl;
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
                if (isinf(costs[v])) {
                    newCost = std::numeric_limits<double>::infinity();
                }
                else {
                    double distance = si_->getStateSpace()->distance(beliefGraph[v].state, beliefGraph[parent].state);
                    newCost = distance + costs[v];
                }
            }
            // if edge is connection between beliefs
            else {
//                Graph::adjacency_iterator pit, pend;
//                std::tie(pit, pend) = boost::adjacent_vertices(parent, beliefGraph);
//                int numChildren = 0;
//                std::vector<VertexTrait> childrenNodes;
//                for (; pit != pend; pit++) {
//                    if (beliefGraph[boost::edge(pit.dereference(), parent, beliefGraph).first].isWorldConnection) { //TODO ?
//                        childrenNodes.push_back(pit.dereference());
//                        numChildren++;
//                    }
//                }
//                for (VertexTrait cn : childrenNodes) {
//                    if (isinf(costs[cn])) {
//                        newCost = std::numeric_limits<double>::infinity();
//                        break;
//                    }
//                    newCost += (1/numChildren) * costs[cn];
//                }
                std::vector<int> children = beliefGraph[parent].beliefChildren;
                if (children.empty()) {
                    newCost = std::numeric_limits<double>::infinity();
                }
                else {
                    newCost = 0;
                    for (int nodeIdx : children) {
                        newCost += world->calcBranchingProbabilitiy(beliefGraph[parent].beliefState, beliefGraph[nodeIdx].beliefState) * costs[nodeIdx];
                    }
                }
            }
            if (newCost < costs[parent]) {
                costs[parent] = newCost;
                pq.push(std::make_pair(newCost, parent));
            }
        }
    }

    std::chrono::steady_clock::time_point t_policy_end = std::chrono::steady_clock::now();
    timePolicyExtraction = (std::chrono::duration_cast<std::chrono::milliseconds>(t_policy_end - t_policy_start).count()) / 1000.0;

    // print costs
    std::cout << "Costs:" << std::endl;
    int n = 0;
    for (double c : costs) {
        std::cout << n << ": " << c << std::endl;
        n++;
    }

    std::chrono::steady_clock::time_point t_pathTree_start = std::chrono::steady_clock::now();

    // create optimal path tree
    VertexTrait currVertex = 0;
    VertexTrait v = add_vertex(pathTree);
    pathTree[v].state = beliefGraph[currVertex].state;
    pathTree[v].fontcolor = beliefGraph[currVertex].fontcolor;
    pathTree[v].color = beliefGraph[currVertex].color;
    pathTree[v].beliefState = beliefGraph[currVertex].beliefState;
    pathTree[v].label = beliefGraph[currVertex].label;
    getSpaceInformation()->getStateSpace()->printState(pathTree[v].state, std::cout);
    pathTree[v].pos = beliefGraph[currVertex].pos;
    constructPathTree(beliefGraph, costs, v, currVertex, std::set<VertexTrait>{});

    std::chrono::steady_clock::time_point t_pathTree_end = std::chrono::steady_clock::now();
    timeOptimalPathTree = (std::chrono::duration_cast<std::chrono::milliseconds>(t_pathTree_end - t_pathTree_start).count()) / 1000.0;
    saveGraph(pathTree, "path", true, true);
    saveGraph(pathTree, "path_no_pos", true, false);

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

    // print random graph
    // 1 solution path per edge
    // 100 solution paths per world
    if (returnRandomGraph) {
        for (int worldState = 0; worldState < world->getNumWorldStates(); worldState++) {
            int c = 0;
            for (EdgeTrait re : randomGraphEdges) {
                if (std::find(randomGraph[re].worldValidities.begin(), randomGraph[re].worldValidities.end(), worldState) != randomGraph[re].worldValidities.end()) {
                    auto path(std::make_shared<PathGeometric>(si_));
                    path->append(randomGraph[re.m_source].state);
                    path->append(randomGraph[re.m_target].state);
                    pdef_->addSolutionPath(path);
                    c++;
                }
            }
            while (c < 100) {
                auto path(std::make_shared<PathGeometric>(si_));
                path->append(randomGraph[0].state);
                pdef_->addSolutionPath(path);
                c++;
            }
        }
    }
    // print belief graph
    else if (returnBeliefGraph) {
        for (int beliefState = 0; beliefState < static_cast<int>(world->getAllBeliefStates().size()); beliefState++) {
            int c = 0;
            for (EdgeTrait re : singleBeliefGraphEdges[beliefState]) {
                auto path(std::make_shared<PathGeometric>(si_));
                path->append(singleBeliefGraph[beliefState][re.m_source].state);
                path->append(singleBeliefGraph[beliefState][re.m_target].state);
                pdef_->addSolutionPath(path);
                c++;
            }
            while (c < 100) {
                auto path(std::make_shared<PathGeometric>(si_));
                path->append(singleBeliefGraph[beliefState][0].state);
                pdef_->addSolutionPath(path);
                c++;
            }
        }
    }
    // return optimal path tree
    else if (returnPathTree) {
        std::cout << "Worlds: ";
        int c = 0;
        for (std::vector<base::ObjectState> w : world->getWorldStates()) {
            std::cout << c << ": ";
            world->printStateFromInt(world->getStateIntFromObjectState(w));
            std::cout << std::endl;
            c++;
        }
        // return solution path
        for (VertexTraitD v : pathTreeFinalStates) {
            auto path(std::make_shared<PathGeometric>(si_));
            std::vector<base::State*> pathR;
            VertexTraitD currNode = v;
            while (currNode != 0) {
                std::cout << "Append to path: ";
                getSpaceInformation()->getStateSpace()->printState(pathTree[currNode].state, std::cout);
                //path->append(pathTree[currNode].state);
                pathR.push_back(pathTree[currNode].state);
                GraphD::in_edge_iterator it, end;
                std::tie(it, end) = boost::in_edges(currNode, pathTree);
                for (; it != end; it++) {
                    // TODO check if only 1 parent
                    currNode = it->m_source;
                }
            }
            //path->append(pathTree[currNode].state);
            pathR.push_back(pathTree[currNode].state);
            for (int i = static_cast<int>(pathR.size()) - 1; i >= 0; i--) {
                path->append(pathR[i]);
            }
            std::cout << "Found solution for belief ";
            world->printBelief(pathTree[v].beliefState);
            std::cout << " with goal state " << v << ":" << std::endl;
            path->print(std::cout);
            pdef_->addSolutionPath(path);
        }
    }
    // return nn paths
    else if (returnNN) {
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

    std::chrono::steady_clock::time_point t_total_end = std::chrono::steady_clock::now();
    timeTotal = (std::chrono::duration_cast<std::chrono::milliseconds>(t_total_end - t_total_start).count()) / 1000.0;

    std::cout << std::endl << std::endl;
    std::cout << "Total time: " << timeTotal << "s" << std::endl;
    std::cout << "Sampling time: " << timeSampling << "s" << std::endl;
    std::cout << "Check motion time: " << timeCheckMotion << "s" << std::endl;
    std::cout << "Belief graph creation time: " << timeBeliefGraph << "s" << std::endl;
    std::cout << "Policy extraction time: " << timePolicyExtraction << "s" << std::endl;
    std::cout << "Optimal path tree creation time: " << timeOptimalPathTree << "s" << std::endl;
    std::cout << std::endl << std::endl;

    // clear memory
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    if (!worldsUnsolved.empty()) {
        return base::PlannerStatus::TIMEOUT;
    }
    // Return a value from the PlannerStatus enumeration.
    // See ompl::base::PlannerStatus for the possible return values; base::PlannerStatus::EXACT_SOLUTION
    return base::PlannerStatus::EXACT_SOLUTION;
}

void ompl::geometric::Partial::constructPathTree(Graph beliefGraph, std::vector<double> costs, VertexTrait v, VertexTrait currVertex, std::set<VertexTrait> visited) {
    while (true) {
        Graph::adjacency_iterator it, end;
        std::tie(it, end) = boost::adjacent_vertices(currVertex, beliefGraph);
        VertexTrait bestVertex = 0;
        bool isConn = !beliefGraph[currVertex].beliefChildren.empty();
        for (; it != end; it++) {
            VertexTrait der = it.dereference();
            if (beliefGraph[boost::edge(it.dereference(), currVertex, beliefGraph).first].isWorldConnection) {
                if (isConn) {
                    if (visited.find(it.dereference()) == visited.end()) {
                        VertexTraitD w = add_vertex(pathTree);
                        pathTree[w].state = beliefGraph[it.dereference()].state;
                        pathTree[w].fontcolor = beliefGraph[it.dereference()].fontcolor;
                        pathTree[w].color = beliefGraph[it.dereference()].color;
                        pathTree[w].beliefState = beliefGraph[it.dereference()].beliefState;
                        pathTree[w].label = beliefGraph[it.dereference()].label;
                        if (pathTree[w].fontcolor == "blue") {
                            pathTreeFinalStates.push_back(w);
                        }
                        std::cout << "State " << pathTree[w].label << std::endl;
                        pathTree[w].pos = beliefGraph[it.dereference()].pos;
                        std::pair<EdgeTraitD, bool> p = add_edge(v, w, pathTree);
                        EdgeTraitD e = p.first;
                        pathTree[e].color = "red";
                        visited.insert(currVertex);
                        constructPathTree(beliefGraph, costs, w, it.dereference(), visited);
                    }
                }
            }
            else if (bestVertex == 0 || costs[it.dereference()] < costs[bestVertex]) {
                bestVertex = it.dereference();
            }
        }

        if (isConn || bestVertex == 0 || costs[bestVertex] == std::numeric_limits<double>::infinity()) {
            break;
        }

        VertexTraitD u = add_vertex(pathTree);
        pathTree[u].state = beliefGraph[bestVertex].state;
        pathTree[u].fontcolor = beliefGraph[bestVertex].fontcolor;
        pathTree[u].color = beliefGraph[bestVertex].color;
        pathTree[u].beliefState = beliefGraph[bestVertex].beliefState;
        pathTree[u].label = beliefGraph[bestVertex].label;
        if (pathTree[u].fontcolor == "blue") {
            pathTreeFinalStates.push_back(u);
        }
        std::cout << "State " << pathTree[u].label << std::endl;
        pathTree[u].pos = beliefGraph[bestVertex].pos;
        std::pair<EdgeTraitD , bool> p = add_edge(v, u, pathTree);
        v = u;
        currVertex = bestVertex;

        if (beliefGraph[bestVertex].fontcolor == "blue") {
            break;
        }
    }
}

// save graph as png
void ompl::geometric::Partial::saveGraph(Graph g, std::string name, bool useLabels) {
    std::ofstream colored_dot_file(name + std::string(".dot"));
    boost::dynamic_properties dp_no_pos;
    dp_no_pos.property("node_id",   get(boost::vertex_index, g));
    dp_no_pos.property("color", get(&EdgeStruct::color, g));
    dp_no_pos.property("color", get(&VertexStruct::color, g));
    dp_no_pos.property("fontcolor", get(&VertexStruct::fontcolor, g));
    if (useLabels) {
        dp_no_pos.property("label", get(&VertexStruct::label, g));
    }
    if (true) {
        dp_no_pos.property("pos", get(&VertexStruct::pos, g));
    }
    boost::write_graphviz_dp(colored_dot_file, g, dp_no_pos);
    std::stringstream command;
    command << "neato -T png " << name << ".dot -o " << name << ".png";
    system(command.str().c_str());
    std::cout << "Graph " << name << " saved." << std::endl;
}

void ompl::geometric::Partial::saveGraph(GraphD g, std::string name, bool useLabels, bool usePos) {
    std::ofstream colored_dot_file(name + std::string(".dot"));
    boost::dynamic_properties dp_no_pos;
    dp_no_pos.property("node_id",   get(boost::vertex_index, g));
    dp_no_pos.property("color", get(&EdgeStruct::color, g));
    dp_no_pos.property("color", get(&VertexStruct::color, g));
    dp_no_pos.property("fontcolor", get(&VertexStruct::fontcolor, g));
    if (useLabels) {
        dp_no_pos.property("label", get(&VertexStruct::label, g));
    }
    if (usePos) {
        dp_no_pos.property("pos", get(&VertexStruct::pos, g));
    }
    boost::write_graphviz_dp(colored_dot_file, g, dp_no_pos);
    std::stringstream command;
    command << "neato -T png " << name << ".dot -o " << name << ".png";
    system(command.str().c_str());
    std::cout << "Graph " << name << " saved." << std::endl;
}

