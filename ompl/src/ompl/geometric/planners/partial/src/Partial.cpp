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

    struct VertexStruct
    {
        base::State* state;
        std::string color;
    };

    struct EdgeStruct
    {
        bool isWorldConnection;
        std::string color;
    };

    std::vector<std::string> colors = {"aquamarine", "blue", "coral", "cyan", "darkred", "gold", "lime", "webpurple"};

    // create belief graph
    typedef boost::adjacency_list< boost::listS, boost::vecS, boost::undirectedS, VertexStruct, EdgeStruct > BeliefGraph;
    typedef boost::graph_traits<BeliefGraph>::vertex_descriptor VertexTrait;
    typedef boost::graph_traits<BeliefGraph>::edge_descriptor EdgeTrait;
    BeliefGraph beliefGraph;
    std::vector<VertexTrait> beliefGraphVertices[numWorldStates];

    // get input states with PlannerInputStates helper, pis_
    while (const base::State *st = pis_.nextStart()) {
    // st will contain a start state.  Typically this state will
    // be cloned here and inserted into the Planner's data structure.
        // si_->copyState() // create suitable data structure
        for (int i = 0; i < numWorldStates; i++) {
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, st);
            motion->beliefs.insert(i);
            motion->idx = 0;
            nn_.at(i)->add(motion);

            VertexTrait v = add_vertex(beliefGraph);
            beliefGraph[v].state = motion->state;
            beliefGraph[v].color = colors[i % static_cast<int>(colors.size())];
            beliefGraphVertices[i].push_back(v);
        }
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    // if needed, sample states from the goal region (and wait until a state is sampled)
    // const base::State *st = pis_.nextGoal(ptc);

    Motion *solution[numWorldStates];
//    std::vector<base::State*> *vertices[numWorldStates];
//    typedef std::pair<base::State*, base::State*> Edge;
//    std::vector<Edge> *edges[numWorldStates];

    for (int i = 0; i < numWorldStates; i++) {
        solution[i] = nullptr;
    }
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;

    int stateIdx[numWorldStates];
    for (int i = 0; i < numWorldStates; i++) {
        stateIdx[i] = 1;
    }

    // <<world_idx, world_idx>, state_idx>
    std::vector<std::pair<std::pair<int, int>, int>> edgesBetweenWorlds;

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
//            if (solution[worldIdx] != nullptr) {
//                continue;
//            }

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

                // the current world is always a possible belief for this state
                motion->beliefs.insert(worldIdx);
                motion->beliefs.insert(motion->parent->beliefs.begin(), motion->parent->beliefs.end());

                motion->idx = stateIdx[worldIdx];

                std::vector<int> currWorldState = world->getStateInt();
                // check which objects are visible from sampled state
                for (int objectIdx : si_->targetFound(dstate)) {
                    std::cout << "Object " << objectIdx << " is visible from current state." << std::endl;
                    int prevValue = currWorldState.at(objectIdx);
                    std::vector<int> newWorldState = currWorldState;
                    newWorldState.at(objectIdx) = abs(prevValue - 1);
                    int newWorldIdx = world->getStateIdx(newWorldState);
                    motion->beliefs.insert(newWorldIdx);
                    std::cout << "New connection between worlds " << worldIdx << " and " << newWorldIdx << std::endl;
                }

                nn_.at(worldIdx)->add(motion);

//                sampledStates[worldIdx]->push_back(motion->state);
//                edges[worldIdx]->push_back(Edge(motion->state, motion->parent->state));
                //BeliefGraph beliefGraph(edges[0]->begin(), edges[0]->end(), static_cast<int>(sampledStates[0]->size()));
                //BeliefGraph beliefGraph(static_cast<int>(sampledStates[0]->size()));
//                Vertex_t v = boost::add_vertex(motion->state);
                //boost::add_edge(edges[0]->at(0).first, edges[0]->at(0).second, beliefGraph);
                VertexTrait v = add_vertex(beliefGraph);
                beliefGraph[v].state = motion->state;
                beliefGraph[v].color = colors[worldIdx % static_cast<int>(colors.size())];
                beliefGraphVertices[worldIdx].push_back(v);

                std::pair<EdgeTrait , bool> p = add_edge(beliefGraphVertices[worldIdx].at(motion->idx), beliefGraphVertices[worldIdx].at(motion->parent->idx), beliefGraph);
                EdgeTrait e = p.first;
                beliefGraph[e].isWorldConnection = false;

                std::set<int> oldBeliefs = motion->parent->beliefs;
                std::set<int> newBeliefs = motion->beliefs;
                if (newBeliefs != oldBeliefs) {
                    for (int belief : newBeliefs) {
                        if (oldBeliefs.find(belief) == oldBeliefs.end()) {
                            edgesBetweenWorlds.push_back(std::pair<std::pair<int, int>, int> {{worldIdx, belief}, motion->idx});
                        }
                    }
                }

                const auto* state3D =
                        motion->state->as<ompl::base::RealVectorStateSpace::StateType>();
                const auto* parent3D =
                        motion->parent->state->as<ompl::base::RealVectorStateSpace::StateType>();

                nmotion = motion;

                std::cout << "New motion added to world " << worldIdx << " from state [" << parent3D->values[0] << ", "
                    << parent3D->values[1] << ", " << parent3D->values[2] << "] to state [" << state3D->values[0] << ", "
                    << state3D->values[1] << ", " << state3D->values[2] << "]" << std::endl;

                stateIdx[worldIdx]++;
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

    // add edges between worlds to belief graph
    for (std::pair<std::pair<int, int>, int> ebw : edgesBetweenWorlds) {
        std::pair<EdgeTrait , bool> p = add_edge(beliefGraphVertices[ebw.first.first].at(ebw.second), beliefGraphVertices[ebw.first.second].at(ebw.second), beliefGraph);
        EdgeTrait e = p.first;
        beliefGraph[e].isWorldConnection = true;
        beliefGraph[e].color = "red";
    }

    // save colroed graph png
    std::ofstream colored_dot_file("colored_grid.dot");
    boost::dynamic_properties dp;
    dp.property("node_id",   get(boost::vertex_index, beliefGraph));
    dp.property("color", get(&EdgeStruct::color, beliefGraph));
    dp.property("color", get(&VertexStruct::color, beliefGraph));
    boost::write_graphviz_dp(colored_dot_file, beliefGraph, dp);
    system("neato -T png colored_grid.dot -o colored_grid.png");

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
