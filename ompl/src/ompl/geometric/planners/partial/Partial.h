//
// Created by janis on 05.04.22.
//

#ifndef OMPL_GEOMETRIC_PLANNERS_PARTIAL_PARTIAL_
#define OMPL_GEOMETRIC_PLANNERS_PARTIAL_PARTIAL_

#include <ompl/base/Planner.h>

// often useful headers:
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
//#include "ompl/base/World.h"
#include <boost/config.hpp>
#include "boost/graph/graph_traits.hpp"
#include "boost/graph/adjacency_list.hpp"
#include <boost/graph/graphviz.hpp>
#include <chrono>

namespace ompl
{
    namespace geometric
    {
        class Partial : public base::Planner {
        public:

            /** \brief Constructor */
            Partial(const base::SpaceInformationPtr &si);

            ~Partial() override;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void setup(void) {
                Planner::setup();
                tools::SelfConfig sc(si_, getName());
                if (nn_.size() < 1) {
                    for (int i = 0; i < si_->getWorld()->getNumWorldStates(); i++) {
                        nn_.push_back(std::shared_ptr<NearestNeighbors<Motion *>>(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this)));
                    }
                }
                for (int i = 0; i < si_->getWorld()->getNumWorldStates(); i++) {
                    if (!nn_.at(i))
                        nn_.at(i).reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
                    nn_.at(i)->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
                }
            }

            virtual void clear(void)
            {
                Planner::clear();
                sampler_.reset();
                freeMemory();
                for (int i = 0; i < si_->getWorld()->getNumWorldStates(); i++) {
                    if (nn_.at(i))
                        nn_.at(i)->clear();
                }
                for (int i = 0; i < static_cast<int>(lastGoalMotion_.size()); i++) {
                    lastGoalMotion_[i] = nullptr;
                }
                pathTree.clear();
                pathTreeFinalStates.clear();
                completeGraphMap.clear();
                debugGraph.clear();
                costs.clear();
                if (pdef_->getSeed() != -1) {
                    int seed = pdef_->getSeed();
                    rng_.setLocalSeed(0);
                    sampler_->setSeed(seed);
                }
                else {
                    rng_.setLocalSeed(0);
                }
            }

            void setPathOptimization(bool b)
            {
                specs_.optimizingPaths = b;
            }

            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};

                /** \brief Stores to which node of the random graph this motion corresponds */
                int nodeIdx;

                bool isGoal;
            };

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distanceBase(a->state, b->state, 2);
            }

            struct VertexStruct
            {
                base::State *state{nullptr};
                std::string color;
                std::string fontcolor;
                std::string pos;
                std::string label;
                std::vector<int> observableObjects;
                std::vector<float> beliefState;
                std::vector<int> beliefChildren;
                int finalSateIdx;
            };

            struct EdgeStruct
            {
                bool isWorldConnection;
                std::string color;
                std::vector<int> worldValidities;
                std::string label;
            };

            typedef boost::adjacency_list< boost::listS, boost::vecS, boost::undirectedS, VertexStruct, EdgeStruct > Graph;
            typedef boost::graph_traits<Graph>::vertex_descriptor VertexTrait;
            typedef boost::graph_traits<Graph>::edge_descriptor EdgeTrait;

            typedef boost::adjacency_list< boost::listS, boost::vecS, boost::bidirectionalS, VertexStruct, EdgeStruct > GraphD;
            typedef boost::graph_traits<GraphD>::vertex_descriptor VertexTraitD;
            typedef boost::graph_traits<GraphD>::edge_descriptor EdgeTraitD;

            std::vector<Graph> createRandomGraph(const ompl::base::PlannerTerminationCondition &ptc);

            Graph createBeliefGraph(Graph randomGraph);

            std::vector<double> calculateCosts(Graph beliefGraph);

            void constructPathTree(Graph beliefGraph, std::vector<double> costs, VertexTrait v, VertexTrait currVertex, std::set<VertexTrait> visited, VertexTrait d_v);

            void pathExtraction();

            void debugSingleVertex(Graph beliefGraph, VertexTrait v);

            void getPlannerData(base::PlannerData &data) const override;

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            void saveGraph(Graph g, std::string name, bool useLabels, bool usePos);
            void saveGraph(GraphD g, std::string name, bool useLabels, bool usePos);
            void saveRandomGraphComplete(Graph g, std::string name);
            Graph readRandomGraph(std::string name);

            void setSeed(int seed) {
                rng_.setLocalSeed(seed);
            }

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::vector<std::shared_ptr<NearestNeighbors<Motion *>>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{.05};

            /** \brief The random number generator */
            RNG rng_;

            // define variables for benchmarking time needed by specific parts of the planner and distances of the planned paths
            std::vector<double> distancesDirect;
            std::vector<double> distancesPlanned;
            double timeTotal = 0;
            double timeSampling = 0;
            double timeCheckMotion = 0;
            double timeBeliefGraph = 0;
            double timePolicyExtraction = 0;
            double timeOptimalPathTree = 0;

            // print graphs etc.
            bool extendedOutput = false;
            // stop sampling if there is a solution for every world
            bool terminateIfSolutionFound = false;

            // define colors for visualizing different beliefs in graphs
            std::vector<std::string> colors = {"aquamarine", "blue", "coral", "cyan", "darkred", "gold", "lime", "webpurple"};

            std::vector<Motion *> lastGoalMotion_;
            int numWorldStates;
            ompl::base::World *world;
            std::vector<base::State *> goalStates;

            GraphD pathTree;
            std::vector<VertexTraitD> pathTreeFinalStates;
            std::map<int, int> completeGraphMap;
            std::vector<double> costs;
            GraphD debugGraph;
        };
    }
}


#endif
