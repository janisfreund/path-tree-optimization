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
                for (int i = 0; i < si_->getWorld()->getNumWorldStates(); i++) {
                    if (nn_.at(i))
                        nn_.at(i)->clear();
                }
            }

        // protected:
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
            };

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
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
        };
    }
}


#endif
