#ifndef OMPL_BASE_WORLD_
#define OMPL_BASE_WORLD_

#include <vector>
#include <set>
#include <functional>
#include <iostream>
#include <algorithm>
#include "math.h"

namespace ompl
{
    namespace base
    {
        // state of partially observable objects
        // TODO don't hardcode states
        enum ObjectState {
            NONEXISTENT,
            EXISTENT
        };

        typedef std::vector<float> BeliefState;

        class World
        {
        public:
            World() {}

            /** \brief Constructor */
            World(int numObjects, bool changeableFinalStates, BeliefState initBelief);

            /** \brief Destructor */
            virtual ~World() = default;

            /** \brief get vector of world states */
            std::vector<std::vector<ObjectState>> getWorldStates() {
                return worldStates_;
            }

            /** \brief get number of distinct world states */
            int getNumWorldStates() {
                return static_cast<int>(worldStates_.size());
            }

            /** \brief get number of partially observable objects */
            int getNumObjects() {
                return numObjects_;
            }

            /** \brief set world state */
            void setState(int idx) {
                objectStates_ = worldStates_.at(idx);
            }

            /** \brief get world state */
            std::vector<ObjectState> getState() {
                return objectStates_;
            }

            /** \brief get world state as ints */
            std::vector<int> getStateInt();

            /** \brief convert object state to a vector of ints */
            std::vector<int> getStateIntFromObjectState(std::vector<ObjectState> st) {
                std::vector<int> intStates;
                for (ObjectState objectState : st) {
                    intStates.push_back(static_cast<int>(objectState));
                }
                return intStates;
            }

            /** \brief get index of a given state of the world */
            // TODO improve
            int getStateIdx(std::vector<int> worldState) {
                int idx = 0;
                for (std::vector<ObjectState> state : worldStates_) {
                    bool found = true;
                    for (int i = 0; i < static_cast<int>(worldState.size()); i++) {
                        if (worldState.at(i) != static_cast<int>(state.at(i))) {
                            found = false;
                        }
                    }
                    if (found == true) {
                        return idx;
                    }
                    idx++;
                }
            }

            /** \brief get beliefStates_ */
            std::vector<BeliefState> getAllBeliefStates() {
                return beliefStates_;
            }

            /** \brief get beliefStateProbabilities[idx] */
            double getBeliefStateProbability(int idx) {
                return beliefStateProbabilities[idx];
            }

            /** \brief return if observing a given object changes the belief */
            bool beliefChanged(std::vector<float> oldBeliefs, int observableObject) {
                if(!calcBelief(oldBeliefs, std::vector<int>{observableObject}, std::vector<ObjectState>{ObjectState(0)}).empty()) {
                    return true;
                }
                return false;
            }

            /** \brief return beliefs after observing a given obejct with an old belief*/
            std::vector<BeliefState> observe(BeliefState oldBeliefs, int observableObject);

            /** \brief get all beliefs that are compatible in given worlds */
            std::pair<std::vector<int>, std::vector<BeliefState>> getCompatibleBeliefs(std::vector<int> worldValidities);

            /** \brief get index of given belief */
            int getBeliefIdx(BeliefState belief) {
                int idx = 0;
                for (BeliefState testBelief : beliefStates_) {
                    bool isEqual = true;
                    for (int i = 0; i < getNumWorldStates(); i++) {
                        if (fabs(testBelief.at(i) - belief.at(i)) > 1e-3) {
                            isEqual = false;
                        }
                    }
                    if (isEqual) {
                        break;
                    }
                    else {
                        idx++;
                    }
                }
                // TODO fails if not found
                return idx;
            }

            /** \brief calculate branching probability from one belief to another */
            double calcBranchingProbabilitiy(BeliefState parentBs, BeliefState childBs) {
                double p = 0;
                for (int i = 0; i < getNumWorldStates(); i++) {
                    if (childBs.at(i) > 0) {
                        p += parentBs.at(i);
                    }
                }
                return p;
            }

            /** \brief returns vector consisting of probabilities for each partially observable object to be present in given belief */
            std::vector<float> beliefToWorld(BeliefState b);

            /** \brief print a belief state */
            void printBelief(BeliefState bs) {
                std::cout << "[";
                for (float f : bs) {
                    std::cout << f << ", ";
                }
                std::cout << "]";
            }

            /** \brief print a vector of ints */
            void printStateFromInt(std::vector<int> s) {
                std::cout << "[";
                for (int i : s) {
                    std::cout << i << ", ";
                }
                std::cout << "]";
            }

            /** \brief current states of the partially observable objects */
            std::vector<ObjectState> objectStates_;

        private:
            /** \brief fills worldStates_ with all possible combinations of objects states */
            void generateCombinations(std::vector<ObjectState> combination, int len, int index);

            /** \brief get world indices in which an object DOESN'T have a certain object state */
            std::set<int> getNegativeWorldIndices(int objIndx, ObjectState state);

            /** \brief fills beliefStates_ vector with all possible belief states given a initial belief */
            void calcReachableBeliefStates(std::vector<float> initialBeliefSate, std::vector<int> undiscoveredObjects);

            /** \brief calculates the resulting beliefs if certain objects are observed to have certain states in a specific belief*/
            std::vector<float> calcBelief(std::vector<float> oldBeliefs, std::vector<int> observableObjects, std::vector<ObjectState> states);

            /** \brief number of partially observable objects */
            int numObjects_;

            /** \brief all the different states the world can have */
            std::vector<std::vector<ObjectState>> worldStates_;

            /** \brief all possible belief states */
            std::vector<BeliefState> beliefStates_;

            /** \brief probability of getting to one specific belief state */
            std::vector<double> beliefStateProbabilities;
        };
    }
}

#endif
