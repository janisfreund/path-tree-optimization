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
        // TODO don't hardcode states
        enum ObjectState {
            NONEXISTENT,
            EXISTENT
        };

        typedef std::vector<float> BeliefState;

        class POObject
        {
        public:
            POObject(std::vector<double> pos, std::vector<double> len) {
                pos_ = pos;
                len_ = len;
                // targetFound_ = targetFound;
            }

            virtual ~POObject() = default;

            std::vector<double> getPos() {
                return pos_;
            }

            std::vector<double> getLen() {
                return len_;
            }

            ObjectState getSate() {
                return objectState_;
            }

            void setState(ObjectState objectState) {
                objectState_ = objectState;
            }

            /// returns whether the object can be seen from the current state of the robot
//            bool observeCurrentState(const State *state) {
//                return targetFound_(state);
//            }

        private:
            /// pos = center; len = extends
            std::vector<double> pos_;
            std::vector<double> len_;
            ObjectState objectState_;
            // std::function<bool(const State *state)> targetFound_;

        };

        class World
        {
        public:
            World() {}

            World(int numObjects, bool changeableFinalStates);

            virtual ~World() = default;

            std::vector<std::vector<ObjectState>> getWorldStates() {
                return worldStates_;
            }

            int getNumWorldStates() {
                return static_cast<int>(worldStates_.size());
            }

            int getNumObjects() {
                return numObjects_;
            }

//            void setState(std::vector<ObjectState> objectStates) {
//                std::cout << "1st setState called." << std::endl;
//                objectStates_ = objectStates;
//                if (static_cast<int>(objectStates_.size()) == 0) {
//                    std::cout << "Invalid state size!" << std::endl;
//                }
//            }

            void setState(int idx) {
                std::cout << "2nd setState called." << std::endl;
                objectStates_ = worldStates_.at(idx);
                if (static_cast<int>(objectStates_.size()) == 0) {
                    std::cout << "Invalid state sizes set!" << std::endl;
                }
                std::cout << "States size: " << static_cast<int>(objectStates_.size()) << std::endl;
            }

            std::vector<ObjectState> getState() {
                if (static_cast<int>(objectStates_.size()) == 0) {
                    std::cout << "Invalid state size!" << std::endl;
                    std::cout << "Real size: "
                              << static_cast<int>(objectStates_.size()) << "; num objects: " << numObjects_
                              << "; world states [0] size: " << static_cast<int>(worldStates_[0].size()) << std::endl;
                }
                return objectStates_;
            }

            std::vector<int> getStateInt();

            std::vector<int> getStateIntFromObjectState(std::vector<ObjectState> st) {
                std::vector<int> intStates;
                for (ObjectState objectState : st) {
                    intStates.push_back(static_cast<int>(objectState));
                }
                return intStates;
            }

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

            std::vector<BeliefState> getAllBeliefStates() {
                return beliefStates_;
            }

            bool beliefChanged(std::vector<float> oldBeliefs, int observableObject) {
                if(!calcBelief(oldBeliefs, std::vector<int>{observableObject}, std::vector<ObjectState>{ObjectState(0)}).empty()) {
                    return true;
                }
                return false;
            }

            std::vector<BeliefState> observe(BeliefState oldBeliefs, int observableObject);

            std::pair<std::vector<int>, std::vector<BeliefState>> getCompatibleBeliefs(std::vector<int> worldValidities);

            int getBeliefIdx(BeliefState belief) {
                int idx = 0;
                for (BeliefState testBelief : beliefStates_) {
                    bool isEqual = true;
                    for (int i = 0; i < getNumWorldStates(); i++) {
                        // std::cout << "#States: " << getNumWorldStates() << "; #Test: " << static_cast<int>(testBelief.size()) << "; #Belief: " << static_cast<int>(belief.size()) << std::endl;
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

            double calcBranchingProbabilitiy(BeliefState parentBs, BeliefState childBs) {
                double p = 0;
                for (int i = 0; i < getNumWorldStates(); i++) {
                    if (childBs.at(i) > 0) {
                        p += parentBs.at(i);
                    }
                }
                return p;
            }

            std::vector<float> beliefToWorld(BeliefState b);

            void printBelief(BeliefState bs) {
                std::cout << "[";
                for (float f : bs) {
                    std::cout << f << ", ";
                }
                std::cout << "]";
            }

            void printStateFromInt(std::vector<int> s) {
                std::cout << "[";
                for (int i : s) {
                    std::cout << i << ", ";
                }
                std::cout << "]";
            }

            std::vector<ObjectState> objectStates_;

        private:
            void generateCombinations(std::vector<ObjectState> combination, int len, int index);

            // get world indices in which an object DOESN'T have a certain object state
            std::set<int> getNegativeWorldIndices(int objIndx, ObjectState state);

            void calcReachableBeliefStates(std::vector<float> initialBeliefSate, std::vector<int> undiscoveredObjects);

            std::vector<float> calcBelief(std::vector<float> oldBeliefs, std::vector<int> observableObjects, std::vector<ObjectState> states);

            int numObjects_;
            std::vector<std::vector<ObjectState>> worldStates_;
            std::vector<BeliefState> beliefStates_;
        };
    }
}

#endif
