#ifndef OMPL_BASE_WORLD_
#define OMPL_BASE_WORLD_

#include <vector>
#include <set>
#include <functional>

namespace ompl
{
    namespace base
    {
        // TODO don't hardcode states
        enum ObjectState {
            NONEXISTENT,
            EXISTENT
        };

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

            World(int numObjects) {
                numObjects_ = numObjects;

                /// fill worldStates with all possible combinations
                generateCombinations(std::vector<ObjectState>{}, numObjects, 0);

                // fill beliefStates with all possible combinations
                std::vector<float> initialBelief;
                for (int i = 0; i < getNumWorldStates(); i++) {
                    initialBelief.push_back(1. / getNumWorldStates());
                }
                std::vector<int> initialObjects;
                for (int i = 0; i < getNumObjects(); i++) {
                    initialObjects.push_back(i);
                }
                beliefStates_.push_back(initialBelief);
                calcReachableBeliefStates(initialBelief, initialObjects);
                for (int i = 0; i < getNumWorldStates(); i++) {
                    std::vector<float> definiteState(getNumWorldStates(), 0);
                    definiteState.at(i) = 1;
                    beliefStates_.push_back(definiteState);
                }

                // targetFound_ = targetFound;
            }

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

            void setState(std::vector<ObjectState> objectStates) {
                objectStates_ = objectStates;
            }

            void setState(int idx) {
                objectStates_ = worldStates_.at(idx);
            }

            std::vector<ObjectState> getState() {
                return objectStates_;
            }

            std::vector<int> getStateInt() {
                std::vector<int> intStates;
                for (ObjectState objectState : objectStates_) {
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

            bool beliefChanged(std::vector<float> oldBeliefs, int observableObject) {
                if(!calcBelief(oldBeliefs, std::vector<int>{observableObject}, std::vector<ObjectState>{ObjectState(0)}).empty()) {
                    return true;
                }
                return false;
            }

            std::vector<std::vector<float>> observe(std::vector<float> oldBeliefs, int observableObject) {
                std::vector<std::vector<float>> newBeliefs;
//                std::vector<std::vector<ObjectState>> *combinations;
//                generateCombinations(combinations, std::vector<ObjectState>{}, static_cast<int>(observableObjects.size()), 0);
//                std::vector<std::vector<ObjectState>> combs = *combinations;
//                for (std::vector<ObjectState> states : *combinations) {
//                    std::vector<float> beliefs = calcBelief(oldBeliefs, observableObjects, states);
//                    newBeliefs.push_back(beliefs);
//                }
                for (int i = 0; i < 2; i++) {
                    std::vector<float> beliefs = calcBelief(oldBeliefs, std::vector<int>{observableObject}, std::vector<ObjectState>{ObjectState(i)});
                    newBeliefs.push_back(beliefs);
                }
                // check if beliefs changed
                if (!newBeliefs.at(0).empty()) {
                    return newBeliefs;
                }
                return std::vector<std::vector<float>>{oldBeliefs};
            }

            std::pair<std::vector<int>, std::vector<std::vector<float>>> getCompatibleBeliefs(std::vector<int> worldValidities) {
                std::vector<int> validIdx;
                std::vector<std::vector<float>> validBeliefs;
                for (int i = 0; i < static_cast<int>(beliefStates_.size()); i++) {
                    bool isValid = true;
                    for (int worldIdx : worldValidities) {
                        if (beliefStates_.at(i).at(worldIdx) == 0) {
                            isValid = false;
                        }
                    }
                    if (isValid) {
                        validIdx.push_back(i);
                        validBeliefs.push_back(beliefStates_.at(i));
                    }
                }
                return std::make_pair(validIdx, validBeliefs);
            }

        private:
            void generateCombinations(std::vector<ObjectState> combination, int len, int index) {
                if (len == index) {
                    worldStates_.push_back(combination);
                    return;
                }

                // TODO be generic, also if enum changes
                for (int i = 0; i < 2; i++) {
                    std::vector<ObjectState> comb(combination);
                    comb.push_back(ObjectState(i));
                    generateCombinations(comb, len, index+1);
                }
            }

            void generateCombinations(std::vector<std::vector<ObjectState>> *states, std::vector<ObjectState> combination, int len, int index) {
                if (len == index) {
                    states->push_back(combination);
                    return;
                }

                // TODO be generic, also if enum changes
                for (int i = 0; i < 2; i++) {
                    std::vector<ObjectState> comb(combination);
                    comb.push_back(ObjectState(i));
                    generateCombinations(comb, len, index+1);
                }
            }

            // get world indices in which an object has a certain object state
            std::set<int> getNegativeWorldIndices(int objIndx, ObjectState state) {
                std::set<int> indices;
                for (int i = 0; i < getNumWorldStates(); i++) {
                    // TODO improve
                    if (worldStates_.at(i).at(objIndx) != state) {
                        indices.insert(i);
                    }
                }
                return indices;
            }

            void calcReachableBeliefStates(std::vector<float> initialBeliefSate, std::vector<int> undiscoveredObjects) {
                for (int obj : undiscoveredObjects) {
                    std::vector<std::vector<float>> newBeliefs = observe(initialBeliefSate, obj);
                    for (std::vector<float> belief : newBeliefs) {
                        bool isDefinite = false;
                        for (float b : belief) {
                            if (b == 1) {
                                isDefinite = true;
                            }
                        }
                        if (isDefinite) {
                            continue;
                        }
                        beliefStates_.push_back(belief);
                        std::vector<int> remainingObjects = undiscoveredObjects;
                        for (std::vector<int>::iterator it = remainingObjects.begin(); it != remainingObjects.end(); ++it) {
                            if (*it == obj) {
                                remainingObjects.erase(it);
                                break;
                            }
                        }
                        calcReachableBeliefStates(belief, remainingObjects);
                    }
                }
            }

            std::vector<float> calcBelief(std::vector<float> oldBeliefs, std::vector<int> observableObjects, std::vector<ObjectState> states) {
                std::vector<float> newBelief;
                std::set<int> invalidWorldIndices;
                float deletedBeliefs = 0;
                for (int i = 0; i < static_cast<int>(observableObjects.size()); i++) {
                    std::set<int> newSet = getNegativeWorldIndices(observableObjects.at(i), states.at(i));
                    invalidWorldIndices.insert(newSet.begin(), newSet.end());
                }
                for (int idx : invalidWorldIndices) {
                    deletedBeliefs += oldBeliefs.at(idx);
                }
                float multiplier = 1 / deletedBeliefs;
                bool beliefsChanged = false;
                for (int i = 0; i < getNumWorldStates(); i++) {
                    if (invalidWorldIndices.find(i) != invalidWorldIndices.end()) {
                        newBelief.push_back(0);
                        if (oldBeliefs.at(i) != 0) {
                            beliefsChanged = true;
                        }
                    }
                    else {
                        newBelief.push_back(oldBeliefs.at(i) * multiplier);
                    }
                }
                if (beliefsChanged) {
                    return newBelief;
                }
                return std::vector<float>{};
            }

            int numObjects_;
            std::vector<ObjectState> objectStates_;
            std::vector<std::vector<ObjectState>> worldStates_;
            std::vector<std::vector<float>> beliefStates_;
            // std::function<bool(const State *state)> targetFound_;
        };
    }
}

#endif
