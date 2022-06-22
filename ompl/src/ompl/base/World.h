#ifndef OMPL_BASE_WORLD_
#define OMPL_BASE_WORLD_

#include <vector>
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

            // TODO
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

//            std::vector<float> observe(std::vector<float> beliefs, State *state) {
//                // TODO hardcoded for 1-object example
//                bool found = targetFound_(state);
//                if (found) {
//                    beliefs.at(0) = 1.;
//                } else {
//                    beliefs.at(0) = 0.;
//                }
//            }

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

            int numObjects_;
            std::vector<ObjectState> objectStates_;
            std::vector<std::vector<ObjectState>> worldStates_;
            // std::function<bool(const State *state)> targetFound_;
        };
    }
}

#endif
