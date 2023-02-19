//
// Created by janis on 02.11.22.
//

#include "ompl/base/World.h"

ompl::base::World::World(int numObjects, bool changeableFinalStates, BeliefState initBelief, bool excludeFullWorld) {
    numObjects_ = numObjects;

    // fill worldStates with all possible combinations
    if (changeableFinalStates) {
        for (int i = 0; i < numObjects; i++) {
            std::vector<ObjectState> ws;
            for (int n = 0; n < numObjects; n++) {
                if (i == n) {
                    ws.push_back(EXISTENT);
                } else {
                    ws.push_back(NONEXISTENT);
                }
            }
            worldStates_.push_back(ws);
        }
    } else {
        generateCombinations(std::vector<ObjectState>{}, numObjects, 0, excludeFullWorld);
    }

    std::vector<float> initialBelief;
    if (initBelief.empty()) {
        // initialize initial belief uniformly
        for (int i = 0; i < getNumWorldStates(); i++) {
            initialBelief.push_back(1. / getNumWorldStates());
        }
    }
    std::vector<int> initialObjects;
    for (int i = 0; i < getNumObjects(); i++) {
        initialObjects.push_back(i);
    }
    beliefStates_.push_back(initialBelief);
    beliefStateProbabilities.push_back(1.);

    // fill beliefStates with all possible combinations
    calcReachableBeliefStates(initialBelief, initialObjects);
}

std::vector<ompl::base::BeliefState> ompl::base::World::observe(BeliefState oldBeliefs, int observableObject) {
    for (float f : oldBeliefs) {
        if (f == 1) {
            return std::vector<std::vector<float>>{oldBeliefs};
        }
        else if (f != 0) {
            break;
        }
    }

    std::vector<std::vector<float>> newBeliefs;
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

void ompl::base::World::generateCombinations(std::vector<ObjectState> combination, int len, int index, bool excludeFullWorld) {
    if (len == index) {
        bool isFull = true;
        for (ObjectState objectState : combination) {
            if (objectState == NONEXISTENT) {
                isFull = false;
            }
        }
        if (!excludeFullWorld || !isFull) {
            worldStates_.push_back(combination);
        }
        return;
    }

    // TODO be generic, also if enum changes
    for (int i = 0; i < 2; i++) {
        std::vector<ObjectState> comb(combination);
        comb.push_back(ObjectState(i));
        generateCombinations(comb, len, index+1, excludeFullWorld);
    }
}

// get world indices in which an object DOESN'T have a certain object state
std::set<int> ompl::base::World::getNegativeWorldIndices(int objIndx, ObjectState state) {
    std::set<int> indices;
    for (int i = 0; i < getNumWorldStates(); i++) {
        // TODO improve
        if (worldStates_.at(i).at(objIndx) != state) {
            indices.insert(i);
        }
    }
    return indices;
}

void ompl::base::World::calcReachableBeliefStates(std::vector<float> initialBeliefSate, std::vector<int> undiscoveredObjects) {
    // iterate over undiscovered objects
    for (int obj : undiscoveredObjects) {
        // get new beliefs when obj is observed
        std::vector<std::vector<float>> newBeliefs = observe(initialBeliefSate, obj);
        // for every new belief
        for (std::vector<float> belief : newBeliefs) {
            // check if new belief is already present in beliefStates_
            bool exists = false;
            for (std::vector<float> oBelief : beliefStates_) {
                bool isEqual = true;
                int i = 0;
                for (float f : oBelief) {
                    if (fabs(f - belief.at(i)) > 1e-3) {
                        isEqual = false;
                    }
                    i++;
                }
                if (isEqual) {
                    exists = true;
                    break;
                }
            }
            if (exists) {
                continue;
            }
            // add belief if it is not final
            beliefStates_.push_back(belief);
            double old_prob = beliefStateProbabilities[getBeliefIdx(initialBeliefSate)];
            double branching_prob = calcBranchingProbabilitiy(initialBeliefSate, belief);
            beliefStateProbabilities.push_back(old_prob * branching_prob);
            // recursively call function with remaining objects
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

std::vector<float> ompl::base::World::calcBelief(std::vector<float> oldBeliefs, std::vector<int> observableObjects, std::vector<ObjectState> states) {
    std::vector<float> newBelief;
    std::set<int> invalidWorldIndices;
    float deletedBeliefs = 0;
    // get world indices of worlds in which observed object is not in observed state
    for (int i = 0; i < static_cast<int>(observableObjects.size()); i++) {
        std::set<int> newSet = getNegativeWorldIndices(observableObjects.at(i), states.at(i));
        invalidWorldIndices.insert(newSet.begin(), newSet.end());
    }
    // calculate sum of beliefs of invalid worlds
    for (int idx : invalidWorldIndices) {
        deletedBeliefs += oldBeliefs.at(idx);
    }
    // check if old belief is final belief
    if (fabs(1. - deletedBeliefs) < 1e-3) {
        return std::vector<float>{};
    }
    // update beliefs
    float multiplier = 1 / (1. - deletedBeliefs);
    bool beliefsChanged = false;
    for (int i = 0; i < getNumWorldStates(); i++) {
        if (invalidWorldIndices.find(i) != invalidWorldIndices.end()) {
            newBelief.push_back(0);
            if (oldBeliefs.at(i) != 0) {
                beliefsChanged = true;
            }
        }
        else {
            float newValue = oldBeliefs.at(i) * multiplier;
            newBelief.push_back(newValue);
        }
    }
    if (beliefsChanged) {
        return newBelief;
    }
    return std::vector<float>{};
}

std::pair<std::vector<int>, std::vector<ompl::base::BeliefState>> ompl::base::World::getCompatibleBeliefs(std::vector<int> worldValidities) {
    std::vector<int> validIdx;
    std::vector<std::vector<float>> validBeliefs;
    for (int i = 0; i < static_cast<int>(beliefStates_.size()); i++) {
        bool isValid = true;
        for (int n = 0; n < getNumWorldStates(); n++) {
            if (beliefStates_.at(i).at(n) > 0 && (std::find(worldValidities.begin(), worldValidities.end(), n) == worldValidities.end())) {
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

std::vector<float> ompl::base::World::beliefToWorld(BeliefState belief) {
    std::vector<float> pWorld;
    for (int objIdx = 0; objIdx < numObjects_; objIdx++) {
        float pObj = 0.;
        int worldStateIdx = 0;
        for (float b : belief) {
            pObj += getStateIntFromObjectState(worldStates_.at(worldStateIdx)).at(objIdx) * b;
            worldStateIdx++;
        }
        pWorld.push_back(pObj);
    }
    return pWorld;
}

std::vector<int> ompl::base::World::getStateInt() {
    std::vector<int> intStates;
    for (ObjectState objectState : objectStates_) {
        intStates.push_back(static_cast<int>(objectState));
    }
    return intStates;
}