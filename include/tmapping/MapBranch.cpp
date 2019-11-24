//
// Created by stumbo on 2019/11/17.
//

#include "MapBranch.h"
#include "Tmapping.h"
#include <iostream>

using namespace std;

tmap::MapBranch::MapBranch(const size_t bornedAt, MapBranch* const father,
                           const size_t nSerial, double confidence) : bornedAt(bornedAt),
                                                                      father(father),
                                                                      nSerial(nSerial),
                                                                      confidence(confidence)
{
    if (father) {
        size_t recordedFirstBornPlace = father->firstChildAtBirthExp.size() + father->bornedAt;
        if (recordedFirstBornPlace < bornedAt) {
            if (father->firstChildAtBirthExp.capacity() < bornedAt - father->bornedAt) {
                father->firstChildAtBirthExp.reserve((bornedAt - father->bornedAt) * 2);
            }
            for (size_t i = recordedFirstBornPlace; i < bornedAt; ++i) {
                father->firstChildAtBirthExp.push_back(father->children.size());
            }
        }
        father->children.push_back(this);
        father->nAliveChildren += 1;
    }
}

void tmap::MapBranch::setExpired()
{
    if (status == MapBranchStatus::EXPIRED) {
        cerr << FILE_AND_LINE << " You expired a mapBranch more than once!" << endl;
    }
    status = MapBranchStatus::EXPIRED;

    if (children.empty()) {
        if (father) {
            size_t childIterBegin =
                    father->firstChildAtBirthExp[this->bornedAt - father->bornedAt - 1];
            size_t childIterEnd;
            if (father->firstChildAtBirthExp.size() > this->bornedAt - father->bornedAt)
            {
                childIterEnd = father->firstChildAtBirthExp[this->bornedAt - father->bornedAt];
                childIterEnd = min(father->children.size(), childIterEnd);
            } else {
                childIterEnd = father->children.size();
            }

            //  TODO
            bool allNullPtr = true;
            for (size_t i = childIterEnd - 1; i >= childIterBegin; --i) {
                auto & checkedChild = father->children[i];
                if (allNullPtr) {
                    if (checkedChild == nullptr) {
                        father->children.pop_back();
                    } else {
                        allNullPtr = false;
                    }
                }
                if (checkedChild == this) {
                    if (!allNullPtr) {
                        checkedChild = nullptr;
                        break;
                    } else {
                        father->children.pop_back();
                    }
                }
            }
        }
    }

}
