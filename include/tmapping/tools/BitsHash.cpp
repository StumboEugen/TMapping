//
// Created by stumbo on 2019/12/2.
//

#include <iostream>
#include <algorithm>

#include "BitsHash.h"
#include "TopoTools.h"

using namespace std;

tmap::BitsHash::BitsHash(const std::initializer_list<std::size_t>& list)
{
    reserve(list.size());
    for (const auto & nInit: list) {
        addBit(nInit);
    }
}

void tmap::BitsHash::addBit(std::size_t newBit)
{
    if (!relatedBit.empty() && relatedBit.back() >= newBit) {

        if (relatedBit.back() == newBit) {
            cerr << FILE_AND_LINE << " You add a repetitive number!" << endl;
            return;
        }

        cerr << FILE_AND_LINE << " You didn't add the bit in order!" << endl;
        for (size_t bit : relatedBit) {
            if (bit == newBit) {
                cerr << FILE_AND_LINE << " You add a repetitive number!" << endl;
                return;
            }
        }

        std::sort(relatedBit.begin(), relatedBit.end());
        /// rehashing
        hashNum = 0;
        for (size_t bit : relatedBit) {
            hashing(bit);
        }
    }

    relatedBit.push_back(newBit);
    hashing(newBit);
}

void tmap::BitsHash::hashing(std::size_t newBit)
{
    size_t one = 1;
    size_t newHash = one << (newBit % 64);
//    hashNum += newHash;
    hashNum ^= newHash + 0x9e3779b9 + (hashNum << 6) + (hashNum >> 2);
}
