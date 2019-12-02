//
// Created by stumbo on 2019/12/2.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_TOOLS_BITSHASH_H
#define TMAPPING_INCLUDE_TMAPPING_TOOLS_BITSHASH_H

#include <vector>

namespace tmap
{

class BitsHash
{
    /// 记录对应的相关bit位, 应当按顺序
    std::vector<std::size_t> relatedBit;
    /// 实时计算的哈希值, 用于提高效率
    std::size_t hashNum = 0;

    void hashing(std::size_t);

public:
    BitsHash(const std::initializer_list<std::size_t>& list);

    BitsHash() = default;

    BitsHash& operator=(const BitsHash& BitsHash) = default;
    BitsHash& operator=(BitsHash&& BitsHash) = default;
    BitsHash(const BitsHash& BitsHash) = default;
    BitsHash(BitsHash&& BitsHash) = default;

    bool operator== (const BitsHash& another) const {
        return this->relatedBit == another.relatedBit;
    }

    std::size_t getHash() const noexcept {
        return hashNum;
    }

    void reserve(std::size_t num) {
        relatedBit.reserve(num);
    }

    void addBit(std::size_t newBit);
};
}

namespace std {
template<> struct hash<tmap::BitsHash> {

    size_t operator()(const tmap::BitsHash& bits) const noexcept {
        return bits.getHash();
    }
};
}


#endif //TMAPPING_INCLUDE_TMAPPING_TOOLS_BITSHASH_H
