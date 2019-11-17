//
// Created by stumbo on 2019/11/17.
//

#ifndef TMAPPING_MAPBRANCH_H
#define TMAPPING_MAPBRANCH_H

#include <vector>
#include <memory>

namespace tmap
{

class MapBranch;

using MapBranchPtr = std::shared_ptr<MapBranch>;
using MapBranchWePtr = std::weak_ptr<MapBranch>;
using MapBranchUnPtr = std::unique_ptr<MapBranch>;

class MapBranch : public std::enable_shared_from_this<MapBranch>
{
    std::size_t bornedAt = 0;
    MapBranchWePtr father;
    /// 不包括 child's child
    std::vector<MapBranchWePtr> children;

    double confidence = 1.0;
    bool isDead = false;

    MapBranch() = default;

    /// 如果一定要用make_shared的话(真么做会导致编译不兼容)
//    friend __gnu_cxx::new_allocator<MapBranch>;

public:

    static MapBranchPtr getTheFirstOne();

    MapBranchPtr born(std::size_t atExp, double xConf);

};

}


#endif //TMAPPING_MAPBRANCH_H
