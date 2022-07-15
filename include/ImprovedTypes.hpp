#pragma once

#include <memory>
#include <utility>
#include <unordered_map>

namespace ORB_SLAM3{

class System;
class Atlas;
class Tracking;
typedef std::shared_ptr<System> System_ptr;
typedef std::weak_ptr<System> System_wptr;
typedef std::shared_ptr<Atlas> Atlas_ptr;
typedef std::weak_ptr<Atlas> Atlas_wptr;
typedef std::shared_ptr<Tracking> Tracking_ptr;
typedef std::weak_ptr<Tracking> Tracking_wptr;

typedef std::pair<int, int> IntPair;
template<typename KEY, typename VALUE> using umap = std::unordered_map<KEY,VALUE>;

}