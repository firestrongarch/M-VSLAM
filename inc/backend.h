#pragma once
#include <memory.h>
#include "map.h"

class Backend
{
public:
    using Ptr = std::shared_ptr<Backend>;
    Backend() = default;

    void SetMap(const Map::Ptr map);
    void Run();

    void OptimizeActiveMap();

private:
    Map::Ptr map_;
};