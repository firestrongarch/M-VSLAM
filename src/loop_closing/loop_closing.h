#pragma once
#include "map.h"

class LoopClosing {
public:
    LoopClosing(/* args */) = default;
    void Run();

    void SetMap(Map::Ptr map){
        map_ = map;
        map_->loop_closing_thread_ = true;
    }

private:
    Map::Ptr map_;

};