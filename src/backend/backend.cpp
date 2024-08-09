#include "backend.h"
#include <iostream>

void Backend::Run()
{
    while (true)
    {
        map_->semaphore_.acquire();
        std::cout << "ID: "<< map_->current_keyframe_->frame_id_ << std::endl;
    }
}

void Backend::SetMap(const Map::Ptr map)
{
    map_ = map;
}