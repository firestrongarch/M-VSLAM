#include "backend.h"
#include <iostream>
#include <thread>
void Backend::Run()
{
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "Backend thread running" << std::endl;
    }
}

void Backend::SetMap(const Map::Ptr map)
{
    map_ = map;
}