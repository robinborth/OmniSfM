#pragma once

#include <iostream>

struct Settings
{
#ifdef ROOT_DIR
    std::string rootDir = ROOT_DIR;
#else
    std::string rootDir = "EDIT me!";
#endif
    std::string dataset = "freiburg_small";
    float siftThreshold = 0.98f;
};