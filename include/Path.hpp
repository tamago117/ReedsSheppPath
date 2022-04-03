#pragma once

#include <string>
#include <vector>

class Path
{
public:
    Path() = default;

    std::vector<double> lengths; //course segment length
    std::vector<std::string> ctypes; //course segment type char ("S": straight, "L": left, "R": right)
    double L = 0; //total length of the path
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;
    std::vector<bool> directions; //directions (true:forward, bool:backward)

};