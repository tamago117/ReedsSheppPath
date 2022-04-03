#include<iostream>
#include <math.h>
#include <vector>
#include <array>
#include <random>
#include "include/matplotlibcpp.h"
#include "include/ReedsSheppPath.hpp"

namespace plt = matplotlibcpp;

//config
int n = 50;
double x_range = 10;
double yaw_range = 3.14;

double get_random(int min, int max)
{
    std::random_device rnd;     // 非決定的な乱数生成器
    std::mt19937 mt(rnd());
    
    std::uniform_int_distribution<> randx(min, max);

    return randx(mt);
}

int main()
{
    ReedsSheppPath reed;
    
    double x1 = get_random(0, x_range);
    double x2 = get_random(0, x_range);
    double y1 = get_random(0, x_range);
    double y2 = get_random(0, x_range);
    double yaw1 = get_random(-yaw_range, yaw_range);
    double yaw2 = get_random(-yaw_range, yaw_range);
    Path path = reed.planning(x1, y1, yaw1, x2, y2, yaw2, 1);

    for(int i=0; i<path.x.size(); ++i){
        std::cout<<path.x[i]<<" "<<path.y[i]<<std::endl;
    }

    // Clear previous plot
    plt::clf();
    plt::plot(path.x, path.y); //goal

    plt::xlim(-1, 15);
    plt::ylim(-1, 15);


    // Add graph title
    plt::title("reeds shepp path");
    // Display plot continuously
    //plt::pause(0.001);
    // show plots
    plt::show();

    return 0;
}