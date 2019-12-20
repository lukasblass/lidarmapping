#include <stdlib.h>
#include <iostream>
#include <vector>
#include "Lidar/Lidar.h"

int main() {
    Lidar lidar(200);
    std::vector<Measurement> ms = lidar.scanACircle(4);

    for (const Measurement m : ms) {
        std::cout << m.distance << " " << m.angle << std::endl;
    }
    //std::cout << "hello " << std::endl;
}