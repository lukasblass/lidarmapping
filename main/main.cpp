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

    for (int i = 0; i<3; i++) {
        for (int j = 0; j<3; j++) {
            std::cout << lidar.matrix(i,j) << std::endl;
        }
    }
    //std::cout << "hello " << std::endl;
}