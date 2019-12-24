#include <Eigen/Dense>

#ifndef MYTYPES_H_
#define MYTYPES_H_

typedef Eigen::Vector3d Vector3;
typedef Eigen::Vector2d Vector2;
typedef Eigen::Vector3d Point3;
typedef Eigen::Vector2d Point2;


class TestDummy {
  public:
  TestDummy(int testint);
  
  int tester = 19;
};

#endif