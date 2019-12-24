#include <stdlib.h>
#include <vector>
#include <types/types.h>

#ifndef TWODROOM_H_
#define TWODROOM_H_

class Room2D {
  public:
  Room2D(std::vector<Point2> corners);
  Room2D();

  ~Room2D() {
    corners.clear();
  }

  const std::vector<Point2>& getCorners() const {
    return corners;
  }

  private:
  std::vector<Point2> corners;
};

#endif