#include <stdlib.h>
#include <iostream>


class PlottingThread {
  public:
  void operator()() {
    system("python ../Plotting/plotting.py");
  }
};
