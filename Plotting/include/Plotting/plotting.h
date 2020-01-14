#include <stdlib.h>
#include <iostream>

#ifndef PLOTTINGTHREAD_H_
#define PLOTTINGTHREAD_H_

class PlottingThread {
  public:
  void operator()() {
    system("python ../Plotting/plotting.py");
  }
};

#endif