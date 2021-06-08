#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include "map.h"

class Optimizer{
public:
  Optimizer(std::shared_ptr<Map> map);
  void OptimizeMap();

private:
  std::shared_ptr<Map> _map;
};


#endif  // OPTIMIZER_H_