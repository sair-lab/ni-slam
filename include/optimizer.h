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

typedef std::shared_ptr<Optimizer> OptimizerPtr;

#endif  // OPTIMIZER_H_