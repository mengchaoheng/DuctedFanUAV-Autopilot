#pragma once

#include "FlightTask.hpp"

class FlightTaskMyTask : public FlightTask
{
public:
  FlightTaskMyTask() = default;
  virtual ~FlightTaskMyTask() = default;

  bool update();
  bool activate(vehicle_local_position_setpoint_s last_setpoint);

private:
//   float _origin_z{0.f};
};
