#ifndef MOTORCONFIG_H
#define MOTORCONFIG_H

#include "driver/ledc.h"
#include "IO.h"

void setup();
extern bool setMotorSpeed(int motor, int duty);
extern double getMotorSpeed(int motor);

#endif// MOTORCONFIG_H