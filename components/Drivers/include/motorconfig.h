#ifndef MOTORCONFIG_H
#define MOTORCONFIG_H

#include "ledc.h"
#include "IO.h"

void setup();
extern bool setMotorSpeed(int motor, int duty);
extern bool getMotorSpeed(int motor);

#endif// MOTORCONFIG_H