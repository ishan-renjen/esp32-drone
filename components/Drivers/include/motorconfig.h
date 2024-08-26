#ifndef MOTORCONFIG_H
#define MOTORCONFIG_H

#include "LEDC.h"
#include "IO.h"

void setup();
bool setMotorSpeed(int motor, int duty);
bool getMotorSpeed(int motor);

#endif// MOTORCONFIG_H