#include <Arduino.h>
#include <I2CStepper.h>
#define stepPin PA8
#define dirPin PA9
#define enPin PA10
#define address 3
#define stepperResolution 0.005 // in meter or radian
#define encoderResolution 0.005 // in meter or radian
#define maxAcceleration 10.0    // in m/s^2 or rad/s^2
#define maxVelocity 10.0        // in m/s or rad/s
#define phA PA_0
#define phB PA_1

I2CStepper* controller {I2CStepper::getInstance(stepPin, dirPin, enPin)};//, phA, phB)};

void setup()
{
  // controller.setErrorThresh(0.007); // 7 mm, doesn't do anything if encoder is not attached.
  controller->setPinsInverted(false, false, true);
  controller->init(address, stepperResolution, maxAcceleration, maxVelocity);
  controller->setAcceleration(1000);
  controller->setMaxSpeed(1000);
}

void loop()
{
  controller->run();
}