#include <I2CStepper.h>

I2CStepper *I2CStepper::ptr{nullptr};

I2CStepper* I2CStepper::getInstance(uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin, PinName phA, PinName phB)
{
  if(ptr)
    return ptr;
  ptr = new I2CStepper(step_pin, dir_pin, enable_pin, phA, phB);
  return ptr;
}

I2CStepper* I2CStepper::getInstance(uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin)
{
  if(ptr)
    return ptr;
  ptr = new I2CStepper(step_pin, dir_pin, enable_pin);
  return ptr;
}

I2CStepper::I2CStepper(uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin, PinName phA, PinName phB) : 
  AccelStepper{1, step_pin, dir_pin, 3, 4, false}, enablePin{enable_pin}, phA{phA}, phB{phB}, encoderAttached{true}
{
}

I2CStepper::I2CStepper(uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin) :
  AccelStepper{1, step_pin, dir_pin, 3, 4, false}, enablePin{enable_pin}, encoderAttached{false}
{
}

void I2CStepper::setPinsInverted(bool directionInvert, bool stepInvert, bool enableInvert)
{
  AccelStepper::setPinsInverted(directionInvert, stepInvert, enableInvert);
}

void I2CStepper::enableOutputs()
{
  AccelStepper::enableOutputs();
}

void I2CStepper::disableOutputs()
{
  AccelStepper::disableOutputs();
}

void I2CStepper::setAcceleration(float acceleration)
{
  AccelStepper::setAcceleration(acceleration);
}

void I2CStepper::setAcceleration(float acceleration, float sqrt_acceleration)
{
  AccelStepper::setAcceleration(acceleration, sqrt_acceleration);
}

void I2CStepper::setMaxSpeed(float speed)
{
  AccelStepper::setMaxSpeed(speed);
}

// modify to handle varying amount of incoming bytes
void receiveEvent(int howMany)
{
  static bool ledState{false};
  // digitalWriteFast(PC_13, ledState = !ledState); // debugging
  _I2CStepper::Converter converter;
  uint8_t command_number = Wire.read();
  // check if any additional bytes need to be read
  if(command_number > 7)
  {
    Wire.readBytes(converter.buffer, 4);
  }
  float temp;
  switch (command_number)
  {
  case 1: // soft stop
    I2CStepper::ptr->stop();
    I2CStepper::ptr->target = I2CStepper::ptr->targetPosition();
    break;
  case 2: // stop running - hard stop
    I2CStepper::ptr->_run_ = false;
    I2CStepper::ptr->_run = false;
    I2CStepper::ptr->_justChanged = true;
    I2CStepper::ptr->_justChanged_ = true;
    break;
  case 3: // loose stop - motor is free to move
    I2CStepper::ptr->disableOutputs();
    break;
  case 4: // start motor - holds the position
    I2CStepper::ptr->enableOutputs();
    break;
  case 5: // start running
    I2CStepper::ptr->_run = true;
    I2CStepper::ptr->_run_ = true;
    I2CStepper::ptr->_justChanged = true;
    I2CStepper::ptr->_justChanged_ = true;
    break;
  case 6:
    I2CStepper::ptr->toSend.fvalue = I2CStepper::ptr->stepperResolution;
    break;
  case 7:
    if(I2CStepper::ptr->encoderAttached)
      I2CStepper::ptr->toSend.fvalue = I2CStepper::ptr->encoderResolution * I2CStepper::ptr->encoderCounter;
    else
      I2CStepper::ptr->toSend.fvalue = I2CStepper::ptr->stepperResolution * I2CStepper::ptr->currentPosition();
    break;
  case 8: // sets absolute target
    I2CStepper::ptr->moveTo(static_cast<long>(converter.fvalue / I2CStepper::ptr->stepperResolution));
    I2CStepper::ptr->target = I2CStepper::ptr->targetPosition();
    break;
  case 9: // sets relative target
    I2CStepper::ptr->move(static_cast<long>(converter.fvalue / I2CStepper::ptr->stepperResolution));
    I2CStepper::ptr->target = I2CStepper::ptr->targetPosition();
    break;
  case 10: // sets acceleration
    temp = converter.fvalue / I2CStepper::ptr->stepperResolutionSqrt;
    temp = temp < I2CStepper::ptr->maxAccelerationSqrt ? temp : I2CStepper::ptr->maxAccelerationSqrt;
    I2CStepper::ptr->setAcceleration(temp * temp, temp);
    break;
  case 11: // updates current position
    I2CStepper::ptr->setCurrentPosition(static_cast<long>(converter.fvalue / I2CStepper::ptr->stepperResolution));
    I2CStepper::ptr->position = I2CStepper::ptr->currentPosition() * I2CStepper::ptr->stepperResolution;
    break;
  case 12: // set max speed
    I2CStepper::ptr->setMaxSpeed(converter.fvalue / I2CStepper::ptr->stepperResolution);
    break;
  default:
    break;
  }
}

void requestEvent()
{
  for(int i{}; i<4; i++)
  {
    Wire.write(I2CStepper::ptr->toSend.buffer[i]);
  }
}

void phAInterruptHandler()
{
  static PinName phB{I2CStepper::ptr->phB};
  digitalReadFast(phB) ? I2CStepper::ptr->encoderCounter++ : I2CStepper::ptr->encoderCounter--;
}

void I2CStepper::init(uint8_t address, float stepperResolution, float encoderResolution, float maxAcceleration, float maxVelocity)
{
  this->maxAcceleration = maxAcceleration / stepperResolution;
  this->maxVelocity = maxVelocity / stepperResolution;
  setAcceleration(this->maxAcceleration);
  setMaxSpeed(this->maxVelocity);
  this->address = address;
  this->stepperResolution = stepperResolution;
  this->encoderResolution = encoderResolution;
  attachInterrupt(digitalPinToInterrupt(pinNametoDigitalPin(phA)), phAInterruptHandler, RISING);
  pinMode(PC13, OUTPUT);
  Wire.begin(address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  setEnablePin(enablePin);
  // Debugging
  bool state{};
  for (uint8_t i{}; i < 10; i++)
  {
    digitalWriteFast(PC_13, state = !state);
    delay(250);
  }
}

void I2CStepper::init(uint8_t address, float stepperResolution, float maxAcceleration, float maxVelocity)
{
  this->maxAcceleration = maxAcceleration / stepperResolution;
  this->maxVelocity = maxVelocity / stepperResolution;
  setAcceleration(this->maxAcceleration);
  setMaxSpeed(this->maxVelocity);
  this->address = address;
  this->stepperResolution = stepperResolution;
  pinMode(PC13, OUTPUT);
  Wire.begin(address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  setEnablePin(enablePin);
  // Debugging
  bool state{};
  for(uint8_t i{}; i < 10; i++)
  {
    digitalWriteFast(PC_13, state = !state);
    delay(250);
  }
}

void I2CStepper::setErrorThresh(float thresh)
{
  errorThresh = thresh;
}

void I2CStepper::run()
{
  if (_justChanged || _justChanged_)
  {
    moveTo(target);
    computeNewSpeed();
    _run = _run_;
    _justChanged = false;
    _justChanged_ = false;
  }
  if (_run_ && _run)
  {
    _run = AccelStepper::run();
    // compensate for error if encoder is attached
    if(encoderAttached && !_run)
    {
      error = encoderCounter * encoderResolution - stepperResolution * target;
      if(error < -errorThresh || error > errorThresh)
      {
        target += error / stepperResolution;
        _justChanged = true;
      }
    }
  }
  else if (!_run_)
  {
    _stepInterval = _n = _speed = 0;
  }
}