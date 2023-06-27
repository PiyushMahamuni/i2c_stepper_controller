#ifndef _I2CStepper_H_
#define _I2CStepper_H_

#include <AccelStepper.h>
#include <Arduino.h>
#include <Wire.h>

namespace _I2CStepper
{
    union Converter
    {
        uint8_t buffer[4];
        float fvalue;
        long lvalue;
    };
}

void receiveEvent(int howMany);
void requestEvent();
void phAInterruptHandler();

/// @brief
/// singleton class StepperController.
/// use getInstance static member function to create or get already initialized StepperController object
class I2CStepper: private AccelStepper
{
private:
    static I2CStepper *ptr;
    uint8_t address;
    uint8_t enablePin;
    volatile bool _run{true}, _run_{true}, _justChanged{false}, _justChanged_{false};
    long target;
    float position;
    float stepperResolution{0.001};
    float stepperResolutionSqrt{std::sqrt(stepperResolution)};
    float encoderResolution{0.001};
    float error{};
    float errorThresh{0.007}; // 7 mm
    PinName phA;
    PinName phB;
    long encoderCounter{};
    bool encoderAttached {false};
    
    // below attributes are in m/s^2, sqrt(m)/s and in m/s or
    // in rad/s^2, sqrt(rad)/s and in rad/s
    float maxAcceleration{1/stepperResolution};
    float maxAccelerationSqrt{std::sqrt(maxAcceleration)};
    float maxVelocity{1/stepperResolution};
    _I2CStepper::Converter toSend;
    
    /// StepperController CLASS HAS A STATIC POINTER TO A StepperController OBJECT, WHICH
    /// POINTS TO FIRST OBJECT OF THE SAME CLASS CREATED. IT IS NECESSARY FOR THE OBJECT TO
    /// BE POINTED BY THIS POINTER TO BE ACTIVATED.
    I2CStepper(uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin, PinName phA, PinName phB);

    I2CStepper(uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin);
public:

    /// @brief creates an instance if no other instance is created before.
    /// this function also initializes interrupt handlers for encoder.
    /// @return pointer to the first steppercontroller object created. (singleton class)
    static I2CStepper* getInstance(uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin, PinName phA, PinName phB);

    /// @brief creates an instance if no other instance is created before.
    /// this function does not initialzes interrupt handlers for encoder assuming encoder is not attached
    /// @returns pointer to the first steppercontroller object created. (singleton class)
    static I2CStepper* getInstance(uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin);


    /// @brief Sets the inverted pins
    void setPinsInverted(bool directionInvert = false, bool stepInvert = false, bool enableInvert = false);

    /// function that gets called for an i2c recieve event.
    friend void receiveEvent(int howMany);
    friend void requestEvent();
    friend void phAInterruptHandler();

    /// @brief initializes the i2c communication and set some motor parameters to initialize motor.
    /// if you don't have encoder attached call the other overloaded version without @param encoderResolution
    void init(uint8_t address, float stepperResolutino, float encoderResolution, float maxAcceleration, float maxVelocity);

    /// @brief initializes the i2c communication and set some motor parameters to initialize motor.
    /// if you have encoder attached call the other overloaded version with @param encoderResolution
    void init(uint8_t address, float stepperResolution, float maxAcceleration, float maxVeloctiy);

    /// runs the motor if a step is due and if hard stop hasn't been requested.
    void run();

    void setErrorThresh(float thresh);

    virtual void enableOutputs();

    virtual void disableOutputs();

    void setMaxSpeed(float speed);

    void setAcceleration(float acceleration);

    void setAcceleration(float acceleration, float sqrt_acceleration);
};
#endif