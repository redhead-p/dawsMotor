/**
 @file dawsMotor.cpp
 @author Paul Redhead on 7/7/2020.
 @copyright (C) 2020, 2021, 2022 Paul Redhead


 This code module contains the traction motor control code.

 */
//  This file is part of DAWS.
//  DAWS is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  DAWS is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

//  You should have received a copy of the GNU General Public License
//  along with DAWS.  If not, see <http://www.gnu.org/licenses/>.

/*!
 */
#include <limits.h>
#include <Arduino.h>
#include <mbed.h>
#include <kvstore_global_api.h>
#include "daws.h"
#include "dawsSensor.h"
#include "dawsPWM.h"
#include "dawsMotor.h"

#define DEBUG false ///< enable Loco Motor debug output to IDE Monitor
#define PLOT false  ///< enable Loco Motor graph output to IDE Plotter
#define DIAG false  ///< enable Loco Motor diagnostic pin (for oscilloscope)

/**
 @ingroup hwAssignments
 @{

 */

#define BEMF_CHNL 3 ///< ADC channel 3 (on Arduino pin A0)

/**
 @}
 */
// thread flags
#define SAMPLE_FLAG 1 ///< Motor thread flag to initiate sample
#define START_FLAG 2  ///< Motor thread flag to start PWM
#define STOP_FLAG 4 ///< Motor thread flag to stop PWM

/**
@brief microseconds between sample gaps

 Each cycle is 20ms so c. 50 samples per sec.
 Each gap is c. 512 µs so about 1% of cycles are lost. This doesn't seem to affect performance.
 */
#define PWM_LEN 20 ///< length of pulse train in ms

#define FAST_DECAY_TIME 512 ///< time allowed for fast decay settling (µs)

/**
 @brief Minimum stop time
 
 Defines the minimum stop time in ms.  Following a stop command restart will be delayed 
 to ensure various levels have settled to quiescent values
 */
#define MIN_STOP 3000


#if DIAG
mbed::DigitalOut diagPin(p43); ///< pin for diagnostic use with oscilloscope
#endif

Motor *Motor::_thisMotor; // there is only one!
                          // This allows static routines to call a method with access to object variables etc

/**
 @brief Construct a motor contoller

 This constructs the motor controller.  This should be a singleton.
 Operational parameters are set to default values.

 */
Motor::Motor() : bemfSensor(MOTOR_BEMF, BEMF_CHNL),
                 _pwmGen(mbed::callback(this, &Motor::_startFastDecay)),
                 _sampleThread(MOTOR_PRIORITY)
{
    if (_thisMotor == nullptr)
    {
        _thisMotor = this; // so we have a pointer to access non static methods
    }

    _cv.gainDB = MIN_GAIN_DB;     // special value for off
    _cv.iTimeLog = MAX_ITIME_LOG; // max value is integral component off

    // quantized bemf conversion constant
    // uno 5V
    //_cv.bemfK = 183;               // 5/7 => 183/256
    // nano 33 ble (3.3V)
    _cv.bemfK = 160; // 5/8 => 160/256

    _dir = STOPPED;
    _targetSpeed = 0;
    _maxSpeed = 0;
    _outputI = 0;
}

/**
 @brief Setup loco motor

 This gets called once e.g. withing setup
 in the main sketch.

 It initialises the pwm hardware and the BEMF sensor.

 It retrieves configuration paramaters (e.g. PI controller values) from the Key Variable store.

 It starts the sampling thread.
  *********************************/

void Motor::setup()
{
    int res;
#if DEBUG || PLOT
    Serial.begin(115200); // connect serial output
    Serial.println(F("Motor debug/plot"));
    if (&(Motor::instance()) != this)
    {
        // This object is not the one referred by Motor::instance()
        // It's a duplicate instance - not allowed
        Serial.println("Attempt to create a second motor");
    }
#endif
#if DIAG
    diagPin = 0;
#endif
    // PWM hardware specific setup
    _pwmGen.setup(); // set up the PWM peripheral

    ADConv::setupHW(BEMF_CHNL); // Analogue to digital (SAADC) hardware set up for BEMF

    bemfSensor.setup(); // and we need to explictily call setup in the BEMF sensor object too

    _lastStopTime = millis(); // initialise last stop time (may not be 0 ms)

    bemfSensor.resetFilters(); // set bemf filters to initial state

    // get the configuration variables from KV

    // if this fails configuration variables will take default values
    // as set up by the constructor

    res = kv_get(KV_KEY, &_cv, sizeof(_cv), &_kvAct);
    if (res == MBED_SUCCESS)
    {
        _kvState = KV_OK;
    }
    else if ((MBED_GET_ERROR_CODE(res)) == MBED_ERROR_CODE_ITEM_NOT_FOUND)
    {
#if DEBUG
        Serial.println("KV not found");
#endif
        _kvState = KV_NOT_FOUND;
    }
    else
    {
#if DEBUG
        Serial.print("KV err:");
        Serial.println(MBED_GET_ERROR_CODE(res));
#endif
        _kvState = KV_ERROR;
    }

    // set runtime ratios according to values read from KV
    _setGainRatio(_cv.gainDB);    // this will set the gain ratio
    _setITimeRatio(_cv.iTimeLog); // and the Integral Time factor
#if DEBUG
    Serial.print("I time ");
    Serial.print(_cv.iTimeLog);
    Serial.print(" ");
    Serial.println((float)_iTimeRatio / 256.0);
#endif
    // start the motor control thread - it will wait on semaphore for instruction
    _sampleThread.start(mbed::callback(this, &Motor::_motorMain));
}

/**
 @brief Reference to the single motor instance

 This provides a reference to the single instance of the motor class.

 @note This is a static routine.

 @return reference to the traction motor
 */
Motor &Motor::instance()
{
    return ((Motor &)(*_thisMotor));
}

/**
 @brief Save configuration to KV store

 This saves the motor configuration (e.g. PI controller params etc) to
 the Key Variable store.

 @return true if successful
 */
bool Motor::syncKV()
{
    int res;
    res = kv_set(KV_KEY, &_cv, sizeof(_cv), 0);
    if (res == MBED_SUCCESS)
    {
        _kvState = KV_OK;
        return (true);
    }
    else
    {
#if DEBUG
        Serial.print("KV err:");
        Serial.println(MBED_GET_ERROR_CODE(res));
#endif
        _kvState = KV_ERROR;
        return (false);
    }
}

/**
 @defgroup MotorAPI Motor application interface routines

 Use of the methods in this group is required to set the operational characteristic of the
 Proportional Integral Controller and issue commands to the motor.  get... commands may be used to read
 these parameters  as well as internal private variables.

 @{
 */

/**
 @brief Set direction

 This routine is called as required to set the motor direction.

 If dir is changed as result of a call, speed is set to 0.
 If dir is unchanged the call has no effect.

 @note If direction is changed to STOPPED while the locomotive is in motion a crash stop will be executed.
 Setting speed to 0 first and waiting for the loco to become stationary will be more controlled.

 @param dir - direction as an enumeration FORWARD, STOPPED or REVERSE.

 @return true if direction change accepted or null else false

 *********************************/

bool Motor::setDir(Dir_t dir)
{
    bool result;
    if (dir == _dir)
    {
        // already running in this direction or stopped - quietly ignore
        result = true;
    }
    else if (dir == STOPPED) // stop command
    {
        // if speed has not already been set to 0
        // this implements an emergency stop
        _dir = dir;
        
         _sampleThread.flags_set(STOP_FLAG); // set the thread flag to stop PWM

        result = true;
    }
    else if (STOPPED == _dir) //  was stopped
    {
        _dir = dir;
        // speed should already be zeroed
        //_targetSpeed = 0;
        //_maxSpeed = 0;
        _sampleThread.flags_set(START_FLAG); // set the thread flag to start PWM
        result = true;
    }
    else // crash direction change not allowed.
    {
        result = false;
    }
    return (result);
}

/**
 @brief Get speed


 This routine retrives the motor speed.

 @return motor speed in range 0 - 255

*********************************/

uint16_t Motor::getSpd()
{
    return (_maxSpeed);
}

/**
 @brief Set Speed

 This routine is called as required to set the motor speed.  This is in the range 0 - 255.  If PI gain ratio is 0
 the correction will be 0 and then this speed equates to the pulse width.  This speed is taken as the maximum speed.  The speed used
 depends on acceleration, braking etc.


 @returns - bool false if speed cannot be set (stopped) or invalid
 *********************************/

bool Motor::setSpd(uint16_t speed)
{
    if ((_dir == STOPPED) || // stopped command - ignore speed
        (speed > 255))
    {
        return (false); // speed not changed
    }
    else
    {
        // mbed::CriticalSectionLock lock;  // apply lock while speed updated
        //  already running
        //  change speed - pulse width will be updated when bemf processed
        //_maxSpeed = speed;
        core_util_atomic_store_u16(&_maxSpeed, speed); // use atomic operation to update command speed.
    }
    return (true);
}

/**
 @brief Get PI controller gain ratio

 This retreives the PI controller gain as a ratio.


 @return gain ratio as a floating point number
*/

float Motor::getGainRatio()
{ // for use in non time critial situations
    return (((float)_gainRatio) / 256);
}

/**
 @brief Get PI Integral Time constant

 This retreives the PI controller Integral Time constant in milliseconds.

 @return time constant in ms.
*/

int Motor::getITimeConst()
{ // for use in non time critical situations - time const in millisecs
    return ((int)((float)PWM_LEN * (float)_iTimeRatio / (256.0)));
}

/**
 @brief Get locomotive direction

 This retreives the current locomotive direction.

 @return direction
 */

Dir_t Motor::getDir()
{
    return (_dir);
}

/**
 @brief Get the Key Variable state

 @return Returns the key variable state as an enumeration.
 */
KVstate Motor::getKVstate()
{
    return (_kvState);
}

/**
 @brief Set the Integral Time Constant Logarithmic

 This sets the Integral Time Constant for the PI controller on a logarithmic basis.  I.e:
 - 0 => 1s
 - 3 => ~ 2s
 - -3 => ~0.5s

 Setting a maximum value (
 @ref MAX_ITIME_LOG
 ) is interpreted as switching the integral component of the PI controller off.

 The pwm setITimeRatio method is called to convert from the log value to a
 time for use in the PI controller.

 @param t - the logarithmic value of integral time constant

 @return true if t is valid and used, otherwise false
 */
bool Motor::setITimeLog(int t)
{
    if (_setITimeRatio(t)) // validate and convert to ratio
    {
        // validated OK
        _cv.iTimeLog = t;
        return (true); // value set
    }
    else
    {
        return (false); // value invalid - not set
    }
}

/**
 @brief Set PI controller gain(dB)

 This sets the PI controller gain.  The gain is specified in deciBells. If gain set to
 @ref MIN_GAIN_DB
 the controller is switched off.  The pwm setGainRatio method is used to validate the input
 and convert from dB to a ratio for use in the PI controller.


 @param gain - controller gain in dB.

 @return true if gain is valid and used, otherwise false
*/

bool Motor::setGainDB(int gain)
{                            // will switch PI on too
    if (_setGainRatio(gain)) // validate and convert to ratio
    {
        _cv.gainDB = gain; // save in CV store
        return (true);
    }
    else
    {
        return (false); // out of range
    }
}

/**
 @brief Get PI controller gain

 This retreives the PI controller gain in deciBells


 @return gain in dB
*/
int Motor::getGainDB()
{
    return (_cv.gainDB);
}

/**
 @brief Get the BEMF conversion constant

 This retrieves the BEMF conversion constant used to convert the measured BEMF to a speed.
 The speed as converted is comparable with speed setting, which uses a value of 255 for maximum
 speed without consideration as what this means in terms of real speed or scale speed. However
 the nominal maximum scale speed is about 25 mph, so dividing by 10 will give an approximate scale speed.

 @return the speed conversion factor as a floating point number
 */
float Motor::getBemfK()
{
    return (((float)_cv.bemfK) / 256);
}
/**
 @brief Get PI Integral Time Constant (logarithmic)

 This retreives the PI controller Integral Time as a logarithmic  value.

 @return time constant in ms.
*/

int Motor::getITimeLog()
{
    return (_cv.iTimeLog);
}

/**
 @brief set stop for obstruction

 Compute deceleration rate to achieve a stop in distance as provided.  This will be used until this routine called again with updated distance to run.

 @param mm - the distance in mm to the obstruction

 @param mmPerSec - the current speed in mm per second

 @note Setting mm to zero results in zero dec  - this is taken as cancelling the stop
 */
void Motor::setObstructionStop(int mm, float mmPerSec)
{
    if (mm == 0) // we are  there!
    {
        // can't continue 'cos of divide by zero
        core_util_atomic_store_s32((int32_t *)&_decPerChange, 0); // reset decrement to 0
    }
    else if (mmPerSec > 0) // make sure mmPerSec non zero
    {
        FixPnt2408 targetSpeed = (core_util_atomic_load_s32((int32_t *)&_targetSpeed)); // copy of current target speed
        // The time to come to a stop using constant deceleration start speed v over distance d is twice the time
        // it takes to travel distance d at a a constant speed of v
        int time = ((float)mm / mmPerSec) * 2000; // time in ms to come to stop
        int num = time / PWM_LEN;                 // number of iterations that deceleration will be applied over

        FixPnt2408 decPerChange = targetSpeed / num; // decleration per iteration
#if DEBUG
        Serial.print((float)targetSpeed / 256.0);
        Serial.print('\t');
        Serial.print(mmPerSec);
        Serial.print('\t');
        Serial.print(time);
        Serial.print('\t');
        Serial.println((float)decPerChange / 256.0);
#endif
        core_util_atomic_store_s32((int32_t *)&_decPerChange, (int32_t)decPerChange);
    }
}

/**
 @brief clear stop for obstruction

 This cancels the stop process.


 */

void Motor::clearObstructionStop()
{
    core_util_atomic_store_s32((int32_t *)&_decPerChange, 0);
}

/**
 @}
 */

/**
 @brief Convert the Integral Time Constant log time value to a ratio

 This converts the Integral Time Constant for the PI controller from a logarithmic value to a value in
 seconds.  I.e:
 - 0 => 1s
 - 3 => ~ 2s
 - -3 => ~0.5s

 This is then converted to a ratio w.r.t to the iteration interval.
 Setting a maximum value (
 @ref MAX_ITIME_LOG
 ) is interpreted as switching the integral component of the PI controller off.

 The ratio is used by the PID controller when calculating the output correction due to the integral component.

 @param t - the logarithmic value of integral time constant

 @return true if t is valid and used, otherwise false
 */
bool Motor::_setITimeRatio(int t)
{
    float iTimeConst;
    if (t == MAX_ITIME_LOG) // magic value for no Integral component
    {
        _iTimeRatio = 0; // another magic value for no Integral component
        _outputI = 0;    // reset accumulated integral value to zero
        return (true);   // value set
    }
    else if ((t < MAX_ITIME_LOG) && (t >= MIN_ITIME_LOG))
    {
        iTimeConst = (float)pow(10.0, ((float)(t) / 10.0) + 6);                  // µs!
        _iTimeRatio = (FixPnt2408)((iTimeConst / (PWM_LEN * 1000) * 256) + 0.5); // convert to fp binary
        return (true);                                                           // value set
    }
    else
    {
        return (false); // out of range
    }
}

/**
 @brief Convert PI controller gain in dB to ratio

 The gain in deciBells is validated and then converted to a ratio.  If gain set to
 @ref MIN_GAIN_DB
 the controller is switched off.

 The PID controller uses the ratio when calculating the output correction.


 @param gain - controller gain in dB.

 @return true if gain is valid and used, otherwise false
 */

bool Motor::_setGainRatio(int gain)
{ // will switch PI on too
    if ((gain > MIN_GAIN_DB) && (gain <= MAX_GAIN_DB))
    {
        // convert dB to amplitude ratio
        float gainR = (float)pow(10.0, (float)gain / 20.0);
        _gainRatio = (FixPnt2408)((gainR * 256) + 0.5); // convert to fixed point binary

        return (true);
    }
    else if (gain == MIN_GAIN_DB) // magic value to switch PI controller off
    {                             // switch off PI controller
        _gainRatio = 0;
        return (true);
    }
    else
    {
        return (false); // out of range
    }
}


/*********************************
 Sample ticker.
 The ticker is enabled when PWM is generated (direction not STOPPED).
 It schedules the insertion of  sample gaps in the PWM chain.
 The first action is to pause PWM generation.  The return here is immediate and
 the next action in the sample process is triggered when the current PWM pulse terminates.

 The ticker runs at ISR priority.
 */

void Motor::_sampleTickProc()
{
#if DIAG
    diagPin = 1;
#endif

    _pwmGen.pausePWM(); // pause pwm generation for sample gap
}

/**
@brief Start fast decay period

 PWM has been paused for the BEMF sample.  The DRV8871 has already been set for fast decay.
 This starts a timout to schedule the sampling within the gap after a period of fast decay.

 */
void Motor::_startFastDecay()
{
    // wait for voltage across motor terminals to settle at BEMF  (currently 512 µs)
    _sampleTimer.attach(mbed::callback(this, &Motor::_sampleTOProc),
                        std::chrono::microseconds(FAST_DECAY_TIME)); // attach timeout to schedule taking of sample
}

// timeout processor - ISR priority
// it's time to run the sample process
void Motor::_sampleTOProc()
{
    if (_pwmGen.getState() == M_GAP_FAST_DECAY)
    {
        bemfSensor.initSample();
        _sampleThread.flags_set(SAMPLE_FLAG); // set thread sample flag to read sample
    }
}

/**
 @brief motor thread control function

 This runs the sampling control thread

 It loops forever and is kicked into action when an external command sets the start flag or when the timeout controlling the
 period of fast decay ends and sets the sample flag.

 */
void Motor::_motorMain()
{


    while (true)
    {
        // wait for one of the flags to be set
        uint32_t flags = rtos::ThisThread::flags_wait_any(START_FLAG | SAMPLE_FLAG | STOP_FLAG);
        if ((flags & START_FLAG) != 0) // start flag set
        {
            _outputI = 0;              // clear Integral correction
            bemfSensor.resetFilters(); // and filters

#if DEBUG
            Serial.print("Time since last stop ");
            Serial.print(millis() - _lastStopTime);
            Serial.println("ms");
#endif

            if ((millis() - _lastStopTime) < MIN_STOP)
            {

                // need to pause before measuring
                // to give things a chance to settle
                rtos::ThisThread::sleep_for(std::chrono::milliseconds(MIN_STOP));
            }
#if DEBUG
            Serial.println("Motor starting");
#endif
            bemfSensor.measureBEMFzero(); // measure BEMF zero reference
            if (_dir != STOPPED)          // check to make sure that we've not been stopped already!
            {
                _pwmGen.startPWM(_dir);
                // start the ticker - the sample will be initiated when it interrupts
                // the first interrupt will after the defined number of milliseconds
                _sampleTicker.attach(mbed::callback(this, &Motor::_sampleTickProc),
                                     std::chrono::milliseconds(PWM_LEN));
            }
        }
        if ((flags & STOP_FLAG) != 0) // stop flag set
        {
            // stop the pwm ticker and sample timeout
            _sampleTicker.detach();
            _sampleTimer.detach();
            _maxSpeed = 0;
            _targetSpeed = 0;
            _pwmGen.stopPWM();
            _lastStopTime = millis(); // save time of stop - if restart is too soon it will be delayed
#if DEBUG
            Serial.print("Stopped at ");
            Serial.print(_lastStopTime);
            Serial.println("ms");
#endif
        }
        if ((flags & SAMPLE_FLAG) != 0) // sample flag set
        {
#if DIAG
            diagPin = 0;
#endif
            // taking the sample has already been initiated
            // wait for sampling to complete and
            // record the sample - it's saved internally within the sensor object
            // as returned by the SAADC.
            bemfSensor.takeSample(); // read sample from ADC and save internally
#if DIAG
            diagPin = 1;
#endif
            _pwmGen.resumePWM(); // restart PWM - this will be at the same pulse width as before

            // odometer.calcRPMperiodic();  // calculate new value for drive shaft rpm0

            // limit speed changes by 0.5 when accelerating and 1 when braking per call e.g. bemf sample
            {
                // mbed::CriticalSectionLock  lock;  // create lock while we read command speed
                FixPnt2408 maxSpeed = _maxSpeed << 8; // only read it once - should be enough & convert to fixed point
                if (maxSpeed >= _targetSpeed)
                {
                    if (_decPerChange == 0)
                    {
                        _targetSpeed += DEFAULT_ACC;
                    }
                    else
                    {
                        _targetSpeed -= _decPerChange;
                    }
                }
                else
                {
                    _targetSpeed -= (DEFAULT_DEC > _decPerChange) ? DEFAULT_DEC : _decPerChange;
                }
                _targetSpeed = (_targetSpeed < 0) ? 0 : _targetSpeed; // target speed can't be less than 0!
            }

            // Calculate speed wrt to current direction by multiplying the
            // filtered reading (returned by processReading) by
            // bemf conversion constant (bemfK)
            // and pass it to PI contoller.
            // these are two fixed point variables
            // hence result has to be divided by 256 and rounded

            _pidController(_targetSpeed, ROUND((bemfSensor.processReading() * _cv.bemfK), 8) * _dir);

#if DIAG
            diagPin = 0;
#endif
        }
    }
}

/**
 @brief  Process new speed measurement using Proportional Integral Differential controller

 This routine is called when a new speed measurement is available
 e.g. following a bemf reading.  It calculates the pulse width to be used
 until the next reading becomes available.  Pulse width is a function of
 measured speed and required speed as determined using a Proportional Integral Differential controller algorithm.

 A general PID controller includes a derivative component - not implemented in this version.

    pw = spdr + output\n
    output = outputP + outputI\n
    outputP = error * gain\n
    outputI = outputI(-1) + (gain * error / i_time_ratio)\n
    i_time_ratio = ITC/∂t  (n.b this set as a ratio)

 Where
 - pw = pulse width (0 - 255)
 - spdr = required speed (0 - 255)
 - output is the PI controller output
 - outputP is proportional element of controller output
 - outputI is integral element of controller output
 - outputI(-1) is outputI from previous sampling iteration
 - gain is controller gain
 - ∂t is sample time (time between BEMF samples))
 - ITC  is the  Integral time constant

    error = spdr - measured speed

 @param targetSpeed - Fractional long representation of the speed as set by control as the target for the PID controller
 @param measuredSpeed - Fractional long representation measured speed adjusted to same
 relative but arbitary units as requested speed.
 */

void Motor::_pidController(const FixPnt2408 targetSpeed, const FixPnt2408 measuredSpeed)
{

    // calculate pulse width for use until next sample processed using a PI controller

    FixPnt2408 spdErr;  // speed error - binary fixed point
    FixPnt2408 outputP; // proportional component of PI controller output
    int nextPw;         // next pulse width - range 0 - 255

    // unsigned long ts = micros();

    // calculate speed error wrt to current direction
    // measured speed cannot be negative as it's been corrected for direction
    // but sometimes A to D conversion  drift causes it to be so
    spdErr = targetSpeed - ((measuredSpeed > 0) ? measuredSpeed : 0);

    // we are multiplying two fixed point numbers here, so the number of binary
    // digits after the point need to be halved to get back to 8
    outputP = ROUND(spdErr * _gainRatio, 8);

    // and finally integral output (which as a class variable, persists between calls)
    if (_iTimeRatio == 0) // no integral component set
    {
        _outputI = 0;
    }
    else
    {
        if (((targetSpeed / 256) == 0) && (ROUND(_outputI, 8) > 0))
        {
            // there may be some residual integral output in these circumstances
            // but if the motor has stopped the bemf should be c.0 as will the speed error
            // the residual will not decay and will stop the pulse width becoming 0
            _outputI = (ROUND(_outputI, 8) - 1) << 8; // force it to decay
        }
        else
        {
            // update _outputI according to error
            // multiplying 2 fractional doubles the fraction length but then we divide
            // so back to where we started and no adjustment required
            _outputI += ((spdErr * _gainRatio) / _iTimeRatio);
        }
    }

    // apply controller output to speed for next pulse width
    // rounding fractional components to nearest integer
    nextPw = ROUND(targetSpeed + outputP + _outputI, 8);
    nextPw = constrain(nextPw, 0, 255); // just in case
    _pwmGen.setPulseWidth(nextPw);

#if PLOT
    // Serial.print(((float)(spdErr)/ 256));
    Serial.print(ROUND(spdErr, 8));
    // calculate a rolling root square average - indicates how well PID controller
    // is working - the lower the better.
    // this is the same low pass as the filter macro but using floats
    // rollingSqrErr = rollingSqrErr * 63 / 64 + sq(((float)spdErr)/256);
    // Serial.print(sqrt(rollingSqrErr/64));
    Serial.print('\t');
    // Serial.print(((float)(_outputI)/ 256));
    Serial.print(ROUND(_outputI, 8));

    Serial.print('\t');

    Serial.println(_pwmGen.getPulseWidth());
#endif
}
