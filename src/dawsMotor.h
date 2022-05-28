/**
 @file dawsMotor.h
 @author Paul Redhead on 7/7/2020.
 @copyright (C) 2020 Paul Redhead
 @version 0.a
 */

//
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
//
//  Version 0.a First released version
//
//
//

#ifndef ____dawsMotor__
#define ____dawsMotor__

/**
 @brief Key Variable states.

 Key variable acess state enumeration.
 */
enum KVstate : byte
{
    KV_AWAITING_INIT, ///< awaiting initial read attempt
    KV_OK,            ///< initial read from KV Store successful
    KV_NOT_FOUND,     ///< not read - key not found
    KV_ERROR,         ///< other read/write error
    KV_CHANGED        ///< data changed since last read or write
};

#define KV_KEY "/kv/conf" ///< Key for KV store access

/**
 @brief Prefefined values for max speed.

 These values are used for the maxium speeds under various situations.
 */
enum Speed_t : byte
{
    S_STOP = 0,   ///< stop speed
    S_SLOW = 30,  ///< ~ 4mph  walking pace - for approach to buffers and other stock (until obstacle avoidance takes over)
    S_MEDIUM = 60 ///< 7 mph
};
/**
 @brief Prefefined values for Acceleration and deceleration

 These values are used for acceleration and deceleration.

 @note these values are held as fixed point.  Values  below 256 are fractional.

 */
enum Acceleration_t : FixPnt2408
{
    DEFAULT_ACC = 100, ///< default value to be added to speed when accelerating to max 100/256
    DEFAULT_DEC = 150  ///< default to be subtracted from speed when braking 150/256
};

/**
 @brief Loco Motor Controller.

 This class controls the electromechanical elements of the locomotive drive mechanism.  This includes the propulsion motor and measurement of the
 back EMF.  BEMF measurements and processing use the BEMFSensor class.  PWM generation uses the PWM class.  It also uses
 rtos Thread, Semaphore, Ticker and Timeout.

 Features:
 - PI (proportional-integral) control using bemf derived speed error,
 - bemf sampling during gap in pwm pulse train,
 - control of rates of acceleration and braking to stop at specific distance.

 The ticker is used to schedule regular gaps in PWM generation which are used to sample the back EMF and other periodic tasks.
 The timout is used to time the sampling within the gap, which runs withing the thread.



 *********************************/

class Motor : mbed::NonCopyable<Motor>

{
public:
    Motor();
    void setup();

    KVstate getKVstate();
    bool syncKV();
    bool setDir(Dir_t);
    Dir_t getDir();

    bool setSpd(uint16_t);
    uint16_t getSpd();

    bool setGainDB(int);
    bool setITimeLog(int);

    int getGainDB();
    int getITimeLog();

    int getITimeConst();
    float getGainRatio();

    float getBemfK();

    static Motor &instance();

    void setObstructionStop(int, float);
    void clearObstructionStop();

    BEMFSensor bemfSensor; ///< bemf sensor object

private:
    PWM _pwmGen;                ///< pwm generator object
    rtos::Thread _sampleThread; ///< motor BEMF sample control thread
    mbed::Ticker _sampleTicker; ///< ticker to insert sample gaps
    mbed::Timeout _sampleTimer; ///< timer to start BEMF measurement in sample gap

    void _motorMain();
    void _sampleTickProc();
    void _sampleTOProc();

    void _startTO();
    void _startFastDecay();

    bool _setGainRatio(int);

    bool _setITimeRatio(int);

    void _pidController(const FixPnt2408, const FixPnt2408);

    // structure for configuration variables saved to KV store
    KVstate _kvState;
    size_t _kvAct; // size of actual data read
    /**
     @brief motor configuration variables

     Motor configuration variables are blocked together.  The are read and written to KV store together.
     */
    struct cv_t
    {
        int gainDB;       // PI controller gain in dB
        int iTimeLog;     // logrithmic  Integral const = 10^(iTimeLog/10)seconds
        FixPnt2408 bemfK; // conversion bemf to speed factor
    } _cv;

    // operational parameters
    Dir_t _dir;                  ///< direction as set by command
    volatile uint16_t _maxSpeed; // maximum speed (as set by command) range 0 to 255
    FixPnt2408 _targetSpeed;     // for input to pid controller - range as maximum speed
    unsigned long _lastStopTime; // time in millis of last stop command (setDir(STOPPED))

    FixPnt2408 _decPerChange; ///< decrement to be applied to target speed per iteration - 0 no decrement set

    // the following are derived from the saved configuration variables
    FixPnt2408 _gainRatio;  // PI controller gain as ratio
    FixPnt2408 _iTimeRatio; // sample period:Integral time constant - 0 for no I calc.

    // PI controller persistant variables
    FixPnt2408 _outputI; // integral component of output correction

    static Motor *_thisMotor;
};

#endif /* defined(____dawsMotor__) */
