/**
@file dawsPWM.h
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

#ifndef ____dawsPWM__
#define ____dawsPWM__







/**
 @ingroup MotorAPI
 @{
 */

#define MAX_GAIN_DB 12   ///< 12 dB ~4
#define MIN_GAIN_DB -2 ///< min value off - 0 gain
#define MAX_ITIME_LOG   4      ///< max value off - 3 => 2sec : 0 => 1 sec.
#define MIN_ITIME_LOG -8       ///< min value ~160 mS

/**
 @}
 */


/**
 @brief Motor PWM driver States
 
 These define the state of the PWM driver finite state machine.  They are used to control
 and monitor the insertion of BEMF measurements.
 */
enum PWMStateType {
    M_IDLE,                 ///< idle waiting to be put in motion
    M_PWM,                  ///< pulse train in production
    M_PWM_PAUSING,          ///< waiting for current pulse to complete before sample
    M_GAP_FAST_DECAY,       ///< period of fast decay  for sample following last pulse in train
    M_PWM_STOPPING          ///< waiting for current pulse to complete before stopping
};




/**
 @brief DAWS Motor PWM Driver
 
 This class controls the motor PWM driver.
 
 This is for a brushed DC motor controlled using a TI DRV8871 H bridge
 which is connected to two pwm capable output pins.
 
 Features:
 - pause and resume to allow for the bemf sampling gap in PWM pulse train,
 - slow decay mode PWM
 
 The DRV8871 has a two signal logic level control interface.
 - one pin per signal - each controls one side of the bridge.
 - 0 0  - high impedance on both bridge outputs -> fast decay (or coast)
 - 0 1 or 1 0 drive on forward/reverse
 - 1 1  - low side gates on - low impedance to 0V -> slow decay (or brake)
 
 On the Arduino uno we can directly address its timer 2 to produce PWM frequencies higher than the default
 arduino frequencies of 490 or 980Hz e.g. 7.8 KHz.
 
 The nano ble uses a different processor with a dedicated PWM
 controllers.  This uses one of the PWM controllers to provide PWM on whichever of the output pin requires modulation
 depending on the direction.  The PWM frequency is 31.25 kHz. Slow decay mode is used for the off phase of the PWM cycle.  When the pulse train is paused
 for sampling, the bridge is set for fast decay.
 
 *********************************/
class PWM
{
public:
    PWM(mbed::Callback<void(void)>);
    void setup();

    static PWM& instance();
 
    void setPulseWidth(uint16_t);
    uint16_t getPulseWidth();
    
    
    void pulseEnded();
    
    
    void startPWM(Dir_t);
    void stopPWM();
    
    
    void pausePWM();
    void resumePWM();
    
    PWMStateType getState();
    
private:
    static NRF_PWM_Type* _pwmp;        // pointer to hardware PWM controller
    static PWM* _thisPWM;
    

    volatile uint16_t _pulseWidth;  // accessed via PWM DMA
    volatile PWMStateType _state;  // used to manage pulse gap for sampling
    volatile PWMStateType _errorState;  // error state for diagnostic purposes
    
    volatile int _errorLine;  // error line for diagnostic purposes

 
    mbed::Callback<void(void)> _pulseEndCB;  // call back for when the pulse train ends


    
    
    void _initHW();
    void _sample();
    void _stateError(int);
    
};


#endif /* defined(____dawsPWM__) */
