/**
@file dawsPWM.cpp
@author Paul Redhead on 7/7/2020.
@copyright (C) 2020, 2021 Paul Redhead
@version 0.a
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
#include "daws.h"
#include "dawsPWM.h"



#define DEBUG false  ///< enable DAWS PWM debug output to IDE Monitor




/** @addtogroup hwAssignments 
 
 @{

 @brief Use first PWM periphal
 
 The motor pwm generation uses PWM0, the first PWM peripheral
 
 @note NRFX_CONCAT_2 from nrfx_common.h is used to create relevant tokens as required at compile time for hw
 pointers, IRQ numbers and vectors etc.
 */
#define MOTOR_PWM PWM0 ///< use first PWM peripheral

/**
 @}
 */


PWM* PWM::_thisPWM;             // pointer to the single instance 
NRF_PWM_Type* PWM::_pwmp;        // pointer to PWM controller



/**
 @brief Construct a motor PWM contoller
 
 This constructs the motor PWM controller.  PWM is generated using the nRF52840 PWM peripheral.  Two of the four
 channels are used for this motor.  Other channels could possibly be used but they would have to run at the same PWM
 frequency and same gap insertion profile.
 
 PWM frequency is set to 31.25kHz - 32µs period.
 */

PWM::PWM(mbed::Callback<void(void)> cb)
{
    if (_pwmp == nullptr)
    {
        _pwmp = NRFX_CONCAT_2(NRF_, MOTOR_PWM);  // pointer to the PWM peripheral in use.
        _thisPWM = this;  // so we have a pointer to access non static methods
    }

    _pulseEndCB = cb;
 
    _state = M_IDLE;
    _pulseWidth = 0;
    _errorLine = 0;
}

/**
 @brief Reference to the single PWM hardware drive
 
 This provides a reference to the single instance of the PWM driver class.
 
 @note this is a static routine.
 
 @return reference to the PWM driver
 */
PWM& PWM::instance()
{
    return((PWM&)*_thisPWM);
}

/**
 @brief Setup loco motor
 
 This gets called once e.g. indirectly from within setup
 in the main sketch.
 
 It initialises the pwm hardware.
 *********************************/


void PWM::setup()
{
#if DEBUG || PLOT
    Serial.begin(115200);    // connect serial output
    Serial.println(F("PWM debug/plot"));
    if (this != _thisPWM)
    {
        // 'this' doesn't point at the single PWM driver!
        Serial.println("Attempt to create second PWM driver")
    }
#endif
    _initHW();      // contains hardware specific setup
}

/*
*********************************
get & set routines
*********************************

These allow access to private class variables


*********************************/





/**
 @addtogroup MotorAPI 
 
 @{
 */


/**
 @brief Get Pulse Width
 
 This method returns the pulse width.
  
 @return - pulse width as a byte
 
 @note It is assumed that interrupts are enabled.
 
 *********************************/


uint16_t PWM::getPulseWidth()
{
    return(_pulseWidth);
}

/**
@}
*/

/**
 @brief Set Pulse Width
 
 This method sets the pulse width.  The pulse width is saved as a class variable.  The PWM peripheral
 is set up to read this via DMA.  A new pulse width will be implemented at the end of the current pulse.
   
 @param pw - the new pulse width
 *********************************/


void PWM::setPulseWidth(uint16_t pw)
{
    _pulseWidth = constrain(pw, 0, 255);  // just in case
}





/* routines that manage hardware*/

/**
 @brief Pause PWM generation
 
 Stop the PWM generation to allow measurement of the sample.  The current pulse will complete
 and the peripheral will interrrupt when completed.
 
 
 @note May be run as part of an ISR.
 @note This is intended for internal usage within the motor controller.
 
 *********************************/


void PWM::pausePWM()
{
    if (_state == M_PWM)
    {
        _pwmp->INTENSET = 0x02;             // enable interrupt for stopped
        _pwmp->TASKS_STOP = 1;         // stop pulse train
        _state = M_PWM_PAUSING;
    }
    else
    {
        _stateError(__LINE__);
    }

}

/**
 @brief Resume PWM generation
 
 Resume the PWM generation following sampling.
 
 
 @note May be run as part of an ISR.
 @note This is intended for internal usage within the motor controller.
 
 *********************************/


void PWM::resumePWM()
{
    if (_state == M_GAP_FAST_DECAY)
    {
#if DIAG
        diagPin = 1;
#endif
        
        
        _state = M_PWM;
        // set pins high (PWM will override whichever is in use of the two
        NRF_P1->OUTSET= 1 << 14;    // port 1.14 aka p46 or D6
        NRF_P1->OUTSET= 1 << 12;    // port 1.12 aka p44 or D3
        _pwmp->TASKS_SEQSTART[0] = 1;  // restart PWM
    }
    else
    {
        _stateError(__LINE__);
    }

}


/**
@brief Start PWM generation
 
 This routine selects  the outputs for for PWM generation and enables the PWM peripheral.
 
@param dir - direction as enumeration

 
 @note This is intended for internal usage within the motor controller.


*/

void PWM::startPWM(Dir_t dir)
{
#if DEBUG
    Serial.print(F("Start pwm:"));
    Serial.println(_state);
#endif
    if (_state == M_IDLE)
    {
        _pulseWidth = 0;
        
        
        // set both output pins high - slow decay
        // these settings will  be used if the pulse train is stopped and no other action
        // taken
        NRF_P1->OUTSET= 1 << 14;    // port 1.14 aka p46 or D6
        NRF_P1->OUTSET= 1 << 12;    // port 1.12 aka p44 or D3
        if (dir == FORWARD)
        {
            _pwmp->PSEL.OUT[0] = ((p46 << PWM_PSEL_OUT_PIN_Pos) |
                                  (PWM_PSEL_OUT_CONNECT_Connected <<
                                   PWM_PSEL_OUT_CONNECT_Pos));
        }
        else if (dir == REVERSE)
        {
            _pwmp->PSEL.OUT[1] = ((p44 << PWM_PSEL_OUT_PIN_Pos) |
                                  (PWM_PSEL_OUT_CONNECT_Connected <<
                                   PWM_PSEL_OUT_CONNECT_Pos));
        }
        _pwmp->ENABLE    = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
        _pwmp->TASKS_SEQSTART[0] = 1;
        
        _state = M_PWM;
    }
    else
    {
        _stateError(__LINE__);
    }

    
}

/**
@brief Stop PWM generation
 
 This stops PWM generation.  It stops the PWM peripheral and waits for the current pulse to complete.  The output pins
 are set low to disable the DRV8871 and cause it to enter sleep mode.  The PWM peripheral is disabled.
 

 
 @note This is intended for internal usage within the motor controller.
 @note This must not be called from ISR or ticker service routine.


*/

void PWM::stopPWM()
{
#if DEBUG
    Serial.print(F("Stop pwm:"));
    Serial.println(_state);
    if (_errorLine != 0)
    {
        Serial.print("StateError line:");
        Serial.print(_errorLine);
        Serial.print(' ');
        Serial.println(_errorState);
    }
    

#endif

    
    if ((_pwmp->ENABLE & (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos)) != 0)
    {
        _pwmp->TASKS_STOP = 1;         // stop pulse train
        // and wait for current pulse to finish
        while (_pwmp->EVENTS_STOPPED == 0)
        {
            rtos::ThisThread::yield();  // blocks lower priority threads
        }
        _pwmp->EVENTS_STOPPED = 0;  // clear the stopped flag
    }
    
    // set out put pins low - put DRV8871 to sleep
    NRF_P1->OUTCLR= 1 << 14;    // port 1.14 aka p46 or D6
    NRF_P1->OUTCLR= 1 << 12;    // port 1.12 aka p44 or D3
    
    _pwmp->ENABLE    = (PWM_ENABLE_ENABLE_Disabled << PWM_ENABLE_ENABLE_Pos);
    _pwmp->PSEL.OUT[0] = NRF_PWM_PIN_NOT_CONNECTED;
    _pwmp->PSEL.OUT[1] = NRF_PWM_PIN_NOT_CONNECTED;
    
    _state = M_IDLE;
    
}

/**
 @brief Expose the current PWM generator state
 
 The PWM state is enumerated (PWMStateType) and held as a private variable. This makes it available for reading.
 
 @return the PWM generator state
 */
PWMStateType PWM::getState()
{
    return(_state);
}

/**
 @brief The current pulse has ended.
 
 This function is called by the ISR after the PWM periphal has generated the STOPPED interrupt following the STOP
 command.  It runs at ISR priority.
 
 The output pins are set low for fast decay.  The pulse end callback is called.
 
 @note This function is not intended for use in other circumstance.
 */

void PWM::pulseEnded()
{
    if (_state == M_PWM_PAUSING)
    {
        // set out puts for fast decay
        NRF_P1->OUTCLR= 1 << 14;    // port 1.14 aka p46 or D6
        NRF_P1->OUTCLR= 1 << 12;    // port 1.12 aka p44 or D3
        _pulseEndCB();              // call back must be set!
        _state = M_GAP_FAST_DECAY;
    }
    else
    {
        _stateError(__LINE__);
    }
}



void PWM::_initHW()
{
    // set count up mode
    _pwmp->MODE      = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
    // set prescaler for 8MHz clock
    _pwmp->PRESCALER = (PWM_PRESCALER_PRESCALER_DIV_2 <<
                           PWM_PRESCALER_PRESCALER_Pos);

    // counter top set to 256 (same as Arduino default)
    // PWM frequency is 31.25 kHz - period 32µs
    _pwmp->COUNTERTOP  = (256 << PWM_COUNTERTOP_COUNTERTOP_Pos);
    // _pwmp->LOOP        = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);
    _pwmp->LOOP        = (1 << PWM_LOOP_CNT_Pos);
    _pwmp->DECODER   = (PWM_DECODER_LOAD_Common << PWM_DECODER_LOAD_Pos) |
                          (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    _pwmp->SEQ[0].PTR  = ((uint32_t)(&_pulseWidth) << PWM_SEQ_PTR_PTR_Pos);
    _pwmp->SEQ[0].CNT  = 1 << PWM_SEQ_CNT_CNT_Pos;
    //((sizeof(_pulseWidth) / sizeof(uint16_t)) <<
      //                       PWM_SEQ_CNT_CNT_Pos);
    _pwmp->SEQ[0].REFRESH  = 0;
    _pwmp->SEQ[0].ENDDELAY = 0;
    
    // it appears that both sequences need to be set up and linked using shorts to
    // ensure that a new pulse width is automatically picked up by the hardware
    _pwmp->SEQ[1].PTR  = ((uint32_t)(&_pulseWidth) << PWM_SEQ_PTR_PTR_Pos);
    _pwmp->SEQ[1].CNT  = 1 << PWM_SEQ_CNT_CNT_Pos;
    //((sizeof(_pulseWidth) / sizeof(uint16_t)) <<
      //                       PWM_SEQ_CNT_CNT_Pos);
    _pwmp->SEQ[1].REFRESH  = 0;
    _pwmp->SEQ[1].ENDDELAY = 0;
    
    _pwmp->EVENTS_LOOPSDONE = 1 << PWM_EVENTS_LOOPSDONE_EVENTS_LOOPSDONE_Pos;
    
    // and link shorts
    _pwmp->SHORTS = PWM_SHORTS_LOOPSDONE_SEQSTART1_Enabled << PWM_SHORTS_LOOPSDONE_SEQSTART1_Pos;
    

    // set out put pins low - put DRV8871 to sleep
    NRF_P1->DIRSET=1  << 14;        // port 1.14 aka p46 or D6
    NRF_P1->OUTCLR= 1 << 14;
    NRF_P1->DIRSET=1  << 12;        // port 1.12 aka p44 or D3
    NRF_P1->OUTCLR= 1 << 12;
    
    NRFX_IRQ_ENABLE(NRFX_CONCAT_2(MOTOR_PWM, _IRQn));  // enable interrupt

/*
    // p46 will be connected to pwm controller 0 output 0 - but not yet
    _pwmp->PSEL.OUT[0] = NRF_PWM_PIN_NOT_CONNECTED;

    // and p44 to pwm controller 0 output 1
    _pwmp->PSEL.OUT[1] = NRF_PWM_PIN_NOT_CONNECTED;
 */

}

void PWM::_stateError(int lineNo)
{
    _errorState = _state;
    _errorLine = lineNo;
}

/**
 @brief PWM pulse train done ISR

 
 Pulse train  complete interrupt service routine.
 This routine is called when the current PWM cycle completes following pulse train suspension
 
 @note NRFX_CONCAT_2 from nrfx_common.h is used to generate PWM peripheral name
 
 @ingroup ISR
*/
extern "C"
{
   void NRFX_CONCAT_2(MOTOR_PWM, _IRQHandler_v)()
   {
       {

           NRFX_CONCAT_2(NRF_, MOTOR_PWM)->EVENTS_STOPPED = 0;  // clear the stopped flag
           NRFX_CONCAT_2(NRF_, MOTOR_PWM)->INTENCLR = 2;        // and disable interrupt
           //NRF_PWM0->INTEN = 0;             // disable all event interrupts
           PWM::instance().pulseEnded(); // trigger the next action
           
       }
   }
}
