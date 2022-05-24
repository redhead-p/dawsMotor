/**
 @file motorTest.ino
 @author Paul Redhead
 @copyright (C) Paul Redhead 2021, 2022
 
 This is the motor test program

 */


//#include <Arduino.h>


// ARM mbed
#include <mbed.h>




#include "daws.h"

// daws lib headers

#include "dawsSensor.h"
#include "dawsPWM.h"
#include "dawsMotor.h"






#define DEBUG false     ///< Debug enable for loco main
#define UI true         ///< Include user interface on IDE monitor


/**
 @brief The motor controller
 
 - pins 6 & 3 are used for the DRV8871
 - pin A0 for BEMF measurement
 - PWM produced by PWM controller peripheral
 
 */
Motor motor;            // motor uses Arduino pins 6 and 3 & A0






/**
 @brief Set up - top level
 
 This is the standard Arduino top level setup function as called by the Arduino environment.
 


 

 
 */

void setup()
{
#if UI || DEBUG
    Serial.begin(115200);    // connect serial output
    while (!Serial)
    {
        rtos::ThisThread::yield();
    }
    
    Serial.println('>');

#endif

    
    motor.setup();  // set up the motor
    
}

/**
@brief Loop - top level

This is the standard Arduino top level loop function as called by the Arduino environment.
 
 
 
 It checks for any user input from the IDE serial connection
 
 
 
 */

void loop()
{

#if UI
    if (Serial.available())
    {
        serialEvent();
    }
#endif

    rtos::ThisThread::yield();
    
}


    
#if UI
/**
@brief Interpret user commmands

 This is the standard seriel event handler called by the Arduino environment when serial data received
 (from IDE monitor/plotter).
 
 Commands
 
  [x][-|y][n]  spaces are ignored
 
  x - command - set speed assumed if missing,   y command qualifier, n number:
 - [n]  new speed
 - S stop
 - >[n] set direction forward, speed n
 - <[n] set direction reverse, speed n
 - ? print operational and configuration variables
 - GZ switch PI off
 - G[n] set PI gain to n dB (n may be 0 or -ve)
 - IZ integral component off
 - I[n] set integral component (logarithmic) 0 => 1s, 3 => 2s, -3 => 0.5s
 - C copy motor cv's to KV Store
 */
void serialEvent()
{
    int negMpy = 1;  // multilpier for negative numbers
    long numberIn = -1;  // number in command string - entered numbers are +ve
    char qualifier = '\0';  // qualifier is not null
    String inputStr = Serial.readStringUntil(0x0a);
    inputStr.toUpperCase();
    inputStr.trim();
    
    // [x][-|y][n]  spaces are ignored
    // x - command - set speed assumed if missing
    // - - indicates negative number following
    // y command qualifier - Z
    // n number
    
    
    if (inputStr.length() > 0)
    {
        
        
        char c = inputStr.charAt(0);  // get the first character,
        if (isDigit(c))  // it's a number - interpret string as a new speed
        {
            if (motor.setSpd(inputStr.toInt()))
            {
                Serial.print(F("Set speed:"));
                Serial.println(motor.getSpd());
            }
            // anything after the number will be quietly ignored
        }
        else
        {
            
            inputStr.remove(0, 1);        // remove first char from the string and
            inputStr.trim();              // any leading spaces
            if (inputStr.length() > 0) // more input
            {
                qualifier = inputStr.charAt(0); // save qualifer for later
                if (qualifier == '-') // negative numbers are possible
                {
                    negMpy = -1;
                    inputStr.remove(0,1);  // remove sign
                }
                
                if ((inputStr.length() > 0) && // even more input
                    isDigit(inputStr.charAt(0)))
                {
                    numberIn = inputStr.toInt();  // get the number
                }
            }

            
            switch (c)
            {
                case 'S': // emergency stop
                    motor.setDir(STOPPED);  // any qualifier and number ignored
                    Serial.println(F("E. Stop"));


                    break;
                    
                case '>': // forward
                    if (motor.getDir() != REVERSE)
                    {  // check no crash reverse!
                        if (motor.getDir() == STOPPED)
                        {
                            motor.setDir(FORWARD);
                            Serial.print(F("Forward"));
                        }
                        if (numberIn >= 0)// its been changed
                        {
                            motor.setSpd((unsigned int)numberIn);
                            Serial.print(':');
                            Serial.print(motor.getSpd());
                        }
                        Serial.println();

                    }
                    else
                    {
                        Serial.println(F("Not permitted - in reverse"));
                    }
                    break;
                    
                case '<': // reverse
                    if (motor.getDir() != FORWARD)
                    {  // check no crash reverse!
                        
                        if (motor.getDir() == STOPPED)
                        {
                            motor.setDir(REVERSE);
                            Serial.print(F("Reverse"));
                        }
                        if (numberIn >= 0)// it's been changed
                        {
                            Serial.print(':');
                            motor.setSpd((unsigned int)numberIn);
                            Serial.print(motor.getSpd());
                        }
                        Serial.println();
                        
                    }
                    else
                    {
                        Serial.println(F("Not permitted - going forward"));
                    }
                    break;
                    
                case '?': //
                    Serial.println(F("Loco"));


                    
                    Serial.print(F("Zero reference:"));
                    Serial.println(motor.bemfSensor.getBemfZero());  // print bemf reference
                    Serial.print(F("Gain dB       :"));
                    Serial.println(motor.getGainDB());
                    
                    Serial.print(F("Gain Ratio    :"));
                    Serial.println(motor.getGainRatio());
                    
                    Serial.print(F("Integral log  :"));
                    Serial.println(motor.getITimeLog());
                    
                    Serial.print(F("Integral TC ms:"));
                    Serial.println(motor.getITimeConst());
                    Serial.print(F("bemf factor   :"));
                    Serial.println(motor.getBemfK(), 3);
                    switch (motor.getDir())
                    {
                        case FORWARD:
                            Serial.print(F("Foward        :"));
                            break;
                            
                        case REVERSE:
                            Serial.print(F("Reverse       :"));
                            break;
                            
                        case STOPPED:
                        default:
                            Serial.print(F("Stopped       :"));
                            break;
                    } // end switch (dir)
                    Serial.println(motor.getSpd());
                    Serial.print(F("BEMF          :"));
                    Serial.println(((float)(motor.bemfSensor.getBemf())) / 256, 3);

                    
                    Serial.print("KV Store      :");
                    switch (motor.getKVstate())
                    {
                        case KV_AWAITING_INIT:
                            Serial.println(F("Awaiting init."));
                            break;
                            
                        case KV_OK:
                            Serial.println("OK");
                            break;
                            
                        case KV_NOT_FOUND:
                            Serial.println("Key not found");
                            break;
                    
                        case KV_CHANGED:
                            Serial.println(F("Write pending"));
                            break;
                            
                        case KV_ERROR:
                            Serial.println(F("Read or write error"));
                            break;
                            
                        default:
                            Serial.println(F("Unknown"));
                            break;
                            
                    } // end switch (getKVstate())
                    
                    break;

                case 'G':
                    if (qualifier == 'Z')  // switch off gain
                    {
                        motor.setGainDB(MIN_GAIN_DB);
                        Serial.println(F("Gain off"));
                    }
                    else if (numberIn >= 0)
                    {
                        motor.setGainDB((int)numberIn * negMpy);
                        Serial.print(F("Gain:"));
                        Serial.println(motor.getGainDB());

                    }
                    break;

                case 'I':
                    if (qualifier == 'Z')  // switch Integral off
                    {
                        motor.setITimeLog(MAX_ITIME_LOG);
                        Serial.println(F("Integral time off"));
                    }
                    else if (numberIn >= 0)
                    {
                        motor.setITimeLog((int)numberIn * negMpy);
                        Serial.print(F("Integral time:"));
                        Serial.println(motor.getITimeLog());
                    }
                    break;

                case 'C':  // motor cv sync
                    if(motor.syncKV())
                    {
                        Serial.println("KV updated");
                    }
                    break;

                default:
                    /*
                     Serial.print(ch);
                     Serial.println(F(" - not recognised"));
                     */
                    break;
                    
                    
            } // end switch (c)
        }
    }
}
#endif
