/**
 @file dawsSensor.cpp
 @author Paul Redhead on 7/7/2020.
 @copyright (C) 2020 Paul Redhead
 @version 0.a
 */
//  
//
//
//  This file is part of LocoOS.
//  LocoOS is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  LocoOS is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

//  You should have received a copy of the GNU General Public License
//  along with LocoOS.  If not, see <http://www.gnu.org/licenses/>.
//
//  Version 0.a First released version
//
//
//
#include <Arduino.h>
#include <limits.h>
#include <mbed.h>
#include "daws.h"

#include "dawsSensor.h"

#define DEBUG false         ///< Debug Loco Sensor to IDE
#define PLOT  false         ///< Plot loco sensor to IDE plotter

#define NUM_SENSORS 8      ///<  number of sensors - these are on pins A0, A1 etc






namespace ADConv
{
// pointers to sensor objects - Nano 33 BLE has six analogue pins -
// at the moment only one in use
VirtualADC* asp[NUM_SENSORS]; ///< array of registered sensors indexed by channel
uint16_t sample;    ///< result of last sample (writen to by SAADC)

}

/**
 @brief Determine zero reference reading
 
 Establish the zero reference voltage for channel. This is for use where the actual voltage being measured may be +ve or -ve
 and an offset has been added using analogue circuitry to make the range compatible with the analogue input.
 This is intended to be called as part of initialisation when the source voltage is known to be 0V.
 
 An average of 64 readings is taken.
 
 @note This function uses blocking IO.  Do not use when time critical.
 
 @param channel - ADC channel
 @return zero refence floating point
 */
float ADConv::getRef(byte channel)
{
    
    float ref = 0.0;
#if DEBUG
    long startTime;
    long elapsedTime;
    int yieldCount = 0;
#endif
    NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;  // and run self calibration
    while (NRF_SAADC->EVENTS_CALIBRATEDONE == 0)
    {
        rtos::ThisThread::yield();
    }
    NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
    
#if DEBUG
    startTime = micros();
#endif
    for (int x = 0; x < 64; x++)  // average over 64 samples
    {
        NRF_SAADC->TASKS_START = 1;
        NRF_SAADC->TASKS_SAMPLE = 1;
        
        while (NRF_SAADC->EVENTS_END == 0)
        {
            rtos::ThisThread::yield();
#if DEBUG
            yieldCount++;
#endif
        }
        NRF_SAADC->EVENTS_END = 0;  // clear end event flag
        
        ref += (float)sample;  // add in to accumulator
    }
#if DEBUG
    elapsedTime = micros() - startTime;
    Serial.print("analogue conv:");
    Serial.print(elapsedTime/64);
    Serial.print(" - ");
    Serial.println(yieldCount);
    Serial.print("SAADC IRQ ");
    Serial.println((NRFX_IRQ_IS_ENABLED(SAADC_IRQn))?"Enabled":"Disabled");
#endif
    return(ref/64.0);  // return average
}

/**
@brief Setup - start of day procedure
 
 Initialise the Analog to Digital Converter (ADC0):
 - Reference is VDD/4 - gain is 4 so full scale deflection  (1023) is VDD
 - Single sample per measurement.
*/
void ADConv::setupHW(byte channel)
{
    // set up analogue to digital conversion registers
    // for ADC0 channel 3 (pin A0)
    NRF_SAADC->RESULT.PTR = (uint32_t)&sample;  // point at result
    NRF_SAADC->RESULT.MAXCNT = 1;           // just 1 result to store
    NRF_SAADC->RESOLUTION = NRF_SAADC_RESOLUTION_10BIT;         // same as Uno
    // defaults for single ended & burst disabled modes
    NRF_SAADC->CH[channel].PSELP = NRF_SAADC_INPUT_AIN2;
    NRF_SAADC->CH[channel].CONFIG =
    (
     (NRF_SAADC_ACQTIME_3US   << SAADC_CH_CONFIG_TACQ_Pos)  |    // minimal sample time
     (NRF_SAADC_REFERENCE_VDD4  << SAADC_CH_CONFIG_REFSEL_Pos) | // VDD/4 referennce
     (NRF_SAADC_GAIN1_4 << SAADC_CH_CONFIG_GAIN_Pos) |             // gain 1/4 - fsd = VDD
     //(NRF_SAADC_RESISTOR_VDD1_2 << SAADC_CH_CONFIG_RESP_Pos)
     (NRF_SAADC_RESISTOR_DISABLED << SAADC_CH_CONFIG_RESP_Pos)   // resistor ladder off
     );

    NRF_SAADC->ENABLE = 1;      // enable SAADC - this must be done before calibration

    
    
}
/**
@brief Register channel in use

Save the reference to the sensor object in the array so that the ISR knows
which sensor object is associated with the channel when a reading completes.

@param asPointer - pointer to the sensor object
@param channel - channel number

*/
void ADConv::regAS(VirtualADC* asPointer, byte channel)
{
    asp[channel] = asPointer;
}

/**
 @brief Init reading
 
 This initiates taking a reading from the SAADC.  Aquistion time for the sample and hold is 3µs.
 Conversion time is <2µs.  This runs at the priority of the thread issuing the read.

 @todo can't use lock - need to check if busy
*/
void ADConv::initReading()
{
    //adLock.lock();  // not legal for ISR
    NRF_SAADC->TASKS_START = 1;
    NRF_SAADC->TASKS_SAMPLE = 1;
}
/**
 @brief take reading
 
 This takes a reading from the SAADC synchronously.  Aquistion time for the sample and hold is 3µs.
 Conversion time is <2µs.  This runs at the priority of the thread issuing the read.  If the sample is not ready, it waits (yields)
 until it is.
 
 @return sample value as a short int.
 
 @note Not to be used in ISR or ticker service context.  Lower priority threads will be blocked.
 
 
 */
short ADConv::takeReading()
{
    short mySample;

    
    while (NRF_SAADC->EVENTS_END == 0)
    {
        rtos::ThisThread::yield();  // blocks lower priority threads for c. 5µs
    }
    NRF_SAADC->EVENTS_END = 0;  // clear end event flag
    mySample = sample;
    return (mySample);
}


/**
 @brief ADC result ready ISR
 
 Analog to digital conversion complete interrupt service routine.
 This reads the measurement and passes this and the reading initiation timestamp
 to the processReading routine which may be channel specific as it
 is virtual in the base class.
 
 @note Not in use!
 
 */
extern "C"
{
void SAADC_IRQHandler_v()
    {
        NRF_SAADC->INTENCLR = 2;
    }
}


/**
 @brief void constructor
 */
VirtualADC::VirtualADC()
{
}

/**
 @brief Construct sensor for given converter channel
 
 @param channel - ADC channel number
 */
VirtualADC::VirtualADC(byte channel)
{
    _channel = channel;       // number of channel on converter
}

/**
  @brief Setup up sensor
 *
 
 This registers the sensor object with the ADC routines so that
 when a new reading is available for the sensor on its channel
 the processReading() routine for this class object that instatiated this object is called.
 *********************************/

void VirtualADC::setup()
{
#if DEBUG
    Serial.println("in ADC begin " + String(_channel));
#endif
    ADConv::regAS(this,_channel);
}

/**
 @brief Get channel number of sensor
*********************************

This returns the ADC multiplexer channel number for this sensor.

@returns channel number
*********************************/
byte VirtualADC::getChannel()
{
    return(_channel);
}




/**
 @brief Construct BEMF sensor
 
 This constructs a BEMF sensor for the given Arduino pin number and ADC channel.  Taking a sample runs at the priority of
 the thread issuing the read.  Processing the sample runs at this thread's priority.
 
 @note On the Arduino Uno there is a simple relationship between channel and pin number.  The
 pin number is the channel plus an offset.  On the Nano Every  and mbed boards the relationship is not obvious and it's
 simpler to provide both.
 
 @param pin - the arduino pin assignment
 @param channel - the ADC mulitplexer channel used
 
 */
BEMFSensor::BEMFSensor(byte pin, byte channel):
VirtualADC(channel)

{
    _pin = pin;  // not used - deprecated & to be removed
}

/**
 @brief Setup BEMF measurements
 
 This calls the setup routine in the base ADC class.

 @note  Unlike reporter based class setup routines,
 this has to be called explicitly.
 */
void BEMFSensor::setup()
{
#if PLOT || DEBUG
    Serial.begin(115200);    // connect serial output
    
#endif


    // call setup in the base ADC class
    VirtualADC::setup();      //  to register the channel
}

/**
 @brief measure BEMF zero reference
 
 
 This determines the ADC reading corresponding to 0V BEMF.
 
 BEMF zero reference is the ADC reading at zero speed.
 The analogue differential amplifier sets the zero reading
 to +5V/2.  The raw analogue reading should be c. 512.
 A reading greater than the reference indicates forward motor rotation; less than the reference, reverse.
 
 It calls the A2D converter routine to obtain the measurement and saves it.
 */
void BEMFSensor::measureBEMFzero()
{
    _bemfZero = (long)(ADConv::getRef(getChannel()) * 256.0) ;  // get 0V reference reading
}



/**
 @brief Initiate the sample reading
 
 This initiates taking a reading from the SAADC.  The time of the sample is taken.

 */
void BEMFSensor::initSample()
{
    // record time reading initiated for use when processing reading
    _timeStamp = micros();
    ADConv::initReading();
}


/**
 @brief Collect a BEMF sample
 
 Collect the result of the analogue conversion.
  The sample is saved.
 */
void BEMFSensor::takeSample()
{
    _sample = ADConv::takeReading();
}



/**
 @brief Process Reading
 
 The reading is
 - filtered using statistical median and IIR LP filters
 - the resulting bemf is then passed to the motor controller for calculation of the new pulse width.
 
 Occasionally exceptionally large or small readings occur.
 This seems primarily to be associated with noise, but can occur as result of
 timer glitches - current timer settings and procedure appear glitch free
 but beware!
 
 The n-stage statistical median filter removes spikey noise:
 - The last n (where n is odd) readings are retained (inclusive of latest reading)
 - The list is sorted
 - The filter output is the median value (i.e. middle reading of the sorted list)
 
 For a 3 element list, a single iteration spike will be rejected as long as it is separated from the nearest spike by
 2 normal readings.
 
 For Low pass filter description
 @see LP_FILTER
 
 @note This method is called once per sample.  It must complete
 before the end of the current sample period otherwise an overrun will occur.
 
 @param x - raw reading from ADC
 @param timeStamp - time stamp from when raw reading was initiated  (not used here)
 
 @returns the filtered bemf reading.
 */

FixPnt2408  BEMFSensor::processReading()
{
    FixPnt2408 medianReading;  // output from median filter
    {  // begin median filter
        unsigned short i;
        unsigned short l = FILTER_N; // number of readings in set to be filtered
        unsigned short temp;
        unsigned short sortedReading[FILTER_N];
        bool swapped;
        _reading[_ri++] = _sample;  // overwrite oldest reading & increment index
        if (_ri == FILTER_N)
        {
            _ri = 0; // back to the beginning again
        }

        for (i = 0; i < FILTER_N; i++)
        {
            sortedReading[i] = _reading[i];  // copy readings for sort
        }
        // simple bubble sort should be OK
        swapped = true;  // must be true for 1st while test!
        while ((l > 0) && swapped)  // if nothing swapped sort is finished
        {
            swapped = false;
            for (i = 0; (i + 1) < l; i++)
            {
                if (sortedReading[i] > sortedReading[i + 1])
                {
                    // swap readings
                    temp = sortedReading[i + 1];
                    sortedReading[i + 1] = sortedReading[i];
                    sortedReading[i] = temp;
                    swapped = true;
                }
            }
            // for any iteration the highest will have 'bubbled' to the end
            l--;    // so it can be ommitted from the next iteration
            
        }
        medianReading = ((FixPnt2408)sortedReading[FILTER_N/2]) << (8);   // take the middle (median) reading
    }   // end median filter

    
    // resulting median reading is now fed into the IIR low pass filter
    _readingFilter = LP_FILTER(medianReading, _readingFilter, FILTER_POWER);
    
    _filteredBemf = (ROUND(_readingFilter - (_bemfZero << FILTER_POWER),
                             FILTER_POWER));
    
#if PLOT
    // time from conversion start to now
    // if done this needs to be first
    // Serial.print(getTimeFromConvInit());
    // Serial.print('\t');
    {
    
        
        Serial.print(_sample - ROUND(_bemfZero, 8));
        Serial.print('\t');     // allows copy and past from IDE window to Excel etc
        Serial.print(ROUND(medianReading - _bemfZero, 8));
        Serial.print('\t');
        Serial.print(((float)(getBemf())) / 256);
        Serial.println();
    }

         
#endif


    return(_filteredBemf);
}
/**
 @brief Determine the time to process a reading
 
 Uses the time stamp taken when the reading was initiated to determine the time taken so far.
 This will include the time taken by the ADC peripheral to process the sample.
 
 @returns time taken in micro seconds
 */
unsigned long BEMFSensor::getTimeFromConvInit()
{
    return (micros() - _timeStamp);
}


/**
 @brief Get the BEMF measurement
 
 This returns the most recent filter output to a BEMF measurement.  The measurement is to the same resolution as
 the original measurement but is at the fixed point precision to reflect the result of filtering.
 It is adjusted to allow for the zero BEMF offset and will be -ve for reverse motion.
 
 @return the fitered BEMF measurement
 */
FixPnt2408 BEMFSensor::getBemf()
{
    return(_filteredBemf);
}

/**
@brief Get the BEMF zero reference

This exposes the zero reference value.  I.e the value measured at start of day equivalent to zero BEMF.

@return the reading corresponding to zero BEMF
 */

float BEMFSensor::getBemfZero()
{
    return((float)_bemfZero /  256.0);
}

/**
@brief Reset Filters
 
 The median and IIR LP filters save values from previous iterations which are used to process
 the current value.  They need to be initalised at start of day and reset when direction is set from
 stationary.  This routine sets the values to be consistent with a history of zero bemf - i.e. a stationary motor.
*/

void BEMFSensor::resetFilters()
{
    // set median filter history values to initial state
    for (int x = 0; x < FILTER_N; x++)
    {
        _reading[x] = ROUND(_bemfZero, 8);
    }
    _ri = 0;  // reset median filter index

    // set IIR LP filter start value to zero reference
    _readingFilter = _bemfZero << FILTER_POWER;
}
