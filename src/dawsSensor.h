/**
 @file  dawsSensor.h
 @author Paul Redhead
 @copyright (C) 2019, 2020, 2021, 2022 Paul Redhead
 @version 0.a  Initial release
 */

/*
 This file is part of DAWS.
 
 DAWS is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 DAWS is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with DAWS.  If not, see <http://www.gnu.org/licenses/>.
 
 
 */

#ifndef ____dawsSensor__
#define ____dawsSensor__



/** @defgroup filterParams Operational Parameters for BEMF filters

These parameters set the operational characteristics of the median and low pass filters for
 use in processing BEMF measurements.  Recompilation is required to changed them.
 
@{
*/

/**
@brief Median filter size

The number of samples held for the median filter.

@note This must be odd.
*/
   #define FILTER_N 3
/**
 @brief Low Pass Filter Power
 
 Defines the power of the low pass filter.  For n = 2 alpha is  3/4
 */
#define FILTER_POWER 1

/**
 @}
 */

/**
 @brief base for analogue input

 This is the base class for analogue input devices
 e.g. sensors etc.
 
 
 @note This class is not intended to be independently
 instantiated.
 */
class VirtualADC
{
public:
    VirtualADC();
    VirtualADC(byte channel);
    void setup();
    //void begin(byte);

    byte getChannel();



    

    
private:
    byte    _channel;               ///< channel corresponding to analogue input pin
};

/**
 @brief Measure the motor's BEMF
 
 
 
 This class is based on the VirtualADC class which provides  hardware access to the
 analogue to digital converter.
 
 BEMF is measured across motor using a differential op. amp.  The op. amp. output is centered on Arduino 5V/2
 or 3.3V/2 depending on the Arduino board used.
 
 8 NiMH cells give a nominal 9.6V
 
 The Arduino supply is used as reference and nominal range at Arduino Vcc/2 +/- 9.6 * op. amp. gain
 Zero bemf is read as c. 512.
 
 Op. amp. output is connected to Arduino Nano 33 BLE pin A3 .
 
 The BEMF readings are filtered
 -  using a statistical median digital filter and,
 - a simple Infinite Impulse Response (IIR) low pass digital filter
 

*/
class BEMFSensor: public VirtualADC
{
public:
    BEMFSensor();
    BEMFSensor(byte, byte);
    void initSample();
    void takeSample();
    void setup();
    FixPnt2408 getBemf();
    float getBemfZero();
    void resetFilters();
    void measureBEMFzero();
    
    FixPnt2408 processReading();

    unsigned long getTimeFromConvInit();
    
    

    
private:
    byte _pin;                          // pin corresponding to ADC channel used
    volatile unsigned long _timeStamp;  // time sample was taken (µs)
    short _sample;                      //sample as taken
    
    



    // following numbers are held as fixed binary point with 8 binary digits
    // to right of point 

    FixPnt2408 _bemfZero;               // zero speed reference for bemf measurement
    FixPnt2408 _readingFilter;          // accumulation register for IIR bemf filter
    FixPnt2408 _filteredBemf;           // most recent filter output
    
    //mbed::Callback<void(FixPnt2408)> _newReadingCB;
    
    unsigned short _reading[FILTER_N];  // circular array for median filter readings
    unsigned int _ri;                   // index for oldest (next to be overwritten)

};

/**
@brief Encapsulates the functions and variables directly associated with ADC hardware interface.
*/
namespace ADConv
{

void setupHW(byte);

void regAS(VirtualADC* asPointer, byte channel);

float getRef(byte);

void initReading();

short takeReading();


};




#endif /* defined(____dawsSensor__) */
