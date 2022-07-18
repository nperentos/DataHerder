
/*
    _____________________________________________________________________________________________________________________
   |Intsy2.0 is a 32(EEG/LFP)+3(accelerometer)+X(auxiliary) *high* sampling rate logger implementation using a Teensy 4.0|
   |microcontroller and an Intan Bioamplifier (RHD2132) chip.                                                            |
   |                                                                                                                     |
   |This development adapts Jon Erickson's 64ch electrogastrography implementation (https://github.com/jonatmudd/Intsy), |
   |with emphasis on increased portability for applications involving freely moving animals and sampling rates compatible|
   |with local field potential and single unit electrophysiological recordings.                                                 |
   |Open source schematics, BOM, and assembly instructions can be found at XYZ                                           |
   |Developed by Andreas Genewsky, Zyian Prince, Jon Erickson and Nikolas Perentos                                       |
   |Contact: Nikolas Perentos perentos.n@unic.ac.cy                                                                      |
   |_____________________________________________________________________________________________________________________|

  This software is covered by the MIT license.  See below for details.

  Copyright (c) 2017 - 2018  Jonathan C Erickson

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
  subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
// preset the CPU speed
#if defined(__IMXRT1062__)
extern "C" uint32_t set_arm_clock(uint32_t frequency);
#endif

// ------------------------------------------
//  Import libraries and declare variables. Voltatile is needed for interrupts to do A/D conversion with convert() cmd
// -------------------------------------------
#include <stdlib.h>
#include <SPI.h>
//#include <SdFat.h>  //requires Bill Greiman's sdFat library (downloaded July 2018 from: https://github.com/greiman/SdFat)
#include "SdFat.h"
#include "IntanDefaultConfig.h"    // Define various 
#include "SPIconfig.h"             // Define SPI hardware pins on Teensy
#include "IntanCmd.h"              // Intan commands for read, write, calibrate, convert
#include "FilterSettingDefaults.h" // Define default Intan bandpass setting, as well as easy options
#include "setSamplingPeriod.h"     // Define sampling period for interval timer object (for Teensyduino interrupts)
#include "AccelerometerConfig.h"   // Define acceleromter pins and variables that store readings
#include "CircularBufferConfig.h"  // Define circular buffer. This is crucial element to control data flow
#include "SDwrite.h"               // setting up SD card for writing data
#include <TimeLib.h>               // for timestamping the log files
#include "arm_math.h"

IntervalTimer SamplingPeriodTimer;          // Create an IntervalTimer object

extern float tempmonGetTemp(void);

const int BAUD_RATE_USB =  115200; // baud rate Serial port, only affects UART, not USB which always runs at 12 MB/s, see: https://www.pjrc.com/teensy/td_serial.html

const unsigned int SPI_PIPE_DELAY = 2; //Intan's results return 2 commands later in SPI command pipe.

int numAmpsVerified; // value returns for SPIgetAmplInfo.  Should be either 32 or 64 (or 0 if error)

int REDpin = 14; // external red LED used for status indicating (solid red: loging, flashing red: error encountered)

uint16_t throwAwayRead;  //place holders for 2 delay in SPI command: throw away data

int fs = 8000; // sampling rate (Hz)
const uint64_t log_file_size = 5000ULL*1024*1024; //file size (bytes)
const uint64_t log_file_size_adj = log_file_size - 1000; // save some bytes for writing clock time to the end of the file
uint32_t cnt = 0;

void setup() {
  // ----------------------------------------------------------------
  // SETUP:  start serial port, initialize and verify SPI interface,
  // setup SD card and prepare a log file for write
  // ----------------------------------------------------------------
  Serial.begin(BAUD_RATE_USB);
  
  // CPU speed adjustment - helps with temperature management https://forum.pjrc.com/threads/57196-T4-heatsink-recommended
  #if defined(__IMXRT1062__)
    set_arm_clock(528000000); // at <= 528MHz voltage on cpu drops to 2.15 from 2.25 thus reducing temperature 
    Serial.print("F_CPU_ACTUAL = ");
    Serial.println(F_CPU_ACTUAL);
  #endif
  
  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);
  pinMode(REDpin, OUTPUT);//status LED
  

  Serial.print(F("Requested file size is: "));Serial.print(log_file_size/(1024*1024));Serial.println(F(" MB"));  
  
  analogReadResolution(16); // Teensy4.0 has native 10 bit resolution, so we pad to 16 so as to have 2 full bytes

  Serial.println(F("VERSION: T40_64ch_v10"));

  if (timeStatus() != timeSet) {
    Serial.println(F("Unable to sync with the RTC"));
    return;
  }

  // Set time callback Used by SDFat when generating new files
  FsDateTime::setCallback(dateTime);
  SPI_init();// Is this reduntant?

  //keep track of how many bytes written to SD card
  //static uint64_t TotalNumBytesWrittenToSD = 0;// replaced with dataFile.fileSize()
  static uint16_t NbytesWrite;

  Serial.println(F("Initialising SPI port..."));
  SPI_init(); delay(100);// initialize 2x RHD2132, see SPIconfig.ino

  Serial.println(F("Veryfying SPI link with INTAN chips..."));
  SPI_verify(); delay(100);  // checks reg 40-44 for "INTAN" characters

  Serial.println(F("Configuring INTAN registers..."));
  numAmpsVerified = SPI_getAmplInfo();  delay(100);  // get more info about amplifiers: number, chip revision, etc. SPI pins defined in SPIconfig.h
  ConfigureRegisters_0_17(); delay(100);// see IntanDefaultConfig, configures both A and B amps

  Serial.println(F("Configuring INTAN filter settings..."));
  configFilterSettings(); delay(100);//see FilterSettingDefaults.h and .ino, configures both A and B amps

  Serial.println(F("Setting the sampling rate..."));
  SAMPLE_PERIOD_MICROS = computeSamplingPeriod(fs); //see setSamplingPeriod.h and .ino
  delay(100);

  Serial.println(F("Calibrating the INTAN chip..."));
  SPI.beginTransaction(SPIsettingsFast);
  calibrateADC(CSpinA);      // see IntanCmd.h and .ino
  calibrateADC(CSpinB);      // configure both A and B amps
  SPI.endTransaction();
  delay(100);

  // Setup the SD card
  SDcardInitStatus = initSDcard(); delay(100);  // initialize SDfat volume
  crashDumpFile(); // check for the presence of a crash log, if one available write it to crashDump.txt
  SDcardFileOpenStatus = SDcardCreateLogFile();//create a data log file  

  Serial.println(F("Configure the SPI bus..."));
  // configure the SPI bus.  This uses faster clock speed 12 MHz for converting channels. JE 01 May 2017
  SPI.beginTransaction(SPIsettingsFast);
  
  Serial.println(F("Starting data logging..."));
  digitalWriteFast(REDpin, HIGH);//redLED for now
  //start the interval timer, turns on interrupts for precisely timed sampling ADC
  SamplingPeriodTimer.begin(scanADC, SAMPLE_PERIOD_MICROS);
  //SamplingPeriodTimer.priority(10);            // 0 is max priority, so 10 grants relatively  high priorty for this ISR.  JE 27 apr 2017.

}  //end setup()


void errBlink(int pin) {
  // ------------------------------------------------
  // errBlink() pulse a digital pin to indicate error
  // ------------------------------------------------
  while (1) {
    digitalWriteFast(pin, !digitalReadFast(pin));
    delay(100);
  }
}


void scanADC(void) {
  // ---------------------------------------------------
  // void scanADC() The interrupt service routine for
  // precisely timed data sampling from INTAN+AUX+ACCEL
  // ---------------------------------------------------
  //time stamp in microseconds at start of conversions
  timestamp = ARM_DWT_CYCCNT;//micros();//ARM_DWT_CYCCNT;

  // ====== First data chunk =======
  for (byte nA = 0; nA < NUM_AMPS_PER_CHIP; nA++) {
    chunkA[nA] = convertChannel(nA, CSpinA); //this reads auxiliary data from commands sent last sampling period
  }
  // Now call 2 auxiliary commands, into which ADC(30) and ADC(31) results are returned
  chunkA[32] = readRegister(40, CSpinA); //Returns 'I' from INTAN, if data link properly established, 2 SPI commands later
  chunkA[33] = readRegister(41, CSpinA); //Returns 'N' from INTAN, if data link properly established
  // add the accelerometer reads
  chunkA[34] = analogRead(PIN_ACCEL_X);
  chunkA[35] = analogRead(PIN_ACCEL_Y);
  chunkA[36] = analogRead(PIN_ACCEL_Z);
  // pipe data over to the circular buffer
  for (unsigned int jj = 1; jj < sizeof(chunkA) / sizeof(uint16_t); jj++) {
    //writeCircBuf(chunkA[jj]);
    int next_head = (head + 1) % BUFSIZE;
    if (next_head != tail) { // buffer not full
      dataBuffer[head] = chunkA[jj];
      head = next_head;
    } else {
      SamplingPeriodTimer.end();
      Serial.println(F("buffer is full"));
      digitalWriteFast(REDpin, LOW);
      Serial.print(F("tail:")); Serial.print(tail);
      Serial.print(F(", head:")); Serial.print(head);
      Serial.print(F(", next)head:")); Serial.print(next_head);
      Serial.print(F(" BUFSIZE:")); Serial.print(BUFSIZE); //      //noInterrupts();
      errBlink(REDpin);
      /* no room left in the buffer */
    }
  }
  // ====== Second data chunk =======

  for (byte nB = 0; nB < NUM_AMPS_PER_CHIP; nB++) {
    chunkB[nB] = convertChannel(nB, CSpinB); //this reads auxiliary data from commands sent last sampling period
  }
  // Now call 2 auxiliary commands, into which ADC(30) and ADC(31) results are returned
  chunkB[32] = readRegister(40, CSpinB); //Returns 'I' from INTAN, if data link properly established, 2 SPI commands later
  chunkB[33] = readRegister(41, CSpinB); //Returns 'N' from INTAN, if data link properly established
  // add the timestamp bytes
  chunkB[34] = timestamp & 0xFFFF; //use 0xFFFF and shift 16 to convert to two ints
  chunkB[35] = (timestamp >> 16) & 0xFFFF;
  //chunkB[36] = 22379;//sync bits to check for checking fidelity of SD writes
  // pipe data over to the circular buffer
  for (unsigned int jj = 1; jj < sizeof(chunkB) / sizeof(uint16_t); jj++) {
    //writeCircBuf(chunkB[jj]);
    int next_head = (head + 1) % BUFSIZE;
    if (next_head != tail) { // buffer not full
      dataBuffer[head] = chunkB[jj];
      head = next_head;
    } else {
      SamplingPeriodTimer.end();
      Serial.println(F("buffer is full"));
      digitalWriteFast(REDpin, LOW);
      Serial.print(F("tail:")); Serial.print(tail);
      Serial.print(F(", head:")); Serial.print(head);
      Serial.print(F(", next)head:")); Serial.print(next_head);
      Serial.print(F(" BUFSIZE:")); Serial.print(BUFSIZE); //      //noInterrupts();
      errBlink(REDpin);
      /* no room left in the buffer */
    }
  }
  //dataAcqComplete = true;
   //__DSB(); //waits for interrupt flag to propagate before exiting ISR? https://forum.pjrc.com/threads/58050-Teensy-4-Interval-Timer
} //end of scanADC

void loop() {
  // ---------------------------------------------------
  // main loop()
  // Handles checking circular buffer and writing any
  // contents to SD and/or stream to serial
  // ---------------------------------------------------

  //Turn off interrupts so we can read head and tail current position and figure out if there is data in the circular buffer that can be written to sd card and/or serial
  noInterrupts();
  int curr_head, curr_tail, NumElementsInBuffer;
  curr_head = head;
  curr_tail = tail;
  interrupts(); //re-enable interrupts

  unsigned long timestampCopy;
  //static uint64_t TotalNumBytesWrittenToSD = 0; //keep track of how many bytes written to SD card
  static uint16_t NbytesWrite;

  // compute how many elements currently in data buffer
  if (curr_head == curr_tail) {
    NumElementsInBuffer = 0;
  }
  else if (curr_head > curr_tail) {
    NumElementsInBuffer =  curr_head - curr_tail;
  }
  else {
    NumElementsInBuffer =  BUFSIZE - abs(curr_tail - curr_head);
  }


  // if buffer contains sufficient number of elements, Copy Nelements into an array and write it to SD and/or Serial
  if (NumElementsInBuffer > MIN_BUFFER_BYTES_TO_WRITE) { // JE 06 Dec 2018, set to 512 to force larger block to be written to SD
    uint16_t bufCopy[NumElementsInBuffer];

    // Loop to copy elements from circular buffer, one at at time.
    for (int m = 0; m < NumElementsInBuffer; m++) {
      //bufCopy[m] = readCircBuf(); //get data from circular buffer (make non-volatile copy)
      if (tail != head) { //there is data
        bufCopy[m]  = dataBuffer[tail];
        tail = (tail + 1) % BUFSIZE;
      }
    }   //end loop copying data from circular buffer

    // Write data to SD card.
    if ((dataFile.fileSize() + NbytesWrite) <= log_file_size_adj) { //ensure sufficient space in current file
      NbytesWrite = dataFile.write((uint8_t*)bufCopy, sizeof(bufCopy));
      //TotalNumBytesWrittenToSD += NbytesWrite;
      if ((unsigned) NbytesWrite != sizeof(bufCopy)) {
        // perhaps here we should switch on two LEDs the green and the red? or reset?
        Serial.println(F("not all data written to card"));
        SamplingPeriodTimer.end();
        errBlink(REDpin);
      }
    }
    else { //filled current log file
      SamplingPeriodTimer.end();
      Serial.println(F("file full..."));     
      digitalWriteFast(REDpin, LOW);
      //FinishLogFile(); //TotalNumBytesWrittenToSD
      dataFile.println("");
      printNow(&dataFile);
      dataFile.truncate();//TotalBytesWritten
      dataFile.close();
      Serial.print(F("tail:")); Serial.print(tail);
      Serial.print(F(", head:")); Serial.println(head);
      // reset circ buff values
      head = 0; tail = 0; NbytesWrite = 0;
      Serial.println(F("starting new file..."));
      SDcardFileOpenStatus = SDcardCreateLogFile();
      digitalWriteFast(REDpin, HIGH);
      flushSDElapsedTime = 0;
      SamplingPeriodTimer.begin(scanADC, SAMPLE_PERIOD_MICROS);
    }

    //  periodically flush SD card to update its FAT: addedJE 20 Nov 2018
    if (flushSDElapsedTime >= SD_FLUSH_TIMER_MS) {
      dataFile.flush();  // flush data to commit SD buffer to physical memory locations.  Inefficient to do this everytime we write a block of data?
      Serial.println( tempmonGetTemp() );
      flushSDElapsedTime = 0; //reset timer, timing precision isn't critical
    }
  } // if there's enough elements in buffer
} //loop
