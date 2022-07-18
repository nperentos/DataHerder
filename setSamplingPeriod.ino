// ----------------------------------------
// Set user defined sampling period
// This sets timerInterval object period on Teensy
// Does not configure anything on Intan chips
//  01 Dec 2016, JE
// ---------------------------------------


const int computeSamplingPeriod(int fs) {  
  long SAMPLE_PERIOD_MICROS;  
  SAMPLE_PERIOD_MICROS = round(1000000 / fs);
  float FS = 1000000 / SAMPLE_PERIOD_MICROS; //sampling rate in Hz
  Serial.print(F("Achieved sampling rate = "));
  Serial.print(FS);
  Serial.println(" Hz");
  return SAMPLE_PERIOD_MICROS; //return sample period. used to set the Teensy Timer Interval
}
