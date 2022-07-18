
//implementing circular buffer: in progress as of 10 July 2018 JE
// This just declares all of the data structures involved to keep main function a bit cleaner
// Can also think about using circular buffer library for implementing e.g.: https://github.com/tonton81/Circular_Buffer
// However, not sure if this is interrupt safe.
// This one seems to be: https://github.com/rlogiacco/CircularBuffer

const size_t BUFSIZE = 454*454;//400*512;//65536*3; //  Teensy data read buffer size. At ~ 150 bytes/scan x 100 scans/s sampling rate, we have a 15 kB/s stream, so this size buffers nearly 2 seconds, which seems generous.
static volatile uint16_t dataBuffer[BUFSIZE];// = {NULL}; //circular array buffer, store 16 bit ints since Intan returns 16 bit numbers, and so does accelerometer. Use volatile for shared variables
static volatile int head, tail;

static volatile uint16_t chunkA [37];
static volatile uint16_t chunkB [36];

volatile boolean  dataAcqComplete = false;        // boolean that indicates when we are finished filling up the adc buffer, will trigger SERIALNAME transmit

volatile unsigned long timestamp;                 // for timestamps (32 bits)

//static volatile  byte accelBytes[6];  //this array will store the 6 bytes of accelerometer readings (=3 analog reads x 2 bytes each)
//                                      //format is accelX lowbyte in accelBytes[0], accelX highbyte in accelBytes[1], etc for Y, Z.

const int MIN_BUFFER_BYTES_TO_WRITE = 512;    // minimum number of bytes in buffer to operate on. 512 is a sensible choice b/c SD cards have 512 byte sectors 15 Dec 2018
