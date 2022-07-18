// ------------------------------
// Configuring SD card for writes
// -----------------------------


// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
//#define SPI_CLOCK SD_SCK_MHZ(50) //Prob not needed since we are using sdio interface?



// Use Teensy SDIO
#define SD_CONFIG  SdioConfig(FIFO_SDIO)

// ------Create sd card volume  and datafile to write to
//SdFatSdioEX sdEx;  //  JE changed from sdio to sdioEX 20 Nov 2018, fastest implementation, requires newer/fast SD card
SdFs sdEx;
//SdFat32 sdEx;
FsFile dataFile;   // data file instance
//File32 dataFile;
boolean SDcardInitStatus = false;
boolean SDcardFileOpenStatus = false;

// file size to be logged. when full, a new file will begin (data lost on the order of 20ms)
extern const uint64_t log_file_size ;
//for flushing buffered SD to FAT
elapsedMillis flushSDElapsedTime;
const int SD_FLUSH_TIMER_MS = 60*10000; // data will be flushed every 60 s (worst case we lose last 60 s of data

// for defining file name
#define FILE_BASE_NAME "Herder_"

// Define function prototype to get user settings 
extern boolean initSDcard();
//extern boolean SDcardOpenFile(); // OLD do not use.  Use SDcardCrateLogFile() instead
extern boolean SDcardCreateLogFile(); // added JE 2018 Nov 20, creating a contiguous, pre-erased file to minimize logging latency
extern boolean crashDumpFile();
