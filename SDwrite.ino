// ----------------------------------------
// Initialize SD card volume and open file
// Get SD card file name from serial input
// Filename is restricted to 64 characters max
//  11 July 2018, JE
// ---------------------------------------
String fnNEW = "";
//#define error(s) sdEx.errorHalt(&Serial, F(s))

boolean initSDcard() {

  boolean SDcardInit_success = false;
  Serial.print(F("Initialize SD card..."));
  if (!sdEx.begin(SD_CONFIG)) {
    Serial.println(F("SdFs begin() failed. Check SD card is present in microSD slot."));
    //errBlink(REDpin);
  }
  else { // SD memory card initiailzed; now open a file
    sdEx.chvol(); // make sdEx the current volume.
    SDcardInit_success = true;
  }
  if (SDcardInit_success) {
    Serial.println("DONE");
  } else {
    //    error("could not initialise SD card...");
    errBlink(REDpin);
  }

  //  Serial.write(TERM_CHAR); //write termination character, used to sync with LabView
  return SDcardInit_success;
}


/* ----------------------------------------------------------------------------------
    Create pre-allocated, pre-erased file. Should drastically reduce SD write latency
    Block of code adapted from:
    https://github.com/tni/teensy-samples/blob/master/SdFatSDIO_low_latency_logger.ino
  ---------------------------------------------------------------------------------------*/

//////////////// initialise sd card //////////////////////
/////////////////////////////////////////////////////////
boolean SDcardCreateLogFile() { // added JE 2018 Nov 20, creating a contiguous, pre-erased file to minimize logging latency

  // define new filename
  FsFile root;
  FsFile entry;
  FsFile crashReport;
  char fileName[32] = "";
  int numFile = 0;
  String fnNewList[] = {"00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10"}; //

  if (!root.open("/")) {
    //error("open root");
    Serial.println("open root failed");
    errBlink(REDpin);
  }

  while (entry.openNext(&root, O_RDONLY)) {
    entry.printFileSize(&Serial);
    Serial.write(' ');
    entry.printModifyDateTime(&Serial);
    Serial.write(' ');
    entry.printName(&Serial);
    entry.getName(fileName, 32);
    if ((String(fileName[0]) + String(fileName[1]) + String(fileName[2]) == "Her")) {
      numFile = numFile + 1;
    }
    if (numFile <= 10) {
      fnNEW = fnNewList[numFile];
    }
    if (numFile > 10) {
      fnNEW = String(numFile);
    }
    Serial.println();
    entry.close();
  }
  root.close();

//  if (root.getError()) {
//    Serial.println("openNext failed");
//  } else {
//    Serial.println("File enumeration done...");
//  }

  fnNEW = "Herder_" + fnNEW + ".tsy";
  Serial.print(F("New file name defined as:  "));
  Serial.println(fnNEW);

  // Set file name suffix.  Base file name is "Herder_"
  // filename for file.createcontigous for SD must be DOS 8.3 format:
  //  http://www.if.ufrj.br/~pef/producao_academica/artigos/audiotermometro/audiotermometro-I/bibliotecas/SdFat/Doc/html/class_sd_base_file.html#ad14a78d348219d6ce096582b6ed74526

  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  const uint8_t FILE_NAME_DIM  = BASE_NAME_SIZE + 7;
  //char binName[FILE_NAME_DIM] = FILE_BASE_NAME "00.tsy";
  char binName[14] ;
  char fname[FILE_NAME_DIM];
  fnNEW.toCharArray(binName, 14);
  strcpy(fname, binName);

  // Open or create file - truncate existing file.
  if (!dataFile.open(fname, O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println(F("Open failed\n"));
    errBlink(REDpin);
    return;
  }
  // File must be pre-allocated to avoid huge delays searching for free clusters.
  if (!dataFile.preAllocate(log_file_size)) {
    Serial.println(F("PreAllocate failed\n"));
    dataFile.close();
    errBlink(REDpin);
    return;
  }

  Serial.println(F("PreAllocate success\n"));
  Serial.print(F("File Size set to: "));
  Serial.print(log_file_size/(1024*1024)); Serial.println(" MB");
  
  if (dataFile.isContiguous()) {
    Serial.println(("DataFile is contiguous...\n"));
  }

  uint32_t first_sector, last_sector;
  if (!dataFile.contiguousRange(&first_sector, &last_sector)) {
    Serial.println(("PreallocatedFile: contiguousRange() failed"));
    // last_error = E_create_contiguous;
    //   return false;
  }
  //  Serial.println("dataFile.fileSize()");
  //  println64(&Serial, dataFile.fileSize());

  // Print current date time to file.
  printNow(&dataFile);
  dataFile.println();
  dataFile.flush();
  Serial.println(F("Sectors pre-allocated and pre-erased"));
  Serial.println(F("The log file is ready"));
  return true;
}

boolean crashDumpFile() {
  FsFile crashReport;
  //char crshName[13] = "crashDump.txt";
  if (CrashReport) {
    Serial.println("dumping crash report to SD...");
    if (!crashReport.open("crashDump.txt", O_WRITE | O_CREAT | O_APPEND)) {
      Serial.println("Cant open crash file");
    }
    Serial.println("file opened");
    crashReport.println(CrashReport);
  }
  crashReport.close();
  return true;
}

//------------------------------------------------------------------------------
// Call back for file timestamps.  Only called for file create and sync().
void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) {

  // Return date using FS_DATE macro to format fields.
  *date = FS_DATE(year(), month(), day());

  // Return time using FS_TIME macro to format fields.
  *time = FS_TIME(hour(), minute(), second());

  // Return low time bits in units of 10 ms.
  *ms10 = second() & 1 ? 100 : 0;
}
//------------------------------------------------------------------------------
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}
//------------------------------------------------------------------------------
void printField(Print* pr, char sep, uint8_t v) {
  if (sep) {
    pr->write(sep);
  }
  if (v < 10) {
    pr->write('0');
  }
  pr->print(v);
}
//------------------------------------------------------------------------------
void printNow(Print* pr) {
  pr->print(year());
  printField(pr, '-', month());
  printField(pr, '-', day());
  printField(pr, ' ', hour());
  printField(pr, ':', minute());
  printField(pr, ':', second());
}
