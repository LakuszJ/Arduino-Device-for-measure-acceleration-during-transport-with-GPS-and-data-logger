#include <math.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

#include <SPI.h>
#include <SD.h>

#include <Wire.h>
#include <LSM6.h>
LSM6 imu;

Sd2Card card;
SdVolume volume;
SdFile root;
const int chipSelect = 10;
File myFile;

// GPS SETINGS
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false
//

#define buttPIN 2
#define ledGreenSaveModePIN 3
#define niebieska 6
#define zielona 5
#define czerwona 4


void setup()
{
  Serial.begin(115200);
  delay(1000);

  // GPS -  SETTINGS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  mySerial.println(PMTK_Q_RELEASE);
  delay(1000);

  /// !!! ACCELEROMETRY - START SETTINGS !!! ///
  Wire.begin();
  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();

  /// ACC. -  SETTINGS
  imu.writeReg(imu.CTRL1_XL, 0b00010000); // int=16 ; 12,5 Hz
  //imu.writeReg(imu.CTRL1_XL, 0b01000010); // int=?? ; 104 Hz // LPF2_XL_EN=1 --> LPF2=ON
  //imu.writeReg(imu.CTRL1_XL, 0b01000010); // int=?? ; 104 Hz // LPF2_XL_EN=1 --> LPF2=ON
  //imu.writeReg(imu.CTRL8_XL, 0b00000000); // 

  imu.writeReg(imu.CTRL2_G, 0); // TurnOffGyro
  imu.writeReg(imu.CTRL5_C, 0b00000000);

  // ACC. - Print current settings
  delay(1000);
  Serial.println("_____________________");
  uint8_t test1 = imu.readReg(imu.CTRL1_XL); 
  Serial.println(test1);
  uint8_t test2 = imu.readReg(imu.CTRL8_XL); 
  Serial.println(test2);
  uint8_t test3 = imu.readReg(imu.CTRL5_C);
  Serial.println(test3);
  Serial.println("_____________________");


  /// !!! SD CARD - START SETTINGS !!! ///
  while (!Serial) {
 
    ; // wait for serial port to connect. Needed for native USB port only

  }
  Serial.println("\nInitializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    while (1);
  }
   
  // myFile = SD.open("TEST6667.TXT", FILE_WRITE);
  // myFile.close();
  testSdCard();

  // BUTTONS and LEDs
  pinMode(buttPIN, INPUT_PULLUP);
  pinMode(ledGreenSaveModePIN, OUTPUT);

  pinMode(czerwona, OUTPUT); // Piny, podłączone do diody jako wyjścia
  pinMode(zielona, OUTPUT);
  pinMode(niebieska, OUTPUT);
 
}


// GPS 
  int gpsFix=0;
  int gpsQuality=0;
  int gpsNoSatelites=0;

  int gpsDay=0;
  int gpsMonth=0;
  int gpsYear=0;
  int gpsHour=0;
  int gpsMinutes=0;
  int gpsSeconds = 0;
  int gpsMilliseconds=0;

  float gpsLatitudeDeg = 0.00;
  float gpsLongitudeDeg = 0.00;
  float gpsSpeed=0.00;
//

// ACC

  // ACC. - calibration cons value
    const float g=9.81;
    const float calib_Zminus=g/16376;
    const float calib_Zplus=g/16609;
    const float calib_Xminus=g/16390;
    const float calib_Xplus=g/16458;
    const float calib_Yminus=g/16809;
    const float calib_Yplus=g/15831;
  //

  float aXm=0.00;
  float aYm=0.00;
  float aZm=0.00;

  BLA::Matrix<3, 3,float> mA = {1, 0, 0,
                          0, 1, 0, 
                          0, 0, 1};
  BLA::Matrix<3, 3, float> mT = {1, 0, 0,
                          0, 1, 0, 
                          0, 0, 1};
//


// GENERAL DATA
  int i=0;
  int n=0;

  unsigned long stratLoopTime = 0;
  unsigned long stopLoopTime = 0;
  
  bool calibrationMode = false;
  bool saveMode = false;
  bool errorDuringSave = false;
  bool backupSaveMode = false;
  String currentFileName="";
  int fileNo=0;
  int nextNewFileSaveStep = 90000; // 90000 every ~2h will crate new file

  // BUTTON
  bool saveStartButton = false;
  byte lastButtonState = HIGH;
  unsigned long debounceDuration = 50; // millis
  unsigned long lastTimeButtonStateChanged = 0;
  //

  // Array
  const int arrSize=50;//250 = 20s ; 375 = 30s
  const int arrNoElements = 26; // Warning ! - at function "calcArrayMediumValue"  is necessary to change manualy
  float mainArray[arrSize][arrNoElements];
  int savePositionAtArray=0;
  //
//


void loop()
{
  // BUTTONS AND LEDS
  if (millis() - lastTimeButtonStateChanged > debounceDuration) 
  {
    byte buttonState = digitalRead(buttPIN);
    if (buttonState != lastButtonState) 
    {
      lastTimeButtonStateChanged = millis();
      lastButtonState = buttonState;
      if (buttonState == LOW ) // && (millis() - lastTimeButtonStateChanged) >= 300
      {
        if(saveStartButton == false)
        {
          saveStartButton=true;
          calibrationMode=true;
          n=0;
        }
        else if (saveStartButton == true)
        {
         saveStartButton=false;
         calibrationMode=false;
        } 
      }
    }    
  }


  if (saveStartButton == true && saveMode == false)
  {
   saveMode = true;
   digitalWrite(ledGreenSaveModePIN, HIGH);
   Serial.println("Start recording");
   delay(2000);

  } else if (saveStartButton == false && saveMode == true)
  {
    saveMode = false;
    digitalWrite(ledGreenSaveModePIN, LOW);
    Serial.println("Stop recording");
    delay(2000);
  }

 // GPS - READ
  char c = GPS.read();
  if (GPS.newNMEAreceived()) 
   {  
    //Serial.println("!!!!!!! XXXXXXX !!!!!!!"); 
    if (!GPS.parse(GPS.lastNMEA()))  
      return; 
   }

  if (millis()-stratLoopTime>=80)
  {
  // BASIC 
  // ACCELERATION MEASURMENTS & CYCLES CALCULATIONS & LEDS RESTART  
  stratLoopTime = millis();

  //Serial.println("_____________________");
  i++; // number of cycle from turn on device
  n++; // number of cycle from start Calibration
  //Serial.print("i=");
  //Serial.println(i); 
  //Serial.print("n=");
  //Serial.println(n); 

  imu.readAcc();
  int aXr= imu.a.x;
  int aYr= imu.a.y;
  int aZr= imu.a.z;

  //Serial.print("Basic:   ");
  //pinrt3Int(aXr,aYr,aZr);
 
  // analogWrite(czerwona, 255);
  // analogWrite(zielona, 255);
  // analogWrite(niebieska, 255);
  digitalWrite(czerwona, HIGH);
  digitalWrite(zielona, HIGH);
  digitalWrite(niebieska, HIGH); 

  // SPIRIT LEVEL
  if (saveMode==false)
  {
    int aXrABS= abs(aXr);
    if (aXrABS>900) // red 
    {
      // analogWrite(czerwona, 0);
      // analogWrite(zielona, 255);
      // analogWrite(niebieska, 255);
      digitalWrite(czerwona, LOW);
      digitalWrite(zielona, HIGH);
      digitalWrite(niebieska, HIGH); 
      
    }
    else if (aXrABS <= 900) // ping // magenta
    {
      // analogWrite(czerwona, 54);
      // analogWrite(zielona, 199);
      // analogWrite(niebieska, 115);
      digitalWrite(czerwona, LOW);
      digitalWrite(zielona, HIGH);
      digitalWrite(niebieska, LOW); 
        if (aXrABS <= 400) // blue
        {
          // analogWrite(czerwona, 255);
          // analogWrite(zielona, 255);
          // analogWrite(niebieska, 0);
          digitalWrite(czerwona, HIGH);
          digitalWrite(zielona, HIGH);
          digitalWrite(niebieska, LOW); 
          if (aXrABS <= 200) // green
          {
            // analogWrite(czerwona, 255);
            // analogWrite(zielona, 0);
            // analogWrite(niebieska, 255);
            digitalWrite(czerwona, HIGH);
            digitalWrite(zielona, LOW);
            digitalWrite(niebieska, HIGH);             
          }
        }
    }
  }
  
  // MAIN PART 1
  // ACCELERATION MEASURMENTS & CALIBARTION & DATA LOGGING
  if (saveMode==true)
  {
    if (n%25 == 0 || n==1) // every 2s
    {
      // if (GPS.hour < 10) { Serial.print('0'); }
      // Serial.print(GPS.hour, DEC); Serial.print(':');
      // if (GPS.minute < 10) { Serial.print('0'); }
      // Serial.print(GPS.minute, DEC); Serial.print(':');
      // if (GPS.seconds < 10) { Serial.print('0'); }
      // Serial.print(GPS.seconds, DEC); Serial.print('.');
      // if (GPS.milliseconds < 10) {
      //   Serial.print("00");
      // } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      //   Serial.print("0");
      // }
      // Serial.println(GPS.milliseconds);
      // Serial.print("Date: ");
      // Serial.print(GPS.day, DEC); Serial.print('/');
      // Serial.print(GPS.month, DEC); Serial.print("/20");
      // Serial.println(GPS.year, DEC);
      // Serial.print("Fix: "); Serial.println((int)GPS.fix);
      
    if (GPS.fix) 
    {
      // Serial.print("Location: ");
      // Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      // Serial.print(", ");
      // Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      // Serial.print("Google Maps location: ");
      // Serial.print(GPS.latitudeDegrees, 4);
      // Serial.print(", ");
      // Serial.println(GPS.longitudeDegrees, 4);
      // Serial.print("Speed (knots): "); Serial.println(GPS.speed);

      // Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
      // Serial.print("Quality: "); Serial.println((int)GPS.fixquality);
      // Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      // Serial.print("Angle: "); Serial.println(GPS.angle);
      // Serial.print("Altitude: "); Serial.println(GPS.altitude);


      delay(20);
      gpsFix=(int)GPS.fix;
      gpsQuality=(int)GPS.fixquality;
      gpsNoSatelites=(int)GPS.satellites;

      gpsDay=(int)GPS.day;
      gpsMonth=(int)GPS.month;
      gpsYear=2000+(int)GPS.year;
      gpsHour=(int)GPS.hour;
      gpsMinutes=(int)GPS.minute;
      gpsSeconds=(int)GPS.seconds;
      gpsMilliseconds=(int)GPS.milliseconds;

      gpsLatitudeDeg = (float)GPS.latitudeDegrees;
      gpsLongitudeDeg = (float)GPS.longitudeDegrees;
      gpsSpeed = 1.852*((float)GPS.speed);
      delay(10);
    }
    }

    float aX = axisValueNormalize(aXr, calib_Xplus, calib_Xminus);
    float aY = axisValueNormalize(aYr, calib_Yplus, calib_Yminus);
    float aZ = axisValueNormalize(aZr, calib_Zplus, calib_Zminus);

    float aXRot = mT(0,0)*aX+mT(0,1)*aY+mT(0,2)*aZ;
    float aYRot = mT(1,0)*aX+mT(1,1)*aY+mT(1,2)*aZ;
    float aZRot = mT(2,0)*aX+mT(2,1)*aY+mT(2,2)*aZ;

    float aXtared = aX-aXm;
    float aYtared = aY-aYm;
    float aZtared = aZ-aZm;

    float aXtarRot = mT(0,0)*aXtared+mT(0,1)*aYtared+mT(0,2)*aZtared;
    float aYtarRot = mT(1,0)*aXtared+mT(1,1)*aYtared+mT(1,2)*aZtared;
    float aZtarRot = mT(2,0)*aXtared+mT(2,1)*aYtared+mT(2,2)*aZtared;

    //Serial.print("Basic - Normalized:   ");
    //pinrt3Float(aX,aY,aZ);

    //Serial.print("Basic-N & Rotated-N:   ");
    //pinrt3Float(aXRot,aYRot,aZRot);

    //Serial.print("Tared-N:   ");
    //pinrt3Float(aXtared,aYtared,aZtared);

    //Serial.print("Tared & Rotated-N:   ");
    //pinrt3Float(aXtarRot,aYtarRot,aZtarRot);

    saveDataToArray(n,                                        // Number of cycle from start Calibration
                    aX,aY,aZ,                                 // Basic - Normalized (Readed from ACC. AFTER normalization, BEFORE taring)
                    aXtared,aYtared,aZtared,                  // Normalized & Tared (Readed from ACC. AFTER normalization, AFTER taring, BEFORE Rotation)
                    aXRot,aYRot,aZRot,                        // Normalized & Rotated (Readed from ACC. AFTER normalization, BEFORE taring, AFTER Rotation)
                    aXtarRot,aYtarRot,aZtarRot,               // Normalized & Tared & Rotated (Readed from ACC. AFTER normalization, AFTER taring, AFTER Rotation)  
                    gpsFix, gpsQuality, gpsNoSatelites,       // GPS 3x int - Fix, Quality, no of Staelites    
                    gpsDay, gpsMonth, gpsYear, gpsHour, gpsMinutes, gpsSeconds, gpsMilliseconds, // Datum and time (int);
                    gpsLatitudeDeg, gpsLongitudeDeg, gpsSpeed);

    if (n%arrSize == 0 && calibrationMode == true)
    {
      Serial.println("!!! First array completed !!!");

      aXm=calcArrayMediumValue(mainArray, 1 ,arrSize);
      aYm=calcArrayMediumValue(mainArray, 2 ,arrSize);
      aZm=calcArrayMediumValue(mainArray, 3 ,arrSize);

      Serial.println("Completed: Calculation of medium value for calibration.");
      Serial.print("Calibration Medium values:   ");
      pinrt3Float(aXm, aYm, aZm);

      //  Create rotation matrix
      BLA::Matrix<3,1,float> vZb = {aXm, aYm, aZm};
      float vZ_leng = sqrt(pow(aXm,2)+pow(aYm,2)+pow(aZm,2));
      BLA::Matrix<3,1,float> vZ1 = vZb * (1/vZ_leng);
      mA(0, 2) = vZ1(0);
      mA(1, 2) = vZ1(1);
      mA(2, 2) = vZ1(2);

      BLA::Matrix<3,1,float> vX1 = {1.00,0.00,0.00} ;
      BLA::Matrix<3,1,float> vY;
      vY(0) = vX1(1) * vZ1(2) - vX1(2) * vZ1(1); // a2*b3−a3*b2
      vY(1) = vX1(2) * vZ1(0) - vX1(0) * vZ1(2); // a3*b1−a1*b3
      vY(2) = vX1(0) * vZ1(1) - vX1(1) * vZ1(0); // a1*b2−a2*b1
      float vY_leng = sqrt(pow(vY(0),2)+pow(vY(1),2)+pow(vY(2),2));
      BLA::Matrix<3,1,float> vY1 = vY * (1/vY_leng);
      Serial.print("vY_leng --> sin(Z1, X1):   ");
      Serial.println(vY_leng,7);
      Serial << "vY1: " << vY1 << '\n';
      mA(0, 1) = vY1(0);
      mA(1, 1) = vY1(1);
      mA(2, 1) = vY1(2);

      Serial << "mA: " << mA << '\n';
      Serial << "vZ1: " << vZ1 << '\n';
      mT = Inverse(mA);
      Serial << "mT: " << mA << '\n';
      BLA::Matrix<3> resultTest ;
      resultTest = mT*vZb;
      Serial << "vZ_leng: " << vZ_leng << '\n';
      Serial << "resultTest: " << resultTest << '\n';

      calibrationMode = false;       // Calibration completed - exit calibration mode

      // 3x time blink green
      for (int iBlink = 0; iBlink <= 2; iBlink++) 
      {
          // analogWrite(czerwona, 255);
          // analogWrite(zielona, 0);
          // analogWrite(niebieska, 255);
          // delay(250);
          // analogWrite(czerwona, 255);
          // analogWrite(zielona, 255);
          // analogWrite(niebieska, 255);
          // delay(250);
          digitalWrite(czerwona, HIGH);
          digitalWrite(zielona, LOW);
          digitalWrite(niebieska, HIGH); 
          delay(250);
          digitalWrite(czerwona, HIGH);
          digitalWrite(zielona, HIGH);
          digitalWrite(niebieska, HIGH); 
          delay(250);          
      }
      
    }
  }

  stopLoopTime = millis()-stratLoopTime;
  //Serial.print("StopWatch:   ");
  //Serial.println(stopLoopTime);
  }
}


void saveDataToArray(int postion ,float x1, float y1, float z1, float x2, float y2, float z2, float x3, float y3, float z3, float x4, float y4, float z4,
                     int gps1, int gps2, int gps3, 
                     int gps4, int gps5, int gps6, int gps7, int gps8, int gps9, int gps10,
                     float gps11, float gps12, float gps13)
{
 //Serial.print("At start: savePostion= ");
 //Serial.println(savePosition);

  mainArray[savePositionAtArray][0] = postion;
  // Basic - Normalized (Readed from ACC. AFTER normalization, BEFORE taring)
  mainArray[savePositionAtArray][1] = x1;
  mainArray[savePositionAtArray][2] = y1;
  mainArray[savePositionAtArray][3] = z1;
  // Normalized & Tared (Readed from ACC. AFTER normalization, AFTER taring, BEFORE Rotation)
  mainArray[savePositionAtArray][4] = x2;
  mainArray[savePositionAtArray][5] = y2;
  mainArray[savePositionAtArray][6] = z2;
  // Normalized & Rotated (Readed from ACC. AFTER normalization, BEFORE taring, AFTER Rotation)
  mainArray[savePositionAtArray][7] = x3;
  mainArray[savePositionAtArray][8] = y3;
  mainArray[savePositionAtArray][9] = z3;
  //Normalized & Tared & Rotated (Readed from ACC. AFTER normalization, AFTER taring, AFTER Rotation)
  mainArray[savePositionAtArray][10] = x4;
  mainArray[savePositionAtArray][11] = y4;
  mainArray[savePositionAtArray][12] = z4;
  //GPS Fix, Quality, Satelites
  mainArray[savePositionAtArray][13] = gps1;
  mainArray[savePositionAtArray][14] = gps2;
  mainArray[savePositionAtArray][15] = gps3;
  //GPS Dataum
  mainArray[savePositionAtArray][16] = gps4;
  mainArray[savePositionAtArray][17] = gps5;
  mainArray[savePositionAtArray][18] = gps6;
  //GPS Time
  mainArray[savePositionAtArray][19] = gps7;
  mainArray[savePositionAtArray][20] = gps8;
  mainArray[savePositionAtArray][21] = gps9;
  mainArray[savePositionAtArray][22] = gps10;
  //GPS Location & Speed
  mainArray[savePositionAtArray][23] = gps11;
  mainArray[savePositionAtArray][24] = gps12;
  mainArray[savePositionAtArray][25] = gps13;

  savePositionAtArray++;

  if(savePositionAtArray==arrSize && saveMode)  // if savePositionAtArray == arrSize (size of array) then start again filling array (from index 0)
  {
    if ((n % nextNewFileSaveStep == 0 || n == arrSize) && backupSaveMode == false)
    {
      fileNo++;
      currentFileName = "";
      currentFileName.concat("acc");
      currentFileName.concat(fileNo);
      currentFileName.concat(".TXT");
      Serial.println(currentFileName);
      delay (1000);
    }
    else if (backupSaveMode == true)
    {
      currentFileName="TEST6667.TXT";
    }

    myFile = SD.open(currentFileName, FILE_WRITE);  // "TEST6667.TXT"
    delay(10);
    
    if (myFile) 
    {
      for (int i=0; i < arrSize; i++)
      {
        String savingSentence = ""; 
        
        for (int j=0; j < arrNoElements; j++)
        {
          if (j==0)
          {
            savingSentence.concat((int)mainArray[i][j]);
          }
          else if (j != 0 && j <= 12)
          {
            savingSentence.concat(';');
            savingSentence.concat(String((float)mainArray[i][j],7));            
          }
          else if (j > 12 && j <= 22)
          {
            savingSentence.concat(';');
            savingSentence.concat((int)mainArray[i][j]); 
          }
          else if (j > 22 && j <= arrNoElements-2)
          {
            if(j == 23) 
            {
                savingSentence.concat(';');
                savingSentence.concat(String((float)mainArray[i][j],5));
            }
            else if(j == (arrNoElements-2)) 
            {
                savingSentence.concat(',');
                savingSentence.concat(String((float)mainArray[i][j],5));
            }         
          }
          else
          {
            savingSentence.concat(';');
            savingSentence.concat(String((float)mainArray[i][j],2));
          }
        }
        
        myFile.println(savingSentence);
      }
      errorDuringSave = false;
    } else 
    {
      Serial.println("error opening .txt");
      backupSaveMode = true;
      errorDuringSave = true;
    }

    delay(10);
    myFile.close();

    savePositionAtArray = 0;

    // 5x time blink red
    if (errorDuringSave == true)
    {
      for (int iBlink = 0; iBlink <= 5; iBlink++) 
      {
          // RED BLINKG
          // analogWrite(czerwona, 0);
          // analogWrite(zielona, 255);
          // analogWrite(niebieska, 255);
          // delay(250);
          // analogWrite(czerwona, 255);
          // analogWrite(zielona, 255);
          // analogWrite(niebieska, 255);
          // delay(250);
          digitalWrite(czerwona, LOW);
          digitalWrite(zielona, HIGH);
          digitalWrite(niebieska, HIGH); 
          delay(250);
          digitalWrite(czerwona, LOW);
          digitalWrite(zielona, HIGH);
          digitalWrite(niebieska, HIGH); 
          delay(250);   
      }      
    }
  
  } else if (savePositionAtArray==arrSize && !saveMode)
  {
    savePositionAtArray = 0;
  }
 // Serial.print("At end: savePostion= ");
 // Serial.println(savePosition);
}

float calcArrayMediumValue(float ar[][26], int position, int sizeOfArray)
{
    float s = 0.00;
    for (int i_s=0; i_s< sizeOfArray; i_s++)
    {
        s += ar[i_s][position];
    }

    float m = s/sizeOfArray;
    // Serial.print("m= ");
    // Serial.println(m);
    return m;
}

float axisValueNormalize(int ar, float aCalibPlus, float aCalibMinus) {
  float arN;
  if (ar >= 0) {
    arN = ar * aCalibPlus;
  } else {
    //Serial.println("INSIDE Z-");
    //Serial.println(calib_Zminus, 7);
    //Serial.println((float)aZr, 7);
    arN = ar * aCalibMinus;
    //Serial.println(aZ, 7);
  }
  return arN;
}


void pinrt3Float(float x, float y, float z)
{
      Serial.print(x,7);
      Serial.print(";   ");
      Serial.print(y,7);
      Serial.print(";   ");
      Serial.print(z,7);
      Serial.println(";");
}

void pinrt3Int(int x, int y, int z)
{
      Serial.print(x);
      Serial.print(";");
      Serial.print(y);
      Serial.print(";");
      Serial.print(z);
      Serial.println(";");
}


void testSdCard()
{
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    while (1);
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  // print the type of card
  Serial.println();
  Serial.print("Card type:         ");

  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32

  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    while (1);
  }

  Serial.print("Clusters:          ");
  Serial.println(volume.clusterCount());
  Serial.print("Blocks x Cluster:  ");
  Serial.println(volume.blocksPerCluster());
  Serial.print("Total Blocks:      ");
  Serial.println(volume.blocksPerCluster() * volume.clusterCount());
  Serial.println();

  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("Volume type is:    FAT");
  Serial.println(volume.fatType(), DEC);
  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
  Serial.print("Volume size (Kb):  ");
  Serial.println(volumesize);
  Serial.print("Volume size (Mb):  ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (Gb):  ");
  Serial.println((float)volumesize / 1024.0);
  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);
  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);
  root.close();
}