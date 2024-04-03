#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <TMC2209.h>
#include "FS.h"
#include "SD.h"
#include "RTClib.h"

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire); //create the OLED Display
HardwareSerial & serial_stream = Serial2; //create the UART connection to stepper driver
TMC2209 stepper_driver; //create the driver
TMC2209::Settings settings = stepper_driver.getSettings();
TMC2209::Status status = stepper_driver.getStatus();
RTC_PCF8523 rtc;

#define BUTTON_A 34 //sel
#define BUTTON_B 4 //dec
#define BUTTON_C 37 //inc
#define BUTTON_D 32 //next
#define BUTTON_E 14  //before

#define BUTTON_1 15 //prime 
#define BUTTON_2 33 //33 = error!!! Start/stop //25=A1 so dont be hopeful it works.... possible to share cs pin?
#define BUTTON_3 27 //cal 
#define BUTTON_4 36 //cw/ccw

#define dirPin 12
#define stepPin 13 //led
#define batterySense 39 //V- Input only

#define cardSelect 33 //Note: This pin is used by the SD and
//shared by the Start/Stop button due to shortage of GPIO pins

int menu = 0;
int submenu = 0;
int pulseDays = 0;
int pulseHours = 0;
int pulseMinutes = 0;
int pulseSeconds = 0;
int pulseTimeTotal = 0;
int pulseTimeTotalReserve = 0;
double pulseVolume = 1.000; //Microliter presision, get preferred default value from SD card maybe?
double onceVolume = 1.000; //ml
double onceSpeed = 2.0;
int pulseRepeat = 0;
double pumpRate = 4.000; // Bring in from card settings "default pump mL/m"
int primeLength = 10; //cm bring from card
int primeTime = 0; //s
int maxLengthPerMinute = 60; //cm
int calTime = 60; //s
int reserve = calTime;
int shutdownTimer = 120; //s
double revolutionsPerformed = calTime/8; //calculate how many revolutions were performed during calibration period
double mlPerRevolution = 3.680/30; // figure out real number from SD card
double maxPumpRate = mlPerRevolution*(calTime/2); //~3.800ml/m
double mlPerEighth = mlPerRevolution/8;
bool direction = HIGH; //High = cw
double maxBatteryVoltage = 26.0; // pull from settings
bool disableShutdown = false;

//Motor Variables
const int stepsPerRevolution = 200;
double primeSpeed = 6;
int primeRotations = 5 * 8;
double primeVolume = (3.14159*1.5*1.5*(primeLength*100))/2000; //ml, 7ml for 10cm tube
double primeRevs = 1; //default


//TMC 2209 UART Variables
const long SERIAL_BAUD_RATE = 115200;
const int RX_PIN = 7;
const int TX_PIN = 8;
const int DELAY = 500;
const int32_t VELOCITY = 20000;
uint16_t microsteps_per_step = 16; //16
const uint8_t RUN_CURRENT_PERCENT = 40;
const uint8_t STOP_CURRENT_PERCENT = 1;
const TMC2209::CurrentIncrement CURRENT_INCREMENT = TMC2209::CURRENT_INCREMENT_8;
const TMC2209::MeasurementCount MEASUREMENT_COUNT = TMC2209::MEASUREMENT_COUNT_1;
const uint32_t COOL_STEP_DURATION_THRESHOLD = 2000;
const uint8_t COOL_STEP_LOWER_THRESHOLD = 1;
const uint8_t COOL_STEP_UPPER_THRESHOLD = 0;
bool cool_step_enabled = true;

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}


void setup() {
  Serial.begin(115200);
  
  Serial.begin(SERIAL_BAUD_RATE);

  //The following is from the Adafruit RTC example library
  #ifndef ESP8266
    while (!Serial); // wait for serial port to connect. Needed for native USB
  #endif

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (! rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC is NOT initialized, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    //
    // Note: allow 2 seconds after inserting battery or applying external power
    // without battery before calling adjust(). This gives the PCF8523's
    // crystal oscillator time to stabilize. If you call adjust() very quickly
    // after the RTC is powered, lostPower() may still return true.
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

  // When the RTC was stopped and stays connected to the battery, it has
  // to be restarted by clearing the STOP bit. Let's do this to ensure
  // the RTC is running.
  rtc.start();

   // The PCF8523 can be calibrated for:
  //        - Aging adjustment
  //        - Temperature compensation
  //        - Accuracy tuning
  // The offset mode to use, once every two hours or once every minute.
  // The offset Offset value from -64 to +63. See the Application Note for calculation of offset values.
  // https://www.nxp.com/docs/en/application-note/AN11247.pdf
  // The deviation in parts per million can be calculated over a period of observation. Both the drift (which can be negative)
  // and the observation period must be in seconds. For accuracy the variation should be observed over about 1 week.
  // Note: any previous calibration should cancelled prior to any new observation period.
  // Example - RTC gaining 43 seconds in 1 week
  float drift = 43; // seconds plus or minus over oservation period - set to 0 to cancel previous calibration.
  float period_sec = (7 * 86400);  // total obsevation period in seconds (86400 = seconds in 1 day:  7 days = (7 * 86400) seconds )
  float deviation_ppm = (drift / period_sec * 1000000); //  deviation in parts per million (μs)
  float drift_unit = 4.34; // use with offset mode PCF8523_TwoHours
  // float drift_unit = 4.069; //For corrections every min the drift_unit is 4.069 ppm (use with offset mode PCF8523_OneMinute)
  int offset = round(deviation_ppm / drift_unit);
  // rtc.calibrate(PCF8523_TwoHours, offset); // Un-comment to perform calibration once drift (seconds) and observation period (seconds) are correct
  // rtc.calibrate(PCF8523_TwoHours, 0); // Un-comment to cancel previous calibration

  Serial.print("Offset is "); Serial.println(offset); // Print to control offset



  stepper_driver.setup(serial_stream, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, RX_PIN, TX_PIN);

  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.setCoolStepCurrentIncrement(CURRENT_INCREMENT);
  stepper_driver.setCoolStepMeasurementCount(MEASUREMENT_COUNT);
  stepper_driver.setCoolStepDurationThreshold(COOL_STEP_DURATION_THRESHOLD);
  stepper_driver.setMicrostepsPerStep(microsteps_per_step);
  //stepper_driver.setHoldCurrent(STOP_CURRENT_PERCENT);
  stepper_driver.enableStealthChop();
  //stepper_driver.setMicrostepsPerStep(8);
  stepper_driver.enable();
  pinMode(13, OUTPUT);
  delay(300);



  
  Serial.println("128x64 OLED FeatherWing test");
  delay(250); // wait for the OLED to power up
  display.begin(0x3C, true); // Address 0x3C default

  Serial.println("OLED begun");

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  display.setRotation(1);

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  pinMode(BUTTON_D, INPUT_PULLUP);
  pinMode(BUTTON_E, INPUT_PULLUP);
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);
  pinMode(BUTTON_3, INPUT_PULLUP);
  pinMode(BUTTON_4, INPUT_PULLUP);

  digitalWrite(BUTTON_A, HIGH); //is this nescessary?
  digitalWrite(BUTTON_B, HIGH);
  digitalWrite(BUTTON_C, HIGH);
  digitalWrite(BUTTON_D, HIGH);
  digitalWrite(BUTTON_E, HIGH);
  digitalWrite(BUTTON_1, HIGH);
  digitalWrite(BUTTON_2, HIGH);
  digitalWrite(BUTTON_3, HIGH);
  digitalWrite(BUTTON_4, HIGH);


  pinMode(stepPin, OUTPUT);
	pinMode(dirPin, OUTPUT);

  //get and display the battery percentage
	pinMode(batterySense, INPUT);
  digitalWrite(batterySense, LOW);

  //add some logic to set the values of R1 and R2 to calculate 31.75 below:
  double voltage = analogRead(batterySense) * (33.2 / 4095.0); //31.75 is the maximum the voltage the divider can handle to be 3.3v
  int percentage = 0;
  Serial.println("Input Voltage: " + String(voltage));
  //Experimental battery percentage vs voltage equation: percentage = -5168.3x^2 + 10617x - 5350.5 (x=voltage/maxBatteryVoltage)
  //this equation is valid for perentages between 25-100%. The batteries should NEVER be below 25% or 11.75v each, 23.5v total.
  double b = voltage/maxBatteryVoltage; //ratio between measured voltage and the maximum voltage set by the user in the settings
  if(voltage >= 23.5){int(percentage = -5168.3*(b*b) + 10617*b - 5350.5);} //calculates the percentage of the batteries using the ratio 'b'

  // text display settings
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);
  display.println("Input V:");
  display.println(String(voltage) + " V");
  display.println("If Batt:");
  display.println(String(percentage) + "%");
  display.display(); // actually display all of the above
  digitalWrite(dirPin, direction); //High = Clockwise
  delay(2000);

  //setup the SD socket
  if(!SD.begin(cardSelect)){ //pin 33 for Feather
        Serial.println("Card Mount Failed");
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(0,0);
        display.println("No SD Card");
        display.println("Use 1GB SD");
        display.println("Format:");
        display.println("FAT32");
        display.display(); // actually display all of the above
        delay(6000);
        return;
    }
    uint8_t cardType = SD.cardType(); //save card type for debugging

    if(cardType == CARD_NONE){ //make sure there is a card inserted
        Serial.println("No SD card attached"); 
        return; //stop the program if there isnt one
    }

    Serial.print("SD Card Type: "); //report the type
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }
    /*

    char settings[2][10] = {char(mlPerRevolution), "Setting 2"}

    uint64_t cardSize = SD.cardSize() / (1024 * 1024); //report the card size
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
 */
    listDir(SD, "/", 0);
    createDir(SD, "/mydir"); //create default directory in case card is fresh
    listDir(SD, "/", 0);
    removeDir(SD, "/mydir");
    listDir(SD, "/", 2);
    writeFile(SD, "/settings.txt", "Hello ");
    appendFile(SD, "/settings.txt", "World!\n");
    readFile(SD, "/settings.txt");
    //deleteFile(SD, "/foo.txt");
    //renameFile(SD, "/hello.txt", "/foo.txt");
    //readFile(SD, "/foo.txt");
    //testFileIO(SD, "/test.txt");
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
    //writeFile(SD, "/hello.txt", "Hello ");
    //readFile(SD, "/hello.txt");
}

//a function that rotates the pump in increments of 1/8th of a revolution
void eighthPump(double eighths, double speed) { //speed should be 1=100%speed, 2=200%speed 0.5=50% etc.
  Serial.println("pumping " + String(eighths) + "/8 of a revolution.");
  int steps = stepsPerRevolution*2; //*eighths
  tempCheck();
  for(int y = 0; y < eighths; y++){
    for(int x = 0; x < steps; x++)
    {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(1000*(1/speed));
      digitalWrite(stepPin, LOW);
      delayMicroseconds(1000*(1/speed));
      if(digitalRead(BUTTON_2) == LOW) {break; menu=1; submenu=2;}
    }
    if(digitalRead(BUTTON_B) == LOW) {if ((menu == 1)&&(submenu == 1)){pumpRate = pumpRate - 0.05; break;}} //Dec
    if(digitalRead(BUTTON_C) == LOW) {if ((menu == 1)&&(submenu == 1)){pumpRate = pumpRate + 0.05; break;}} //Inc
    if(digitalRead(BUTTON_D) == LOW) {if ((menu == 1)&&(submenu == 1)){pumpRate = pumpRate + 1; break;}} //Next
    if(digitalRead(BUTTON_E) == LOW) {if ((menu == 1)&&(submenu == 1)){pumpRate = pumpRate - 1; break;}} //Before
  }
}

void velocityPump(double rate){ //rate is user input of ml/min
  double mlPerSecond = rate/60;
  double ratio = (mlPerSecond/mlPerRevolution)*(6.67);
  eighthPump(2048,ratio);
  
}

void batteryCheck(){
  double voltage = analogRead(batterySense) * (33.2 / 4095.0); //gets the battery voltage. typical range: 23-26V
  if(((voltage < 23.5) && (voltage > 5)) && (!disableShutdown)){ //23.5
    display.oled_command(SH110X_DISPLAYON);
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0,0);
    display.println("Battery");
    display.println("Critical!!");
    display.println(String(voltage) + " V");
    display.display(); // actually display all of the above
    for (int i = 0; i < 20; i++)
    {
      if(digitalRead(BUTTON_A) == LOW) 
      {
        disableShutdown = true;
        delay(200);
        menu = 0; submenu = 0;
        return;
      }
    delay(100);
    }
    display.clearDisplay();
    display.setContrast(0);
    display.display();
    display.oled_command(SH110X_DISPLAYOFF);
    stepper_driver.enableStealthChop();
    stepper_driver.setHoldCurrent(STOP_CURRENT_PERCENT);
    stepper_driver.disable();
    esp_sleep_enable_timer_wakeup(86400e6);//sleep for a day
    esp_deep_sleep_start();
  }

}

void tempCheck(){
    if(status.over_temperature_120c == 1){
      delay(120); //Wait 2 minutes if the driver is overheating.
      Serial.println("Driver Overheating, Waiting 2 Minutes.");
    }
}

int pastSecond = 0;
bool secondPassed(){
  DateTime now = rtc.now();
  int currentSecond = now.second();
  if((currentSecond > pastSecond) || ((currentSecond == 0)&&(pastSecond == 59)))
  {
    pastSecond = currentSecond;
    Serial.print("Second Passed: ");
    Serial.println(currentSecond);
    return true;
  }
  else
  {
    return false;
  }
}

uint8_t i=0;
void loop() {
  DateTime now = rtc.now();

  if ((disableShutdown) && (secondPassed())){ //Allows the user 120 seconds to use the pump if shutdown is disabled
      shutdownTimer = shutdownTimer - 1;
    if (shutdownTimer == 0){
      disableShutdown = false;
      shutdownTimer = 120;
    }
  }
  /*
  if(secondPassed()){
    stepper_driver.disableStealthChop();
    eighthPump(4,8);
    stepper_driver.enableStealthChop();
    }
  */
  /*
  Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print('/');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
  */
  display.oled_command(SH110X_DISPLAYON);
  double voltage = analogRead(batterySense) * (33.2 / 4095.0); //gets the battery voltage. typical range: 23-26V
  batteryCheck();
  tempCheck();
  //Serial.println(String(menu) + " " + String(submenu));

  if (not stepper_driver.isSetupAndCommunicating())
  {
    Serial.println("Stepper driver not setup and communicating. ESP may have crashed.");
    //return;
  }

  if (cool_step_enabled)
    {
      stepper_driver.enableCoolStep(COOL_STEP_LOWER_THRESHOLD,COOL_STEP_UPPER_THRESHOLD);
    }
    else
    {
      stepper_driver.disableCoolStep();
    }

  int wait = 300;
  if(digitalRead(BUTTON_A) == LOW) {
    Serial.println("Sel");
    if ((menu == 0) && (submenu == 0)) {menu = 1; submenu = 0;}
    else if ((menu == 0) && (submenu == 1)) {menu = 2; submenu = 0;}
    else if ((menu == 4) && (submenu == 2)) {menu = 4; submenu = 3;}
    else if ((menu == 0) && (submenu == 3)) {menu = 3; submenu = 0;}
    else if ((menu == 3) && (submenu == 0)) {menu = 6; submenu = 0;}
    else if ((menu == 3) && (submenu == 1)) {menu = 7; submenu = 0;}
    else if ((menu == 3) && (submenu == 2)) {menu = 8; submenu = 0;}
    else if ((menu == 2) && (submenu == 4)) {menu = 2; submenu = 5;}
    else if ((menu == 3) && (submenu == 3)) {menu = 9; submenu = 0;}
    else if ((menu == 9) && (submenu == 0)) {menu = 0; submenu = 0;}
    else if ((menu == 0) && (submenu == 2)) {menu = 10; submenu = 0;}
    else if ((menu == 10) && (submenu == 0)) {menu = 10; submenu = 2;}
    else if ((menu == 10) && (submenu == 2)) {menu = 10; submenu = 0;}

    delay(wait); }
  
  if(digitalRead(BUTTON_B) == LOW) {
    Serial.println("Dec");
    if((menu == 1) && (submenu == 0)){pumpRate = pumpRate - 0.005;}
    else if((menu == 2) && (submenu == 0) && (pulseDays > 0)) {pulseDays = pulseDays - 1;}
    else if((menu == 2) && (submenu == 1) && (pulseHours > 0)) {pulseHours = pulseHours - 1;}
    else if((menu == 2) && (submenu == 2) && (pulseMinutes > 0)) {pulseMinutes = pulseMinutes - 1;}
    else if((menu == 2) && (submenu == 3) && (pulseSeconds > 0)) {pulseSeconds = pulseSeconds - 1;}
    else if((menu == 2) && (submenu == 4) && (pulseVolume > 0)) {pulseVolume = pulseVolume - 0.005;}
    else if((menu == 2) && (submenu == 5)) {pulseRepeat = pulseRepeat - 1;}
    else if((menu == 4) && (submenu == 2)) {maxPumpRate = maxPumpRate - 0.005;}
    else if((menu == 10) && (submenu == 0)) {onceVolume = onceVolume - 0.005;}
    else if((menu == 10) && (submenu == 2)) {onceSpeed = onceSpeed - 0.05;}
    else if((menu == 6) && (submenu == 0) && (primeLength > 0)) {primeLength = primeLength - 1;}
    else if((menu == 7) && (submenu == 0) && (calTime > 60)) {calTime = calTime - 60;}
    delay(wait); }

  if(digitalRead(BUTTON_C) == LOW) {
    Serial.println("Inc");
    if((menu == 1) && (submenu == 0)) {pumpRate = pumpRate + 0.005;}
    else if((menu == 2) && (submenu == 0)) {pulseDays = pulseDays + 1;}
    else if((menu == 2) && (submenu == 1)) {pulseHours = pulseHours + 1;}
    else if((menu == 2) && (submenu == 2)) {pulseMinutes = pulseMinutes + 1;}
    else if((menu == 2) && (submenu == 3)) {pulseSeconds = pulseSeconds + 1;}
    else if((menu == 2) && (submenu == 4)) {pulseVolume = pulseVolume + 0.005;}
    else if((menu == 2) && (submenu == 5)) {pulseRepeat = pulseRepeat + 1;}
    else if((menu == 4) && (submenu == 2)) {maxPumpRate = maxPumpRate + 0.005;}
    else if((menu == 10) && (submenu == 0)) {onceVolume = onceVolume + 0.005;}
    else if((menu == 10) && (submenu == 2)) {onceSpeed = onceSpeed + 0.05;}
    else if((menu == 6) && (submenu == 0)) {primeLength = primeLength + 1;}
    else if((menu == 7) && (submenu == 0)) {calTime = calTime + 60;}
    delay(wait); }

  if(digitalRead(BUTTON_D) == LOW) {
    Serial.println("Next");
    if((menu != 4) && (menu != 5) && (menu != 1) && ((menu != 2) && (menu != 9) || (submenu != 4) && (submenu != 5)) && (menu != 10))
      submenu = submenu + 1;
    else if((menu == 4) && (submenu == 2)) {maxPumpRate = maxPumpRate + 1.000;}
    else if((menu == 1) && (submenu == 0)) {pumpRate = pumpRate + 1.000;}
    else if((menu == 2) && (submenu == 4)) {pulseVolume = pulseVolume + 1.000;}
    else if((menu == 10) && (submenu == 0)) {onceVolume = onceVolume + 1.000;}
    else if((menu == 10) && (submenu == 2)) {onceSpeed = onceSpeed + 1.000;}
    delay(wait); }

  if(digitalRead(BUTTON_E) == LOW) {
    Serial.println("Before");
    if((menu != 4) && (menu != 5) && (menu != 1) && ((menu != 2) && (menu != 9) || ((submenu != 4) && (submenu = !5))) && (menu != 10))
      submenu = submenu - 1;
    else if((menu == 4) && (submenu == 2)) {maxPumpRate = maxPumpRate - 1.000;}
    else if((menu == 1) && (submenu == 0)) {pumpRate = pumpRate - 1.000;}
    else if((menu == 2) && (submenu == 4)) {pulseVolume = pulseVolume - 1.000;}
    else if((menu == 10) && (submenu == 0)) {onceVolume = onceVolume - 1.000;}
    else if((menu == 10) && (submenu == 2)) {onceSpeed = onceSpeed - 1.000;}
    delay(wait); }

  if(digitalRead(BUTTON_1) == LOW) {
    Serial.println("Prime");
    menu = 5;
    submenu = 0;
    delay(wait);}

  if(digitalRead(BUTTON_2) == LOW) {
    Serial.println("Start");
    if((menu == 4) && (submenu == 0)){submenu = 1;}
    else if((menu == 5) && (submenu == 0)){submenu = 1;}
    else if((menu == 1) && (submenu == 0)){submenu = 1;}
    else if((menu == 1) && (submenu == 1)){submenu = 2;}
    else if((menu == 2) && (submenu == 5)){submenu = 6;}
    else if((menu == 9) && (submenu == 0)){submenu = 1;}
    else if((menu == 10) && (submenu == 0)){submenu = 1;}
    else if((menu == 10) && (submenu == 2)){submenu = 1;}
    delay(wait);}
  if(digitalRead(BUTTON_3) == LOW) {
   Serial.println("Cal");
    menu = 4;
    submenu = 0;
    delay(wait);}
  if(digitalRead(BUTTON_4) == LOW) {
   Serial.println("CWCCW");
   direction = !direction;
   digitalWrite(dirPin, direction); //High = Clockwise
   delay(wait);}
  yield();
  display.display();
  //Serial.println("Menu: " + String(menu) + " Submenu : " + String(submenu));

  switch(menu) {
    case 0: //Main Menu
      if(submenu == 0) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Mode:");
        display.println("Constant\nFlow");
        display.println("Push Sel");
        
      }
      else if(submenu == 1) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Mode:");
        display.println("Pulse Flow");
        display.println("Push Sel");
      }
      else if(submenu == 2) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Mode:");
        display.println("Once Pump");
        display.println("Push Sel");
      }
      else if(submenu == 3) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Mode:");
        display.println("Settings");
        display.println("Push Sel");
      }
      if(submenu == 4) {
        submenu = 3;
      }
      if(submenu == -1) {
        submenu = 0;
      }
      break;
    case 1: //Constant Flow Menu
      if(submenu == 0) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Enter mL/m");
        display.println(" " + String(pumpRate, 3));
        display.println("Max:" + String(maxPumpRate));
        display.println("Push Start");
        stepper_driver.disableStealthChop();
      }
      if(submenu == 1) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println(" Pumping:");
        display.println(" " + String(pumpRate, 3));
        display.println("Hold Stop");
        display.display();
        velocityPump(pumpRate);
      }
      if(submenu == 2) {
        stepper_driver.enableStealthChop();
        stepper_driver.setHoldCurrent(STOP_CURRENT_PERCENT);
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Stopping..");
        display.display();
        delay(1000);
        menu = 0; submenu = 0;
      }
      if(submenu == 3) {
        submenu = 2;
      }
      if(submenu == -1) {
        submenu = 0;
      }
      break;
    case 2: //Pulse Flow Menu
      if(submenu == 0) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Pulse Time");
        display.println("Days: " + String(pulseDays));
        display.println("Push Next");
      }
      if(submenu == 1) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Pulse Time");
        display.println("Hours: " + String(pulseHours));
        display.println("Push Next");
      }
      if(submenu == 2) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Pulse Time");
        display.println("Mins: " + String(pulseMinutes));
        display.println("Push Next");
      }
      if(submenu == 3) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Pulse Time");
        display.println("Secs: " + String(pulseSeconds));
        display.println("Push Next");
      }
      if(submenu == 4) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Pump Vol");
        display.println(String(pulseVolume, 3) + " mL");
        display.println("Push Sel");
        pulseRepeat = 0;
      }
      if(submenu == 5) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Repeat");
        if(pulseRepeat == 0) {display.println("Inf Times");}
        else{display.println(String(pulseRepeat) + " Times");}
        display.println("Push Start");
        if(pulseRepeat < 0) {pulseRepeat = 0;}
      }
      if(submenu == 6) {
        display.clearDisplay();
        display.setCursor(0,0);
        pulseTimeTotal = (pulseDays*86400)+(pulseHours*3600)+(pulseMinutes*60)+pulseSeconds;
        pulseTimeTotalReserve = pulseTimeTotal;
        if(pulseRepeat == 0) {display.println("Pumping Inf");}
        else{display.println("Pump " + String(pulseRepeat)) + "x";}
        display.println("every " + String(pulseTimeTotal));
        display.println("seconds...");
        display.display();
        delay(2000);
        submenu=7;
      }
      if(submenu == 7) {
        bool powerSaver = false;
        double numEighths = pulseVolume/mlPerEighth; //pump once before starting (requested)
        stepper_driver.disableStealthChop();
        stepper_driver.setHoldCurrent(STOP_CURRENT_PERCENT);
        eighthPump(numEighths,2.5);
        stepper_driver.enableStealthChop();

        int j = 0; int k = 0;
        if(pulseRepeat == 0) {pulseRepeat = 99999999999999;}
        for (int j = 0; j < pulseRepeat; j++)
        {
          if(powerSaver){
            display.oled_command(SH110X_DISPLAYOFF);
            esp_sleep_enable_timer_wakeup(pulseTimeTotal*1e6);//5 seconds
            batteryCheck();
            esp_light_sleep_start();
            double numEighths = pulseVolume/mlPerEighth; 
            stepper_driver.disableStealthChop();
            eighthPump(numEighths,2.5);
            stepper_driver.enableStealthChop();
            stepper_driver.setHoldCurrent(STOP_CURRENT_PERCENT);

            if(digitalRead(BUTTON_2) == LOW) {break;}
          }
          else{
          for (int k = 0; k <= pulseTimeTotal; k++)
          {
          display.clearDisplay();
          display.setContrast(10);
          display.setCursor(0,0);
          display.println(String(pulseTimeTotal-k) + " s");
          display.println("remaining.");
          display.println("Hold Stop");
          display.setTextSize(1);
          display.println("Hold Sel for 1 sec");
          display.println("to enter sleep mode.");
          display.setTextSize(2);
          display.display();
          delay(1000);
          if(digitalRead(BUTTON_2) == LOW) {break;}
          if(digitalRead(BUTTON_A) == LOW) {//this detects if the user wants power saving on, if they do, the time remaining must finish in sleep
            powerSaver = true;
            Serial.println("power save enabled");
            display.oled_command(SH110X_DISPLAYOFF);
            esp_sleep_enable_timer_wakeup((pulseTimeTotal-k)*1e6);//1 second
            esp_light_sleep_start();
            Serial.println("power save finished");
            k = pulseTimeTotal;
            Serial.println(k);
            }
        }//for
        if(digitalRead(BUTTON_2) == LOW) {menu = 1; submenu = 2; display.setContrast(100); break;}

        double numEighths = pulseVolume/mlPerEighth; 
        stepper_driver.disableStealthChop();
        eighthPump(numEighths,2.5);
        stepper_driver.enableStealthChop();
        menu = 0; submenu = 0; display.setContrast(100);
          }//else
        }//for
      }
      if(submenu == 8) {
        submenu = 7;
      }
      if(submenu == -1) {
        submenu = 0;
      }
      break;
    case 3: //settings selection
      if(submenu == 0) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Prime");
        display.println("Length");
        display.println("Push Sel");
      }
      if(submenu == 1) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Cal Time");
        display.println("Push Sel");
      }
      if(submenu == 2) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("HW Info");
        display.println("Push Sel");
      }
      if(submenu == 3) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Battery");
        display.println("Cal.");
        display.println("Push Sel");
      }
      if(submenu == 4) {
          submenu = 3;
      }
      if(submenu == -1) {
          submenu = 0;
      }
      break;
    case 4: //calibration procedure
      if(submenu == 0) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Calibrate");
        display.println("Push Start");
        reserve = calTime;
      }
      if(submenu == 1) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Calibrate");
        display.println(String(calTime / 60) + " m " + String(calTime % 60) + " s");
        display.println("remaining");
        display.display();
        if(digitalRead(BUTTON_2) == LOW) {menu=1; submenu=2;}
          
        if(secondPassed()){
        stepper_driver.disableStealthChop();
        eighthPump(4,8);//pump one half revolution for exactly 1 second. 1/8 = 0.8333 speed
        stepper_driver.enableStealthChop();
        calTime = calTime - 1;
        //stepper_driver.setHoldCurrent(STOP_CURRENT_PERCENT);
        }
        if(calTime == 0) {submenu = 2; calTime = reserve; maxPumpRate = maxPumpRate * (reserve / 60);} //pull this 60 from sd card "calibration time"
      }
      if(submenu == 2) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Enter Vol");
        display.println(String(maxPumpRate,3) + " mL");
        display.println("Press Sel");
      }
      if(submenu == 3) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Saving...");
        display.display();
        calTime = reserve;
        maxPumpRate = maxPumpRate/(calTime/60); //1  min cal remains unchanged. 2 min divides by 2
        mlPerRevolution = maxPumpRate/(calTime/2); //calculate how many mL per revolution
        mlPerEighth = mlPerRevolution/8;
        //Write to SD card. Update Settings: mlPerRevolution
        delay(500);
        display.println("Done!");
        display.display();
        delay(1000);
        menu = 0;
        submenu = 0;
      }
        if(submenu == 4) {
          submenu = 3;
        }
        if(submenu == -1) {
          submenu = 0;
        }
      break;
    case 5: //prime procedure
      if(submenu == 0) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Prime");
        display.println("Push Start");
      }
      if(submenu == 1) {
        display.clearDisplay();
        display.setCursor(0,0);
        primeRevs = ((primeVolume/10)/mlPerEighth);
        display.println("Priming...");
        display.println("..........");
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        stepper_driver.disableStealthChop();
        eighthPump(primeRotations,primeSpeed);
        eighthPump(primeRevs,4);

        display.println("Priming...");
        display.println("#.........");
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        eighthPump(primeRevs,4);

        display.println("Priming...");
        display.println("##........");
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        eighthPump(primeRevs,4);

        display.println("Priming...");
        display.println("###.......");
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        eighthPump(primeRevs,4);

        display.println("Priming...");
        display.println("####......");
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        eighthPump(primeRevs,4);

        display.println("Priming...");
        display.println("#####.....");
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        eighthPump(primeRevs,4);

        display.println("Priming...");
        display.println("######....");
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        eighthPump(primeRevs,4);

        display.println("Priming...");
        display.println("#######...");
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        eighthPump(primeRevs,4);

        display.println("Priming...");
        display.println("########..");
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        eighthPump(primeRevs,4);

        display.println("Priming...");
        display.println("#########.");
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        eighthPump(primeRevs,4);

        display.println("Priming...");
        display.println("##########");
        display.display();
        display.clearDisplay();
        display.setCursor(0,0);
        eighthPump(primeRevs,4);

        stepper_driver.enableStealthChop();
        stepper_driver.setHoldCurrent(STOP_CURRENT_PERCENT);
        display.println("Done!");
        display.println("Primed ");
        display.println(String(primeLength) + " cm");
        display.display();
        delay(2000);
        menu = 0;
        submenu = 0;
      }
      break;
    case 6: //prime length
      if(submenu == 0) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Enter");
        display.println("Length");
        display.println(String(primeLength) + " cm");
        display.println("Push Next");
      }
      if(submenu == 1) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Saving...");
        display.display();
        //Write to SD card
        delay(500);
        display.println("Done!");
        display.display();
        delay(1000);
        menu = 0;
        submenu = 0;
      }
        if(submenu == 2) {
          submenu = 1;
        }
        if(submenu == -1) {
          submenu = 0;
        }
      break;
    case 7: //calibration time setting
      if(submenu == 0) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Enter");
        display.println("cal time:");
        display.println(String(calTime/60) + " min");
        display.println("Push Next");
      }
      if(submenu == 1) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Saving...");
        display.display();
        //Write to SD card
        delay(500);
        display.println("Done!");
        display.display();
        delay(1000);
        menu = 0;
        submenu = 0;
      }
        if(submenu == 2) {
          submenu = 1;
        }
        if(submenu == -1) {
          submenu = 0;
        }
      break;
    case 8: //HW info setting
      if(submenu == 0) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Push Next");
      }
      if(submenu == 1) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(1);
        display.println("FWV: 1.0.b");
        display.println("BTT TMC 2209 V1.3");
        display.println("Controller:");
        display.println("Adafruit Fether ESPV2");
        display.printf("Total SD space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
        display.printf("Used SD space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
        display.println("Push Next");
        display.setTextSize(2);
      }

        if(submenu == 2) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(2);
        menu = 0;
        submenu = 0;
        }
        if(submenu == 3) {
          submenu = 2;
        }
        if(submenu == -1) {
          submenu = 0;
        }
      break;
    case 9: //Battery calibration menu
      if(submenu == 0) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(1);
        display.println("Battery Calibration");
        display.println("Ensure BOTH batteries");
        display.println("are fully charged.");
        display.println("Press Start to Start.");
        display.println("Press Sel to cancel.");
        display.println("Current Max: " + String(maxBatteryVoltage));
        display.println("Battery V: " + String(voltage));
        display.setTextSize(2);
        display.display();
        delay(250);
      }
      if(submenu == 1) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(1);
        display.println("Begin Calibration");
        display.println("Do not turn off pump.");
        display.display();
        delay(4000); //allow battery voltage to stabilize
        maxBatteryVoltage = voltage;
        display.println("Calibration Finished!");
        display.println("Final Voltage:");
        display.println(maxBatteryVoltage);
        display.display();
        delay(6000);
        display.setTextSize(2);
        menu = 0; submenu = 0; 
      }
        if(submenu == 3) {
          submenu = 2;
        }
        if(submenu == -1) {
          submenu = 0;
        }
      break;
    case 10: //Once Pump menu
      if(submenu == 0) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(2);
        display.println("Push Start");
        display.println("to pump:");
        display.println(String(onceVolume,3) + " ml");
        display.setTextSize(1);
        display.println("Push Sel to set speed");
        display.setTextSize(2);
        display.display();
      }
      if(submenu == 1) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(2);
        display.println("Pumping");
        display.println(String(onceVolume,3) + " at");
        display.println(String(onceSpeed));
        display.display();
        double numEighths = onceVolume/mlPerEighth; 
        stepper_driver.disableStealthChop();
        eighthPump(numEighths,onceSpeed);
        stepper_driver.enableStealthChop();
        display.setTextSize(2);
        display.println("Done!");
        display.display();
        delay(1000);
        menu = 10; submenu = 0; 
      }
      if(submenu == 2) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(2);
        display.println("Speed:");
        display.println(String(onceSpeed,3));
        display.setTextSize(1);
        display.println("Press Sel to set volume");
        display.setTextSize(2);
        display.display();
      }
        if(submenu == 3) {
          submenu = 2;
        }
        if(submenu == -1) {
          submenu = 0;
        }
      break;
  } //switch
}