
/*
Based on:
https://github.com/empierre/arduino/blob/master/SoilMoistSensor.ino
https://github.com/javos65/WDTZero
Installation process:
    Unzip the downloaded .zip file
    Sketch > Include Library > Add .ZIP Library
    Select the WDTZero subfolder of the unzipped folder (not the .zip file).
    Click the "Open" button.
    Despite the name, Add .ZIP Library can be used to install libraries from folders as well as .zip files.
This version does:
- reads wp and temperature data
- mesures battery voltage
- has several led information
- sends the data to H2Optimum

*/

#include <SparkFun_RV8803.h> //Get the library here:http://librarymanager/All#SparkFun_RV-880
RV8803 rtc;


//Make sure to change these values to the decimal values that you want to match
uint8_t minuteAlarmValue = 59; //0-60, change this to a minute or two from now to see the alarm get generated
uint8_t hourAlarmValue = 0; //0-24
uint8_t weekdayAlarmValue = 0; //Or together days of the week to enable the alarm on those days.
uint8_t dateAlarmValue = 0; //1-31

//Define which alarm registers we want to match, make sure you only enable weekday or date alarm, enabling both will default to a date alarm
//In its current state, an alarm will be generated once an hour, when the MINUTES registers on the time and alarm match. Setting MINUTE_ALARM_ENABLE to false would trigger an alarm every minute
#define MINUTE_ALARM_ENABLE true
#define HOUR_ALARM_ENABLE false
#define WEEKDAY_ALARM_ENABLE false
#define DATE_ALARM_ENABLE false

//NB Lybrary
#include <MKRNB.h>

#include "secrets.h"

const char PINNUMBER[]     = SECRET_PINNUMBER;
// APN data
const char GPRS_APN[]      = SECRET_GPRS_APN;

// initialize the library instance
NBClient client(false);

GPRS gprs;
NB nbAccess(1);

// URL, path and port (for example: arduino.cc)
char server[] = "api.2adapt.pt";
int port = 80; // port 80 is the default for HTTP
String cmac ="01:00:00:00:00:11";


//////

#if !( defined(ARDUINO_SAMD_ZERO) || defined(ARDUINO_SAMD_MKR1000) || defined(ARDUINO_SAMD_MKRWIFI1010) \
      || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_SAMD_MKRFox1200) || defined(ARDUINO_SAMD_MKRWAN1300) || defined(ARDUINO_SAMD_MKRWAN1310) \
      || defined(ARDUINO_SAMD_MKRGSM1400) || defined(ARDUINO_SAMD_MKRNB1500) || defined(ARDUINO_SAMD_MKRVIDOR4000) \
      || defined(ARDUINO_SAMD_CIRCUITPLAYGROUND_EXPRESS) || defined(__SAMD51__) || defined(__SAMD51J20A__) \
      || defined(__SAMD51J19A__) || defined(__SAMD51G19A__) || defined(__SAMD51P19A__)  \
      || defined(__SAMD21E15A__) || defined(__SAMD21E16A__) || defined(__SAMD21E17A__) || defined(__SAMD21E18A__) \
      || defined(__SAMD21G15A__) || defined(__SAMD21G16A__) || defined(__SAMD21G17A__) || defined(__SAMD21G18A__) \
      || defined(__SAMD21J15A__) || defined(__SAMD21J16A__) || defined(__SAMD21J17A__) || defined(__SAMD21J18A__) )
  #error This code is designed to run on SAMD21/SAMD51 platform! Please check your Tools->Board setting.
#endif

/////////////////////////////////////////////////////////////////

// These define's must be placed at the beginning before #include "SAMDTimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
// Don't define TIMER_INTERRUPT_DEBUG > 2. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     1

// Select only one to be true for SAMD21. Must must be placed at the beginning before #include "SAMDTimerInterrupt.h"
#define USING_TIMER_TC3         true      // Only TC3 can be used for SAMD51
#define USING_TIMER_TC4         false     // Not to use with Servo library
#define USING_TIMER_TC5         false
#define USING_TIMER_TCC         false
#define USING_TIMER_TCC1        false
#define USING_TIMER_TCC2        false     // Don't use this, can crash on some boards

// Uncomment To test if conflict with Servo library
//#include "Servo.h"

/////////////////////////////////////////////////////////////////

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "SAMDTimerInterrupt.h"

#ifndef LED_BUILTIN
  #define LED_BUILTIN       13
#endif

volatile uint32_t preMillisTimer = 0;

//////////////////////////////////////////////

// TC3, TC4, TC5 max permissible TIMER_INTERVAL_MS is 1398.101 ms, larger will overflow, therefore not permitted
// Use TCC, TCC1, TCC2 for longer TIMER_INTERVAL_MS
#define TIMER_INTERVAL_MS        1000

volatile uint32_t preMillisTimer1 = 0;

#if USING_TIMER_TC3
  #define SELECTED_TIMER      TIMER_TC3
#elif USING_TIMER_TC4
  #define SELECTED_TIMER      TIMER_TC4
#elif USING_TIMER_TC5
  #define SELECTED_TIMER      TIMER_TC5
#elif USING_TIMER_TCC
  #define SELECTED_TIMER      TIMER_TCC
#elif USING_TIMER_TCC1
  #define SELECTED_TIMER      TIMER_TCC1
#elif USING_TIMER_TCC2
  #define SELECTED_TIMER      TIMER_TCC
#else
  #error You have to select 1 Timer  
#endif

// Init selected SAMD timer
SAMDTimer ITimer(SELECTED_TIMER);

//////////////////////////////////////////////

int timeInterval       = 0;

void disconnecting()//disconnecting to network
{
  
// CLEAR FLAGS AND SET NEW INTERRUPT
  Serial.println("Alarm Triggered, clearing flag");
  
  // JUST FOR TESTING START
   rtc.set24Hour();
   String ReadMinutes;
   if (rtc.updateTime() == true) //Updates the time variables from RTC
  {
  ReadMinutes = rtc.getMinutes();
  Serial.print("Current Minutes ");
  Serial.println(ReadMinutes);
  }

  int Minutes = ReadMinutes.toInt();

  minuteAlarmValue = Minutes + 2 ; //0-60, change this to a minute or two from now to see the alarm get generated
  
  if(minuteAlarmValue>=60){
    minuteAlarmValue = 2;
    
  }
  Serial.print("Next alarm at: ");
  Serial.println(minuteAlarmValue);
  // JUST FOR TESTING END
 
  //rtc.disableAllInterrupts();
  
  
  rtc.setItemsToMatchForAlarm(MINUTE_ALARM_ENABLE, HOUR_ALARM_ENABLE, WEEKDAY_ALARM_ENABLE, DATE_ALARM_ENABLE); //The alarm interrupt compares the alarm interrupt registers with the current time registers. We must choose which registers we want to compare by setting bits to true or false
  rtc.setAlarmMinutes(minuteAlarmValue);
  //rtc.setAlarmHours(hourAlarmValue);
  //rtc.setAlarmWeekday(weekdayAlarmValue);
  rtc.enableHardwareInterrupt(ALARM_INTERRUPT);

  rtc.clearAllInterruptFlags();//Clear all flags in case any interrupts have occurred.

}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void TimerHandler()
{
  static bool toggle = false;

  //Serial.println(toggle? "ON" : "OFF");

  //timer interrupt toggles pin LED_BUILTIN
  digitalWrite(LED_BUILTIN, toggle);
  toggle = !toggle;
  
  if(timeInterval == 60){

    Serial.println(rtc.getInterruptFlag(FLAG_ALARM));
    Serial.println(timeInterval);
    timeInterval = 0;
    
    //resetFunc();  //call reset
    disconnecting();
  }
  timeInterval++;
}

//////////////////////////////////////////////


// LEDs lybrary
#include <SparkFun_PCA9536_Arduino_Library.h>
PCA9536 io;


#include <SPI.h>
//attachr SD lib
#include <SD.h>
File myFile;



// Temperature Lybrary
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS_T1 7  // DS18B20 pin
OneWire oneWireT1(ONE_WIRE_BUS_T1);
DallasTemperature DS18B20T1(&oneWireT1);




#include <math.h>       // Conversion equation from resistance to %

// Setting up format for reading 3 soil sensors
#define NUM_READS 10    // Number of sensor reads for filtering
long buffer[NUM_READS];
int indice;

typedef struct {        // Structure to be used in percentage and resistance values matrix to be filtered (have to be in pairs)
  int moisture;
  long resistance;
} values;


const long knownResistor = 4700;  // Constant value of known resistor in Ohms

int supplyVoltage;                // Measured supply voltage
int sensorVoltage;                // Measured sensor voltage

values valueOf[NUM_READS];        // Calculated moisture percentages and resistances to be sorted and filtered


int i;                            // Simple indice variable

int WaterPotential1;
int WaterPotential2;
int WaterPotential3;

int TIME_BETWEEN_READINGS; //default time between poweer saving mode

float betterychargesignal = 0;
float batteryVoltage;
int batteryMode = 0;
String readServerResponse;

void setup() {

  //START RTC
  Wire.begin();
  
  delay(200);
  


 //Initialize serial and wait for port to open:
  Serial.begin(9600);
  Serial.println("Set Time on RTC");

 //START RTC
  if (rtc.begin() == false)
  {
    Serial.println("Something went wrong, check wiring on the RTC");
    while(1);
  }
    Serial.println("RTC online!");
    

  //error_shutdow_func();


  
  //Using built in led to wakeup from deepsleep
  pinMode(LED_BUILTIN, OUTPUT);
  
  //SAMD_TimerInterrupt timer code
  //digitalWrite(LED_BUILTIN, LOW);
  
  Serial.print(F("\nStarting Argument_None on ")); Serial.println(BOARD_NAME);
  Serial.println(SAMD_TIMER_INTERRUPT_VERSION);
  Serial.print(F("CPU Frequency = ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));
  
  // Interval in millisecs
  if (ITimer.attachInterruptInterval_MS(TIMER_INTERVAL_MS, TimerHandler))
  {
    preMillisTimer = millis();
    Serial.print(F("Starting  ITimer OK, millis() = ")); Serial.println(preMillisTimer);
  }
  else {
    Serial.println(F("Can't set ITimer. Select another freq. or timer"));
  }

  
  //LEDs OFF
  

  // Initialize the PCA9536 with a begin function
  if (io.begin() == false)
  {
    Serial.println("PCA9536 not detected. Please check wiring. Freezing...");
    while (1)
      ;
  }
  
  //LEDs START CHECK
   for (int i = 0; i < 4; i++)
  {
    // pinMode can be used to set an I/O as OUTPUT or INPUT
    io.pinMode(i, OUTPUT);
    delay(1000);
    io.write(i, LOW);
    delay(1000);
  }
  
  //LEDs

  
  // Energy pin of temperature sensor
   pinMode(6, OUTPUT);
   
  //watermark readings start
  // initialize the digital pins as an output.

  // Digital Pins: 5,4 Analog pins: A1,A2 for sensor 1
  pinMode(5, OUTPUT);    
  pinMode(4, OUTPUT);  

  // Digital Pins: 3,2 Analog pins: A3,A4 for sensor 2
  pinMode(3, OUTPUT);    
  pinMode(2, OUTPUT); 

  // Digital Pins: 1,0 Analog pins: A5,A6 for sensor 3
  pinMode(1, OUTPUT);    
  pinMode(0, OUTPUT);
  
  Serial.println ("Starting readings ;-)" );

}

void loop() {

  stateOne();
     
   
    delay(10000);//just to give some time to upload the sketch to the controler before it connects to the internet or goes to sleep 

    
    
    
    //error_shutdow_func();
    
    String rtc_Date = GetTime();
    
     //send data to H2Optimum  
    nbConnect();

    delay(3000);
    //lightsOff();
    
    delay(1000);
    //rtc.standbyMode();
    //disconnecting();
    
}


void error_shutdow_func()
{
  // TURNS OFF IF WATCHDOG RESTARTED AFTER TRYING 20MIN TO CONNECT
    rtc.set24Hour();
    String ReadMinutes;
    if (rtc.updateTime() == true) //Updates the time variables from RTC
    {
    ReadMinutes = rtc.getMinutes();
    Serial.print("Current Minutes ");
    Serial.println(ReadMinutes);
    }
  
    int Minutes = ReadMinutes.toInt();
  
    if(Minutes >= 25){
      
      disconnecting();
  
      }
    
    delay(1000);
}


String GetTime()
{
   
   
  String RTCtime;
   
   rtc.set24Hour();
   if (rtc.updateTime() == true) //Updates the time variables from RTC
  {
    //String currentDate = rtc.stringDateUSA(); //Get the current date in mm/dd/yyyy format (we're weird)
    String currentDate = rtc.stringDate(); //Get the current date in dd/mm/yyyy format
    String currentTime = rtc.stringTime(); //Get the time
    Serial.print(currentDate);
    Serial.print(" ");
    Serial.println(currentTime);
    RTCtime = currentDate + " " + currentTime;
  }
  else
  {
    Serial.print("RTC read failed");
  }
  
  return RTCtime;
}









void nbConnect(){
  
  Serial.println("Starting Arduino web client.");
   
  // connection state
  boolean connected = false;

  //added "bool restart = false" on 18/06/2022 for testing
  bool restart = false;
  

  connecting();
    
  int notConnectedCounter = 0;
  
  //WDT
  //MyWatchDoggy.clear();
  
  while (!connected && notConnectedCounter < 3) {

          
    
    //added ","", false, true" on 18/06/2022 for testing
    //https://github.com/arduino-libraries/MKRNB/issues/71
    if ((nbAccess.begin(PINNUMBER,"", restart, true) == NB_READY) &&
        (gprs.attachGPRS(GPRS_APN) == GPRS_READY)) {
          connected = true;
    } else {
  
          Serial.println("Not connected");
          delay(1000);
          stateTwoConnectedWithErrors();
          //added "restart = true" on 18/06/2022 for testing
          restart = true;
          //for power saving
          // nbAccess.shutdown();
          delay(5000);
      }
      notConnectedCounter++;
      //WDT
      //MyWatchDoggy.clear();
    }
    
   
  
    if(!connected) {
          Serial.println("Can't initialize, retrying.");
          delay(3000); // Let's SARA breath.

          //WDT
          //MyWatchDoggy.clear();
          
          stateTwoNotConnected();
          //ADDED FROM V5
          //client.stop();
          
          // ADDED SLEEP MODE FROM V5
          delay(10000);
          lightsOff();
          //for power saving
          nbAccess.shutdown();
          delay(5000);
          //rtc.standbyMode();
          //disconnecting();
    }
  
  
  // you're connected now, so print out the data:
  Serial.println("You're connected to the network");
  

  stateTwoConnected();
 
}



void lightsOff()
{
  for (int i = 0; i < 4; i++)
  {
    // pinMode can be used to set an I/O as OUTPUT or INPUT
    //io.pinMode(i, OUTPUT);
    io.write(i, LOW);
  }
  
}

void stateOne()//waking up from sleep and/or starting sketch
{
  io.digitalWrite((1) % 4, HIGH); // Turn last pin HIGH 0- RED; 1-GREEN; 2 - BLUE; 3 - ORANGE
}


void SDCardStart()//Done writing to SD card
{
  io.digitalWrite((3) % 4, HIGH); // Turn last pin HIGH 0- RED; 1-GREEN; 2 - BLUE; 3 - ORANGE
}

void stateSDerror()//Done writing to SD card
{
  io.digitalWrite((0) % 4, HIGH); // Turn last pin HIGH 0- RED; 1-GREEN; 2 - BLUE; 3 - ORANGE
  delay(1000);
  for (int i = 0; i < 4; i++)
  {
    // pinMode can be used to set an I/O as OUTPUT or INPUT
    //io.pinMode(i, OUTPUT);
    io.write(i, LOW);
  }
}

void stateSDwirte()//Done writing to SD card
{
  io.digitalWrite((2) % 4, HIGH); // Turn last pin HIGH 0- RED; 1-GREEN; 2 - BLUE; 3 - ORANGE
  delay(1000);
  for (int i = 0; i < 4; i++)
  {
    // pinMode can be used to set an I/O as OUTPUT or INPUT
    //io.pinMode(i, OUTPUT);
    io.write(i, LOW);
  }
}

void connecting()//Connecting to network
{
  io.digitalWrite((1) % 4, HIGH); // Turn last pin HIGH 0- RED; 1-GREEN; 2 - BLUE; 3 - ORANGE
  //io.digitalWrite((3) % 4, HIGH); // Turn last pin HIGH 0- RED; 1-GREEN; 2 - BLUE; 3 - ORANGE
}

void stateTwoConnected()//Connecting to network
{
  //io.write(3, LOW);
  io.digitalWrite((2) % 4, HIGH); // Turn last pin HIGH 0- RED; 1-GREEN; 2 - BLUE; 3 - ORANGE
}

void stateTwoConnectedWithErrors()//Connecting to network
{
  io.digitalWrite((3) % 4, HIGH); // Turn last pin HIGH 0- RED; 1-GREEN; 2 - BLUE; 3 - ORANGE
}

void stateTwoNotConnected()//Connecting to network
{
  io.digitalWrite((0) % 4, HIGH); // Turn last pin HIGH 0- RED; 1-GREEN; 2 - BLUE; 3 - ORANGE
}


void stateThreeCheckGSMGPRS()
{
  io.digitalWrite((1) % 4, HIGH); // Turn last pin HIGH 0- RED; 1-GREEN; 2 - BLUE; 3 - ORANGE
}

void stateThreeCheckError()
{
  io.digitalWrite((3) % 4, HIGH); // Turn last pin HIGH 0- RED; 1-GREEN; 2 - BLUE; 3 - ORANGE
}

void stateThreeCheckOk()
{
  io.digitalWrite((2) % 4, HIGH); // Turn last pin HIGH 0- RED; 1-GREEN; 2 - BLUE; 3 - ORANGE
}



void stateFourDataSent()
{
for (int i = 0; i < 4; i++)
  {
    // pinMode can be used to set an I/O as OUTPUT or INPUT

    io.write(i, LOW);
    
  }
  delay(500);
  io.digitalWrite((1) % 4, HIGH); // Turn last pin HIGH 0- RED; 1-GREEN; 2 - BLUE; 3 - ORANGE
  io.digitalWrite((2) % 4, HIGH); // Turn last pin HIGH 0- RED; 1-GREEN; 2 - BLUE; 3 - ORANGE
  delay(200);
  io.write(1, LOW);
  io.write(2, LOW);
  delay(200);
  io.digitalWrite((1) % 4, HIGH); // Turn last pin HIGH 0- RED; 1-GREEN; 2 - BLUE; 3 - ORANGE
  io.digitalWrite((2) % 4, HIGH); // Turn last pin HIGH 0- RED; 1-GREEN; 2 - BLUE; 3 - ORANGE
  delay(200);
  io.write(1, LOW);
  io.write(2, LOW);
}


void stateFourChekingConnection()
{
    for (int i = 0; i < 4; i++)
  {
    // pinMode can be used to set an I/O as OUTPUT or INPUT

    io.write(i, LOW);
    
  }
  io.digitalWrite((3) % 4, HIGH); // Turn last pin HIGH 0- RED; 1-GREEN; 2 - BLUE; 3 - ORANGE
}

void stateFourDataNotSent()
{
  io.digitalWrite((0) % 4, HIGH); // Turn last pin HIGH 0- RED; 1-GREEN; 2 - BLUE; 3 - ORANGE
}
