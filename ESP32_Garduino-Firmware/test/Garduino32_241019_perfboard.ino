#include <Arduino.h>
#include <U8g2lib.h>
#include "uRTCLib.h"
#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "time.h"
#include "sntp.h"
#include "DHT.h"


#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

// define pins
#define BLUE 17
#define RED 25
#define GREEN 26

#define BUZZER 5

#define PUMP 4 //

#define Relay_1 33 // Pump
//#define Relay_2 2
#define Relay_3 14 // Azalea
#define Relay_4 27 // Ivy

#define CLK 21
#define DT 22
#define SW 13

#define sensorPin 35
#define sensorPin2 34
#define sensorPin3 16
#define DHTTYPE DHT22


/* ======================================================================================================


  Notes during Programming & TO-DO:

  X --- 1. Add state to remove relays triggering on each loop
  2. Calibrate wet / dry references and update map function
  X --- 3. Ensure on auto trigger there is a timeout so it doesn't keep watering until sensor catches up.  Must wait at least a day for auto trigger
  4. Add timeout to switch back to Passive state(green led)
  X --- |||||||||||||| Tried this and reverted because readings were too smooth..  5. Average readings to reduce jumpy values
  X --- 6. Incorporate scheduling
  7. Switch for Modes / auto-trigger + Schedule + manual, auto-trigger + manual, Schedule + manual



 ======================================================================================================
*/

 // ------- Wifi Creds --------
const char* ssid       = "GoBears-2.4";
const char* password   = "Marlin22#";


const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = -21600;
const int   daylightOffset_sec = 3600;

const int dry = 595;
const int wet = 239;

//const char* time_zone = "CET-1CEST,M3.5.0,M10.5.0/3";  // TimeZone rule for Europe/Rome including daylight adjustment rules (optional)

// Oled Setup
U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 15, /* dc=*/ 32, /* reset=*/ 4); 

struct plantNumber
{
  //char name[12];
  String name;
  int DayNumber;
  int hour;
  int minute;
  int mL;
  bool wateredToday;
  int plantState;       // 1. Dry   2. Fair   3. Good   4. Wet
};

const int BOX_HEIGHT = 15;
const int BOX_WIDTH = 62;

unsigned long lastButtonPress = 0;   
int currentStateCLK;
int lastStateCLK;       
int btnState = HIGH;   
int lastbtnState;     
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
int diffState = false;

byte k = 0;
// define color mode
int mode = 0;

int selector = 0;
int rotPos = 0;

bool menuState = false;
int startState = 0;
int counter = 42;
int counter2 = 42;
float temperatureVal = 69;            // TODO -- fix this later
bool wateringState = false;
bool manualTrigger = false;
bool running = false;
bool flashing = false;
bool initSetup = false;
bool sleepMode = false;
char buffer[18];

float moisture = 0;
float moisture2 = 0;

unsigned long currentMillis = 0;
unsigned long animationPreviousMillis = 0;
unsigned long readingPreviousMillis = 0;
unsigned long postPreviousMillis = 0;
unsigned long postInfoPreviousMillis = 0;
unsigned long menuPreviousMillis = 0;
unsigned long sleepPreviousMillis = 0;
unsigned long wakePreviousMillis = 0;

const int animationDelay = 200;
const int readingDelay = 3000;

// ---------------------------------------------     this pump runs 360mL in 30sec >> ~720mL in 60sec       60sec == 60000, 12mL/sec   || desired mL / 12 == wateringDelay
int wateringDelay1 = 3000;         
int wateringDelay2 = 3000;
const int menuDelay = 20000;
const int sendDataDelay = 5000;
const int sleepDelay = 600000;
const int wakeDelay = 5000;

String serverName = "https://bluegarden-6f0bc98539d7.herokuapp.com/api/plants";
String serverSchedule = "https://bluegarden-6f0bc98539d7.herokuapp.com/api/device_info";

String DEVICE_ID = "2341";

struct tm timeinfo;

// Temp sensor --- Not being used
 DHT dht(sensorPin3, DHTTYPE);

char DaysoftheWeek[8][5] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun", "N/A"};
char daysOfTheWeek[8][12] = {"Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday", "N/A"};

plantNumber myPlant1 = {"1:", 7, 0, 0, 0, false, 3};
plantNumber myPlant2 = {"2:", 7, 0, 0, 0, false, 3};

// ====================================================================================================================================

//                      FUNCTIONS 

// ====================================================================================================================================



// 'New Piskel-1', 47x47px
const unsigned char epd_bitmap_New_Piskel_1 [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 
	0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x10, 0x00, 0x00, 0x00, 0x04, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0xf0, 0x87, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 
	0x88, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x84, 0x20, 0x04, 0x00, 0x00, 0x00, 0xd6, 0x20, 0x00, 0x00, 
	0x00, 0x00, 0x44, 0xf0, 0x10, 0x00, 0x00, 0x00, 0x06, 0xb8, 0x03, 0x00, 0x00, 0x00, 0x02, 0x3c, 
	0x06, 0x00, 0x00, 0x00, 0x0c, 0x37, 0x08, 0x00, 0x00, 0x00, 0xfc, 0x71, 0x08, 0x00, 0x00, 0x00, 
	0x00, 0x70, 0x10, 0x00, 0x00, 0x00, 0x02, 0x70, 0x10, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x10, 0x00, 
	0x00, 0x00, 0x00, 0x30, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 
	0x20, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x02, 0x30, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x1f, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x5f, 0x00, 
	0x00, 0x00, 0xf8, 0xff, 0x1f, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x1f, 0x00, 0x00, 0x00, 0xf0, 0xff, 
	0x0f, 0x00, 0x00, 0x00, 0x10, 0x00, 0x08, 0x00, 0x00, 0x00, 0x10, 0x00, 0x08, 0x00, 0x00, 0x00, 
	0x10, 0x00, 0x08, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 
	0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 0x00, 0x00, 0x20, 0x00, 
	0x04, 0x00, 0x00, 0x00, 0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 
	0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 0x80, 0x00, 0x01, 0x00, 
	0x00, 0x00, 0x80, 0x00, 0x01, 0x00, 0x00, 0x00, 0x80, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'New Piskel-1', 47x47px
const unsigned char epd_bitmap_New_Piskel_2 [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x88, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x04, 0x00, 0x00, 0x00, 0x12, 0x00, 0x08, 0x00, 0x00, 0x00, 0x80, 0x1f, 0x01, 0x00, 0x00, 0x00, 
	0xe0, 0x2c, 0x00, 0x00, 0x00, 0x00, 0x32, 0xa6, 0x00, 0x00, 0x00, 0x00, 0x10, 0x23, 0x00, 0x00, 
	0x00, 0x00, 0x08, 0x31, 0x00, 0x00, 0x00, 0x00, 0x88, 0xf1, 0x10, 0x00, 0x00, 0x00, 0x8c, 0xb8, 
	0x01, 0x00, 0x00, 0x00, 0x84, 0x38, 0x0b, 0x00, 0x00, 0x00, 0x04, 0x3c, 0x06, 0x00, 0x00, 0x00, 
	0x04, 0x36, 0x2c, 0x00, 0x00, 0x00, 0xc4, 0x71, 0x08, 0x00, 0x00, 0x00, 0x3c, 0x70, 0x10, 0x00, 
	0x00, 0x00, 0x02, 0xf0, 0x10, 0x02, 0x00, 0x00, 0x00, 0xb0, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x30, 
	0x0e, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x1f, 0x00, 0x00, 0x00, 0xfa, 0xff, 0x1f, 0x00, 
	0x00, 0x00, 0xf8, 0xff, 0x1f, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x1f, 0x00, 0x00, 0x00, 0xf0, 0xff, 
	0x0f, 0x00, 0x00, 0x00, 0x10, 0x00, 0x08, 0x00, 0x00, 0x00, 0x10, 0x00, 0x08, 0x00, 0x00, 0x00, 
	0x10, 0x00, 0x08, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 
	0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 0x00, 0x00, 0x20, 0x00, 
	0x04, 0x00, 0x00, 0x00, 0x60, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 
	0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 0x80, 0x00, 0x01, 0x00, 
	0x00, 0x00, 0x80, 0x00, 0x01, 0x00, 0x00, 0x00, 0x80, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'New Piskel-1', 47x47px
const unsigned char epd_bitmap_New_Piskel_3 [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x00, 
	0x00, 0x00, 0x88, 0x0f, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x70, 0x93, 
	0x00, 0x00, 0x00, 0x00, 0x18, 0x31, 0x28, 0x00, 0x00, 0x00, 0xc8, 0x30, 0x00, 0x00, 0x00, 0x00, 
	0x46, 0x30, 0x00, 0x00, 0x00, 0x00, 0x22, 0x32, 0x22, 0x00, 0x00, 0x00, 0x32, 0x30, 0x00, 0x00, 
	0x00, 0x00, 0x92, 0xf8, 0x07, 0x00, 0x00, 0x00, 0x02, 0x3c, 0x0c, 0x00, 0x00, 0x00, 0x02, 0x36, 
	0x10, 0x00, 0x00, 0x00, 0x82, 0x33, 0x30, 0x00, 0x00, 0x00, 0xfe, 0x30, 0x20, 0x00, 0x00, 0x00, 
	0x00, 0x30, 0x20, 0x00, 0x00, 0x00, 0x00, 0x30, 0x20, 0x00, 0x00, 0x00, 0x00, 0x30, 0x60, 0x00, 
	0x00, 0x00, 0x00, 0x70, 0x60, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x41, 0x00, 0x00, 0x00, 0x00, 0x30, 
	0x7f, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x1f, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x1f, 0x02, 
	0x00, 0x00, 0xf8, 0xff, 0x1f, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x1f, 0x00, 0x00, 0x00, 0xf0, 0xff, 
	0x0f, 0x00, 0x00, 0x00, 0x10, 0x00, 0x08, 0x00, 0x00, 0x00, 0x12, 0x00, 0x08, 0x00, 0x00, 0x00, 
	0x10, 0x00, 0x08, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 
	0x00, 0x00, 0x20, 0x00, 0x44, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 0x00, 0x00, 0x20, 0x00, 
	0x04, 0x00, 0x00, 0x00, 0x60, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 
	0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 0x80, 0x00, 0x01, 0x00, 
	0x00, 0x00, 0x80, 0x00, 0x01, 0x00, 0x00, 0x00, 0x80, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
  // 'New Piskel-1', 47x47px
const unsigned char epd_bitmap_New_Piskel_4 [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x1f, 0x00, 0x00, 0x00, 0x00, 
	0xe0, 0x2c, 0x00, 0x00, 0x00, 0x00, 0x30, 0x26, 0x00, 0x00, 0x00, 0x00, 0x10, 0x23, 0x00, 0x00, 
	0x00, 0x00, 0x08, 0x31, 0x00, 0x00, 0x00, 0x00, 0x88, 0xf1, 0x00, 0x00, 0x00, 0x00, 0x8c, 0xb8, 
	0x01, 0x00, 0x00, 0x00, 0x84, 0x38, 0x03, 0x00, 0x00, 0x00, 0x04, 0x3c, 0x06, 0x00, 0x00, 0x00, 
	0x04, 0x36, 0x0c, 0x00, 0x00, 0x00, 0xc4, 0x71, 0x08, 0x00, 0x00, 0x00, 0x3c, 0x70, 0x10, 0x00, 
	0x00, 0x00, 0x00, 0xf0, 0x10, 0x00, 0x00, 0x00, 0x00, 0xb0, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x30, 
	0x0e, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x1f, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x1f, 0x00, 
	0x00, 0x00, 0xf8, 0xff, 0x1f, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x1f, 0x00, 0x00, 0x00, 0xf0, 0xff, 
	0x0f, 0x00, 0x00, 0x00, 0x10, 0x00, 0x08, 0x00, 0x00, 0x00, 0x10, 0x00, 0x08, 0x00, 0x00, 0x00, 
	0x10, 0x00, 0x08, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 
	0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 0x00, 0x00, 0x20, 0x00, 
	0x04, 0x00, 0x00, 0x00, 0x60, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 
	0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 0x80, 0x00, 0x01, 0x00, 
	0x00, 0x00, 0x80, 0x00, 0x01, 0x00, 0x00, 0x00, 0x80, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 912)
const int epd_bitmap_allArray_LEN = 4;
const unsigned char* epd_bitmap_allArray[4] = {
	epd_bitmap_New_Piskel_1,
	epd_bitmap_New_Piskel_2,
	epd_bitmap_New_Piskel_3,
  epd_bitmap_New_Piskel_4
};


void printLocalTime(void)
{
  if(!getLocalTime(&timeinfo)){
    Serial.println("No time available (yet)");
    return;
  }
  tm* myTime = &timeinfo;

  time_t rawtime;
  struct tm * timeinfo;
  char buffer[15];

  time(&rawtime);
  timeinfo = localtime (&rawtime);

  //strftime (buffer, 80, "%I:%M%p", timeinfo);
  strftime (buffer, 15, "%H%M%S", timeinfo);
  puts(buffer);

  int val = atoi(buffer);
  printf("--- %d --- \t", val);

  // if the time is between 11:59:53PM and 6secs later(for readtime) reset watered today bool check
  if (val >= 235953 && val <= 235959)
  {
    myPlant1.wateredToday = false;
    myPlant2.wateredToday = false;
  }

  //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.println(myTime, "%H:%M \n");
  //Serial.println(myPlant1.DayNumber);
  //Serial.println(myPlant2.DayNumber);
}

// Callback function (get's called when time adjusts via NTP)
void timeavailable(struct timeval *t)
{
  Serial.println("Got time adjustment from NTP!");
  printLocalTime();
}

void animation(byte k)
  {    
    u8g2.setDrawColor(1);
    
      if(selector == 1 && menuState == false)
      {
        u8g2.drawXBMP(3, 1, 47 , 47, epd_bitmap_allArray[k]);
        u8g2.drawXBMP(67, 1, 47 , 47, epd_bitmap_allArray[3]);
      }
      else if(selector == 2 && menuState == false)
      {
        u8g2.drawXBMP(3, 1, 47 , 47, epd_bitmap_allArray[3]);
        u8g2.drawXBMP(67, 1, 47 , 47, epd_bitmap_allArray[k]);
      }
      else if(menuState == false)
      {
        u8g2.drawXBMP(3, 1, 47 , 47, epd_bitmap_allArray[3]);
        u8g2.drawXBMP(67, 1, 47 , 47, epd_bitmap_allArray[3]);
      }
      else if(menuState == true && startState == 0)
      {
        u8g2.drawXBMP(3, 1, 47 , 47, epd_bitmap_allArray[3]);
      }
      else
      {
        u8g2.drawXBMP(3, 1, 47 , 47, epd_bitmap_allArray[k]);
      }
  }

void colorSelect(char color){

  switch (color) {
    case 'r':
      analogWrite(RED, 0);
      analogWrite(GREEN, 255);
      analogWrite(BLUE, 255);
      break;
    case 'g':
      analogWrite(RED, 255);
      analogWrite(GREEN, 0);
      analogWrite(BLUE, 255);
      break;
    case 'b':
      analogWrite(RED, 255);
      analogWrite(GREEN, 255);
      analogWrite(BLUE, 0);
      break;
    case 'w':
      analogWrite(RED, 0);
      analogWrite(GREEN, 0);
      analogWrite(BLUE, 0);
      break;
    case 'o':
      analogWrite(RED, 255);
      analogWrite(GREEN, 255);
      analogWrite(BLUE, 255);
      break;
    default :
      analogWrite(RED, 255);
      analogWrite(GREEN, 255);
      analogWrite(BLUE, 255);
  }
}

void u8g2_prepare(void)
{
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void buzz()
{
  digitalWrite(BUZZER, HIGH);
  delay(250);
  digitalWrite(BUZZER, LOW);
}

void update() 
{

 	// Read the current state of CLK
	currentStateCLK = digitalRead(CLK);

  // If there is a minimal movement of 1 step
  if ((currentStateCLK != lastStateCLK) && (currentStateCLK == 1)) {
    
    if (digitalRead(DT) != currentStateCLK) {      // If Pin B is HIGH
      Serial.println("Right");             // Print on screen
      rotPos++;
    } else {
      Serial.println("Left");            // Print on screen
      rotPos--;
    }
    
    Serial.print("Direction: ");
		Serial.print(rotPos);
		Serial.print(" | Counter: ");
		Serial.println(rotPos);
  }
  
  lastStateCLK = currentStateCLK;  
}

void u8g2_lastWater(void)
{
  u8g2_prepare();
  if(menuState == 0)
  {
    u8g2.setFontMode(1);
    u8g2.setDrawColor(1);
    u8g2.drawStr(41, 5, DaysoftheWeek[myPlant1.DayNumber]);
    u8g2.drawStr(103, 5, DaysoftheWeek[myPlant2.DayNumber]);
  }
  else if(menuState == 1)
  {
    if(selector == 1)
    {
    u8g2.setFontMode(1);
    u8g2.setDrawColor(1);
    u8g2.drawStr(92, 5, DaysoftheWeek[myPlant2.DayNumber]);
    }
    else if(selector == 2)
    {
      u8g2.setFontMode(1);
      u8g2.setDrawColor(1);
      u8g2.drawStr(92, 5, DaysoftheWeek[myPlant2.DayNumber]);
    }
  }
}

void waterStart(char plant)
{
  buzz();
  Serial.println("Running cycle\n");
  
  switch(plant)
  {
    case '1':
    // Low is active
    digitalWrite(Relay_1, LOW);
    //digitalWrite(Relay_2, HIGH);
    digitalWrite(Relay_3, HIGH);
    digitalWrite(Relay_4, LOW);
    Serial.println("Watering Ivy\n");
    Serial.println(wateringDelay1);
    delay(wateringDelay1);
    running = false;
    wateringState = false;
    myPlant1.wateredToday = true;
    break;
  case '2':
  
    digitalWrite(Relay_1, LOW);
    //digitalWrite(Relay_2, HIGH);
    digitalWrite(Relay_3, LOW);
    digitalWrite(Relay_4, HIGH);
    Serial.println("Watering Azalea\n");
    Serial.println(wateringDelay2);
    delay(wateringDelay2);
    running = false;
    wateringState = false;
    myPlant2.wateredToday = true;
    break;
  
  default: 
  
    digitalWrite(Relay_1, HIGH);
    //digitalWrite(Relay_2, HIGH);
    digitalWrite(Relay_3, HIGH);
    digitalWrite(Relay_4, HIGH);
    Serial.print("Something aint working\n");
    Serial.println(plant);
    Serial.println(wateringState);
  } 
}

void waterEnd(void)
{
  digitalWrite(Relay_1, HIGH);
  //digitalWrite(Relay_2, HIGH);
  digitalWrite(Relay_3, HIGH);
  digitalWrite(Relay_4, HIGH);

  startState = 0;
}

void checkSchedule()    // ================ NEEDS WORK
{
  int hour = timeinfo.tm_hour;
  int minute = timeinfo.tm_min;
  int day = timeinfo.tm_wday - 1;
  Serial.println(day);
  Serial.println(hour);
  Serial.println(minute);

  // CHANGE THIS LATER
  if (
    day == myPlant1.DayNumber && 
    hour == myPlant1.hour && 
    minute == myPlant1.minute && 
    myPlant1.wateredToday == false)
  {
    if(!myPlant1.wateredToday)
    {
      selector = 1;
      startCycleActive(counter, counter2, k, selector);
    }
    else{
      Serial.println("My plant1 was already watered today");
    }
  }
  if (
    day == myPlant2.DayNumber && 
    hour == myPlant2.hour && 
    minute == myPlant2.minute && 
    myPlant2.wateredToday == false)    
    {
      if(!myPlant2.wateredToday)
      {
        selector = 2;
        startCycleActive(counter, counter2, k, selector);
      }
      else{
        Serial.println("My plant2 was already watered today");
      }
    }
}

void checkPlantState(void)  // For automatic watering based on plant sate
{
  if(myPlant1.plantState == 1 )
  {
    if(!myPlant1.wateredToday)
    {
      selector = 1;
      startCycleActive(counter, counter2, k, selector);
    }
    else{
      Serial.println("My plant1 was already watered today");
    }
  }
    if(myPlant2.plantState == 1)
  {
    selector = 2;
    menuState = true;
    //wateringState = true;
    if(!myPlant2.wateredToday)
    {
      selector = 2;
      startCycleActive(counter, counter2, k, selector);
    }
    else{
      Serial.println("My plant2 was already watered today");
    }
  }
}

void determineMoisture(int moisture, int moisture2)
{
  
  if(moisture > 80)
  {
    myPlant1.plantState = 4;
  }
  else if(moisture < 30)
  {
    myPlant1.plantState = 1;
    
  }
  else if(moisture >= 30 && moisture <= 50)
  {
    myPlant1.plantState = 2;
  }
  else
  {
    myPlant1.plantState = 3;
  }

  if(moisture2 > 80)
  {
    myPlant2.plantState = 4;
  }
  else if(moisture2 < 30)
  {
    myPlant2.plantState = 1;
  }
  else if(moisture2 >= 30 && moisture2 <= 50)
  {
    myPlant2.plantState = 2;
  }
  else
  {
    myPlant2.plantState = 3;
  }
}

int determineWateringDelay(int mL)
{
  // ------ this pump runs 360mL in 30sec >> ~720mL in 60sec   ||   60sec == 60000, 12mL/sec   ||   (desired mL / 12) * 1000== wateringDelay
  return (mL / 12) * 1000;
}

void checkMenu(void)
{
  if(rotPos == 2 && menuState == false)
  {
    selector++;
  }
  else if(rotPos == 0 && menuState == false)
  {
    selector--;
  }
  else if(rotPos == 2 && menuState == true)
  {
    startState++;
    if(startState > 1)
    {
      startState = 0;
    }
  }
  else if(rotPos == 0 && menuState == true)
  {
    menuState = 0;
  }
  else if(diffState == true && menuState == true && startState == 1)
  {
    startState = 2;
    delay(200);
  }

  else if((diffState == true && selector == 1) || (diffState == true && selector == 2))
  {
    if(menuState == true)
    {
      menuState = 0;
    }
    else
    {
      menuState = 1;
    }
    delay(200);
  }

  if(selector > 3)
    {
      selector = 1;
      Serial.println("Selector reset");
    }
  else if(selector < 0)
  {
    selector = 0;
  }

  rotPos = 1;
}

void multibox(void) 
{
  u8g2_prepare();
  animation(k);
  colorSelect('g');
  u8g2.drawRFrame(1, 1, 62, 62, 3);
  u8g2.drawRFrame(65, 1, 62, 62, 3);

  char plant1Schedule[15];
  char plant2Schedule[15];
  //u8g2.setDrawColor(2);

  if(myPlant1.minute < 10)
  {
    sprintf(plant1Schedule, "%d:0%d", myPlant1.hour, myPlant1.minute);
  }
  else
  {
    sprintf(plant1Schedule, "%d:%d", myPlant1.hour, myPlant1.minute);
  }

  if(myPlant2.minute < 10)
  {
    sprintf(plant2Schedule, "%d:0%d", myPlant2.hour, myPlant2.minute);
  }
  else
  {
    sprintf(plant2Schedule, "%d:%d", myPlant2.hour, myPlant2.minute);
  }

    u8g2.drawStr(30, 16, plant1Schedule);
    u8g2.drawStr(93, 16, plant2Schedule);

  if (selector == 1)
  {
    u8g2.drawRBox(1, 48, BOX_WIDTH, BOX_HEIGHT, 3);
    u8g2.drawRFrame(65, 48, BOX_WIDTH, BOX_HEIGHT, 3);
  }
  else if(selector == 2)
  {
    u8g2.drawRFrame(1, 48, BOX_WIDTH, BOX_HEIGHT, 3);
    u8g2.drawRBox(65, 48, BOX_WIDTH, BOX_HEIGHT, 3);
  }
  else
  {
    u8g2.drawRFrame(1, 48, BOX_WIDTH, BOX_HEIGHT, 3);
    u8g2.drawRFrame(65, 48, BOX_WIDTH, BOX_HEIGHT, 3);
  }

  if(myPlant1.plantState == 1)
    {
      u8g2.drawStr(35, 35, "DRY");
    }
    else if(myPlant1.plantState == 2)
    {
      u8g2.drawStr(34, 35, "FAIR");
    }
    else if(myPlant1.plantState == 4)
    {
      u8g2.drawStr(35, 35, "WET");
    }
    else
    {
      u8g2.drawStr(32, 35, "GOOD");
    }

    if(myPlant2.plantState == 1)
    {
      u8g2.drawStr(98, 35, "DRY");
    }
    else if(myPlant2.plantState == 2)
    {
      u8g2.drawStr(97, 35, "FAIR");
    }
    else if(myPlant2.plantState == 4)
    {
      u8g2.drawStr(98, 35, "WET");
    }
    else
    {
      u8g2.drawStr(95, 35, "GOOD");
    }


  u8g2.setFontMode(2);
  u8g2.setDrawColor(2);
  u8g2.drawStr(4, 50, myPlant1.name.c_str());
  u8g2.drawStr(68, 50, myPlant2.name.c_str());

  u8g2_lastWater();
}

void singleBoxBig(int counter, byte k)
{

  int hour = timeinfo.tm_hour;
  int minutes = timeinfo.tm_min;
  //int day = timeinfo.tm_wday;
  char* wday = DaysoftheWeek[timeinfo.tm_wday];
  char mL_string1[5]; 
  char mL_string2[5];

  itoa(myPlant1.mL, mL_string1, 10);
  itoa(myPlant2.mL, mL_string2, 10);

  u8g2_prepare();
  animation(k);
  u8g2.drawRFrame(1, 1, 126, 62, 3);
  u8g2.drawRBox(1, 48, 126, BOX_HEIGHT, 3);
  if(minutes < 10)
  {
    sprintf(buffer, "%s %d:0%d", wday, hour, minutes);
  }
  else
  {
    sprintf(buffer, "%s %d:%d", wday, hour, minutes);
  }

  if(selector == 1)
  {
    u8g2.setFontMode(2);
    u8g2.setDrawColor(2);
    u8g2.drawStr(4, 50, myPlant1.name.c_str());
    u8g2.drawStr(69, 50, buffer);

    if(myPlant1.plantState == 1)
    {
      u8g2.drawStr(84, 25, "DRY");
    }
    else if(myPlant1.plantState == 2)
    {
      u8g2.drawStr(84, 25, "FAIR");
    }
    else if(myPlant1.plantState == 4)
    {
      u8g2.drawStr(84, 25, "WET");
    }
    else
    {
      u8g2.drawStr(81, 25, "GOOD");
    }

    u8g2.setFont(u8g2_font_fub20_tr);
    u8g2.setFontMode(1);
    u8g2.setDrawColor(1);
    u8g2.drawStr(28, 25, mL_string1);
    u8g2.setFont(u8g2_font_profont11_mf);
    u8g2.setFontMode(1);
    u8g2.setDrawColor(1);
    if (myPlant1.mL < 100)
    {
      u8g2.drawStr(60, 36, "mL");
    }
    else
    {
      u8g2.drawStr(80, 36, "mL");
    }
    u8g2_lastWater();
    u8g2.drawFrame(114, 4, 10, 42);
    
  }
  else if(selector == 2)
  {
    u8g2.setFontMode(2);
    u8g2.setDrawColor(2);
    u8g2.drawStr(4, 50, myPlant2.name.c_str());
    u8g2.drawStr(69, 50, buffer);

    if(myPlant2.plantState == 1)
    {
      u8g2.drawStr(84, 25, "DRY");
    }
    else if(myPlant2.plantState == 2)
    {
      u8g2.drawStr(84, 25, "FAIR");
    }
    else if(myPlant2.plantState == 4)
    {
      u8g2.drawStr(81, 25, "WET");
    }
    else
    {
      u8g2.drawStr(81, 25, "GOOD");
    }

    u8g2.setFont(u8g2_font_fub20_tr);
    u8g2.setFontMode(1);
    u8g2.setDrawColor(1);
    u8g2.drawStr(28, 25, mL_string2);
    u8g2.setFont(u8g2_font_profont11_mf);
    u8g2.setFontMode(1);
    u8g2.setDrawColor(1);
    if (myPlant2.mL < 100)
    {
      u8g2.drawStr(60, 36, "mL");
    }
    else
    {
      u8g2.drawStr(80, 36, "mL");
    }
    u8g2_lastWater();
    u8g2.drawFrame(114, 4, 10, 42);
  }
}

float readSensor() 
{
  float Plantval = analogRead(sensorPin);	// Read the analog value form sensor
	return Plantval;							// Return analog moisture value
}

float readSensor2() 
{
	float Plantval2 = analogRead(sensorPin2);	// Read the analog value form sensor
	return Plantval2;							// Return analog moisture value
}

void startCyclePassive(int counter, int counter2)
{
  u8g2_prepare();

    u8g2.drawRFrame(40, 4, 45, 12, 3);
    if(selector == 1)
    {
      u8g2.drawBox(114, 46 - counter, 9, counter);
    }
    else
    {
      u8g2.drawBox(114, 46 - counter2, 9, counter2);
    }
    u8g2.setFontMode(2);
    u8g2.setDrawColor(2);
    u8g2.drawStr(47, 6, "START");
}

void startCycleReady(int counter, int counter2)
{
    u8g2_prepare();
    u8g2.drawRBox(40, 4, 45, 12, 3);
    if(selector == 1)
    {
      u8g2.drawBox(114, 46 - counter, 9, counter);
    }
    else
    {
      u8g2.drawBox(114, 46 - counter2, 9, counter2);
    }

    u8g2.setFontMode(2);
    u8g2.setDrawColor(2);
    u8g2.drawStr(47, 6, "START");
    colorSelect('r');
}

bool startCycleActive(int counter, int counter2, byte k, int selector)
{

  u8g2_prepare();
  //animation(k);
  u8g2.drawRBox(40, 4, 45, 12, 3);
  u8g2.setFontMode(2);
  u8g2.setDrawColor(2);
  u8g2.drawStr(45, 6, "ACTIVE");

  if(selector == 1)
  {
    Serial.print("Selector: ");
    Serial.println(selector);
    u8g2.drawBox(114, 46 - counter, 9, counter);
    manualTrigger = true;
    running = true;
    colorSelect('b');
    wateringState = true;
    u8g2.sendBuffer();
    waterStart('1'); 
    waterEnd();
  }
  if(selector == 2)
  {
    Serial.print("Selector: ");
    Serial.print(selector);
    u8g2.drawBox(114, 46 - counter2, 9, counter);
    manualTrigger = true;
    running = true;
    colorSelect('b');
    wateringState = true;
    u8g2.sendBuffer();
    waterStart('2');
    waterEnd();
  }
  return true;
}

void menuStateReturn(void)
{
  if(currentMillis - menuPreviousMillis >= menuDelay) {
    
  menuPreviousMillis = currentMillis;
  menuState = false;
  }
}

int updatePlantSchedule(String plantName, int nextDay, int nextHour, int nextMinute, int amount)
{
  Serial.println(plantName);

  if (plantName[0] == myPlant1.name[0])
  {
    Serial.println("Plant 1 info updated");
    myPlant1.name = plantName;
    myPlant1.DayNumber = nextDay;
    myPlant1.hour = nextHour;
    myPlant1.minute = nextMinute;
    myPlant1.mL = amount;
    wateringDelay1 = determineWateringDelay(amount);
  }
  else if (plantName[0] == myPlant2.name[0])
  {
    Serial.println("Plant 2 info updated");
    myPlant2.name = plantName;
    myPlant2.DayNumber = nextDay;
    myPlant2.hour = nextHour;
    myPlant2.minute = nextMinute;
    myPlant2.mL = amount;
    wateringDelay2 = determineWateringDelay(amount);
  }
  else{
    return -1;
  }
  return 0;
}

void retrieveSchedule(void)
{
  //  if(currentMillis - postInfoPreviousMillis >= 10000) {
  //  postInfoPreviousMillis = currentMillis;
    HTTPClient http;

    String serverPath = serverSchedule; 

    int update1Confirm = 0;
    int update2Confirm = 0;
    
    // Your Domain name with URL path or IP address with path
    http.begin(serverPath.c_str());
    http.addHeader("Content-Type", "application/json");
    
    // If you need Node-RED/server authentication, insert user and password below
    //http.setAuthorization("REPLACE_WITH_SERVER_USERNAME", "REPLACE_WITH_SERVER_PASSWORD");
    StaticJsonDocument<200> doc;
    // Add values in the document
    
    doc["device_id"] = DEVICE_ID;

    String requestBody;
    serializeJson(doc, requestBody);
    
    int httpResponseCode = http.POST(requestBody);

    if (httpResponseCode>0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      //String payload = http.getString();
      deserializeJson(doc, http.getString());

      String plantName = doc["Plant1"]["Plant_name"];
      int nextDay = doc["Plant1"]["Day"];
      int nextHour = doc["Plant1"]["Hour"];
      int nextMinute = doc["Plant1"]["Minute"];
      int nextAmount = doc["Plant1"]["Amount"];

      String plant2Name = doc["Plant2"]["Plant_name"];
      int plant2_nextDay = doc["Plant2"]["Day"];
      int plant2_nextHour = doc["Plant2"]["Hour"];
      int plant2_nextMinute = doc["Plant2"]["Minute"];
      int plant2_nextAmount = doc["Plant2"]["Amount"];

      Serial.println(plantName);
      Serial.println(nextDay);
      Serial.println(nextHour);
      Serial.println(nextMinute);
      Serial.println(nextAmount);
      Serial.println("\n");

      Serial.println(plant2Name);
      Serial.println(plant2_nextDay);
      Serial.println(plant2_nextHour);
      Serial.println(plant2_nextMinute);
      Serial.println(plant2_nextAmount);
      Serial.println("\n");

      update1Confirm = updatePlantSchedule(plantName, nextDay, nextHour, nextMinute, nextAmount);
      update2Confirm = updatePlantSchedule(plant2Name, plant2_nextDay, plant2_nextHour, plant2_nextMinute, plant2_nextAmount);

      Serial.print(update1Confirm);
      Serial.println(update2Confirm);
      Serial.println("\n");
    }
    else {
      Serial.print("Error code 2: ");
      Serial.println(httpResponseCode);
    }
    http.end();
}

void postData(void)
{
   if(currentMillis - postPreviousMillis >= 900000 || initSetup == false) {           //300000 for every 5 min      
    postPreviousMillis = currentMillis;
    colorSelect('r');
    HTTPClient http;

    String serverPath = serverName; 
    
    // Your Domain name with URL path or IP address with path
    http.begin(serverPath.c_str());
    http.addHeader("Content-Type", "application/json");
    
    // If you need Node-RED/server authentication, insert user and password below
    //http.setAuthorization("REPLACE_WITH_SERVER_USERNAME", "REPLACE_WITH_SERVER_PASSWORD");
    StaticJsonDocument<200> doc;
    // Add values in the document
    if(myPlant1.name != "N/A")
    {
      // TODO -- FIX temp value reading
      temperatureVal = 69;
      doc["user_id"] = 5;
      doc["temperature"] = temperatureVal;
      doc["plant1_name"] = myPlant1.name;
      doc["plant1_moisture"] = moisture;
      doc["plant2_name"] = myPlant2.name;
      doc["plant2_moisture"] = moisture2;
      
      String requestBody;
      serializeJson(doc, requestBody);
      
      int httpResponseCode = http.POST(requestBody);

      if (httpResponseCode>0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
      }
      else {
        Serial.print("Error code 1: ");
        Serial.println(httpResponseCode);
      }
    }
    // Free resources
    http.end();

    retrieveSchedule();
 
    colorSelect('o');
  }
}

void sleep()
{
  // turn off display 
  u8g2.setPowerSave(true);

  sleepMode = true;

  // turn off led
  colorSelect('o');
}

void wakeup()
{
  // turn on display
  u8g2.setPowerSave(false);

  sleepMode = false;

  // turn on green led 
  colorSelect('g');
}

void sleepWakeTimer()
{
  int hour = timeinfo.tm_hour;
  int minute = timeinfo.tm_min;

  // check if display is currently sleeping
  if (sleepMode)
  {
    // if it is.. check how long it has been sleeping for ||  if it's been long enough, wake it up to flash details
    if(currentMillis - sleepPreviousMillis >= sleepDelay) 
    {
      sleepPreviousMillis = currentMillis;
      wakeup();
    }
  }

  // check if time is after 10pm and before 6:30am && it has been on for at least 5 seconds
  if ((!sleepMode && hour > 22) || (!sleepMode && hour < 6))
  {
    if(currentMillis - wakePreviousMillis >= wakeDelay)
    {
      // put display in sleepmode
      sleep();
    }
  }
}

// ====================================================================================================================================

//                      SETUP 

// ====================================================================================================================================

void setup(void) 
{
  // set notification call-back function
  sntp_set_time_sync_notification_cb( timeavailable );

    /**
   * NTP server address could be aquired via DHCP,
   *
   * NOTE: This call should be made BEFORE esp32 aquires IP address via DHCP,
   * otherwise SNTP option 42 would be rejected by default.
   * NOTE: configTime() function call if made AFTER DHCP-client run
   * will OVERRIDE aquired NTP server address
   */
  //sntp_servermode_dhcp(1);    // (optional)

  /**
   * This will set configured ntp servers and constant TimeZone/daylightOffset
   * should be OK if your time zone does not need to adjust daylightOffset twice a year,
   * in such a case time adjustment won't be handled automagicaly.
   */
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);

  /**
   * A more convenient approach to handle TimeZones with daylightOffset 
   * would be to specify a environmnet variable with TimeZone definition including daylight adjustmnet rules.
   * A list of rules for your zone could be obtained from https://github.com/esp8266/Arduino/blob/master/cores/esp8266/TZ.h
   */
  //configTzTime(time_zone, ntpServer1, ntpServer2);

  //connect to WiFi
  //Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);

  //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
  //WiFiManager wm;

  // reset settings - wipe stored credentials for testing
  // these are stored by the esp library
  //wm.resetSettings();                         // ---------------- UNCOMMENT THIS TO RESET THE WIFI AND REQUIRE AP CREDENTIALS AGAIN (FOR TESTING)

  //bool res;
    // res = wm.autoConnect(); // auto generated AP name from chipid
    // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
    //res = wm.autoConnect("Garduino","swordfish"); // password protected ap

    //if(!res) {
        //Serial.println("Failed to connect");
        // ESP.restart();
    //}
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println(" CONNECTED");

  u8g2.begin();

  dht.begin();

  delay(500);

  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // LED startup sequence
  colorSelect('o');
  delay(25);
  colorSelect('r');
  delay(500);
  colorSelect('g');
  delay(500);
  colorSelect('b');
  delay(500);

  colorSelect('g');

  // Rotary encoder
  pinMode(SW, INPUT_PULLUP);       // Enable the switchPin as input 
  pinMode(DT, INPUT);                   // Set PinA as input
  pinMode(CLK, INPUT);                   // Set PinB as input

  // Relay module startup
  pinMode(Relay_1, OUTPUT);               // Pump
  // pinMode(Relay_2, OUTPUT);               // Solenoid 1
  pinMode(Relay_3, OUTPUT);               // Solenoid 2
  pinMode(Relay_4, OUTPUT);  
  
  digitalWrite(Relay_1, HIGH);
  // digitalWrite(Relay_2, HIGH);
  digitalWrite(Relay_3, HIGH);
  digitalWrite(Relay_4, HIGH);

  digitalWrite(BUZZER, HIGH);

  // Atach a CHANGE interrupt to PinB and exectute the update function when this change occurs.
  attachInterrupt(digitalPinToInterrupt(22), update, CHANGE);

  lastStateCLK = digitalRead(CLK);

  Serial.begin(115200);

  digitalWrite(BUZZER, LOW);

  retrieveSchedule();
  
  initSetup = true;
}

// ====================================================================================================================================

//                      LOOP 

// ====================================================================================================================================

void loop(void) 
{
  bool justRan = false;

  currentMillis = millis();

  if(currentMillis - animationPreviousMillis >= animationDelay) {
    
    animationPreviousMillis = currentMillis;

    k++;

    if(k > 3)
    {
      k = 0;
    }
  }

  running = false;
  
  if(currentMillis - readingPreviousMillis >= readingDelay) {
  
  readingPreviousMillis = currentMillis;

  sleepWakeTimer();
  
  uint32_t readings1 = 0;
  uint32_t readings2 = 0;

  // take 5 readings
  // for(int i = 0; i < 5; i++)
  // {
  //   readings1 = readings1 + analogRead(sensorPin);
  //   readings2 = readings2 + analogRead(sensorPin2);
  // }

  // int sensorVal1 = readings1 / 5; // average the last 5 readings;
  // int sensorVal2 = readings2 / 5; 

  int sensorVal1 = analogRead(sensorPin);
  int sensorVal2 = analogRead(sensorPin2);

  //temperatureVal = dht.readTemperature(true);

  moisture = map(sensorVal1,  1000, 2500, 100, 0);
  moisture2 = map(sensorVal2, 1000, 2500, 100, 0);
  
  Serial.print(sensorVal1);
  Serial.print("\t");
  Serial.print(moisture);
  Serial.print("\t");
  Serial.print(sensorVal2);
  Serial.print("\t");
  Serial.print(moisture2);
  Serial.print("\t\t");

  printLocalTime();     // it will take some time to sync time :)

  counter = map(moisture, 0, 100, 0, 41);
  counter2 = map(moisture2, 0, 100, 0, 41);
  }

  postData();

  u8g2.clearBuffer();

  int reading = digitalRead(SW);

  // If the switch changed, due to noise or pressing:
  if (reading != lastbtnState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) 
  {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != btnState) 
    {
      btnState = reading;

      if(btnState == LOW)
      {
        diffState = true;
        delay(300);
      }
    }
  }

  // save the btn reading. Next time through the loop, it'll be the lastButtonState:
  lastbtnState = reading;

  checkMenu();

    if (menuState == false)
    {
      multibox();
      startState = 0;
    }
    else if(menuState == true && startState == 0)
    {
      singleBoxBig(counter, k);
      startCyclePassive(counter, counter2);
      colorSelect('g');
    }
    else if(menuState == true && startState == 2)
    {
      singleBoxBig(counter, k);
      justRan = startCycleActive(counter, counter2, k, selector);
      colorSelect('b');
    }
    else if(menuState == true && startState == 1)
    {
      singleBoxBig(counter, k);
      startCycleReady(counter, counter2);
      colorSelect('r');
    }

  determineMoisture(moisture, moisture2);

  checkPlantState();

  checkSchedule();

  if (!justRan)
  {
    u8g2.sendBuffer();
  }
  
  diffState = false;
}