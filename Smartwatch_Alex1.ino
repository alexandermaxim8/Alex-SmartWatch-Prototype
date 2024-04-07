#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ezButton.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <ESP32Time.h>
#include <HTTPClient.h>
#include <UrlEncode.h>
#include "time.h"
#include "sntp.h"
#include <ESP32Time.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define PROCEED_PIN 26
#define SCROLL_PIN 13
#define buzzer 19
#define LCD_POWER 4

ezButton button1(PROCEED_PIN);
ezButton button2(SCROLL_PIN);
 
String IPaddress;
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_MPU6050 mpu;

const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0;

ESP32Time rtc(25200);

int state=0;
bool dsInit=true;
int dsInitTime;
bool saver = true;
bool saverInit = false;
bool healthInit = false;
bool firstInit = true;
int timeState=0;
int startTime, minute, second, minuteCopy, secondCopy;


const char* ssid = "Galaxy A512844";
const char* password = "12341234";

float accMagnitude;
int stepCount = 0, flag = 0;
float accX, accY, accZ;

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;


// Create AsyncWebServer object on port 80
AsyncWebServer server(80);


// Create an Event Source on /events
AsyncEventSource events("/events");


// Json Variable to Hold Sensor Readings
JSONVar readings;


TinyGPSPlus gps;


// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 10000;
unsigned long sleepTime1 = 0;
unsigned long sleepTime2;


const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred


int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
int32_t SPO2Val;
int32_t heartRateVal;


float beatsPerMinute;
int beatAvg;


MAX30105 particleSensor;


#define MAX_BRIGHTNESS 255


#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

void callback(){
  //placeholder callback function
}

void initTime(){
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
  struct tm timeinfo;
  while(!getLocalTime(&timeinfo)){
    Serial.println("  Failed to obtain time");
    delay(500);
  }
  rtc.setTime(timeinfo.tm_sec, timeinfo.tm_min, timeinfo.tm_hour, timeinfo.tm_mday, timeinfo.tm_mon+1, timeinfo.tm_year+1900);
  Serial.print(timeinfo.tm_hour);
  Serial.println(timeinfo.tm_mon);
  Serial.println(timeinfo.tm_year);
}

void displayTime(){
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(8, 10);
  display.print(String(rtc.getTime()));
  display.setTextSize(1);
  display.setCursor(8, 30);
  display.print(String(rtc.getDate()));
  if(!saver){
    display.setCursor(8, 50);
    display.print(WiFi.localIP().toString());
  }
}


void mainMenu(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(8, 5);
  display.print("Running");
  display.setCursor(8, 15);
  display.print("Health Test");
  display.setCursor(8, 25);
  display.print("Timer");
  display.setCursor(8, 35);
  if(!saver){
    display.print("Saver Mode");
  }
  else{
    display.print("Active Mode");
  }
  display.setCursor(8, 45);
  display.print("Home");
}


void runningMenu(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(8, 5);
  display.print("BPM: "+String(heartRateVal));
  display.setCursor(8, 15);
  display.print("#Steps: "+String(stepCount));
  display.setCursor(8, 25);
  display.print("Lat: "+String(gps.location.lat(),4));
  display.setCursor(8, 35);
  display.print("Lon: "+String(gps.location.lng(),4));
  display.setCursor(8, 45);
  display.print("Button1 to End");
}


void healthMenu(){
  if(!healthInit){
    for(int i=0; i<=10; i++){
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(8, 5);
      display.print("Reading Data");
      display.setCursor(8, 15);
      display.print(".");
      display.display();
      delay(500);
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(8, 5);
      display.print("Reading Data");
      display.setCursor(8, 15);
      display.print(" ");
      display.display();
      delay(500);
    }
    healthInit=true;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(8, 5);
  display.print("You're fit!");
  display.setCursor(8, 25);
  display.print("BPM: "+String(heartRateVal));
  display.setCursor(8, 35);
  display.print("SpO2: "+String(SPO2Val));
  display.setCursor(8, 45);
  display.print("Button1 to End");
  display.display();
  if(button1.isPressed()){
    state=1;
    healthInit=false;
  }
}


void saverMode(){
  if(saver){
    initWiFi();
    initSPIFFS();
    initServer();
    initTime();
    saverInit=true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(8, 25);
    display.print("Activated");
    display.display();
    saverInit=false;
    saver=false;
    state=0;
    sleepTime1 = millis();
    delay(500);
  }
  else{
    if(!saverInit){
      WiFi.disconnect();
      for(int i=0; i<=10; i++){
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(8, 25);
        display.print("Low Powering");
        display.setCursor(8, 35);
        display.print(".");
        display.display();
        delay(500);
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(8, 25);
        display.print("Low Powering");
        display.setCursor(8, 35);
        display.print(" ");
        display.display();
        delay(500);
      }
      saverInit=true;
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(8, 25);
    display.print("Done");
    display.display();
    saverInit=false;
    saver=true;
    state=0;
    delay(500);
    sleepTime1 = millis();
  }
}


void timerMenu(){
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(45, 10);
  display.print("Timer");
  display.setCursor(30, 30);
  display.setTextSize(2);
  display.print(minute);
  display.setCursor(55, 30);
  display.print(":");
  display.setCursor(70, 30);
  display.print(second);
  display.display();
  if(button1.isPressed()&&timeState==0){
    timeState=1;
  }
  else if(button1.isPressed()&&timeState==1){
    timeState=2;
    startTime=millis();
  }
  if(button2.isPressed()&&timeState==0){
    if(minute<99){
      minute++;
    }
    else{
      minute=0;
    }
    minuteCopy=minute;
  }
  else if(button2.isPressed()&&timeState==1){
    if(second<59){
      second++;
    }
    else{
      second=0;
    }
    secondCopy=second;
  }
  if(timeState==2){
    if(second==0){
      minute=minute-1;
      secondCopy=59;
      second=59;
      delay(1000);
      if(minute==-1){
        minute=0;
        second=0;
        digitalWrite(2, HIGH);
        timeState=3;
        tone(buzzer, 450);
        delay(500);
      }
    }
    if(timeState!=3){
      second=secondCopy-((millis()-startTime)%60000)/1000;
    }
    delay(100);
    if(button2.isPressed()){
      minute=0;
      second=0;
      state=0;
      sleepTime1 = millis();
      timeState=0;
      noTone(buzzer);
    }
  }
  if(button1.isPressed()&&timeState==3){
    state=0;
    sleepTime1 = millis();
    timeState=0;
    noTone(buzzer);
  }
}


void initMAX(){
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384


  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}


String getSensorReadings(){
  readings["heartrate"] = String(heartRateVal);
  readings["steps"] = String(stepCount);
  readings["SpO2"] = String(SPO2Val);
  readings["latitude"] = String(gps.location.lat());
  readings["longitude"] = String(gps.location.lng());
  String jsonString = JSON.stringify(readings);
  return jsonString;
}


void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}


void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(8, 25);
    display.print("Activating");
    display.setCursor(8, 35);
    display.print(".");
    display.display();
    delay(500);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(8, 25);
    display.print("Activating");
    display.setCursor(8, 35);
    display.print(" ");
    display.display();
    delay(500);
  }
  Serial.println(String(WiFi.localIP()));
}


void initServer(){
    // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });


  server.serveStatic("/", SPIFFS, "/");


  // Request for the latest sensor readings
  server.on("/readings", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = getSensorReadings();
    request->send(200, "application/json", json);
    json = String();
  });


  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);


  // Start server
  server.begin();
}


void initMPU() {
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}


void readAccelerometerData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accX = a.acceleration.x;
  float accY = a.acceleration.y;
  float accZ = a.acceleration.z;

  float accMagnitude = sqrt(accX * accX + accY * accY + accZ * accZ);

  // Peak detection
  if (accMagnitude < 8.5 & flag==0) {
    stepCount++;
    flag = 1;
  }
  if (accMagnitude > 10){
    flag = 0;
  }
  //Serial.print(accMagnitude);
  //Serial.print(",");
  Serial.println(stepCount);
}


void displayStepCount() {
  Serial.print("Steps: ");
  Serial.println(stepCount);
}

void initGPS() {
  Serial2.begin(GPSBaud, SERIAL_8N1, 16, 17);
  readGPS();
}

void readGPS() {
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      displayInfo();


  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

void displayInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  Serial.println();
}

void taskOne( void * parameter ){
  for(;;){
    button1.loop(); // MUST call the loop() function first
    button2.loop(); // MUST call the loop() function first
    //Serial.println(state);
    if(state!=0){
      sleepTime2 = 0;
    }
    if(state==0){
      displayTime();
      display.display();
      if(button1.isPressed()){
        state=1;
        if(firstInit){
          state=0;
          firstInit=false;
        }
      }
      if(saver){
        sleepTime2 = millis()-sleepTime1;
        if(sleepTime2>30000){
          sleepTime2 = 0;
          digitalWrite(LCD_POWER, LOW);
          esp_deep_sleep_start();
        }
      }
    }
    else if(state==1){
      mainMenu();
      display.setCursor(0, 5);
      display.print(">");
      display.display();
      if(button2.isPressed()){
        state=2;
      }
      else if(button1.isPressed()){
        state=6;
      }
    }
    else if(state==2){
      mainMenu();
      display.setCursor(0, 15);
      display.print(">");
      display.display();
      if(button2.isPressed()){
        state=3;
      }
      else if(button1.isPressed()){
        state=7;
      }
    }
    else if(state==3){
      mainMenu();
      display.setCursor(0, 25);
      display.print(">");
      display.display();
      if(button2.isPressed()){
        state=4;
      }
      else if(button1.isPressed()){
        state=8;
      }
    }
    else if(state==4){
      mainMenu();
      display.setCursor(0, 35);
      display.print(">");
      display.display();
      if(button2.isPressed()){
        state=5;
      }
      else if(button1.isPressed()){
        saverMode();
      }
    }
    else if(state==5){
      mainMenu();
      display.setCursor(0, 45);
      display.print(">");
      display.display();
      if(button2.isPressed()){
        state=1;
      }
      else if(button1.isPressed()){
        state=0;
        sleepTime1 = millis();
      }
    }
    ////////////
    else if(state==6){
      readAccelerometerData();
      displayStepCount();
      readGPS();
      runningMenu();
      display.display();
      if(button1.isPressed()){
        state=0;
        sleepTime1 = millis();
      }
    }
    else if(state==7){
      healthMenu();
      if(button1.isPressed()){
        state=0;
        sleepTime1 = millis();
      }
    }
    else if(state==8){
      timerMenu();
    }
    if ((millis() - lastTime) > timerDelay) {
      // Send Events to the client with the Sensor Readings Every 10 seconds
      events.send("ping",NULL,millis());
      events.send(getSensorReadings().c_str(),"new_readings" ,millis());
      lastTime = millis();
    }
    //Serial.println(sleepTime2);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}


void taskTwo( void * parameter ){
  for(;;){
    bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps


  //read the first 100 samples, and determine the signal range
    for (byte i = 0 ; i < bufferLength ; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data


      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample


//      Serial.print(F("red="));
//      Serial.print(redBuffer[i], DEC);
//      Serial.print(F(", ir="));
//      Serial.println(irBuffer[i], DEC);
    }


    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);


    //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
    while (1)
    {
      //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
      for (byte i = 25; i < 100; i++)
      {
        redBuffer[i - 25] = redBuffer[i];
        irBuffer[i - 25] = irBuffer[i];
      }


      //take 25 sets of samples before calculating the heart rate.
      for (byte i = 75; i < 100; i++)
      {
        while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data


        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample(); //We're finished with this sample so move to next sample
//        Serial.print(F("red="));
//        Serial.print(redBuffer[i], DEC);
//        Serial.print(F(", ir="));
//        Serial.print(irBuffer[i], DEC);
//
//
//        Serial.print(F(", HR="));
//        Serial.print(heartRate, DEC);
//
//
//        Serial.print(F(", HRvalid="));
//        Serial.print(validHeartRate, DEC);
//
//
//        Serial.print(F(", SPO2="));
//        Serial.print(spo2, DEC);
//
//
//        Serial.print(F(", SPO2Valid="));
//        Serial.println(validSPO2, DEC);


        if(validHeartRate){
          heartRateVal=heartRate;
        }
        if(validSPO2){
          SPO2Val=spo2;
        }
      }
      maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
      vTaskDelay(20 / portTICK_PERIOD_MS);
    }
  }
}


void setup(){
  Serial.begin(115200);
  Wire.begin();
  initMAX();
  initMPU();
  initGPS();
  pinMode(SCROLL_PIN, INPUT);
  pinMode(PROCEED_PIN, INPUT);
  pinMode(LCD_POWER, OUTPUT);
  digitalWrite(LCD_POWER, HIGH);
  button1.setDebounceTime(10); // set debounce time to 50 milliseconds
  button2.setDebounceTime(10); // set debounce time to 50 milliseconds
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  touchAttachInterrupt(T9, callback, 20);
  esp_sleep_enable_touchpad_wakeup();
  xTaskCreate(
                    taskOne,          /* Task function. */
                    "TaskOne",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */
 
  xTaskCreate(
                    taskTwo,          /* Task function. */
                    "TaskTwo",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */
 
  delay(2000);
}


void loop(){
 
}
