//includes for ESP8266 Wifi and server
#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#include <EEPROM.h>
//EEPROM map
//bytes 0 - 7 ControlAddress
//bytes 8 - 15 double P
//bytes 16 - 23 double I
//bytes 24 - 31 double D
//bytes 32 - 39 double targetTemp
//bytes 40 - 47 LimiterAdress
//bytes 48 - 55 double limiterMax
//bytes 56 - 88 SSID char[33]
//bytes 89 - 153 WifiPasswd char[65]


// Pin definitions
#define ONE_WIRE_BUS D1 // Data wire for sensors
#define outputPin D2 //Output pin for heat power
#define SERVO1PIN D3 //Pin for servo 1
#define SERVO2PIN D4 //Pin for servo 2

// Pins for LEDs
int LEDpin[4]={D5, D6, D7, D8};

//includes for Servo
#include <Servo.h>

//includes for temperature sensors
#include <OneWire.h>
#include <DallasTemperature.h>

//include for PID-library
#include <PID_v1.h> //http://playground.arduino.cc/Code/PIDLibrary, https://github.com/br3ttb/Arduino-PID-Library

//Temperature resolution
#define TEMPRES 12

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress Thermometer[5];

//Address of the controling themometer
byte ControlAddress[8];
int ControlDevice=-1;

//Adress of the temperature limiting sensor thormometer
byte LimiterAddress[8];
int LimiterDevice=-1;

// Variable to hold number of devices found
int numDev;

//Set up servo
Servo Servo1;
Servo Servo2;

//Vars for temp
double targetTemp;
double currentTemp;
double Temp[5];
double output = 0;

//Var for temp limiter sensor
double limiterMax=50;
double limiterTemp;

#define LOWLIMIT 0.0 //lowest acceptable value to limiter sensor
#define HIGHLIMIT 100.0 //highest acceptable limit to limiter sensor

//Vars for PID
double Kp;
double Ki;
double Kd;

//Var for Servo positions
int Servo1Pos=0;
int Servo2Pos=0;

//Var for timings
unsigned long conversionStart=0; //time of last tempoerature conversion start
unsigned long heatPulseStart=0; // time of last heatpulse start
unsigned long heatDutyOff=0; //time when heat duty cycle ends and heater shuts off
unsigned long servoStart=0; //var to hold time of servo start

//var to know if conversion happened this cycle
boolean converted=false;

//var to know if servo was enabled this move
boolean servoMoved=true;

//Time settings
#define CONVTIME 1000 //time of conversion in ms
#define TEMPCYCLE 2000 //time to next temperature reading
#define HEATPULSE 2000 //time of heat pulse cycle in ms
#define SERVOTIME 1000 //time to activate servo for a move

//LED status
boolean LED[4];

//Specify the links and initial tuning parameters
PID myPID(&currentTemp, &output, &targetTemp, Kp, Ki, Kd, DIRECT);

// Network credentials
char ssid[33];
char password[65];
char appasword[65];

//SoftAP data
IPAddress apIP(192,168,0,1);
IPAddress apgateway(192,168,0,1);
IPAddress apsubnet(255,255,255,0);

//Staion mode data
IPAddress ip;

ESP8266WebServer server(80);

void handleRoot() {
  server.send(200, "text/html", "OK");
}

void handleReport() {

  /*
   * Reports:
   * CntrlTemp: temp of control sensor
   * LimitTemp: temp of limiter sensor
   * TargetTemp: target temperature
   * LimitMax: maximal limiter temperature
   * Temps: all sensors' values
   * TempsOut: all sensors and output, with index e.g. C:37.00 1:37.00 2:40.34 O:20, C=contrl temp, N:sensor with index N, O=output
   * NumDev: number of sensors
   * DevAddrs: address of sensors   
   * CntrlAddr: index and address of control sentor
   * LimitAddr: indes and address of limiter sensor
   * Servos: position of servos
   * Output: current outputvalue
   * PID: values of Kp, Ki, Kd 
   * EEPROM: report values in EEPROM
   * LEDs: report status of all LEDs
   */

  String text="";
  int i;
  byte Addr[8];
  double fl;
  
  if (server.arg("cmd")=="CntrlTemp") {text = String("CntrlTemp ") + currentTemp;}
  else if (server.arg("cmd")=="LimitTemp") {text = String("LimitTemp ") + limiterTemp;}
  else if (server.arg("cmd")=="TargetTemp") {text = String("TargetTemp ") + targetTemp;}
  else if (server.arg("cmd")=="LimitMax") {text = String("LimitMax ") + limiterMax;}
  else if (server.arg("cmd")=="Temps") {
    text = "Temps ";
    for (int i=0;i<numDev;i++) {
      text += Temp[i];
      text += ' ';
    }
  }
  else if (server.arg("cmd")=="TempsOutput") {
    text = "C:";
    text += currentTemp;
    text += " ";
    for (int i=0;i<numDev;i++) {
      text += i;
      text += ":";
      text += Temp[i];
      text += ' ';
    }
    text += "O:";
    text += output;
  }
  else if (server.arg("cmd")=="NumDev") {text=String("NumDev ") + numDev;}
  else if (server.arg("cmd")=="DevAddrs") {
    text="DevAddrs ";
    for (int i=0;i<numDev;i++) {
      text += Address(Thermometer[i]);
      text += ' ';
    }
  }
  else if (server.arg("cmd")=="CntrlAddr") {
    if (ControlDevice==-1) {
      text="Error: No control device assiged.";
    } else {
      text = "CntrlAddr ";
      text += ControlDevice;
      text += ' ';
      text += Address(Thermometer[ControlDevice]);
    }
  }
  else if (server.arg("cmd")=="LimitAddr") {
    if (LimiterDevice==-1) {
      text="Error: No limiter device assiged.";
    } else {
      text = "LimitAddr ";
      text += LimiterDevice;
      text += ' ';
      text += Address(Thermometer[LimiterDevice]);
    }
  }
  else if (server.arg("cmd")=="Servos") {
    text = "Servos ";
    text += Servo1Pos;
    text += ' ';
    text += Servo2Pos;
  }
  else if (server.arg("cmd")=="Output") {text = String("Output ") + output;}
  else if (server.arg("cmd")=="PID") {
    text = "PID ";
    text += Kp;
    text += ' ';
    text += Ki;
    text += ' ';
    text += Kd;
  }
  else if (server.arg("cmd")=="EEPROM") {
    EEPROM.get(0, Addr);
    text = "ControlAddress " + Address(Addr);
    text += "<br>";
    EEPROM.get(8, fl);
    text +="Kp             " + String(fl);
    text += "<br>";
    EEPROM.get(16, fl);
    text +="Ki             " + String(fl);
    text += "<br>";
    EEPROM.get(24, fl);
    text +="Kd             " + String(fl);
    text += "<br>";
    EEPROM.get(32, fl);
    text +="Target temp    " + String(fl);
    text += "<br>";
    EEPROM.get(40, Addr);
    text +="LimiterAddress " + Address(Addr);
    text += "<br>";
    EEPROM.get(48, fl);
    text +="Limiter max    " + String(fl);
  }
  else if (server.arg("cmd")=="LEDs") {
    text = "LEDs ";
    for (i=0;i<4;i++) {
      text+=LED[i];
      text+=' ';
    }
  }
  else {text += "Error: Unrecognised command";
  }
  server.send(200, "text/html", text);
}

void handleCmd() {
  
  /*
   * cmd:value:ch
   * LimitMax: set temp of limiter sensor, above this no heating
   * TargetTemp: set target temperature
   * CntrlDev: assign control sensor by index
   * LimitDev: assign limit sensor by index
   * Servo1: set position of servo 1
   * Servo2: set position of servo 2
   * Servos: set position of both servos to value, value2
   * Kp: set Kp
   * Ki: set Ki
   * Kd: set Kd
   * Autotune: perform PID autotune (not implementd)
   * LED: set value of LED with channel No. "ch"
   */

  int i;
  String cmd = server.arg("cmd");
  String value = server.arg("value");
  String value2 = server.arg("value2");
  String ch = server.arg("ch");
  int pin=ch.toInt();
  String text = "";

  Serial.print(cmd);
  Serial.print(' ');
  Serial.println(value);
  
  if (cmd=="LimitMax") {
    if (limiterMax != value.toFloat()) {
      limiterMax = value.toFloat();
      text = "OK LimitTemp set to ";
      text += limiterMax;
      EEPROM.put(48, limiterMax);
      EEPROM.commit();
    }
  }
  else if (cmd=="TargetTemp") {
    if (targetTemp != value.toFloat()) {
      targetTemp = value.toFloat();
      text = "OK TargetTemp set to ";
      text += targetTemp;
      EEPROM.put(32,targetTemp);
      EEPROM.commit();
    }
  }
  else if (cmd=="CntrlDev") {
    if (ControlDevice != value.toInt()) {
      ControlDevice = value.toInt();
      if (ControlDevice>numDev-1 || ControlDevice < 0) {
        ControlDevice=-1;
        text = "Error Control device does not exist.";
      } else {
        for (i=0;i<8;i++) {ControlAddress[i] = Thermometer[ControlDevice][i];}
        EEPROM.put(0,Thermometer[ControlDevice]);
        EEPROM.commit();
        text = "OK Control Device Address set to ";
        text += Address(Thermometer[ControlDevice]);
      }
    }
  }
  else if (cmd=="LimitDev") {
    if (LimiterDevice != value.toInt()) {
      LimiterDevice = value.toInt();
      if (LimiterDevice>numDev-1 || LimiterDevice < 0) {
        LimiterDevice=-1;
        text = "Error Limiter device does not exist.";
      } else {
        for (i=0;i<8;i++) {LimiterAddress[i] = Thermometer[LimiterDevice][i];}
        EEPROM.put(40,Thermometer[LimiterDevice]);
        EEPROM.commit();
        text = "OK Limiter Device Address set to ";
        text += Address(Thermometer[LimiterDevice]);
      }
    }
  }
  else if (cmd=="Servo1") {
    Servo1Pos=value.toInt();
    Servo1.write(Servo1Pos);
  }
  else if (cmd=="Servo2") {
    Servo2Pos=value.toInt();
    Servo2.write(Servo2Pos);
  }
  else if (cmd=="Servos") {
    Servo1Pos=value.toInt();
    Servo1.write(Servo1Pos);
    Servo2Pos=value2.toInt();
    Servo2.write(Servo2Pos);
  }
  else if (cmd=="Kp") {
    Kp=value.toFloat();
    myPID.SetTunings(Kp,Ki,Kd);
    EEPROM.put(8, Kp);
    EEPROM.commit();
  }
  else if (cmd=="Ki") {
    Ki=value.toFloat();
    myPID.SetTunings(Kp,Ki,Kd);
    EEPROM.put(16, Ki);
    EEPROM.commit();
  }
  else if (cmd=="Kd") {
    Kd=value.toFloat();
    myPID.SetTunings(Kp,Ki,Kd);
    EEPROM.put(24, Kd);
    EEPROM.commit();
  }
  else if (cmd=="LED") {
    digitalWrite(LEDpin[pin],value.toInt());
    LED[pin]=value.toInt();
    
    for (i=0;i<4;i++) {
      text+=LED[i];
      text+=' ';
    }
  }
  else if (cmd=="LEDs") {
    for (i=0;i<4;i++) {
      digitalWrite(LEDpin[i],value.toInt());
      LED[i]=value.toInt();
      
      text+=LED[i];
      text+=' ';
    }
  }
  else if (cmd=="Autotune") {text="Error: Autotune not implemented.";}
  else {text += "Error: Unrecognised command";}
  
  server.send(200, "text/html", text);
}

void handleNetwork() {
  
}

void setup() {
  delay(1000);
  
  //Read from EEPROM
  EEPROM.begin(154);
  EEPROM.get(0, ControlAddress);
  EEPROM.get(8, Kp);
  EEPROM.get(16, Ki);
  EEPROM.get(24, Kd);
  EEPROM.get(32, targetTemp);
  EEPROM.get(40, LimiterAddress);
  EEPROM.get(48, limiterMax);
  EEPROM.get(56, ssid);
  EEPROM.get(89, password);
  
  Serial.begin(250000); //Start serial
  Serial.println();

  //network config
  WiFi.mode(WIFI_AP);
  WiFi.disconnect();

  delay(500);

  // WiFi.begin(ssid, password); //Network credentials
  Serial.print("Soft AP Config: ");
  Serial.println(WiFi.softAPConfig(apIP,apgateway, apsubnet) ? "OK" : "Fail");
  
  Serial.print("Soft AP: ");
  Serial.println(WiFi.softAP("ATLIS", "ATLISkey") ? "OK" : "Fail");
  
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());  
  /*
  // Wait for connection
  int i=0;
  while ( WiFi.status() != WL_CONNECTED && i < 60 ) {
    delay ( 500 );
    Serial.print i;
    Serial.print ( " " );
    i++;
  }

  //Go into ap mode
  if (i=>60) {
    WiFi.disconnect();
    Wifi.mode(WIFI_AP);
    delay(500);
    
  }*/

  
  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.on("/report", handleReport);
  server.on("/network", handleNetwork);
      
  server.begin();
  Serial.println("HTTP server started");

  //Set ouputpin and LED pin to ouput
  pinMode(outputPin, OUTPUT); //output pin
  pinMode(BUILTIN_LED, OUTPUT); //LED pin

  Serial.print("Control Address from EEPROM: ");
  printAddress(ControlAddress);
  Serial.println("");

  Serial.print("Limiter Address from EEPROM: ");
  printAddress(LimiterAddress);
  Serial.println("");
  
  Serial.print("Locating devices.");
  //initialise the temp sensors on onewWire bus
  sensors.begin();

  // print number of sensors
  numDev=sensors.getDeviceCount();
  
  Serial.print("Found ");
  Serial.print(numDev);
  Serial.println(" devices");

  //assign temp sensor address and print it
  for (int i=0;i<numDev;i++) {
    if (!sensors.getAddress(Thermometer[i], i)) {
      Serial.print("Error: device ");
      Serial.print(i);
      Serial.println("");
      delay(1000);
    } else {
      Serial.print("Device ");
      Serial.print(i);
      Serial.print(": ");
      printAddress(Thermometer[i]);
      Serial.println("");
      sensors.setResolution(Thermometer[i], TEMPRES);
      delay(1000);

      //Check if this thermometer is the controlling one
      if (ByteArrayCompare(Thermometer[i],ControlAddress,8)) {ControlDevice=i;}
      
      //Check if this thermometer is the limiter      
      if (ByteArrayCompare(Thermometer[i],LimiterAddress,8)) {LimiterDevice=i;}
    }
  }

  if (ControlDevice>-1) {
    Serial.print("Control themometer is: ");
    Serial.print(ControlDevice);
    Serial.print("");
  } else {
    Serial.println("No control thermometer assigned.");
  }

  if (LimiterDevice>-1) {
    Serial.print("Limiter themometer is: ");
    Serial.print(LimiterDevice);
    Serial.println("");
  } else {
    Serial.println("No limiter thermometer assigned.");
  }

  //set no wait for conversion, let loop run instead
  sensors.setWaitForConversion(false);
  
  //initialise PID library
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0.0,100.0);
  
  printPID();
  
  myPID.SetTunings(Kp, Ki, Kd);

  //Servo initialisation
  Servo1.attach(SERVO1PIN);
  Servo2.attach(SERVO2PIN);

  for (int i=0;i<4;i++) {
    pinMode(LEDpin[i],OUTPUT);
    LED[i]=0;
  }
  
  delay(1000);
}

void loop() {
  // Request temperature from sensors
  if (millis()>conversionStart+TEMPCYCLE) {
    sensors.requestTemperatures();
    conversionStart=millis();
    converted = false;
  }

  //Read temp if time is right
  if (millis()>conversionStart+CONVTIME && converted==false) {   
    
    converted=true;
    
    for (int i=0;i<numDev;i++) {
      
      //Read
      double oldTemp = Temp[i];
      Temp[i] = sensors.getTempC(Thermometer[i]);

      //Debounce
      //if (Temp[i] == -127.0) {Temp[i] = oldTemp;}  //removed to check interference
      //if (Temp[i] == 0.0) {Temp[i] = oldTemp;}
      //if (Temp[i] == 85.0) {Temp[i] = oldTemp;}
      
      if (i==ControlDevice) {currentTemp=Temp[i];}
      if (i==LimiterDevice) {limiterTemp=Temp[i];}

      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(Temp[i]);
      Serial.println();
    }
    Serial.println();
    //PID recalculate with new temps
    myPID.Compute();

  }
  
  //Output heatpulse
  if (millis()>heatPulseStart+HEATPULSE && limiterTemp <= limiterMax && limiterTemp <= HIGHLIMIT && limiterTemp >= LOWLIMIT) {
    digitalWrite(outputPin, LOW); //signal inverted, low=on
    digitalWrite(BUILTIN_LED,LOW); //signal inverted, low=on
    heatPulseStart=millis();
    heatDutyOff=map(int(output),0,100,heatPulseStart,heatPulseStart+HEATPULSE);
  }

  if (millis()>heatDutyOff) {
    digitalWrite(outputPin,HIGH); //signal inverted, high=off
    digitalWrite(BUILTIN_LED,HIGH); //signal inverted, high=off
  }
  
  server.handleClient();
}

void printPID() {
  Serial.print("PID set to: ");
  Serial.print(Kp);
  Serial.print(" ");
  Serial.print(Ki);
  Serial.print(" ");
  Serial.print(Kd);
  Serial.println("");
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) {
       Serial.print("0");
    }
    Serial.print(deviceAddress[i], HEX);
  }
}

String Address(DeviceAddress deviceAddress)
{
  String a = "";
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) {
       a += '0';
    }
    a += String(deviceAddress[i], HEX);
  }
  return a;
}

boolean ByteArrayCompare(byte a[], byte b[], int array_size)
{
   for (int i = 0; i < array_size; ++i)
     if (a[i] != b[i])
       return(false);
   return(true);
}
