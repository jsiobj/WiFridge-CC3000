#define ECHO_TO_SERIAL 1
#define DEBUG

#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include "utility/debug.h"

#include <SPI.h>
#include <PString.h>
//#include <avr/wdt.h>

// For DS18B20 temp sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// For AM2302 temp & humidity sensor
#include "DHT.h"

//char API_EMONCMS_PRIV[33];
//char HOST_EMONCMS_PRIV[33];
//char API_EMONCMS_LOCAL[33];
//char HOST_EMONCMS_LOCAL[17];

#define API_EMONCMS_PRIV "youremoncmskey"
#define HOST_EMONCMS_PRIV "your.emoncms.whatever"

#define NODE 0

PROGMEM char HTTP_OK[]="HTTP/1.1 200 OK";
PROGMEM char EMON_CMS_OK[]="ok";

//========================================================================================================
// CC3000 Configuration
//========================================================================================================
#define ADAFRUIT_CC3000_IRQ   3
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT, SPI_CLOCK_DIV2);

//========================================================================================================
// Pin Configuration
//========================================================================================================

// IN
const char pinDS18B20 = 8;
const char pinAM2302 = 9;

// OUT
const int pinLedRed=2;
const int pinLedYellow=6;
const int pinLedGreen=7;

// Buffers & Strings
char buf[16];
String payLoad;

//char payLoadBuf[255];
//PString payLoad(payLoadBuf,sizeof(payLoadBuf));

char floatBuf[8];
int webErrCount=0;
int failedToSend=0;
char freeMem[6];

//========================================================================================================
// DS18B20 Temp sensor configuration
//========================================================================================================
OneWire oneWire(pinDS18B20);          // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature. 
const int TempPrecision = 9;
DeviceAddress sensorFridge, sensorFreezer;
boolean isSensorFridgeActive,isSensorFreezerActive;

//========================================================================================================
// AM2302 Temp sensor configuration
//========================================================================================================
DHT dht(pinAM2302, DHT22);

//========================================================================================================
// Update interval
//========================================================================================================
const unsigned long minIntervalTempRead = 60000;    // Interval between temp reads (in ms)
unsigned long lastTempRead=0;                       // last millis() Temp was read
unsigned long lastNtpRequest=0;                     // last millis() NTP request sent
unsigned long currentEpoch=0;
unsigned long readLoopDuration;

//========================================================================================================
void setup()
{
  Serial.begin(115200);

  Serial.print(F("==== Setup"));
  pinMode(pinLedRed, OUTPUT);
  pinMode(pinLedYellow, OUTPUT);
  pinMode(pinLedGreen, OUTPUT);

  ledTrain();

  // -----------------------------------------------------------------------------------------------------
  // Start Wifi - CC3000 was configured using SmartConfig
  // -----------------------------------------------------------------------------------------------------
  Serial.println(F("\nInitializing Wifi..."));
  if (!cc3000.begin(false, true))
  {
    Serial.println(F("Unable to re-connect!? Did you run the SmartConfigCreate"));
    Serial.println(F("sketch to store your connection details?"));
    while(1) {
      allLedBlink(3,1000);
      delay(1000);
    }
  }
  
  Serial.println(F("Reconnected!"));
  
  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  //delay(500);
  while (!cc3000.checkDHCP())
  {
    Serial.println(F("Waiting for DHCP..."));
    delay(100); // ToDo: Insert a DHCP timeout!
    //ledBlink(pinLedYellow,3,100);
  }  

  displayConnectionDetails();
  
  Serial.println(F("Initializing DS18B20..."));
  sensors.begin();
  isSensorFridgeActive=sensors.getAddress(sensorFridge, 0);
  isSensorFreezerActive=sensors.getAddress(sensorFreezer, 1);
  
  if (!isSensorFridgeActive) { Serial.println("Unable to find Fridge temperarture sensor"); }
  else { Serial.print("Fridge  temperarture sensor (0) : ");  Serial.println((int) sensorFridge,DEC); }
  
  if (!isSensorFreezerActive) Serial.println("Unable to find Freezer temperarture sensor");  
  else { Serial.print("Freezer temperarture sensor (4) : ");  Serial.println((int) sensorFreezer,DEC); }

  Serial.println(F("Sending boot flag..."));
  payLoad += "WiFridgeStartup:1";
  sendData2EmonCms("/input/post.json",HOST_EMONCMS_PRIV,80,API_EMONCMS_PRIV,&payLoad);
  //payLoad="";

  Serial.println(F("End of setup"));
  Serial.println();
  
  ledBlink(pinLedGreen,3,100);
}

//=======================================================================================================
void loop()
{
  if(!lastTempRead || millis()-lastTempRead>minIntervalTempRead) {

    Serial.println(F("==== Start loop..."));
    digitalWrite(pinLedGreen, HIGH);
    lastTempRead=millis();

    // -----------------------------------------------------------------------------------------------------
    // Getting temp from DS18B20 and sending it to emoncms
    // -----------------------------------------------------------------------------------------------------
    Serial.println(F("    Reading 1-wire sensors..."));
    sensors.requestTemperatures(); // Send the command to get temperatures
    
    if(isSensorFridgeActive) {
      dtostrf(sensors.getTempC(sensorFridge), 0, 2, floatBuf);
      addAttr(&payLoad,"tempFridge",floatBuf);
    }
    
    if(isSensorFreezerActive) {
      dtostrf(sensors.getTempC(sensorFreezer), 0, 2, floatBuf);
      addAttr(&payLoad,"tempFreezer",floatBuf);
    }
    
    // -----------------------------------------------------------------------------------------------------
    // Getting temp & humidity from AM2302
    // -----------------------------------------------------------------------------------------------------
    #if ECHO_TO_SERIAL
    Serial.println(F("    Reading AM2302 sensor..."));
    #endif
    
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    
    if(isnan(h)) { Serial.println(F("    Could not read humidity")); } 
    else {
      dtostrf(h, 0, 2, floatBuf);
      addAttr(&payLoad,"humidityKitchen",floatBuf);
    }
    
    if(isnan(h)) { Serial.println(F("    Could not read temp")); } 
    else {
      dtostrf(t, 0, 2, floatBuf);
      addAttr(&payLoad,"tempKitchen",floatBuf);
    }
   
    Serial.println(F("\nSending..."));
    
    sendData2EmonCms("/input/post.json",HOST_EMONCMS_PRIV,80,API_EMONCMS_PRIV,&payLoad);

    payLoad="";

    readLoopDuration=millis() - lastTempRead;
    Serial.print(F("\nRead & send : took "));
    Serial.println(readLoopDuration);
    digitalWrite(pinLedGreen, LOW);
  }
}

// -----------------------------------------------------------------------------------------------------
// Building pay load
// -----------------------------------------------------------------------------------------------------
void addAttr(String *payLoad, char *attrName, char *attrValue) {
  if(payLoad->length()>0) { payLoad->concat(","); }
  payLoad->concat(attrName);
  payLoad->concat(":");
  payLoad->concat(attrValue);
}

// -----------------------------------------------------------------------------------------------------
// Sending data to EmonCms
// -----------------------------------------------------------------------------------------------------
void sendData2EmonCms(char *urlprefix,char host[],int port, char apikey[], String *payLoad) {
 
  uint8_t d1,d2,d3,d4; 
  uint32_t ip=0;

  if(sscanf(host,"%u.%u.%u.%u",&d1,&d2,&d3,&d4)==4) {
    Serial.println(F("    Converting dotted IP into int32"));
    Serial.print(d1);Serial.print(".");Serial.print(d2);Serial.print(".");Serial.print(d3);Serial.print(".");Serial.println(d4);
    ip = cc3000.IP2U32(d1,d2,d3,d4);
  }
  else {
    Serial.print(F("    Get host by name :"));
    Serial.println(host);
    while (ip == 0) {
      if (! cc3000.getHostByName(host, &ip)) {
        Serial.println(F("    Couldn't resolve!"));
      }
      delay(500);
    }
  }
  
  char httpRequestBuf[255];
  PString httpRequest(httpRequestBuf,sizeof(httpRequestBuf));
  httpRequest.print(F("GET "));
  httpRequest.print(urlprefix);
  httpRequest.print(F("?apikey="));
  httpRequest.print(apikey);
  httpRequest.print(F("&node="));
  httpRequest.print(NODE);
  httpRequest.print(F("&json={"));
  httpRequest.print(*payLoad);
  httpRequest.print(F("} HTTP/1.1\r\n"));
  httpRequest.print(F("Host: "));
  httpRequest.print(host);
  httpRequest.print("\r\n\r\n");
 
  #if ECHO_TO_SERIAL
  Serial.println(F("\nDestination:"));
  Serial.print(ip); Serial.print("/");cc3000.printIPdotsRev(ip); Serial.print(F(":")); Serial.println(port);
  Serial.print(F("\nHTTP Request: "));
  Serial.println(httpRequestBuf);
  #endif
  
  Serial.println(F("\n    Connecting"));
  unsigned long sendStart=millis();
  digitalWrite(pinLedYellow, HIGH);
  Adafruit_CC3000_Client www = cc3000.connectTCP(ip, port);
  if (www.connected()) {

    #if ECHO_TO_SERIAL
    Serial.println(F("    Sending"));
    #endif

    www.fastrprint(httpRequestBuf);
    www.println();
  }
  else {
    #if ECHO_TO_SERIAL
    Serial.println(F("    Could not even connect..."));
    #endif
    allLedBlink(5,50);
    webErrCount++;
  }    
    
  #if ECHO_TO_SERIAL
  Serial.println(F("    Processing Answer"));
  #endif
    
  unsigned long timeout  = 3000;
  unsigned long lastRead = millis();
  while(www.connected() && (millis()-lastRead < timeout)) {
    while (www.available()) {
      char c = www.read();
      Serial.print(c);
      lastRead=millis();
    }
  }
  
  digitalWrite(pinLedYellow, LOW);
  Serial.println(F("    Closing & disconnecting"));
  www.close();
  cc3000.disconnect();

  Serial.print(F("\n    Request processed in "));
  Serial.print(millis() - sendStart); Serial.println();
}

// -----------------------------------------------------------------------------------------------------
// Reset Arduino (using Watchdog)
// -----------------------------------------------------------------------------------------------------
//void resetArduino(void) {
//  allLedBlink(5,200);
//  for(;;);
//}

// -----------------------------------------------------------------------------------------------------
// Leds utilities
// -----------------------------------------------------------------------------------------------------
void ledBlink(int led,int count,int wait) {
  int i;
  for(i=0;i<count;i++) {
    digitalWrite(led,HIGH); delay(wait);
    digitalWrite(led,LOW); delay(wait);    
  }
}

void allLedBlink(int count,int wait) {
  int i;
  for(i=0;i<count;i++) {
    digitalWrite(pinLedRed, HIGH); digitalWrite(pinLedYellow, HIGH); digitalWrite(pinLedGreen, HIGH);
    delay(wait);
    digitalWrite(pinLedRed, LOW); digitalWrite(pinLedYellow, LOW); digitalWrite(pinLedGreen, LOW);
    delay(wait);    
  }
}

void ledTrain() {
    digitalWrite(pinLedRed, HIGH); delay(200);
    digitalWrite(pinLedYellow, HIGH); delay(200);
    digitalWrite(pinLedGreen, HIGH); delay(200);
    
    digitalWrite(pinLedRed,LOW); delay(200);
    digitalWrite(pinLedYellow, LOW); delay(200);
    digitalWrite(pinLedGreen, LOW); delay(200);
}

/**************************************************************************/
/*!
    @brief  Tries to read the IP address and other connection details
*/
/**************************************************************************/
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}

