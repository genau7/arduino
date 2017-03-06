#include <WiFi101.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiSSLClient.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <WiFi101.h>
#include <Adafruit_NeoPixel.h>

#define PIN_A 6
#define PIN_B 7
#define STRIPNUM 2  //number of led strips - has to match number of pins
#define STRIP_A 0   //each strip has to have a unique id starting from 0
#define STRIP_B 1
#define PORT 5555
#define NO_MORE_ACTIVE_LEDS -1
#define LEDNUM 324
#define ON 1
#define OFF 0


char ssid[] = "Asiak";      //  your network SSID (name)
char pass[] = "eciepecie";   // your network password
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
int ledpin = 7;
bool val = true;

int status = WL_IDLE_STATUS;
WiFiServer server(PORT);

Adafruit_NeoPixel strip = Adafruit_NeoPixel(LEDNUM, PIN_A, NEO_GRB + NEO_KHZ800);
uint32_t colors[STRIPNUM][LEDNUM]={0}; // for each strip there is a uint32_t variable placed in the array
int activeLeds;                       // blink flag (8bits) + r (8bits) + g (8bits) + b(8bits)

//functions declarations:
void initLeds();
void switchOffAllLed(uint8_t stripNr);
void saveNewData(WiFiClient* client, uint8_t stripNr);
int getNextActiveLed(WiFiClient* client);
void saveLedColor(uint8_t ledId, WiFiClient* client, uint8_t stripNr);
void switchLedOff(int ledId, uint8_t stripNr);
void setLights(uint8_t stripNr, uint8_t on);
uint8_t red(uint32_t color);
uint8_t green(uint32_t color);
uint8_t blue(uint32_t color);
uint8_t blinkFlag(uint32_t color);
uint8_t pinNr(uint8_t stripNr);

void setup() {
  Serial.begin(9600);      // initialize serial communication
  Serial.print("Start Serial ");
  pinMode(ledpin, OUTPUT);      // set the LED pin mode
  initLeds();
 
  // Check for the presence of the shield
  Serial.print("WiFi101 shield: ");
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("NOT PRESENT");
    return; // don't continue
  }
  Serial.println("DETECTED");
  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) {
    digitalWrite(ledpin, LOW);
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   // print the network name (SSID);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  server.begin();                           // start the web server
  printWifiStatus();                        // you're connected now, so print out the status
}

void initLeds(){
  strip.begin();
  strip.show();
}

uint8_t pinNr(uint8_t stripNr){
  switch(stripNr){
    case STRIP_A:
      return PIN_A;
    case STRIP_B:
      return PIN_B;
  }
  return 0;
}
void loop() {
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        Serial.print("Client.available()=");
        Serial.println(client.available());
        int pinNr = client.readStringUntil(' ').toInt();
        Serial.println(pinNr);                   
        if(pinNr!=PIN_A && pinNr != PIN_B) {
            Serial.print("Wrong PIN nr: should be 8 or 7. It is instead  _");
            Serial.println(pinNr);
        }
        else{
          uint8_t stripNr = STRIP_A;
          strip.setPin(PIN_A);
          if(pinNr == PIN_A)
            strip.setPin(PIN_A);
          else{
            strip.setPin(PIN_B); stripNr=STRIP_B;}
        
          activeLeds = client.parseInt();//number of LEDs whose color will change
          Serial.print(activeLeds);
          Serial.println(" is the nr of LEDs which will change color\n");

          if(activeLeds <= 0)
            switchOffAllLed(stripNr);
          else
            saveNewData(&client, stripNr);
          client.stop();
        }       
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disonnected");   
  }
  
  setLights(STRIP_A, ON);
  setLights(STRIP_B, ON);
  delay(1000);
  setLights(STRIP_A, OFF);
  setLights(STRIP_B, OFF);
  delay(1000);
}

void setLights(uint8_t stripNr, uint8_t on){
  strip.clear();
  strip.setPin(pinNr(stripNr));
  //Serial.print("StripNr, pinNr: "); Serial.print(stripNr); Serial.print(", "); Serial.println(pinNr(stripNr));
  for (uint16_t i=0; i<LEDNUM; i++){
    uint32_t color = colors[stripNr][i];
    if(!on && blinkFlag(color)== ON)
      strip.setPixelColor(i, 0, 0, 0);
    else
      strip.setPixelColor(i, red(color), green(color), blue(color)); 
   /* if(on){
      strip.setPixelColor(i, red(color), green(color), blue(color));
      if(stripNr==0 && color!=0)
        Serial.println(color, HEX);
    }
    else if(blinkFlag(color) == ON)
      strip.setPixelColor(i, 0, 0, 0);
    else
      strip.setPixelColor(i, red(color), green(color), blue(color));*/
   }
   strip.show();
}

void switchOffAllLed(uint8_t stripNr){
  for (uint16_t i=0; i<LEDNUM; i++){
    strip.setPixelColor(i, 0, 0, 0);//
    colors[stripNr][i]=0;   
  }  
  strip.show();
}

void saveNewData(WiFiClient* client, uint8_t stripNr){
  int activeLedIndex=getNextActiveLed(client);
  for(uint16_t LED=0; LED<LEDNUM; LED++){
    if(LED == activeLedIndex){
      saveLedColor(LED, client, stripNr);
      activeLedIndex=getNextActiveLed(client);      
    }
    else
      switchLedOff(LED, stripNr);
  }
  strip.show();
}

int getNextActiveLed(WiFiClient* client){
  int nextLedactive = NO_MORE_ACTIVE_LEDS;  
  if(activeLeds>0){
    activeLeds--; 
    nextLedactive= client->parseInt() - 1;
  }
  return nextLedactive;  
}

void saveLedColor(uint8_t ledId, WiFiClient* client,uint8_t stripNr){
  //find the number representing color and check if led should blink
  uint8_t blinkFlag = client->parseInt();
  uint8_t r=client->parseInt();
  uint8_t g=client->parseInt();
  uint8_t b=client->parseInt();
  colors[stripNr][ledId] = ((uint32_t)blinkFlag << 24) | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
  Serial.print("SaveLedColor: StripNr, pinNr: "); Serial.print(stripNr); Serial.print(", "); Serial.println(pinNr(stripNr));
  Serial.print("(");   Serial.print(r);   Serial.print(", ");    Serial.print(g);   Serial.print(",");  Serial.print(b);  Serial.print(")   ");
  Serial.print("(");   Serial.print(r, HEX);   Serial.print(", ");    Serial.print(g, HEX);   Serial.print(",");  Serial.print(b,HEX);  Serial.print(")   ");
  Serial.println(colors[stripNr][ledId], HEX);
}

void switchLedOff(int ledId, uint8_t stripNr){
   strip.setPixelColor(ledId, 0, 0, 0);
}

uint8_t red(uint32_t color){
  return (uint8_t)((color >> 16) & 0xFF);
}

uint8_t green(uint32_t color){
  return (uint8_t)((color >> 8) & 0xFF);
}

uint8_t blue(uint32_t color){
  return (uint8_t)(color  & 0xFF);
}

uint8_t blinkFlag(uint32_t color){
  return (uint8_t)(color >> 24);
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

