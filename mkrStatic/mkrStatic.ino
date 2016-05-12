#include <WiFi101.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiSSLClient.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <WiFi101.h>
#include <Adafruit_NeoPixel.h>

#define PIN_A 7
#define PIN_B 8
#define PORT 5555
#define NO_MORE_ACTIVE_LEDS -1
#define LEDNUM 324

char ssid[] = "Asiak";      //  your network SSID (name)
char pass[] = "eciepecie";   // your network password
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
int ledpin = 7;
bool val = true;

int status = WL_IDLE_STATUS;
WiFiServer server(PORT);

Adafruit_NeoPixel strip = Adafruit_NeoPixel(LEDNUM, PIN_A, NEO_GRB + NEO_KHZ800);
int activeLeds;

//functions declarations:
void switchOffAllLed();
void lightLeds(WiFiClient* client);
int getNextActiveLed(WiFiClient* client);
void setLedColor(int ledId, WiFiClient* client);
void switchLedOff(int ledId);

void setup() {
  strip.begin();
  strip.show();
  Serial.begin(9600);      // initialize serial communication
  Serial.print("Start Serial ");
  pinMode(ledpin, OUTPUT);      // set the LED pin mode
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
    digitalWrite(ledpin, HIGH);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  server.begin();                           // start the web server on port 80
  printWifiStatus();                        // you're connected now, so print out the status
  digitalWrite(ledpin, HIGH);
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
        int pinNum = client.readStringUntil(' ').toInt();
        Serial.println(pinNum);                    // print it out the serial monitor
        if(pinNum!=PIN_A && pinNum!=PIN_B) {
            Serial.print("Wrong PIN nr: should be 8 or 7. It is instead  _");
            Serial.println(pinNum);
        }
        else{
          if(pinNum==PIN_A)
            strip.setPin(PIN_A);
          else
            strip.setPin(PIN_B);
          
          activeLeds = client.parseInt();//number of LEDs whose color will change
          Serial.print(activeLeds);
          Serial.println(" is the nr of LEDs which will change color\n");

          if(activeLeds <= 0)
            switchOffAllLed();
          else
            lightLeds(&client);
           client.stop();
        }       
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disonnected");
  }
}

void switchOffAllLed(){
  for (uint16_t i=0; i<LEDNUM; i++){
    strip.setPixelColor(i, 0, 0, 0);   
  }  
  strip.show();
}

void lightLeds(WiFiClient* client){
  int activeLedIndex=getNextActiveLed(client);
  for(uint16_t LED=0; LED<LEDNUM; LED++){
    if(LED==activeLedIndex){
      setLedColor(LED, client);
      activeLedIndex=getNextActiveLed(client);      
    }
    else
      switchLedOff(LED);
  }
  strip.show();
}

int getNextActiveLed(WiFiClient* client){
  int nextLedactive=NO_MORE_ACTIVE_LEDS;  
  if(activeLeds>0){
    activeLeds--; 
    nextLedactive= client->parseInt()-1;
  }
  return nextLedactive;  
}

void setLedColor(int ledId, WiFiClient* client){
  //find the number representing color
  int r=client->parseInt();
  int g=client->parseInt();
  int b=client->parseInt();
   
  Serial.print("(");
  Serial.print(r);
  Serial.print(", ");
  Serial.print(g);
  Serial.print(",");
  Serial.print(b);
  Serial.println(")");
  
  //set color
  strip.setPixelColor(ledId, r, g, b);
}

void switchLedOff(int ledId){
   strip.setPixelColor(ledId, 0, 0, 0);
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

