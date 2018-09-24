/*Top Comment Lines
 * 
 * 
 * Meeting Room Indicator Light - changes colour based on status of the room recieved from Fusion
 * 
 * 
 * Published sept 2018
 * Author: NJ
 */
/*************************Headers***************************/
#include <Time.h>
#include <TimeLib.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>    //Wifi header
#include <ESP8266WiFiMulti.h> 
#include <WiFiUdp.h>      //Used to maintain time (sync to NTP server)
#include <ESP8266HTTPClient.h>  //create web clients
#include <ESP8266httpUpdate.h>  //over the air updates
#include <PubSubClient.h>     //MQTT library
#include <Adafruit_NeoPixel.h>  //Neopixel light control
#include <WiFiConnect.h>
/*************************Constants***************************/
#define LED_COUNT 12
#define LED_PIN D2    //control pin from ESP
#define TIMER_MS 5000
#define MQTT_KEEPALIVE 120
#define MQTT_MAX_PACKET_SIZE 512

/*************************global Variables***************************/
const int mqttPort = 1883;
const char* clientName = "";  //these three variables used for setting the client name to the Macaddress
String topicString;
char topicChar[18];
//int transitions = 6;
//int sides = 2;
//int segment_size = LED_COUNT/(transitions*sides);//best if light strip is in increments of 12

const char* topic_sub_roomupdate = "MRL/roomupdate";  //listen to this topic
const char* topic_pub = "MRL/status";
const char* topic_sub_firmware = "MRL/commands/firmware";  //listen for firmware update
time_t currenttime = 0;    //used for getting linux time
bool iscurrent = 0;           //is there a current meeting
bool iscurrentprev = 0;     //previous status current state
bool iscurrentstatechng = 1;
uint32_t starttime =0;                //start time of meeting - linux time
int duration = 0;                 //duration of meeting(minutes)
int elapsed = 0;                  //time elapsed (minutes)
int next = 0;                     //minutes to next meeting
unsigned long connect_time;
int Red = 0;
int Green = 0;
int Blue = 0;
int transition_effect=0;   
int effect=0;


WiFiClient espClient;         //wifi client
PubSubClient client(espClient); //MQTT client requires wifi client
ESP8266WiFiMulti wifiMulti;     //creates instance of wifi multi class
/*************************Setup time server******************************************/
//setup time server
WiFiUDP UDP;   //creates instance of UDP class to send and recieve 
IPAddress timeServerIP;
const char* NTPServerName = "time.nist.gov";
const int NTP_PACKET_SIZE = 48;  // NTP time stamp is in the first 48 bytes of the message
byte NTPBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming and outgoing packets
unsigned long intervalNTP = 60000; // Request NTP time every minute
unsigned long prevNTP = 0;
unsigned long lastNTPResponse = millis();
uint32_t timeUNIX = 0;
uint32_t actualTime=0;
uint32_t remainingUnix=0;
int remaining=0;

unsigned long prevActualTime = 0;


/************************setup light strip*****************************************/
// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);


/****************setup wifi************************************/
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
//  WiFiMulti.addAP(ssid, password);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }


  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  long rssi = WiFi.RSSI();
  Serial.print("RSSI:");
  Serial.println(rssi);

  
  topicString = WiFi.macAddress();          //sets up the unique clientName using the MacAddress
  topicString.toCharArray(topicChar, 18);
  clientName = topicChar;
  Serial.println(topicChar);
}

/*****************Firmware UPdate**********************************/

void updateFirmware(){
  
  t_httpUpdate_return ret = ESPhttpUpdate.update("http://99.231.14.167/UpdateMRL");
    //t_httpUpdate_return ret = ESPhttpUpdate.update("http://nj2299.duckdns.org/UpdateMRL");

      Serial.println(ret);
        switch(ret) {
            case HTTP_UPDATE_FAILED:
                Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
                break;
                
            case HTTP_UPDATE_OK:
                Serial.println("HTTP_UPDATE_OK");
                break;

            case HTTP_UPDATE_NO_UPDATES:
                Serial.println("NO UPDATES");
                break;

           default:
              Serial.println("Something else...");
              break;
        }
}


/*****************Connect to MQTT Broker**********************************/
void ConnectBroker(PubSubClient client, const char* clientName)
{
    while (!client.connected())
    {
        Serial.print("Connecting to MQTT: ");
        Serial.println(clientName);
        if(client.connect(clientName))      //command to connect to MQTT broker with the unique client name
        {
          Serial.println("Connected");
        }
        else
        {
          Serial.print("Failed with state ");
          Serial.println(client.state());
          delay(200);
        }
    }
} 
/*****************reconnect to MQTT Broker if it goes down**********************************/
void reconnect() {
  // Loop until we're reconnected
  delay(10);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }


  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  long rssi = WiFi.RSSI();
  Serial.print("RSSI:");
  Serial.println(rssi);
  
  
  
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect, just a name to identify the client
    if (client.connect(clientName)) {
      Serial.println("connected");
      
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


/************************setup lights***********************************/
void setup_lights(){
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

/************************send status***********************************/

void send_status(){

  
    StaticJsonBuffer<300> JSONbuffer;
    JsonObject& JSONencoder = JSONbuffer.createObject();
    JSONencoder["ID"] = clientName;
    JSONencoder["Red"] = Red;
    JSONencoder["Green"] = Green;
    JSONencoder["Blue"] = Blue;
    char JSONmessageBuffer[300];
    JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
    client.publish(topic_pub, JSONmessageBuffer);
    Serial.println(JSONmessageBuffer);
}


/*****************MQTT Listener******************************************************/
void callback(char* topic, byte* payload, unsigned int length2){
  //topicString = WiFi.macAddress();
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

   
  Serial.print("Message: ");
  for(int i = 0; i<length2;i++){
    Serial.print((char)payload[i]);
  }

  Serial.println();
  Serial.println("-------------");
  payload[length2] = 0;
  StaticJsonBuffer<500> JSONbuffer; 
  String inData = String((char*)payload);
  JsonObject& root = JSONbuffer.parseObject(inData);

  String ID = root["ID"];
  if(ID == "All" || ID == topicString){
    iscurrent = root["isCurrent"];
    starttime = root["start"];
    duration = root["duration"];
    elapsed = root["elapsed"];
    next = root["next"];
    remaining = duration - elapsed;
    


    Serial.println(iscurrent);
    Serial.println(starttime);
    Serial.println(duration);
    Serial.println(elapsed);
    Serial.println(next);
    Serial.println(actualTime);
    Serial.println(remaining);
    //Serial.println(remainingUnix);

    if (iscurrent != iscurrentprev){
      iscurrentstatechng = 1;
      iscurrentprev = iscurrent;
    }
  }

  
         
}

/***************Startup Message - sent on bootup connection******************************************************/
//To universal Topic
void sendStartupMessage(){
  
  //time_t now = time(nullptr);
  time_t connect_time = now();
  Serial.println("Time is now: ");
  Serial.println(connect_time);
  
  StaticJsonBuffer<100> JSONbuffer;            //Creates JSON message
  JsonObject& JSONencoder = JSONbuffer.createObject();
  JSONencoder["id"] = clientName;
  JSONencoder["startuptime"] = connect_time;
  char JSONmessageBuffer[100];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  client.publish(topic_pub, JSONmessageBuffer, false);
  
}


/***************Effect Control*****************************************************/
  void effect_control (){
   
    if (iscurrent==0 && iscurrentstatechng==1){
      LightOutMiddle (strip.Color(0, 255, 0));
      iscurrentstatechng = 0;
    }

    if (iscurrent==1 && iscurrentstatechng==1){
      clear_strip();
      iscurrentstatechng = 0;
    }

    if (remainingUnix<=300 && remainingUnix > 240){
      if (effect == 1){
        meeting_ending(strip.Color(255, 0, 0), effect);
        effect = effect+1;
        
      }
    }


    if (remainingUnix<=240 && remainingUnix > 180){
      if (effect == 2){
        meeting_ending(strip.Color(255, 0, 0), effect);
        effect = effect+1;
        
      }
    }

    if (remainingUnix<=180 && remainingUnix > 120){
      if (effect == 3){
        meeting_ending(strip.Color(255, 0, 0), effect);
        effect = effect+1;
        
      }
    }

    if (remainingUnix<=120 && remainingUnix > 60){
      if (effect == 4){
        meeting_ending(strip.Color(255, 0, 0), effect);
        effect = effect+1;
        
      }
    }

    if (remainingUnix <= 60 && remainingUnix > 2){
      if (effect == 5){
        meeting_ending(strip.Color(255, 0, 0), effect);
        effect = effect+1;
        if (remaining == next){
          transition_effect = 1;   //meeting
        }
      }
    }

    if (remainingUnix <= 2){
      if (effect == 6){
        meeting_ending(strip.Color(255, 0, 0), effect);
        effect = effect+1;
        if(transition_effect == 1){
          colorWipe(strip.Color(0,0,0),50); 
        }
        else{
          LightOutMiddle (strip.Color(0, 255, 0));
        }
       }
    }
  }

/************************Time Functions***********************************/
void startUDP() {
  Serial.println("Starting UDP");
  UDP.begin(123);                          // Start listening for UDP messages on port 123
  Serial.print("Local port:\t");
  Serial.println(UDP.localPort());
  Serial.println();
}

uint32_t getTime() {
  if (UDP.parsePacket() == 0) { // If there's no response (yet)
    return 0;
  }
  UDP.read(NTPBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
  // Combine the 4 timestamp bytes into one 32-bit number
  uint32_t NTPTime = (NTPBuffer[40] << 24) | (NTPBuffer[41] << 16) | (NTPBuffer[42] << 8) | NTPBuffer[43];
  // Convert NTP time to a UNIX timestamp:
  // Unix time starts on Jan 1 1970. That's 2208988800 seconds in NTP time:
  const uint32_t seventyYears = 2208988800UL;
  // subtract seventy years:
  uint32_t UNIXTime = NTPTime - seventyYears;
  return UNIXTime;
}

void sendNTPpacket(IPAddress& address) {
  memset(NTPBuffer, 0, NTP_PACKET_SIZE);  // set all bytes in the buffer to 0
  // Initialize values needed to form NTP request
  NTPBuffer[0] = 0b11100011;   // LI, Version, Mode
  // send a packet requesting a timestamp:
  UDP.beginPacket(address, 123); // NTP requests are to port 123
  UDP.write(NTPBuffer, NTP_PACKET_SIZE);
  UDP.endPacket();
}

/************************LIGHT EFFECTS***********************************/

// start in the middle and fill outwards
void LightOutMiddle(uint32_t c) {
  int LED = strip.numPixels()/2;
  int LED2 = LED-1;
  if(LED%2==0){                       //checks if LEDS are even or odd
    strip.setPixelColor(LED, c);
    strip.setPixelColor(LED2, c);
    strip.show();
    stall(4);
  }

  else {
    strip.setPixelColor(LED, c);
    strip.show();
    stall(4);
  }
  for(uint16_t i=0; i<(strip.numPixels()/2)+1; i++) {
    strip.setPixelColor(LED+i,c);
    if(LED%2==0){
      strip.setPixelColor(LED2-i,c);
    }
    else{
      strip.setPixelColor(LED-i,c);
    }
   
    strip.show();
    stall(4);
    
    }

}

//meeting Ending
void meeting_ending(uint32_t c, uint8_t seg) {
  for(uint8_t i=0; i<5; i++) {
    for(uint8_t j=0; j<seg;j++){
      strip.setPixelColor(j,c);
      strip.setPixelColor(strip.numPixels()-1-j,c);
    }
    strip.show();
    stall(4);
    if(i<4){
     clear_strip();
     stall(4); 
    }

  }
 
}



// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
    }
  
//   stall(40); 
//   clear_strip();
}

void colorWipeReverse (uint32_t c, uint8_t wait) {
  for(uint16_t i= 0 ; i<LED_COUNT+1; i++) {
    strip.setPixelColor(strip.numPixels()-i,c);
    strip.show();
    delay(wait);
    }
   stall(40); 
   clear_strip();
}

void stall(uint16_t s){
  for (uint16_t i = 0; i<s; i++){
  client.loop();
  delay(50);
  }
}


void clear_strip(){
  for (uint16_t i=0; i<strip.numPixels(); i++){
    strip.setPixelColor(i,strip.Color(0,0,0));
    strip.show();
  }
}






/************************SETUP***********************************/
void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqttServer,mqttPort);
  ConnectBroker(client, clientName);    //connect to MQTT borker
  client.setCallback(callback);
  client.subscribe(topic_sub_roomupdate); 
  client.subscribe(topic_sub_firmware);
  setup_lights();  
  startUDP();
  
  if(!WiFi.hostByName(NTPServerName, timeServerIP)) { // Get the IP address of the NTP server
    Serial.println("DNS lookup failed. Rebooting.");
    Serial.flush();
    ESP.reset();
  }
  Serial.print("Time server IP:\t");
  Serial.println(timeServerIP);
  
  Serial.println("\r\nSending NTP request ...");
  sendNTPpacket(timeServerIP);  

  

  
  sendStartupMessage();
  

}

/************************LOOP***********************************/
void loop() {
//now = millis();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();

  if (currentMillis - prevNTP > intervalNTP) { // If a minute has passed since last NTP request
    prevNTP = currentMillis;
    Serial.println("\r\nSending NTP request ...");
    sendNTPpacket(timeServerIP);               // Send an NTP request
  }

  uint32_t time = getTime();                   // Check if an NTP response has arrived and get the (UNIX) time
  if (time) {                                  // If a new timestamp has been received
    timeUNIX = time;
    Serial.print("NTP response:\t");
    Serial.println(timeUNIX);
    lastNTPResponse = currentMillis;
  } else if ((currentMillis - lastNTPResponse) > 3600000) {
    Serial.println("More than 1 hour since last NTP response. Rebooting.");
    Serial.flush();
    ESP.reset();
  }

  actualTime = timeUNIX + (currentMillis - lastNTPResponse)/1000;
  if (actualTime != prevActualTime && timeUNIX != 0) { // If a second has passed since last print
    prevActualTime = actualTime;
    remainingUnix = starttime+(duration*60)-actualTime;   //remaining time in meeting
      if(remainingUnix <=0){
        remainingUnix =0;           
      }
      if( remainingUnix >300){
      effect = 1;
    }
    effect_control();
  }
}
