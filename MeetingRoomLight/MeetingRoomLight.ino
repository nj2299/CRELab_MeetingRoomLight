/*Top Comment Lines
 * 
 * 
 * Meeting Room Indicator Light - changes colour based on status of the room recieved from Fusion
 * refactored to use start and end time only
 * 
 * Published sept 2018
 * Last update: Oct 15, 2018
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
//#include <Adafruit_NeoPixel.h>  //Neopixel light control
#include <NeoPixelBus.h>    //Neopixel library that uses DMA (Direct Memory Access)
#include <WiFiConnect.h>
#include <Esp.h>
/*************************Constants***************************/
#define PixelCount 36
//#define LED_PIN D2    //control pin from ESP
#define TIMER_MS 5000
#define MQTT_KEEPALIVE 120
#define MQTT_MAX_PACKET_SIZE 512

/*************************global Variables***************************/
const char* mqttServer = "192.168.1.13";
const int mqttPort = 1883;
const char* clientName = "";  //these three variables used for setting the client name to the Macaddress
String topicString;
char topicChar[18];
int segmultiplier = PixelCount/12;

const char* topic_sub_roomupdate = "MRL/roomupdate";  //listen to this topic
const char* topic_pub = "MRL/status";
const char* topic_sub_firmware = "MRL/commands/firmware";  //listen for firmware update
time_t currenttime = 0;    //used for getting linux time
bool iscurrent = 0;           //is there a current meeting
bool iscurrentprev = 0;     //previous status current state
bool iscurrentstatechng = 1;

unsigned long connect_time;
//int Red = 0;
//int Green = 0;
//int Blue = 0;
int transition_effect=0;   
int effect=0;
bool firmware = 0;
int lightStatus = 0;


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
long remainingUnix=0;
uint32_t nextmeeting;
int remaining=0;
uint32_t starttime = 1549073939;                //start time of meeting - linux time
int duration = 0;                 //duration of meeting(minutes)
int elapsed = 0;                  //time elapsed (minutes)
int next = 0;                     //minutes to next meeting

unsigned long prevActualTime = 0;


/************************setup light strip*****************************************/
// For Esp8266, the Pin is omitted and it uses GPIO3 due to DMA hardware use.  
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount);
#define colorSaturation 255

RgbColor red(colorSaturation, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor blue(0, 0, colorSaturation);
RgbColor white(colorSaturation);
RgbColor black(0);

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
  

  //t_httpUpdate_return ret = ESPhttpUpdate.update("http://99.231.14.167/UpdateMRL");

  t_httpUpdate_return ret = ESPhttpUpdate.update("http://nj2299.duckdns.org/UpdateMRL");

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
  unsigned long currentMillis=0;
  unsigned long startTimer = millis(); 
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
      currentMillis = millis();
      if ((currentMillis - startTimer) > 30000) {                                     //frozen for 5 minutes, restart
      Serial.println("More than 5 minutes since last NTP response. Rebooting.");
      Serial.flush();
      ESP.restart();        //restart chip if MQTT connection is lost for more than 5 minutes - should reset WiFi etc
        }
    }
  }
}

/************************setup lights***********************************/
void setup_lights(){
  strip.Begin();
  strip.Show(); // Initialize all pixels to 'off'
}

/************************send status***********************************/

void send_status(){

  
    StaticJsonBuffer<300> JSONbuffer;
    JsonObject& JSONencoder = JSONbuffer.createObject();
    JSONencoder["ID"] = clientName;
//    JSONencoder["Red"] = Red;
//    JSONencoder["Green"] = Green;
//    JSONencoder["Blue"] = Blue;
    char JSONmessageBuffer[300];
    JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
    client.publish(topic_pub, JSONmessageBuffer);
    Serial.println(JSONmessageBuffer);
    Serial.flush();
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

    iscurrent = root["isCurrent"];    //0 no meeting, 1 meeting occuring
    starttime = root["start"];    //start time in unix time
    duration = root["duration"];    //duration in minutes
    elapsed = root["elapsed"];    //minutes into meeting
    next = root["next"];    //minutes to next meeting
    firmware = root["firmware"];  //check for firmware update
    remaining = duration - elapsed;   //in minutes
    nextmeeting = actualTime+(next*60);  //used for transitioning the light at the end of the meeting
    //    Serial.println(iscurrent);
    //    Serial.println(starttime);
    //    Serial.println(duration);
    //    Serial.println(elapsed);
    //    Serial.println(next);
    //    Serial.println(actualTime);
    //    Serial.println(nextmeeting);
    //    Serial.println(remaining);
    //    Serial.println(remainingUnix);
    //    Serial.flush();

/*    
    if (iscurrent != iscurrentprev){        //used to track when there is a change of meeting room state
      iscurrentstatechng = 1;
      iscurrentprev = iscurrent;
    }
*/

    if (firmware == 1){
      firmware = 0;
      updateFirmware();
      
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


/***************Effect Control*****************************************************
 * 
 * 
 * sets the timing for the various lighting effects
 * the effect variable is used to ensure the function runs once, and is reset after full animation has been run.
 */

  void effect_control (){
   
 //Room is available
    if (iscurrent==0 && lightStatus != 1){       //turn on light -> no meeting at the moment
      LightOutMiddle (green);                   //lightStatus used to control if effect has already run
      lightStatus = 1;
      Serial.println("No Meeting");
    Serial.println (iscurrent);
    Serial.println(remainingUnix);
    
    //    Serial.println(iscurrent);
    //    Serial.println(starttime);
    //    Serial.println(duration);
    //    Serial.println(elapsed);
    //    Serial.println(next);
    //    Serial.println(actualTime);
    //    Serial.println(nextmeeting);
    //    Serial.println(remaining);
    //    Serial.println(remainingUnix);
    }
    
//Meeting effect control
    if (iscurrent==1 || (actualTime > nextmeeting)){    //second part of || update from fusion every 3 minutes -> leads to missed transitions.  

      if(remainingUnix > 300 && lightStatus != 2){
        LightOutMiddle (black);
        lightStatus = 2;
             Serial.println("Meeting");
    Serial.println (iscurrent);
    Serial.println(remainingUnix);
    Serial.println(transition_effect);
       }

      if (remainingUnix <=300 && remainingUnix >240 && lightStatus !=3){
        effect = 1;                                   //effect controls the segments that light up on the LED strip
        meeting_ending(red, effect*segmultiplier);
        lightStatus = 3;
      }

      if (remainingUnix <=240 && remainingUnix > 180 && lightStatus !=4){
        effect = 2;
        meeting_ending(red, effect*segmultiplier);
        lightStatus = 4;
      }

      if (remainingUnix <= 180 && remainingUnix > 120 && lightStatus !=5){
        effect = 3;
        meeting_ending(red, effect*segmultiplier);
        lightStatus = 5;
      }

      if (remainingUnix<=120 && remainingUnix > 60 && lightStatus !=6){
        effect = 4;
        meeting_ending(red, effect*segmultiplier);
        lightStatus = 6;     
      }


      if (remainingUnix<=60 && remainingUnix > 30 && lightStatus !=7){
        effect = 5;
        meeting_ending(red, effect*segmultiplier);
        lightStatus = 7;
        transition_effect = 0;       //reset transition effect
        if (remaining == next){     //if remaining minutes == minutes to next meeting
          transition_effect = 1;   //meeting occurs directly after     
        }
      }

      if (remainingUnix <=30 && remainingUnix !=0 && lightStatus != 8){
        effect = 6;
        meeting_ending(red, effect*segmultiplier);
        lightStatus = 8;
      }

      if(remainingUnix <=0 && lightStatus != 9){
        if(transition_effect ==1){    //meeting occurs direcly after
          LightOutMiddle(black);
           Serial.println ("Transision 1");
        }
        else{
          LightOutMiddle (green);    //room is available
          Serial.println ("Transision 2");
          }
        lightStatus = 9;
      }
      
    }
  }

    
    
/***********************Time Functions***********************************/
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
void LightOutMiddle(RgbColor c) {
  int LED = PixelCount/2;
  int LED2 = LED-1;
  if(LED%2==0){                       //checks if LEDS are even or odd
    strip.SetPixelColor(LED, c);
    strip.SetPixelColor(LED2, c);
    strip.Show();
    stall(4);
    
  }

  else {
    strip.SetPixelColor(LED, c);
    delay(1);
    strip.Show();
    stall(4);
  }
  for(uint16_t i=0; i<(PixelCount/2)+1; i++) {
    strip.SetPixelColor(LED+i,c);
    if(LED%2==0){
      strip.SetPixelColor(LED2-i,c);
    }
    else{
      strip.SetPixelColor(LED-i,c);
    }
    strip.Show();
    stall(4);
    
    }

}

//meeting Ending
void meeting_ending(RgbColor c, uint8_t seg) {
  for(uint8_t i=0; i<5; i++) {
    for(uint8_t j=0; j<seg;j++){
      strip.SetPixelColor(j,c);
      strip.SetPixelColor(PixelCount-1-j,c);
    }
    strip.Show();
    stall(4);
    if(i<4){
     clear_strip();
     stall(4); 
    }

  }
 
}



// Fill the dots one after the other with a color
void colorWipe(RgbColor c, uint8_t wait) {
  for(uint16_t i=0; i<PixelCount; i++) {
    strip.SetPixelColor(i, c);
    strip.Show();
    delay(wait);
    }
  
//   stall(40); 
//   clear_strip();
}

void colorWipeReverse (RgbColor c, uint8_t wait) {
  for(uint16_t i= 0 ; i<PixelCount+1; i++) {
    strip.SetPixelColor(PixelCount-i,c);
    strip.Show();
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
  for (uint16_t i=0; i<PixelCount; i++){
    strip.SetPixelColor(i,black);
    
  }
  strip.Show();
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
      //if( remainingUnix >300){
      //effect = 1;
      //}
    effect_control();
  }
}
