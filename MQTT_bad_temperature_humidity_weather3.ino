// Get ESP8266 going with Arduino IDE
// - https://github.com/esp8266/Arduino#installing-with-boards-manager
// Required libraries (sketch -> include library -> manage libraries)
// - PubSubClient by Nick ‘O Leary
// - DHT sensor library by Adafruit

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

#define wifi_ssid "xxx"
#define wifi_password "xxx"

#define mqtt_server "192.168.1.xx"
#define mqtt_port 00000
#define mqtt_user "xxx"
#define mqtt_password "xxx"

#define humidity_topic "sensor/humidity/bad"
#define humidity_x2_topic "sensor/humidity/badx2"
#define temperature_topic "sensor/temperature/bad"
#define light_topic "sensor/light/bad"

#define t1 "sensor/temperature/1"
#define t2 "sensor/temperature/2"
#define t3 "sensor/temperature/3"
#define t4 "sensor/temperature/4"
#define t5 "sensor/temperature/5"
#define t6 "sensor/temperature/6"
#define t7 "sensor/temperature/7"
#define t8 "sensor/temperature/8"

#define h1 "sensor/humidity/1"
#define h2 "sensor/humidity/2"
#define h3 "sensor/humidity/3"
#define h4 "sensor/humidity/4"
#define h5 "sensor/humidity/5"
#define h6 "sensor/humidity/6"
#define h7 "sensor/humidity/7"
#define h8 "sensor/humidity/8"

#define motion_topic "sensor/motion/bad"

//#define rfstatus "rfth/status"

// Variables for Manchester Receiver Logic:
word    sDelay     = 242;  //Small Delay about 1/4 of bit duration
word    lDelay     = 484;  //Long Delay about 1/2 of bit duration, 1/4 + 1/2 = 3/4
byte    polarity   = 1;    //0 for lo->hi==1 or 1 for hi->lo==1 for Polarity, sets tempBit at start
byte    tempBit    = 1;    //Reflects the required transition polarity
boolean firstZero  = false;//flags when the first '0' is found.
boolean noErrors   = true; //flags if signal does not follow Manchester conventions
//variables for Header detection
byte    headerBits = 10;   //The number of ones expected to make a valid header
byte    headerHits = 0;    //Counts the number of "1"s to determine a header
//Variables for Byte storage
boolean sync0In=true;      //Expecting sync0 to be inside byte boundaries, set to false for sync0 outside bytes
byte    dataByte   = 0;    //Accumulates the bit information
byte    nosBits    = 6;    //Counts to 8 bits within a dataByte
byte    maxBytes   = 6;    //Set the bytes collected after each header. NB if set too high, any end noise will cause an error
byte    nosBytes   = 0;    //Counter stays within 0 -> maxBytes
//Variables for multiple packets
byte    bank       = 0;    //Points to the array of 0 to 3 banks of results from up to 4 last data downloads 
byte    nosRepeats = 3;    //Number of times the header/data is fetched at least once or up to 4 times
//Banks for multiple packets if required (at least one will be needed)
byte  manchester[4][20];   //Stores 4 banks of manchester pattern decoded on the fly

// Variables to prepare recorded values (used to create CSV output) for Ambient

byte   stnId = 0; //Identifies the channel number
int    dataType = 0;  //Identifies the Ambient Thermo-Hygrometer code
int    differencetemp = 0;
float  Newtemp = 0;
float  Newhum = 0;
float  Ch1temp = 100; // Stored value for channel 1 temperature
float  Ch1hum = 0; // Stored value for channel 1 humidity
float  Ch2temp = 100; // Stored value for channel 2 temperature
float  Ch2hum = 0; // Stored value for channel 2 humidity
float  Ch3temp = 100; // Stored value for channel 3 temperature
float  Ch3hum = 0; // Stored value for channel 3 humidity
float  Ch4temp = 100; // Stored value for channel 4 temperature
float  Ch4hum = 0; // Stored value for channel 4 humidity
float  Ch5temp = 100; // Stored value for channel 5 temperature
float  Ch5hum = 0; // Stored value for channel 5 humidity
float  Ch6temp = 100; // Stored value for channel 6 temperature
float  Ch6hum = 0; // Stored value for channel 6 humidity
float  Ch7temp = 100; // Stored value for channel 7 temperature
float  Ch7hum = 0; // Stored value for channel 7 humidity
float  Ch8temp = 100; // Stored value for channel 8 temperature
float  Ch8hum = 0; // Stored value for channel 8 humidity


long lastMsg = 0;
float temp = 0.0;
float hum = 0.0;
float hum2 = 0.0;
float diff = 0.5;
float diff_t = 0.5;
float diff_h = 1.0;
bool previous = 0; //0 = OFF = HIGH, 1= ON = LOW
int  light_state;  /* Holds the last digital value */

int RxPin           = 2;   //The number of signal from the Rx
int ledPin          = 4;  //The number of the onboard LED pin
int PirPin          = 12;   //The number of signal from the Rx

#define DHTTYPE DHT22
#define DHTPIN  14
#define DIGITAL_LIGHT_SENSOR_PIN  13
#define RF_RECEIVER_PIN  2

int counter = 0;
int previousReading = LOW;

WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE, 11); // 11 works fine for ESP8266

void setup() {
  Serial.begin(115200);
  pinMode(RxPin, INPUT);
  pinMode(ledPin, OUTPUT);

  dht.begin();
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);

  Serial.println("Ready!"); 

  eraseManchester();  //clear the array to different nos cause if all zeroes it might think that is a valid 3 packets ie all equal
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
    // if (client.connect("ESP8266Client")) {
    if (client.connect("ESP8266Client", mqtt_user, mqtt_password)) {
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

bool checkBound(float newValue, float prevValue, float maxDiff) {
  return !isnan(newValue) &&
         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
// temp and humidity
//
  long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;

    float newTemp = dht.readTemperature();
    float newHum = dht.readHumidity();

    if (checkBound(newTemp, temp, diff_t)) {
      temp = newTemp;
      Serial.print("New temperature:");
      Serial.println(String(temp).c_str());
      client.publish(temperature_topic, String(temp).c_str(), true);
    }

    if (checkBound(newHum, hum, diff_h)) {
      hum = newHum;
      hum2 = newHum * 2.0;
      Serial.print("New humidity:");
      Serial.println(String(hum).c_str());
      Serial.println(String(hum2).c_str());
      client.publish(humidity_topic, String(hum).c_str(), true);
      client.publish(humidity_x2_topic, String(hum2).c_str(), true);
    }
  }
// light sensor
//  
  light_state = digitalRead(DIGITAL_LIGHT_SENSOR_PIN);  
  if (light_state == LOW && previous == 0)
  {
    Serial.println("Light ON ");
    client.publish(light_topic, "ON", true);
    previous = 1;
  }  else if (light_state == HIGH && previous == 1)
  {
    Serial.println("Light OFF ");
    client.publish(light_topic, "OFF", true);
    previous = 0;
  }

// get motion
//
  int reading = digitalRead(PirPin);
  //Serial.print(reading);
  if (previousReading == LOW && reading == HIGH) {
    counter++;
    client.publish(motion_topic, "ON");
//    motion_topic.publish("ON");  
    Serial.print("ON");
    //Serial.print("Triggered ");
    //Serial.print(counter);
    //Serial.print("x Times! ");
    Serial.println();
    delay(1000);
  }
  if (previousReading == HIGH && reading == LOW) {
    counter++;
    client.publish(motion_topic, "OFF");
//    motion_topic.publish("OFF");  
    Serial.print("OFF ");
    Serial.println();
    delay(1000);
  }
  previousReading = reading;

// weatherstation
//
  tempBit=polarity; //these begin the same for a packet
  noErrors=true;
  firstZero=false;
  headerHits=0;
  nosBits=0;
  nosBytes=0;
  while (noErrors && (nosBytes<maxBytes)){
    while(digitalRead(RxPin)!=tempBit){
      //pause here until a transition is found
    }//at Data transition, half way through bit pattern, this should be where RxPin==tempBit
    delayMicroseconds(sDelay);//skip ahead to 3/4 of the bit pattern
    // 3/4 the way through, if RxPin has changed it is definitely an error
    digitalWrite(ledPin,0); //Flag LED off!
    if (digitalRead(RxPin)!=tempBit){
      noErrors=false;//something has gone wrong, polarity has changed too early, ie always an error
    }//exit and retry
    else{
      delayMicroseconds(lDelay);
      //now 1 quarter into the next bit pattern,
      if(digitalRead(RxPin)==tempBit){ //if RxPin has not swapped, then bitWaveform is swapping
        //If the header is done, then it means data change is occuring ie 1->0, or 0->1
        //data transition detection must swap, so it loops for the opposite transitions
        tempBit = tempBit^1;
      }//end of detecting no transition at end of bit waveform, ie end of previous bit waveform same as start of next bitwaveform

      //Now process the tempBit state and make data definite 0 or 1's, allow possibility of Pos or Neg Polarity 
      byte bitState = tempBit ^ polarity;//if polarity=1, invert the tempBit or if polarity=0, leave it alone.
      if(bitState==1){ //1 data could be header or packet
        if(!firstZero){
          headerHits++;
          if (headerHits==headerBits){
            digitalWrite(ledPin,1); //valid header accepted, minimum required found
            Serial.print("H");
          }
        }
        else{
          add(bitState);//already seen first zero so add bit in
        }
      }//end of dealing with ones
      else{  //bitState==0 could first error, first zero or packet
        // if it is header there must be no "zeroes" or errors
        if(headerHits<headerBits){
          //Still in header checking phase, more header hits required
          noErrors=false;//landing here means header is corrupted, so it is probably an error
        }//end of detecting a "zero" inside a header
        else{
          //we have our header, chewed up any excess and here is a zero
          if (!firstZero){ //if first zero, it has not been found previously
            firstZero=true;
            if(sync0In){
              add(bitState);//Add zero to bytes
              dataByte = B11111111;
              nosBits = 7;
            }
            Serial.print("!");
          }//end of finding first zero
          else{
            add(bitState);
          }//end of adding a zero bit
        }//end of dealing with a first zero
      }//end of dealing with zero's (in header, first or later zeroes)
    }//end of first error check
  }//end of while noErrors=true and getting packet of bytes
  //digitalWrite(ledPin,0); //data processing exited, look for another header
  
}

//Read the binary data from the bank and apply conversions where necessary to scale and format data
void analyseData(){ 
}

void add(byte bitData){
  dataByte=(dataByte<<1)|bitData;
  nosBits++;
  if (nosBits==8){
    nosBits=0;
    manchester[bank][nosBytes]=dataByte;
    nosBytes++;
    Serial.print("B");
  }
  if(nosBytes==maxBytes){
//    hexBinDump();//for debug purposes dump out in hex and bainary
    analyseData();//later on develop your own analysis routines

//Subroutines to check, analyse and format data for F007th
  
int stnId = ((manchester[0][3]&B01110000)/16)+1;  // looks at 3 bits in byte 3 used to identify channels 1 to 8
    dataType = manchester[0][1];  // looks in byte 1 for the F007th Ambient Thermo-Hygrometer code (0x45)
    Newtemp = (float((((manchester[0][3]&B00000111)*256)+ manchester[0][4])-720)*0.0556); // looks in bytes 3 and 4 for temperature and then converts to C
    Newhum =(manchester [0][5]); // looks in byte 5 for humidity data
   if ((dataType == 0x45) && (stnId == 1) && (Newhum <= 100)){ // if the packet is from a F007th sensor on channel 1 and humidity equal or less than 100
    if (Ch1temp == 100){ // if the channel 1 temperature is 100C (default when sketch started so first reading)
    Ch1temp = Newtemp; // take the new reading as the offical channel 1 temperature
    Ch1hum = Newhum; // take the new reading as the offical channel 1 humidity
    }
    if (Ch1temp != 100){ // if the channel 1 temperature is other than 100C(so a subsequent reading)
    differencetemp = Newtemp - Ch1temp; // subtract the previous reading from the new reading to find the difference
    if (differencetemp < 10 && differencetemp > -10){ // if the new reading is within 10 degrees of the old one
    Ch1temp = Newtemp; // take the new reading as the offical channel 1 temperature
    Ch1hum = Newhum;} // take the new reading as the offical channel 1 humidity
    }
   } 
   if ((dataType == 0x45) && (stnId == 2) && (Newhum <= 100)){
    if (Ch2temp == 100){ 
    Ch2temp = Newtemp;
    Ch2hum = Newhum;
    }
    if (Ch2temp != 100){ 
    differencetemp = Newtemp - Ch2temp;
    if (differencetemp < 10 && differencetemp > -10){
    Ch2temp = Newtemp;
    Ch2hum = Newhum;}
    }
   } 
   if ((dataType == 0x45) && (stnId == 3)&& (Newhum <= 100)){
    if (Ch3temp == 100){ 
    Ch3temp = Newtemp;
    Ch3hum = Newhum;
    }
    if (Ch3temp != 100){ 
    differencetemp = Newtemp - Ch3temp;
    if (differencetemp < 10 && differencetemp > -10){ 
    Ch3temp = Newtemp;
    Ch3hum = Newhum;}
    }
   } 
   if ((dataType == 0x45) && (stnId == 4) && (Newhum <= 100)){
    if (Ch4temp == 100){ 
    Ch4temp = Newtemp;
    Ch4hum = Newhum;
    }
    if (Ch4temp != 100){ 
    differencetemp = Newtemp - Ch4temp;
    if (differencetemp < 10 && differencetemp > -10){
    Ch4temp = Newtemp;
    Ch4hum = Newhum;}
    }
   } 
   if ((dataType == 0x45) && (stnId == 5) && (Newhum <= 100)){
    if (Ch5temp == 100){
    Ch5temp = Newtemp;
    Ch5hum = Newhum;
    }
    if (Ch5temp != 100){
    differencetemp = Newtemp - Ch5temp;
    if (differencetemp < 10 && differencetemp > -10){
    Ch5temp = Newtemp;
    Ch5hum = Newhum;}
    }
   } 
   if ((dataType == 0x45) && (stnId == 6) && (Newhum <= 100)){
    if (Ch6temp == 100){ 
    Ch6temp = Newtemp;
    Ch6hum = Newhum;
    }
    if (Ch6temp != 100){
    differencetemp = Newtemp - Ch6temp;
    if (differencetemp < 10 && differencetemp > -10){
    Ch6temp = Newtemp;
    Ch6hum = Newhum;}
    }
   } 
//add additional copies of above code here if using channels 7 and 8   
  }

 const unsigned long SampleTime = 1 * 60 * 1000UL;
 static unsigned long lastSampleTime = 0 - SampleTime;
 
 unsigned long now = millis();
 
 if (now - lastSampleTime >= SampleTime)
  {
    lastSampleTime += SampleTime;
    Serial.println("***");
    Serial.print("Channel 1 data "); //Indicates channel number/name
    Serial.print(Ch1temp,1);
    Serial.print("C ");
    Serial.print(Ch1hum,0);
    Serial.println("%");
    Serial.print("Channel 2 data "); 
    Serial.print(Ch2temp,1);
    Serial.print("C ");
    Serial.print(Ch2hum,0);
    Serial.println("%");
    Serial.print("Channel 3 data "); 
    Serial.print(Ch3temp,1);
    Serial.print("C ");
    Serial.print(Ch3hum,0);
    Serial.println("%");
    Serial.print("Channel 4 data "); 
    Serial.print(Ch4temp,1);
    Serial.print("C ");
    Serial.print(Ch4hum,0);
    Serial.println("%");
    Serial.print("Channel 5 data "); 
    Serial.print(Ch5temp,1);
    Serial.print("C ");
    Serial.print(Ch5hum,0);
    Serial.println("%");
    Serial.print("Channel 6 data "); 
    Serial.print(Ch6temp,1);
    Serial.print("C ");
    Serial.print(Ch6hum,0);
    Serial.println("%");
    Serial.print("Channel 7 data "); 
    Serial.print(Ch7temp,1);
    Serial.print("C ");
    Serial.print(Ch7hum,0);
    Serial.println("%");
    Serial.print("Channel 8 data "); 
    Serial.print(Ch8temp,1);
    Serial.print("C ");
    Serial.print(Ch8hum,0);
    Serial.println("%");
    if (Ch1temp != 100) {
    	client.publish(t1, String(Ch1temp).c_str(), true);
    	client.publish(h1, String(Ch1hum).c_str(), true);
	}
    if (Ch2temp != 100) {
	    client.publish(t2, String(Ch2temp).c_str(), true);
    	client.publish(h2, String(Ch2hum).c_str(), true);
	}
    if (Ch2temp != 100) {
	    client.publish(t3, String(Ch3temp).c_str(), true);
    	client.publish(h3, String(Ch3hum).c_str(), true);
	}
    if (Ch2temp != 100) {
	    client.publish(t4, String(Ch4temp).c_str(), true);
    	client.publish(h4, String(Ch4hum).c_str(), true);
	}
    if (Ch2temp != 100) {
	    client.publish(t5, String(Ch5temp).c_str(), true);
    	client.publish(h5, String(Ch5hum).c_str(), true);
	}
    if (Ch2temp != 100) {
	    client.publish(t6, String(Ch6temp).c_str(), true);
    	client.publish(h6, String(Ch6hum).c_str(), true);
	}
    if (Ch2temp != 100) {
	    client.publish(t7, String(Ch7temp).c_str(), true);
    	client.publish(h7, String(Ch7hum).c_str(), true);
	}
    if (Ch2temp != 100) {
	    client.publish(t8, String(Ch8temp).c_str(), true);
    	client.publish(h8, String(Ch8hum).c_str(), true);
	}
  }
}

void hexBinDump(){
  //Print the fully aligned binary data in manchester[bank] array
Serial.print("D ");
  for( int i=0; i < maxBytes; i++){ 
   byte mask = B10000000;
   if (manchester[bank][i]<16){
      Serial.print("0"); //Pad single digit hex
    }
    Serial.print(manchester[bank][i],HEX);
    Serial.print(" ");
    for (int k=0; k<8; k++){
      if (manchester[bank][i] & mask){
        Serial.print("1");
      }
      else{
        Serial.print("0");
      }
     mask = mask >> 1;
    }
    Serial.print(" ");
  }
  Serial.println();
}

void eraseManchester(){
  //Clear the memory to non matching numbers across the banks
  //If there is only one packet, with no repeats this is not necessary.
  for( int j=0; j < 4; j++){ 
    for( int i=0; i < 20; i++){ 
      manchester[j][i]=j+i;
    }
  }
}
