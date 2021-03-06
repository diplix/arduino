/* 
 Andrew's WIP sketch for capturing data from Ambient F007th Thermo-Hygrometer
 
 Inspired by the many weather station hackers who have gone before,
 Only possible thanks to the invaluable help and support on the Arduino forums.

 With particular thanks to
   
 Rob Ward (whose Manchester Encoding reading by delay rather than interrupt
 is the basis of this code)
 https://github.com/robwlakes/ArduinoWeatherOS
 
 The work of 3zero8 capturing and analysing the data
 http://forum.arduino.cc/index.php?topic=214436.0
 
 The work of Volgy capturing and analysing the data
 https://github.com/volgy/gr-ambient
 
 The forum contributions of;
   dc42: showing how to create 5 minute samples.
   jremington: suggesting how to construct error checking using averages (although not used this is where the idea of using newtemp and newhum for error checking originated)
   Krodal: for his 6 lines of code in the forum thread "Compare two sensor values"
   
 This example code is in the public domain.
 
 What this code does:
   Captures Ambient F007th Thermo-Hygrometer data packets by
     Identifying a header of at least 10 rising edges (manchester encoding binary 1s)
     Synchs the data correctly within byte boundaries
     Distinguishes between F007th data packets and other 434Mhz signals with equivalent header by checking value of sensor ID byte 
   Correctly identifies positive and negative temperature values for up to 8 channels
   Correctly identifies humidity values for up to 8 channels 8 channels
   Prints data for all channels to the serial port every 5 minutes
   Error checks data by rejecting
     humidity value outside the range 1 to 100%
     temperature changes of 10C per minute or greater
 What it does not yet do (and needs to)
   Send the data by wifi to a website
   
Hardware to use with this code
  Up to 8 F007th Thermo-Hygrometer set to different channels
  A 434Mhz receiver
  17cm strand of CAT-5 cable as an antenna.
  
  
F007th Ambient Thermo-Hygrometer
Sample Data:
0        1        2        3        4        5        6        7
FD       45       4F       04       4B       0B       52       0
0   1    2   3    4   5    6   7    8   9    A   B    C   D    E
11111101 01000101 01001111 00000100 01001011 00001011 01010010 0000
hhhhhhhh SSSSSSSS NRRRRRRR bCCCTTTT TTTTTTTT HHHHHHHH CCCCCCCC ????
 
Channel 1 F007th sensor displaying 21.1 Centigrade and 11% RH

hhhhhhhh = header with final 01 before data packet starts (note using this sketch the header 01 is omitted when the binary is displayed)
SSSSSSSS = sensor ID, F007th = Ox45
NRRRRRRR = Rolling Code Byte? Resets each time the battery is changed
b = battery indicator?
CCC = Channel identifier, channels 1 to 8 can be selected on the F007th unit using dipswitches. Channel 1 => 000, Channel 2 => 001, Channel 3 => 010 etc.
TTTT TTTTTTTT = 12 bit temperature data.
      To obtain F: convert binary to decimal, take away 400 and divide by 10 e.g. (using example above) 010001001011 => 1099
      (1099-400)/10= 69.9F
      To obtain C: convert binary to decimal, take away 720 and multiply by 0.0556 e.g.
      0.0556*(1099-720)= 21.1C
HHHHHHHH = 8 bit humidity in binary. e.g. (using example above) 00001011 => 11
CCCCCCCC = checksum? Note that this sketch only looks at the first 6 bytes and ignores the checksum
   
*/

#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// MQTT
#define AIO_SERVER      "192.168.1.xx"
#define AIO_SERVERPORT  00000
#define AIO_USERNAME    "xxx"
#define AIO_KEY         "xxx"

#define WLAN_SSID       "xxx"
#define WLAN_PASS       "xxx"

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Setup a feeds
Adafruit_MQTT_Publish t1  = Adafruit_MQTT_Publish(&mqtt, "sensor/temperature/1");
Adafruit_MQTT_Publish t2  = Adafruit_MQTT_Publish(&mqtt, "sensor/temperature/2");
Adafruit_MQTT_Publish t3  = Adafruit_MQTT_Publish(&mqtt, "sensor/temperature/3");
Adafruit_MQTT_Publish t4  = Adafruit_MQTT_Publish(&mqtt, "sensor/temperature/4");
Adafruit_MQTT_Publish t5  = Adafruit_MQTT_Publish(&mqtt, "sensor/temperature/5");
Adafruit_MQTT_Publish t6  = Adafruit_MQTT_Publish(&mqtt, "sensor/temperature/6");
Adafruit_MQTT_Publish t7  = Adafruit_MQTT_Publish(&mqtt, "sensor/temperature/7");
Adafruit_MQTT_Publish t8  = Adafruit_MQTT_Publish(&mqtt, "sensor/temperature/8");

Adafruit_MQTT_Publish h1  = Adafruit_MQTT_Publish(&mqtt, "sensor/humidity/1");
Adafruit_MQTT_Publish h2  = Adafruit_MQTT_Publish(&mqtt, "sensor/humidity/2");
Adafruit_MQTT_Publish h3  = Adafruit_MQTT_Publish(&mqtt, "sensor/humidity/3");
Adafruit_MQTT_Publish h4  = Adafruit_MQTT_Publish(&mqtt, "sensor/humidity/4");
Adafruit_MQTT_Publish h5  = Adafruit_MQTT_Publish(&mqtt, "sensor/humidity/5");
Adafruit_MQTT_Publish h6  = Adafruit_MQTT_Publish(&mqtt, "sensor/humidity/6");
Adafruit_MQTT_Publish h7  = Adafruit_MQTT_Publish(&mqtt, "sensor/humidity/7");
Adafruit_MQTT_Publish h8  = Adafruit_MQTT_Publish(&mqtt, "sensor/humidity/8");

Adafruit_MQTT_Publish status  = Adafruit_MQTT_Publish(&mqtt, "rfth/status");

int debug = 1; // Turn debugging on (1) or off (0)

void MQTT_connect();

/**/

//Interface Definitions
int RxPin           = 2;   //The number of signal from the Rx
int ledPin          = 4;  //The number of the onboard LED pin

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
//float  Ch7temp = 0; // Stored value for channel 7 temperature
//float  Ch7hum = 0; // Stored value for channel 7 humidity
//float  Ch8temp = 0; // Stored value for channel 8 temperature
//float  Ch8hum = 0; // Stored value for channel 8 humidity

void setup() {
  Serial.begin(115200);//make it fast so it dumps quick!
  Serial.println("setup completed!");
  
  pinMode(RxPin, INPUT);
  pinMode(ledPin, OUTPUT);

  if(debug) { 
    Serial.println(F("Adafruit MQTT demo"));

    // Connect to WiFi access point.
    Serial.println(); Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WLAN_SSID);
  }

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if(debug) { 
      Serial.print(".");
    }
  }
  if(debug) { 
    Serial.println();

    Serial.println("WiFi connected");
    Serial.println("IP address: "); Serial.println(WiFi.localIP());
  }

  status.publish("online");


  // uncomment to display workings
/*  
  Serial.println();
  lDelay=2*sDelay;//just to make sure the 1:2 ratio is established. They can have some other ratio if required
  Serial.print("Using a delay of 1/4 bitWaveform ");// +-15% and they still seem to work ok, pretty tolerant!
  Serial.print(sDelay,DEC);
  Serial.print(" uSecs 1/2 bitWaveform ");//these may not be exactly 1:2 ratio as processing also contributes to these delays.
  Serial.print(lDelay,DEC);
  Serial.println(" uSecs ");
  if (polarity){
    Serial.println("Negative Polarity hi->lo=1"); 
  }
  else{
    Serial.println("Positive Polarity lo->hi=1"); 
  }
  Serial.print(headerBits,DEC); 
  Serial.println(" bits expected for a valid header"); 
  if (sync0In){
    Serial.println("Sync Zero inside Packet"); 
  }
  else{
    Serial.println("Sync Zero outside Packet"); 
  }
  Serial.println("D 00 00001111 01 22223333 02 44445555 03 66667777 04 88889999 05 AAAABBBB"); 
  //if packet is repeated then best to have non matching numbers in the array slots to begin with
  //clear the array to different nos cause if all zeroes it might think that is a valid 3 packets ie all equal
*/
  Serial.println("Ready!"); 

  eraseManchester();  //clear the array to different nos cause if all zeroes it might think that is a valid 3 packets ie all equal
} //end of setup


// Main routines, find header, then sync in with it, get a packet, and decode data in it, plus report any errors.
void loop(){
  MQTT_connect();

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
  digitalWrite(ledPin,0); //data processing exited, look for another header
}//end of mainloop

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
    /*
    zwei.publish("ON");
    */
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
    t1.publish(Ch1temp);
    h1.publish(Ch1hum);
    t2.publish(Ch2temp);
    h2.publish(Ch2hum);
    t3.publish(Ch3temp);
    h3.publish(Ch3hum);
    t4.publish(Ch4temp);
    h4.publish(Ch4hum);
    t5.publish(Ch5temp);
    h5.publish(Ch5hum);
    t6.publish(Ch6temp);
    h6.publish(Ch6hum);
  }
}


//decomment to print out recieved binary

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

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  if(debug) { 
    Serial.print("Connecting to MQTT... ");
  }

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
     if(debug) { 
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
     }
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  if(debug) { Serial.println("MQTT Connected!"); }
  status.publish("online");
}
