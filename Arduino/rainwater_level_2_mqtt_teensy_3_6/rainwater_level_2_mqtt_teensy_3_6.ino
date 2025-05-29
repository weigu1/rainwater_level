/*
 * rainwater_level_2_mqtt_teensy_3_6.ino
 * 
 * weigu.lu
 * 
 * Get rainwater level with ultrasonic sensor MB7060 XL-MaxSonar-WR 
 * and hydrostatic pressure sensor ALS-MPM-2F. 
 * Publish with MQTT over Ethernet (W5500 board) and save to SD card
 * teensy 3.6 with teensyduino 
 */

#pragma execution_character_set("utf-8") 

#include <Ethernet.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <SD.h>
#include <TimeLib.h>
#include <time.h>
#include <ArduinoJson.h>

/* we have code to process time sync messages from the serial port 
 * uncomment the following define if needed
 * sent time with linux command: 
 * date -d '+2 hour' +T%s > /dev/ttyACM0
 * (to do it manually add 
 * Teensy3Clock.set(1594747890); //https://www.epochconverter.com/ GMT!
 * to setup())
*/
//#define SET_TEENSY_TIME

const unsigned int MQTT_PACKET_SIZE_MAX = 1024; //override value in PubSubClient.h
byte mac[]    = { 0xDE, 0xED, 0xBC, 0xFA, 0xFE, 0xED };
IPAddress eth_ip     (192,168,178,12); //static IP
IPAddress dns_ip     (192,168,178,1);
IPAddress gateway_ip (192,168,178,1);
IPAddress subnet_mask(255,255,255,0);

const char *MQTT_SERVER_IP = "192.168.178.22";
const int  MQTT_PORT = 1883;
const char *MQTT_CLIENT_ID = "rainwater_level_1";
const String MQTT_TOPIC = "garden/rainwater";
const String MQTT_ST[] = {"/rainwater_level"};

EthernetClient eth_Client;
PubSubClient MQTT_Client(eth_Client);

const byte LED_TEENSY = 13; // PD6 Teensy LED
const byte LED_POWER = 35;
const byte LED_ACTIVITY = 36;
const byte PIN_REQ_ULTRASONIC_SENSOR = 37;
const byte PIN_PRESSURE_SENSOR = A22;

const byte CS_ETH_pin = 24; // MOSI0 11 MISO0 12
const byte SCK_ETH_pin = 14;
const byte CS_SD = BUILTIN_SDCARD;

char msg[MQTT_MAX_PACKET_SIZE];
char SDline[1200];

byte tsec, tmin, thour, tweekd, tday, tmon, tyear;
unsigned int capacity_u_l, capacity_p_l, distance_p_cm, distance_u_cm;  


/********** SETUP *************************************************************/

void setup() {  
  init_gpios();  
  delay(100);  
  SPI.setSCK(SCK_ETH_pin);
  init_serial();  
  Serial.print("Initializing SD card...");
  if (!SD.begin(CS_SD)) { // see if the card is present and can be initialized:
    Serial.println("Card failed, or not present");
    return; }
  Serial.println("card initialized.");
  setSyncProvider(getTeensy3Time); // set the Time lib for Teensy 3 RTC
  MQTT_Client.setBufferSize(MQTT_PACKET_SIZE_MAX);
  MQTT_Client.setServer(MQTT_SERVER_IP,MQTT_PORT);
  Ethernet.init(CS_ETH_pin); //needed to avoid conflict with SD card!!
  Ethernet.begin(mac, eth_ip);
  delay(1500);  // Allow the hardware to sort itself out  
  digitalWrite(LED_TEENSY,LOW);   // Off, less energy
}

/********** LOOP  **************************************************************/
  
void loop() {  
 #ifdef SET_TEENSY_TIME
    //used to set the time from PC    
    if (Serial.available()) {
      time_t t = processSyncMessage();
      if (t != 0) {
        Teensy3Clock.set(t); // set the RTC
        setTime(t);
      }
    }
    digitalClockDisplay();
    delay(1000);
  #endif   
  if (non_blocking_delay(1000)) { // every second 1000ms    
    distance_p_cm = get_pressure_sensor_cm(PIN_PRESSURE_SENSOR);  
  }  
  if (second()==0) {             // and publish only once a minute
    digitalClockDisplay();
    String dt = get_datetime();
    digitalWrite(LED_ACTIVITY,HIGH);           // LED On
    if (!MQTT_Client.connected()) {
      reconnect();
    }
    Serial.println("-----------------------------------");    
    get_rainwater_decrypt_and_publish(dt);// rainwater second
    log_to_file(dt);    
    delay(1500);                          // wait for second to end
    digitalWrite(LED_ACTIVITY,LOW);            // LED Off
    delay(1500);                 // wait for second to end    
  } 
  delay(10);
  MQTT_Client.loop();  
}

/********** INIT functions ****************************************************/

void init_gpios() {
  pinMode(LED_TEENSY, OUTPUT);                 // initialize LED outputs
  pinMode(LED_POWER, OUTPUT);
  pinMode(LED_ACTIVITY, OUTPUT);
  pinMode(PIN_REQ_ULTRASONIC_SENSOR, OUTPUT);  // init request_pin();
  digitalWrite(LED_POWER,HIGH);                // power LED on
  digitalWrite(LED_TEENSY,HIGH);               // teensy LED on  
} 
 
void init_serial() {
  Serial.begin(115200);  // for debugging
  delay(500);
  Serial.println("Serial is working!");
  Serial6.begin(9600); //rainwater sensor XL-MaxSonar MB7060
}

/********** GET functions *****************************************************/

String get_datetime() {
  snprintf (msg, 110, "%d-%02d-%02dT%02d:%02d:%02d",year(),month(),day(),hour(),minute(),second());
  return(msg);
}

unsigned int get_pressure_sensor_cm(byte analog_pin) {
  const byte alen = 40; // less than 255
  static unsigned int v_array[alen] = {0};
  unsigned long long p_sensor_sum = 0;
  unsigned int p_sensor, distance_p_cm;
  p_sensor = analogRead(PIN_PRESSURE_SENSOR); 
  if ((p_sensor > 299) && (p_sensor < 901)) {
    for (byte i=1; i<alen; i++) { // rotate array
      v_array[i-1]= v_array[i];
      p_sensor_sum += v_array[i];
    }
    v_array[alen-1] = p_sensor;
    p_sensor_sum += v_array[alen-1];  
    p_sensor = round(p_sensor_sum/alen);
    // 4-20mA, 220 Ohm, U*I = 880mV-4400mV (5m), 880mV-2640mV (3m), ADC 300-900
    distance_p_cm = map(p_sensor, 300, 900, 0, 300);
  }
  else {
    distance_p_cm = 0;
  }
  return (distance_p_cm);
}

unsigned int get_distance_mb7060_cm(byte data_req_pin) {
  String MB7060_data;
  char mb7060_data[6];
  unsigned int byte_counter = 0, distance_cm = 0;
  bool valid_data = true;  
  digitalWrite(data_req_pin,LOW);         // Request serial data Off
  delay(10);
  digitalWrite(data_req_pin,HIGH);        // Request serial data On
  while (Serial6.available() > 0) {        // Clear the buffer!
    Serial6.read();
  }   
  delay(200);                             // Wait for the message
  while ((Serial6.available()) && (byte_counter < 5)) {
    mb7060_data[byte_counter] = Serial6.read();
    if (mb7060_data[0] != 'R') {          // wrong data, clear buffer, get out
      distance_cm = 0;
      while (Serial6.available() > 0) { 
        Serial6.read();
      }
      return 0;
    }  
    byte_counter++;
  }
  for (byte i = 0; i<4; i++) {            // ignore carriage return
    valid_data = valid_data && isAlphaNumeric(mb7060_data[i]);
  }    
  if (valid_data) {        
    MB7060_data = String(mb7060_data);        
    MB7060_data = MB7060_data.substring(1,4);      
    distance_cm = MB7060_data.toInt();    
  }  
  digitalWrite(data_req_pin,LOW);         // Request serial data Off
  return (distance_cm);
}

void get_rainwater_decrypt_and_publish(String datetime) {  
  DynamicJsonDocument doc_out(1024);  
  String mqtt_msg;  
  distance_u_cm = get_distance_mb7060_cm(PIN_REQ_ULTRASONIC_SENSOR);  
  // inner diameter: 2,55m height about 2m, A = 5,107m2, 
  capacity_u_l = int(round(((201+3.5)-float(distance_u_cm))*51.07));        
  capacity_p_l = int((float(distance_p_cm))*51.07);
  doc_out["datetime"] = datetime;
  //doc_out["distance_u_cm"] = distance_u_cm;
  doc_out["capacity_u_l"] = capacity_u_l;    
  //doc_out["distance_p_cm"] = distance_p_cm;
  doc_out["capacity_p_l"] = capacity_p_l;
  serializeJson(doc_out, mqtt_msg);
  Serial.println(mqtt_msg);    
  MQTT_Client.publish((MQTT_TOPIC+MQTT_ST[0]).c_str(), mqtt_msg.c_str());
}

/********** TIME and helper functions ********************************************/

// from TimeTeensyTime example

time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

void digitalClockDisplay() { // digital clock display of the time
  byte temp;
  Serial.print(year());
  Serial.print('-');
  Serial.print(month());
  Serial.print('-');
  Serial.print(day());
  Serial.print(' ');
  Serial.print(hour());
  Serial.print(':');
  temp = minute();
  if(temp < 10) {
    Serial.print('0');
  }  
  Serial.print(temp);
  Serial.print(':');
  temp = second();
  if(temp < 10) {
    Serial.print('0');
  }  
  Serial.println(temp);
}

#ifdef SET_TEENSY_TIME
#define TIME_HEADER  "T"   // Header tag for serial time sync message
unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 
  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     return pctime;
     if( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
       pctime = 0L; // return 0 to indicate that the time is not valid
     }
  }
  return pctime;
}
#endif // SET_TEENSY_TIME

void reconnect() {
  while (!MQTT_Client.connected()) { // Loop until we're reconnected
    if (MQTT_Client.connect(MQTT_CLIENT_ID)) { // Attempt to connect publish announcement
      //MQTT_Client.publish(topic, "{\"smartmeters_P1\":\"connected\"}");
    }
    else {
      delay(5000); // Wait 5 seconds before retrying
    }
  }
}

// non blocking delay using millis(), returns true if time is up
bool non_blocking_delay(unsigned long milliseconds) {
  static unsigned long nb_delay_prev_time = 0;
  if(millis() >= nb_delay_prev_time + milliseconds) {
    nb_delay_prev_time += milliseconds;
    return true;
  }
  return false;
}

void log_to_file(String datetime) {
  char filename[13];
  String Data = datetime + ' ';
  snprintf (filename, 13, "%02d_%02d_%02d.txt",year()-2000, month(), day());  
  File dataFile = SD.open(filename, FILE_WRITE); // open file
  Data = Data + String(capacity_u_l) + ' ' + String(capacity_p_l) + '\n';  
  if (dataFile) {  // if the file is available, write to it:    
    dataFile.println(Data);
    dataFile.close();
    Serial.println("Data was saved in " + String(filename));
  }
  else {
    Serial.println("Error opening " + String(filename));
  }
}
