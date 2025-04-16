/* mb7060_tester_ESP8266.ino
 * weigu.lu 
 * Test ultrasonic sensor MB7060 from MaxBotix
 * serial 8N1 9600bit/s
 * Wemos D1 mini pro with OLED shield
 * Wemos GPIO 2 (D4) to Pin 4 (Rx) MB7060
 * Wemos GPIO 3 (RX) to Pin 5 (Tx) MB7060
 */

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306_Wemos_OLED.h>

#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET); // Create display object

const byte PIN_MB7060_REQUEST = D4; // GPIO 2 of D1 mini pro (also LED)

void setup() {
  Serial.begin(9600, SERIAL_8N1, SERIAL_FULL, 1, true);  // init serial (true = invert)
  pinMode(PIN_MB7060_REQUEST, OUTPUT);                   // init request_pin();
  init_display(); 
  delay(2000);
}

void loop() {
  display_distance_and_capacity();
  delay(1000);
}

void init_display() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);      
  display.setTextColor(WHITE);
  display.setCursor(4, 8);
  display.println("Distance: ");  
  display.display();
}

unsigned int get_distance_mb7060_cm(byte data_req_pin) {
  String MB7060_data;
  char mb7060_data[6];  
  unsigned int byte_counter = 0, distance_cm = 0;  
  bool valid_data = true;    
  digitalWrite(data_req_pin,LOW);         // Request serial data Off
  delay(10);
  digitalWrite(data_req_pin,HIGH);        // Request serial data On
  while (Serial.available() > 0) {        // Clear the buffer!
    Serial.read();
  } 
  delay(200);                             // Wait for the message
  while ((Serial.available()) && (byte_counter < 5)) {
    mb7060_data[byte_counter] = Serial.read();
    if (mb7060_data[0] != 'R') {          // wrong data, clear buffer, get out
      distance_cm = 0;
      while (Serial.available() > 0) { 
        Serial.read();
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

void display_distance_and_capacity() {  
  // inner diameter: 2,55m height about 2m, A = 5,107m², 
  unsigned int distance_cm, capacity_l;  
  distance_cm = get_distance_mb7060_cm(PIN_MB7060_REQUEST);
  capacity_l = int(round(((201+3.5)-float(distance_cm))*51.07));
  display.clearDisplay();
  display.display();
  display.setCursor(4, 8);
  display.println("Distance: ");  
  display.setCursor(20, 20);
  display.println(distance_cm);  
  display.setCursor(4, 30);
  display.println("Capacity: ");  
  display.setCursor(20, 40);
  display.println(capacity_l);
  display.display();  
}
