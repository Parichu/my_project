#include <Arduino.h>

#include <DHT.h>

// --- DHT Configuration ---
#define DHTPIN 27         
DHT dht;

// --- Sensor Pins ---
#define soilAnalogPin_in 34
#define soilDigitalPin 18
#define lightAnalogPin 35
#define relayPin 26 // ขา ESP32 ที่ควบคุมรีเลย์

void setup() {
  Serial.begin(57600);
  
  dht.begin();

  pinMode(soilDigitalPin, INPUT); //อ่านค่า Sensor 
  pinMode(relayPin, OUTPUT); //กำหนดให้ Delay Pin เป็น Output
  digitalWrite(relayPin, LOW); // ปิดปั๊มน้ำไว้ก่อนเริ่ม
}

void loop() {
  float temp = dht.readTemperature();
  float humi = dht.readHumidity();


  /*Check การทำงานของ Sensor ไม่จำเป็นถ้ามั่นใจว่าต่อวงจรถูก100%*/
  // if (isnan(temp) || isnan(humi)) {
  //   Serial.println("Failed to read from DHT11 sensor!");
  //   delay(2000);
  //   return;
  // }

  int soilAnalog = analogRead(soilAnalogPin_in);   // ความชื้นในดิน (ยิ่งมาก = เปียก)
  int lightLevel = analogRead(lightAnalogPin);  // ค่าความสว่าง

  // แสดงค่าบน Serial Monitor
  Serial.print("Temp: "); 
  Serial.print(temp); 
  Serial.println(" °C");
  Serial.print("Humi: "); 
  Serial.print(humi); 
  Serial.println(" %");
  Serial.print("Soil Moisture: "); 
  Serial.println(soilAnalog);
  Serial.print("Light: "); 
  Serial.println(lightLevel);

  // -----------------------------
  // เงื่อนไขรดน้ำว่านหางจระเข้ (Truth Table ~= ตรรกศาสตร์)
  // -----------------------------
  bool isSoilDry = soilAnalog < 2000; //แปลงจากแรงดัน 3.3V = 4095 
  bool isAirDry = humi < 50;
  bool isHot = temp > 35;

  if (isSoilDry && (isAirDry || isHot)){
    digitalWrite(relayPin, HIGH); // เปิดปั๊มน้ำ
    Serial.println("Open water pump...");
  } 
  else {
    digitalWrite(relayPin, LOW);  // ปิดปั๊มน้ำ
    Serial.println("Close water pump");
  }

  Serial.println("-------------------------");
  delay(10000); // รอ 10 วินาที ก่อนอ่านรอบใหม่
}