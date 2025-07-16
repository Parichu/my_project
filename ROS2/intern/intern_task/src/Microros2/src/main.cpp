#include <WiFi.h>

// ⚙️ กำหนดชื่อ Wi-Fi และรหัสผ่านของคุณ
const char* ssid = "Public-wifi/pass:1234";         // <-- แก้เป็นชื่อ Wi-Fi ของคุณ
const char* password = "password1234"; // <-- แก้เป็นรหัส Wi-Fi

void setup() {
  // เปิด Serial Monitor
  Serial.begin(115200);
  delay(1000);  // รอ Serial ทำงาน

  // เริ่มเชื่อมต่อ Wi-Fi
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  // รอจนกว่าจะเชื่อมต่อ
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // แสดงผลเมื่อเชื่อมต่อสำเร็จ
  Serial.println();
  Serial.println("✅ Connected to WiFi!");
  Serial.print("📡 ESP32 IP Address: ");
  Serial.println(WiFi.localIP());
}
 
void loop() {
  // สามารถเพิ่มโค้ดอื่นใน loop ได้ เช่น ping, LED, MQTT ฯลฯ
}
