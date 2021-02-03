#include <AsyncTCP.h>
#include <ESPmDNS.h>
#include <WiFiManager.h>
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include "FirebaseESP32.h"
/*#include <SPI.h>
  #define BME_SCK 18
  #define BME_MISO 19
  #define BME_MOSI 23
  #define BME_CS 5*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


<<<<<<< HEAD
=======
//Firebase service account : "firebase-adminsdk-trg39@esp32-f9c31.iam.gserviceaccount.com"

#define FIREBASE_HOST "https://esp32-f9c31-default-rtdb.europe-west1.firebasedatabase.app/" //Change to your Firebase RTDB project ID e.g. Your_Project_ID.firebaseio.com
#define FIREBASE_AUTH "" //Change to your Firebase RTDB secret password
>>>>>>> 181002ad54428ac72454975ef0ce17e6694e2c4c

#define FIREBASE_HOST "" //Change to your Firebase RTDB project ID e.g. Your_Project_ID.firebaseio.com
#define FIREBASE_AUTH "" //Change to your Firebase RTDB secret password
#define ONBOARD_LED  2
#define PIN_RESET_BUTTON 2
#define PIN_BUZZER 0
//Define FirebaseESP32 data object
FirebaseData firebaseData;
FirebaseJson json;
String path = "/sensor1";
//WebServer
AsyncWebServer server(80);
WiFiManager wm;
int RESET = 0;
Adafruit_BME280 bmp; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI
hw_timer_t *timer = NULL; //faz o controle do temporizador (interrupção por tempo)


//------------SET FIREBASE------------
void setFirebase() {

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  //Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");
  /*
    This option allows get and delete functions (PUT and DELETE HTTP requests) works for device connected behind the
    Firewall that allows only GET and POST requests.

    Firebase.enableClassicRequest(firebaseData, true);
  */
}

//------------Return BME Temperature------------
String readBME280Temperature() {
  // Read temperature as Celsius (the default)
  float t = bmp.readTemperature();
  // Convert temperature to Fahrenheit
  //t = 1.8 * t + 32;
  if (isnan(t)) {
    Serial.println("Failed to read from BME280 sensor!");
    return "";
  }
  else {
    //Serial.println(t);
    return String(t);
  }
}

//------------Return BME Humidity------------
String readBME280Humidity() {
  float h = bmp.readHumidity();
  if (isnan(h)) {
    Serial.println("Failed to read from BME280 sensor!");
    return "";
  }
  else {
    //Serial.println(h);
    return String(h);
  }
}

//------------Return BME Pressure------------
String readBME280Pressure() {
  float p = bmp.readPressure() / 100.0F;
  if (isnan(p)) {
    Serial.println("Failed to read from BME280 sensor!");
    return "";
  }
  else {
    //Serial.println(p);
    return String(p);
  }
}

String readLdr() {
  // Read LDR Value (the default)
  int LDR_PIN = A4;
  int sensorValue;
  sensorValue = analogRead(LDR_PIN); // read analog input pin 4
  if (isnan(sensorValue)) {
    Serial.println("Failed to read from LDR sensor!");
    return "";
  }
  else {
    //Serial.println(sensorValue, DEC); // prints the value read
    //float voltagem = sensorValue * (3.3 / 1024.0);   // Converter a leitura analógica (que vai de 0 - 1023) para uma voltagem (0 - 3.3V), quanto de acordo com a intensidade de luz no LDR a voltagem diminui.
    //Serial.println(voltagem);   // Mostrar valor da voltagem no monitor serial
    return String(sensorValue);
  }
}

//------------Return Wifi RSSI------------
long getRssi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Couldn't get a wifi connection!");
    // return "";
    return 0;
  }
  else {
    // print the received signal strength:
    long rssi = WiFi.RSSI();
    //Serial.print("RSSI:");
    //Serial.println(rssi);
    //  return String(rssi);
    return  rssi;
  }
}

//------------Return streamToFirebase------------
void streamToFirebase() {
  //Get the value from getRssi() convert to json and update the cloud with the path /data + path( id of node e.g)
  int value = getRssi();
  json.set("/rssi", value);
  Firebase.updateNode(firebaseData, "/data" + path, json);
}

//------------Buzz active buzzer------------
void buzzing() {
  int i;//freq
  for (i = 0; i < 80; i++) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(1);//wait for 1ms
    digitalWrite(PIN_BUZZER, LOW);
    delay(1);//wait for 1ms
  }
}

//função que o temporizador irá chamar, para reiniciar o ESP32
void IRAM_ATTR resetModule() {
  ets_printf("(watchdog) reiniciar\n"); //imprime no log
  ESP.restart(); //reinicia o chip
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PIN_RESET_BUTTON, INPUT);
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  //reset saved settings
  //wm.resetSettings();
  //wm.setBreakAfterConfig(true);
  //wm.setTimeout(180); //  timeout until configuration portal gets turned off
  //Assign fixed IP
  //wifiManager.setAPStaticIPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  //Try to connect WiFi, then create AP
  wm.autoConnect("ESP32_AP", "");

  pinMode(ONBOARD_LED, OUTPUT);//Onboard Blue Led
  pinMode(PIN_BUZZER, OUTPUT); //initialize the buzzer pin as an output

  //Setup Bme
  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  //the library is blocking. Once connected, the program continues
  Serial.println("ESP32 is connected to Wi-Fi network");

  setFirebase();

  //Setup Spiff
  if (!SPIFFS.begin()) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  //Setup MDNS
  if (!MDNS.begin("esp32")) {
    Serial.println("Error starting mDNS");
    return;
  }

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html", "text/html" );
  });
  server.on("/light", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readLdr().c_str());
  });
  server.on("/wifi", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", String(getRssi()).c_str());
  });

  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readBME280Temperature().c_str());
  });

  server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readBME280Pressure().c_str());
  });

  server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readBME280Humidity().c_str());
  });

  server.begin();
  Serial.println("HTTP server started");
  delay(100);

  timer = timerBegin(0, 80, true); //timerID 0, div 80
  //timer, callback, interrupção de borda
  timerAttachInterrupt(timer, &resetModule, true);
  //timer, tempo (us), repetição
  timerAlarmWrite(timer, 10000000, true);
  timerAlarmEnable(timer); //habilita a interrupção
  delay(100);
}
void loop() {
  timerWrite(timer, 0); //reseta o temporizador (alimenta o watchdog)
  RESET = digitalRead(PIN_RESET_BUTTON);
  if ( RESET == HIGH) {
    Serial.println("Erase settings and restart ...");
    delay(1000);
    wm.resetSettings();
    buzzing();
    ESP.restart();
  }

  //Stream to firebase
  streamToFirebase();

  //blink Onboard led
  digitalWrite(ONBOARD_LED, HIGH);
  delay(100);
  digitalWrite(ONBOARD_LED, LOW);

}
