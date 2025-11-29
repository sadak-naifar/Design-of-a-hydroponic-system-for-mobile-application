#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <DHT.h>
#include <time.h>

// Pin configuration
#define DHTPIN 4          // GPIO 4 for DHT11
#define BUTTON_PIN 13     // GPIO 13 for push button
#define LED_FIREBASE 33   // GPIO 21 for Firebase mode LED
#define LED_BLUETOOTH 32  // GPIO 23 for Bluetooth mode LED
#define GAS_SENSOR_PIN 34 // GPIO 34 for gas sensor
const int trigPin = 5;           // Ultrasonic sensor TRIG
const int echoPin = 18;          // Ultrasonic sensor ECHO
const int pumpRelay = 19;        // Water pump control (LOW = ON)
const int valveRelay = 22;       // Drain valve control (LOW = ON)

// Thresholds
const int minWaterLevel = 2;     // Minimum water level (cm)
const int maxWaterLevel = 5;     // Maximum water level (cm)

// Water control states
enum WaterState { FILLING, WAITING_TO_DRAIN, DRAINING };
WaterState waterState = FILLING;
unsigned long stateStartTime = 0;
const unsigned long DRAIN_DELAY = 30000; // 30 seconds delay before draining

// Sensor type
#define DHTTYPE DHT11

// WiFi credentials
#define WIFI_SSID "TT_E598"
#define WIFI_PASSWORD "c611sfk7d5"

// Firebase credentials
#define FIREBASE_HOST "https://hydroponic-system-be5a7-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "GmaJoIAPQJaQbbjZfyjkOUBHwtndqysSlmwZXbXY"

// Time configuration
#define GMT_OFFSET_SEC 3600      
#define DAYLIGHT_OFFSET_SEC 0    

// BLE UUIDs
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define TEMP_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define HUMID_CHAR_UUID "a0996798-aa52-4858-be39-00924cae77cb"
#define GAS_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define WATER_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26aa"
#define STATE_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26ab"

// Global variables
DHT dht(DHTPIN, DHTTYPE);
FirebaseData firebaseData;
FirebaseConfig firebaseConfig;
FirebaseAuth firebaseAuth;
String outputPath = "ESP";

BLEServer* pServer = NULL;
BLEService* pService = NULL;
BLECharacteristic* pTempChar = NULL;
BLECharacteristic* pHumidChar = NULL;
BLECharacteristic* pGasChar = NULL;
BLECharacteristic* pWaterChar = NULL;
BLECharacteristic* pStateChar = NULL;

float temperature = 0;
float humidity = 0;
int gasValue = 0;
float waterLevel = 0;

// Timing
unsigned long lastMeasurementTime = 0;
const unsigned long measurementInterval = 2000; // 2 seconds
bool deviceConnected = false;
bool currentMode = false; // false = Bluetooth, true = Firebase
bool lastButtonState = HIGH;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Device connected");
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Device disconnected");
        // Restart advertising to allow reconnection
        pServer->startAdvertising();
    }
};

class PumpCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            int command = value[0];
            if (command == 1) {
                digitalWrite(pumpRelay, LOW);
                Serial.println("Pump turned ON via BLE");
            } else {
                digitalWrite(pumpRelay, HIGH);
                Serial.println("Pump turned OFF via BLE");
            }
        }
    }
};

float measureWaterLevel() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;
  return 10 - distance; // Assuming 10cm is full tank height
}

void controlWaterSystem(float waterLevel) {
  switch(waterState) {
    case FILLING:
       pinMode(pumpRelay, OUTPUT);  // Pump ON
       pinMode(valveRelay, INPUT);  // Valve OFF
     
      if (waterLevel >= maxWaterLevel) {
         pinMode(pumpRelay, INPUT); // Pump OFF
        waterState = WAITING_TO_DRAIN;
        stateStartTime = millis();
        Serial.println("Max level reached - waiting 30 seconds to drain");
      }
      break;

    case WAITING_TO_DRAIN:
      if (millis() - stateStartTime >= DRAIN_DELAY) {
        waterState = DRAINING;
        Serial.println("Starting drainage");
      }
      break;

    case DRAINING:
       pinMode(pumpRelay, INPUT);  // Pump OFF
       pinMode(valveRelay, OUTPUT);   // Valve ON
     
      if (waterLevel < minWaterLevel) {
        pinMode(valveRelay, INPUT); // Valve OFF
        waterState = FILLING;
        Serial.println("Min level reached - starting refill");
      }
      break;
  }
}

const char* getStateName(WaterState state) {
  switch(state) {
    case FILLING: return "FILLING";
    case WAITING_TO_DRAIN: return "WAITING";
    case DRAINING: return "DRAINING";
    default: return "UNKNOWN";
  }
}

void readSensors() {
    float newTemp = dht.readTemperature();
    float newHumid = dht.readHumidity();
    waterLevel = measureWaterLevel();
   
    if (!isnan(newTemp)) temperature = newTemp;
    if (!isnan(newHumid)) humidity = newHumid;
   
    gasValue = analogRead(GAS_SENSOR_PIN);  
    controlWaterSystem(waterLevel);
   
    Serial.printf("Temp: %.1f°C, Humid: %.1f%%, Gas: %d, Water: %.1fcm, State: %s\n",
                 temperature, humidity, gasValue, waterLevel, getStateName(waterState));
}

void setupBluetooth() {
    BLEDevice::init("SmartFarm-ESP32");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
   
    pService = pServer->createService(SERVICE_UUID);

    // Create characteristics
    pTempChar = pService->createCharacteristic(
        TEMP_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pTempChar->addDescriptor(new BLE2902());

    pHumidChar = pService->createCharacteristic(
        HUMID_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pHumidChar->addDescriptor(new BLE2902());

    pGasChar = pService->createCharacteristic(
        GAS_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pGasChar->addDescriptor(new BLE2902());

    pWaterChar = pService->createCharacteristic(
        WATER_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pWaterChar->addDescriptor(new BLE2902());

    pStateChar = pService->createCharacteristic(
        STATE_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ
    );

    pService->start();
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.println("BLE Server Ready");
}

void setupFirebase() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());

    firebaseConfig.host = FIREBASE_HOST;
    firebaseConfig.signer.tokens.legacy_token = FIREBASE_AUTH;
    Firebase.begin(&firebaseConfig, &firebaseAuth);
    Firebase.reconnectWiFi(true);
   
    // Sync time
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.nist.gov");
    Serial.println("Synchronizing time...");
    while (!time(nullptr)) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nTime synchronized!");
   
    Serial.println("Firebase initialized");
}

void sendDataBluetooth() {
    if (!deviceConnected) return;

    uint8_t tempBuffer[4];
    memcpy(tempBuffer, &temperature, 4);
    pTempChar->setValue(tempBuffer, 4);
    pTempChar->notify();
   
    uint8_t humidBuffer[4];
    memcpy(humidBuffer, &humidity, 4);
    pHumidChar->setValue(humidBuffer, 4);
    pHumidChar->notify();
   
    uint8_t gasByte = map(gasValue, 0, 4095, 0, 255);
    pGasChar->setValue(&gasByte, 1);
    pGasChar->notify();
   
    uint8_t waterByte = constrain(waterLevel * 10, 0, 255); // Convert to cm*10 for better precision
    pWaterChar->setValue(&waterByte, 1);
    pWaterChar->notify();
   
    String stateStr = getStateName(waterState);
    pStateChar->setValue(stateStr.c_str());
}

void sendDataFirebase() {
    FirebaseJson json;
    json.set("temperature", temperature);
    json.set("humidity", humidity);
    json.set("gas", gasValue);
    json.set("water_level", waterLevel);
    json.set("state", getStateName(waterState));

    if (Firebase.set(firebaseData, outputPath, json)) {
        Serial.println("Data sent to Firebase successfully");
    } else {
        Serial.println("Failed to send data to Firebase");
        Serial.println("REASON: " + firebaseData.errorReason());
    }
}

void setup() {
    Serial.begin(115200);
   
    // Initialize pins
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_FIREBASE, OUTPUT);
    pinMode(LED_BLUETOOTH, OUTPUT);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(GAS_SENSOR_PIN, INPUT);
   
   

   
    // Start with both LEDs OFF
    digitalWrite(LED_BLUETOOTH, LOW);
    digitalWrite(LED_FIREBASE, LOW);
   
    dht.begin();
   
    // Initialize both modes
    setupBluetooth();
    setupFirebase();
   
    Serial.println("System started");
}

void loop() {
    // Check mode switch button
    bool buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == LOW && lastButtonState == HIGH) {
        currentMode = !currentMode;
        Serial.println("Mode switched to " + String(currentMode ? "Firebase" : "Bluetooth"));
        delay(200); // Debounce
    }
    lastButtonState = buttonState;

    // Read sensors and control water system
    if (millis() - lastMeasurementTime >= measurementInterval) {
        readSensors();
        lastMeasurementTime = millis();
       
        if (currentMode) {
            // Firebase mode
            digitalWrite(LED_BLUETOOTH, LOW);
            digitalWrite(LED_FIREBASE, HIGH);
            sendDataFirebase();
        } else {
            // Bluetooth mode (only send data if connected)
            digitalWrite(LED_FIREBASE, LOW);
            digitalWrite(LED_BLUETOOTH, deviceConnected ? HIGH : LOW);
            if (deviceConnected) {
                sendDataBluetooth();
            }
        }
    }
   
    delay(100);
}
