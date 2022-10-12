
  /*
Sensation ICE Monitor
2022 Jimmy Hough Jr.
*/

#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>
#include "max6675.h" 

bool hasNotifiedDisconnect = false;

const char* firmware_version = "0.1.0";

int usb_serial_baud = 9600;
int adc_resolution = 12;

byte SO = 12;
byte CS = 10;
byte sck = 13;
byte tachPin = A2;
byte speedoPin = A1;
byte batteryPin = A0;

unsigned long speedPulseCount = 0;
unsigned long tachPulseCount = 0;

MAX6675 module(sck, CS, SO);

// Custom characteristic guids
    const char* accelXGUID = "589636a3-4a3c-41f4-a62d-057183b7bc07";
    const char* accelYGUID = "b4499eb6-0b53-4e2e-b87e-4a7a0dc81bd3";
    const char* accelZGUID = "b0773a30-0c61-489d-a3e0-c1e66fef6b25";
    const char* gyroXGUID = "2069b95b-61c0-4f20-9e35-7656c5fb7932";
    const char* gyroYGUID = "a4cc7540-4782-4902-9626-176c218ea850";
    const char* gyroZGUID = "9ede0bc2-ca7d-47c3-9c25-24984bd1a383";
    const char* speedGUID = "487c132a-e771-410f-b677-a95b56c5cf97";
    const char* magneticFieldGUID = "c5396eb2-2f22-439c-90ce-e06e9311f1dd";

    const char* temperatureGUID = "5da82033-af9d-495f-bb39-89ecb3fbe73e";
    const char* envTemperatureGUID = "04800f4c-112c-4a27-bdfa-de3c7134e386";

    const char* isLiveGUID = "44f2747f-693e-478c-8340-b251cebcf021";

    const char* batteryGUID = "eee4fd2d-db70-4cb8-b2cb-4a4dfcd232ef";
    const char* regulatorGUID = "9CD11CDA-38E1-4552-9293-98D5293A7F51";

    const char* tachGUID = "3d94d295-f174-432c-9eec-8dbbace7c79d";
    const char* speedDiffGUID = "c01e3037-5e89-45d0-a2ac-4d68bd669eeb";

// Custom service GUIDS
  const char* commandServiceGUID = "0fec88ee-5036-41ce-846d-d3831fd60e6a";
  const char* tachServiceGUID = "a50cca46-44a0-4f90-9022-889bbcab5972";


// Services
BLEService batteryService("939a3ede-dd78-4f3c-b17f-96ef5e165fa4");
BLEService accelerationService("88369fea-5048-4e9c-9433-3b1f99752d74");
BLEService gyroService("42af00aa-4387-4a84-962f-6e74aecfd818");
BLEService magneticService("d4b67eb7-86fa-4af3-b1ae-a57dfad6acb3");
BLEService speedService("ff208e73-30c1-4ecd-80b4-e41a2a38c59b");//0x2712
BLEService temperatureService("57d20d31-f7e6-48c0-adfa-a7386f087174");//0x2712
BLEService commandService(commandServiceGUID);
BLEService tachService(tachServiceGUID);

BLEBoolCharacteristic isLiveModeChar(isLiveGUID, BLEWrite | BLERead);

BLEFloatCharacteristic batteryLevelChar(batteryGUID, BLERead | BLENotify);
BLEFloatCharacteristic regulatorLevelChar(regulatorGUID, BLERead | BLENotify);
BLEFloatCharacteristic temperatureChar(temperatureGUID, BLERead | BLENotify);
BLEFloatCharacteristic envTemperatureChar(envTemperatureGUID, BLERead | BLENotify);

BLEFloatCharacteristic accelXChar(accelXGUID,  BLERead | BLENotify);
BLEFloatCharacteristic accelYChar(accelYGUID,  BLERead | BLENotify);
BLEFloatCharacteristic accelZChar(accelZGUID,  BLERead | BLENotify);

BLEFloatCharacteristic angleXChar(gyroXGUID,  BLERead | BLENotify);
BLEFloatCharacteristic angleYChar(gyroYGUID,  BLERead | BLENotify);
BLEFloatCharacteristic angleZChar(gyroZGUID,  BLERead | BLENotify);

BLEFloatCharacteristic magneticChar(magneticFieldGUID,  BLERead | BLENotify);

BLEFloatCharacteristic speedChar(speedGUID,  BLERead | BLENotify);
BLEFloatCharacteristic speedDiffChar(speedDiffGUID, BLERead | BLENotify);

BLEFloatCharacteristic tachChar(tachGUID, BLERead | BLENotify);

// sensor data holders
float oldBatteryLevel, batteryLevel = 0.0;  // last battery level reading from analog input

float gyroX, gyroY, gyroZ = 0.0;
float oldGyroX, oldGyroY, oldGyroZ = 0.0;

float accelX, accelY, accelZ = 0.0;
float oldAccelX, oldAccelY, oldAccelZ = 0.0;

float oldMagneticField, magneticField = 0.0;
float oldSpeed, speed = 0.0;
float oldTach, tach  = 0.0;

// update rates all in millis
unsigned long previousBatMillis = 0;  // last time the battery level was checked, in ms
unsigned long previousAccMillis = 0;
unsigned long previousGyroMillis = 0;
unsigned long previousSpeedMillis = 0;
unsigned long previousMagMillis = 0;
unsigned long previousTempMillis = 0;
unsigned long previousTachMillis = 0;

unsigned long batteryUpdateRate = 200; //ms
unsigned long accUpdateRate = 200;
unsigned long gyroUpdateRate = 200;
unsigned long tempUpdateRate = 1000;
unsigned long magneticFieldUpdateRate = 200;
unsigned long speedUpdateRate = 200;
unsigned long tachUpfateRate = 200;

unsigned long minSpeedPulseTime = 1;// 
unsigned long lastSpeedPulse, speedPulse = 0;
unsigned long speedInterval = 0;

void speedIRQ() {

  lastSpeedPulse = speedPulse;
  speedPulse = millis();
  speedInterval = speedPulse - lastSpeedPulse;
  speedDiffChar.writeValue(speedInterval);

  speedPulseCount++;
}

void tachIRQ() {
  tachPulseCount++;
}

void setupServices() {

  BLE.setAdvertisedService(batteryService); 

  batteryService.addCharacteristic(batteryLevelChar); 
  batteryService.addCharacteristic(regulatorLevelChar);
  BLE.addService(batteryService); // Add the battery service

  temperatureService.addCharacteristic(temperatureChar);
  temperatureService.addCharacteristic(envTemperatureChar);

  BLE.addService(temperatureService);

  accelerationService.addCharacteristic(accelXChar);
  accelerationService.addCharacteristic(accelYChar);
  accelerationService.addCharacteristic(accelZChar);

  BLE.addService(accelerationService);

  gyroService.addCharacteristic(angleXChar);
  gyroService.addCharacteristic(angleYChar);
  gyroService.addCharacteristic(angleZChar);

  BLE.addService(gyroService);

  speedService.addCharacteristic(speedChar);
  speedService.addCharacteristic(speedDiffChar);
  BLE.addService(speedService);

  tachService.addCharacteristic(tachChar);
  BLE.addService(tachService);

  magneticService.addCharacteristic(magneticChar);
  BLE.addService(magneticService);

  isLiveModeChar.writeValue(true);
  commandService.addCharacteristic(isLiveModeChar);
  BLE.addService(commandService);
}

void setup() {
  delay(10000);
  Serial.begin(115200);    // initialize serial communication
  Serial.print("baud rate ");
  Serial.println(usb_serial_baud);

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected
  analogReadResolution(12); // 12 bit ADC on Nano 33 
   pinMode(speedoPin,INPUT);
  pinMode(batteryPin, INPUT);
  pinMode(tachPin, INPUT);
  Serial.println("Arduino Nano 33 IoT");
  Serial.print("Firmware Version ");
  Serial.print(firmware_version);
  Serial.print("- PIN MAP -");
  Serial.print("LED :");
  Serial.println(LED_BUILTIN);
  Serial.print("Speedo :");
  Serial.println(speedoPin);

  Serial.print("Tacho :");
  Serial.println(tachPin);
  Serial.print("battery :");
  Serial.println(batteryPin);
  Serial.println("MAX6675 on ");
  Serial.print("CS :");
  Serial.print(CS);
  Serial.print("SO: ");
  Serial.print(SO);
  Serial.print("SCK: ");
  Serial.print(sck);

  IMU.begin();
  Serial.println("LSM6DS3 initialized.");
  

  if (!BLE.begin()) {
    Serial.println("FATAL: starting BLE failed!");
    while (1);
  }

  BLE.setDeviceName("Sensation");
  BLE.setLocalName("Sensation");

  setupServices();
 

// initialize cahracteristic values
  batteryLevelChar.writeValue(oldBatteryLevel); // set initial value for this characteristic
  accelXChar.writeValue(oldAccelX);
  accelYChar.writeValue(oldAccelY);
  accelZChar.writeValue(oldAccelZ);

  angleXChar.writeValue(oldGyroX);
  angleYChar.writeValue(oldGyroY);
  angleZChar.writeValue(oldGyroZ);

  speedChar.writeValue(oldSpeed);
  speedDiffChar.writeValue(0.0);

  tachChar.writeValue(oldTach);

  magneticChar.writeValue(oldMagneticField);
  temperatureChar.writeValue(0.0);

  // default to live mode
  isLiveModeChar.writeValue(true);

  // interrupts for pulse counting/timing
   attachInterrupt(speedoPin, speedIRQ, FALLING ); // pin 2 looks for HIGH to LOW change
  //  attachInterrupt(tachPin, tachIRQ, FALLING);
  
  // start advertising
  BLE.advertise();

}

void loop() {
  BLEDevice central = BLE.central(); 

  if (central) {
    hasNotifiedDisconnect = false;
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {

      // check for live mode toggle, later check for commands mtoo
      if (isLiveModeChar.written()) {
        Serial.println("wrote isLive : ");
        Serial.println(isLiveModeChar.value() ? "true" : "false");
      }
      // if live mode push data at intervals
      if (isLiveModeChar.value()) {
        long currentMillis = millis();

        long battime = currentMillis - previousBatMillis;
        long gyrotime = currentMillis - previousGyroMillis;
        long acctime = currentMillis - previousAccMillis;
        long temptime = currentMillis - previousTempMillis;
        long speedTime = currentMillis - previousSpeedMillis;
        long tachTime = currentMillis - previousTachMillis;
        
        if (battime >= batteryUpdateRate) {
          previousBatMillis = currentMillis;
          updateBatteryLevel();
        }
        if (gyrotime >= gyroUpdateRate) {
          previousGyroMillis = currentMillis;
          updateGyros();
        }
        if (acctime >= accUpdateRate) {
          previousAccMillis = currentMillis;
          updateAcceleration();
        }
        if (temptime >= tempUpdateRate) {
          previousTempMillis = currentMillis;
          updateTermperature();
        }

        if (speedTime >= minSpeedPulseTime) {
          previousSpeedMillis = currentMillis;
          updateSpeed();
        }
        if (tachTime >= tachUpfateRate) {
          previousTachMillis = currentMillis;
          updateTach();
        }
      }
      // if not live mode, can poll from the central

    }
    
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");

  }
  else {
    if (!hasNotifiedDisconnect) {
      hasNotifiedDisconnect = true;
        Serial.println("Bluetooth® device active, waiting for connections...");
    }
  }
}

void updateTermperature() {
     float temperature = module.readCelsius();
     float envTemp = 0.0;
     temperatureChar.writeValue(temperature);
     envTemperatureChar.writeValue(envTemp);
}

void updateGyros() {
if (IMU.gyroscopeAvailable())
  {

    IMU.readGyroscope(gyroX, gyroY, gyroZ);

        angleXChar.writeValue(gyroX);
        oldGyroX = gyroX;
        angleYChar.writeValue(gyroY);
        oldGyroY = gyroY;
        angleZChar.writeValue(gyroZ);
        oldGyroZ = gyroZ;

  }  
  else {
    Serial.println("gyro unavailable");
  }
}

void updateAcceleration() {
  float delta = 400;
  if (IMU.accelerationAvailable())
  {
    IMU.readAcceleration(accelX, accelY, accelZ);

    if ((accelX != oldAccelX) /*&& (abs(accelX - oldAccelX) > 0.1)*/) {
        accelXChar.writeValue(accelX);
        oldAccelX = accelX;
    }
    if ((accelY != oldAccelY) /*&& (abs(accelY - oldAccelY) > 0.1)*/) {
      accelYChar.writeValue(accelY);
      oldAccelY = accelY;
    }
    if ((accelZ != oldAccelZ) /*&& (abs(accelZ - oldAccelZ) > 0.1)*/ ) {
      accelZChar.writeValue(accelZ);
      oldAccelZ = accelZ;
    }
    
}
  else {
     Serial.println("IMU unavailable");
  }

}

void updateBatteryLevel() {
  int battery = analogRead(batteryPin);
  float batteryLevel = map(battery, 0, 4096, 0.0, 18.0);

  if (batteryLevel != oldBatteryLevel) {      
    batteryLevelChar.writeValue(batteryLevel);
    oldBatteryLevel = batteryLevel;
  }
}

void updateSpeed() {
  speedDiffChar.writeValue(speedInterval);
  speedChar.writeValue(speed);
  speedPulseCount = 0;
}

void updateTach() {
  tachChar.writeValue(tachPulseCount);
  tachPulseCount = 0;
}

// template <class X, class M, class N, class O, class Q>
// X map_Generic(X x, M in_min, N in_max, O out_min, Q out_max){
//   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }