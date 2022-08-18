
  /*
Sensation ICE Monitor
2022 Jimmy Hough Jr.
*/

#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>

// Services
 // Bluetooth® Low Energy Battery Service
BLEService batteryService("180F");
BLEService accelerationService("acceleration");
BLEService gyroService("gyro");
BLEService magneticService("magneticField");
BLEService speedService("speedometer");//0x2712

// Bluetooth® Low Energy Battery Level Characteristic
BLEUnsignedCharCharacteristic batteryLevelChar("2A19",  // standard 16-bit characteristic UUID
    /*BLERead |*/ BLENotify); // remote clients will be able to get notifications if this characteristic changes

BLEFloatCharacteristic accelXChar("accX", BLENotify);
BLEFloatCharacteristic accelYChar("accY", BLENotify);
BLEFloatCharacteristic accelZChar("accZ", BLENotify);

BLEFloatCharacteristic angleXChar("angX", BLENotify);
BLEFloatCharacteristic angleYChar("angY", BLENotify);
BLEFloatCharacteristic angleZChar("angZ", BLENotify);

BLEDoubleCharacteristic speedChar("speed", BLENotify);
BLEDoubleCharacteristic magneticChar("mag", BLENotify);

// sensor data holders
int oldBatteryLevel = 0;  // last battery level reading from analog input
float oldGyroX,oldGyroY, oldGyroZ;
float accelX,accelY,accelZ;
float oldAccelX,oldAccelY, oldAccelZ;
float gyroX,gyroY,gyroZ;
float oldMagneticField;
float magneticField;

long previousMillis = 0;  // last time the battery level was checked, in ms


void setup() {
  Serial.begin(9600);    // initialize serial communication

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected
  analogReadResolution(12); // 12 bit ADC on Nano 33 
  IMU.begin();
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("FATAL: starting BLE failed!");
    while (1);
  }
  BLE.setDeviceName("Sensation");
  BLE.setLocalName("Sensation");

  BLE.setAdvertisedService(batteryService); 

  batteryService.addCharacteristic(batteryLevelChar); 
  BLE.addService(batteryService); // Add the battery service

  accelerationService.addCharacteristic(accelXChar);
  accelerationService.addCharacteristic(accelYChar);
  accelerationService.addCharacteristic(accelZChar);

  BLE.addService(accelerationService);

  gyroService.addCharacteristic(angleXChar);
  gyroService.addCharacteristic(angleYChar);
  gyroService.addCharacteristic(angleZChar);

  BLE.addService(gyroService);

  speedService.addCharacteristic(speedChar);
  BLE.addService(speedService);

  magneticService.addCharacteristic(magneticChar);
  BLE.addService(magneticService);

// initialize cahracteristic values
  batteryLevelChar.writeValue(oldBatteryLevel); // set initial value for this characteristic
  accelXChar.writeValue(oldAccelX);
  accelYChar.writeValue(oldAccelY);
  accelZChar.writeValue(oldAccelZ);

  angleXChar.writeValue(oldGyroX);
  angleYChar.writeValue(oldGyroY);
  angleZChar.writeValue(oldGyroZ);

  speedChar.writeValue(0.0);
  magneticChar.writeValue(0.0);
  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
  Serial.println("12 bit ADC enabled");
  Serial.println("Battery, Acceleration, Angle, Speed, MagneticField services installed.");
}

void loop() {
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());

    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    // check the battery level every 200ms
    // while the central is connected:
    while (central.connected()) {
      long currentMillis = millis();
      // if 200ms have passed, check the battery level:
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
        updateBatteryLevel();
        updateGyros();
        updateAcceleration();
      }
    }
    
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
   

  }
  else {
    Serial.print("waiting for connection.");
  }
}

void updateGyros(){
if (IMU.gyroscopeAvailable())
  {
    Serial.println("updating gyro");
    float delta = 400;
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    if (gyroX != oldGyroX) {
        angleXChar.writeValue(gyroX);
        oldGyroX = gyroX;
    }
    if (gyroY != oldGyroY) {
        angleYChar.writeValue(gyroY);
        oldGyroY = gyroY;
    }
    if (gyroZ != gyroZ) {
        angleZChar.writeValue(gyroZ);
        oldGyroZ = gyroZ;
    }

  }  
  else {
    Serial.println("gyro unavailable");
  }
}

void updateAcceleration() {
  Serial.println("updating Accel");
  float delta = 400;
  if (IMU.accelerationAvailable())
  {
    IMU.readAcceleration(accelX, accelY, accelZ);
 
    if(accelY <= delta && accelY >= -delta)
          Serial.println("flat");
    else if(accelY > delta && accelY < 1 - delta)
          Serial.println("tilted to the left");
    else if(accelY >= 1 - delta)
          Serial.println("left");
    else if(accelY < -delta && accelY > delta - 1)
          Serial.println("tilted to the right");
    else
          Serial.println("right");

    if (accelX != oldAccelX) {
        accelXChar.writeValue(accelX);
        oldAccelX = accelX;
    }
    if (accelY != oldAccelY) {
      accelYChar.writeValue(accelY);
      oldAccelY = accelY;
    }
    if (accelZ != oldAccelZ) {
      accelZChar.writeValue(accelZ);
      oldAccelZ = accelZ;
    }
    Serial.print("Read");
    Serial.println(accelX);
    Serial.println(accelY);
    Serial.println(accelZ);
}
  else {
     Serial.println("IMU unavailable");
  }

}

void updateBatteryLevel() {
  int battery = analogRead(A0);
  int batteryLevel = map(battery, 0, 4096, 0, 100);

  if (batteryLevel != oldBatteryLevel) {      // if the battery level has changed
    Serial.print("Battery Level % is now: "); // print it
    Serial.println(batteryLevel);
    batteryLevelChar.writeValue(batteryLevel);  // and update the battery level characteristic
    oldBatteryLevel = batteryLevel;           // save the level for next comparison
  }
}
