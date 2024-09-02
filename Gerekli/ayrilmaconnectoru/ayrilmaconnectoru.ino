//guvenlik konectoru

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
//#define BNO055_SAMPLERATE_DELAY_MS (100)

const int ayrilma = 8;
const int ledPin = 13;
int butdurumu = 0;

RF24 radio(9, 10);  // CE, CSN
const byte address[6] = "00007";

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void displaySensorDetails(void) {
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" xxx");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" xxx");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displaySensorStatus(void) {
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

void displayCalStatus(void) {

  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system) {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

void setup() {

  pinMode(ledPin, OUTPUT);
  pinMode(ayrilma, INPUT_PULLUP);
  pinMode(1, OUTPUT);  // Röle kontrol ucu çıkış olarak tanımlanmıştır
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  radio.begin();
  radio.openReadingPipe(0, address);

  radio.startListening();
  Serial.println("Orientation Sensor Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  delay(1000);
  displaySensorDetails();
  displaySensorStatus();

  bno.setExtCrystalUse(true);
}

void loop() {
  butdurumu = digitalRead(ayrilma);  //buton basma kontrolü
  if (butdurumu == LOW)              //butona basılırsa
  {
    Serial.println("Güvenlik konektöründen ayrılma gerçekleşmedi.");
    digitalWrite(1, HIGH);
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);

    digitalWrite(ledPin, HIGH);  //LED AÇIK
  } else {
    while (1) {
      if (!radio.available()) {
        char text[32] = { 0 };
        radio.read(&text, sizeof(text));
        Serial.println(text);
        sensors_event_t event;
        bno.getEvent(&event);

        /* Display the floating point data */
        Serial.print("X: ");
        Serial.print(event.orientation.x, 4);
        Serial.print("\tY: ");
        Serial.print(event.orientation.y, 4);
        Serial.print("\tZ: ");
        Serial.print(event.orientation.z, 4);

        /* Optional: Display calibration status */
        displayCalStatus();

        //Eksenlerin yerleri çok önemli bunu kontrol etmeyi unutmadan atışa gitme.
        if (event.orientation.z > 0 && event.orientation.y > 0) {
          digitalWrite(1, LOW);
          digitalWrite(2, HIGH);
          digitalWrite(3, HIGH);
          digitalWrite(4, HIGH);
          digitalWrite(5, LOW);
        }
        if (event.orientation.z < 0 && event.orientation.y > 0) {
          digitalWrite(1, HIGH);
          digitalWrite(2, LOW);
          digitalWrite(3, HIGH);
          digitalWrite(4, HIGH);
          digitalWrite(5, LOW);
        }
        if (event.orientation.z < 0 && event.orientation.y < 0) {
          digitalWrite(1, HIGH);
          digitalWrite(2, HIGH);
          digitalWrite(3, LOW);
          digitalWrite(4, HIGH);
          digitalWrite(5, LOW);
        }
        if (event.orientation.z > 0 && event.orientation.y < 0) {
          digitalWrite(1, HIGH);
          digitalWrite(2, HIGH);
          digitalWrite(3, HIGH);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
        } else if (event.orientation.z == 0 && event.orientation.y == 0) {
          digitalWrite(1, HIGH);
          digitalWrite(2, HIGH);
          digitalWrite(3, HIGH);
          digitalWrite(4, HIGH);
          digitalWrite(5, LOW);
        }

        Serial.println("");
        //delay(BNO055_SAMPLERATE_DELAY_MS);
      }
    }
  }
}