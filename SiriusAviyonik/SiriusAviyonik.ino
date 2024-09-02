#include <Wire.h>
#include <SPI.h>
#include <LPS.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <MPU6050.h>
#include <SD.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

const int ayrilma = 8;
const int ledPin = 13;
int butdurumu = 0;

RF24 radio(9, 10);  // CE, CSN 
const byte address[6] = "00007";

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
LPS ps;
MPU6050 mpu;

// Genel değişkenler
float verticalVelocity = 0.0;
float baseAccelerationZ = 0.0;
unsigned long previousTime = 0;

// Kalman filtresi parametreleri
float x_est = 0.0;
float P_est = 1.0;
const float Q = 0.0001;  // Daha düşük sistem gürültüsü
const float R = 10.0;    // Daha yüksek ölçüm gürültüsü

// Hız sıfırlama için değişkenler
const float VELOCITY_THRESHOLD = 0.03;  // Artırılmış hız eşiği
const unsigned long RESET_INTERVAL = 500;  // Daha sık sıfırlama kontrolü
unsigned long lastResetTime = 0;

// Hareketli ortalama filtresi için değişkenler
const int FILTER_WINDOW = 20;  // Daha büyük filtre penceresi
float velocityReadings[FILTER_WINDOW];
int readIndex = 0;
float velocityTotal = 0;

// Yeni: İvme için "ölü bölge" tanımı
const float ACCELERATION_DEADZONE = 0.02;  // g cinsinden

// Yeni: Hareket algılama için değişkenler
bool isMoving = false;
const int MOTION_THRESHOLD_COUNT = 5;
int motionCount = 0;

// SD kart tanımlamaları
const int chipSelect = BUILTIN_SDCARD;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(ayrilma, INPUT_PULLUP);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.startListening();

  Serial.println("Orientation Sensor Test");
  Serial.println("");

  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
  displaySensorDetails();
  displaySensorStatus();

  bno.setExtCrystalUse(true);

  // LPS25HB sensörünü başlat
  Wire.begin();
  if (!ps.init()) {
    Serial.println("Failed to autodetect pressure sensor!");
    while (1);
  }
  ps.enableDefault();

  // MPU6050 başlat
  Serial.println("MPU6050 başlatılıyor...");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 ile bağlantı kurulamadı!");
    while (1);
  }
  Serial.println("MPU6050 bağlandı.");

  // Kalibrasyon
  Serial.println("Kalibrasyon başlıyor. Lütfen sensörü sabit tutun...");
  float sumAz = 0;
  for (int i = 0; i < 2000; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumAz += az / 16384.0;
    delay(1);
  }
  baseAccelerationZ = sumAz / 2000.0;
  Serial.println("Kalibrasyon tamamlandı.");
  Serial.print("Temel Z ivmesi: ");
  Serial.println(baseAccelerationZ);

  previousTime = millis();
  lastResetTime = previousTime;

  for (int i = 0; i < FILTER_WINDOW; i++) {
    velocityReadings[i] = 0;
  }

  // SD kart başlatma
  if (!SD.begin(chipSelect)) {
    Serial.println("SD kart başlatılamadı!");
    while (1);
  }
  Serial.println("SD kart başlatıldı.");
}

void loop() {
  butdurumu = digitalRead(ayrilma);
  if (butdurumu == LOW) {
    Serial.println("Güvenlik konektöründen ayrılma gerçekleşmedi.");
    digitalWrite(1, LOW);
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(ledPin, HIGH);
  } else {
    while (1) {
      if (!radio.available()) {
        char text[32] = { 0 };
        radio.read(&text, sizeof(text));
        Serial.println(text);
        sensors_event_t event;
        bno.getEvent(&event);

        Serial.print("X: ");
        Serial.print(event.orientation.x, 4);
        Serial.print("\tY: ");
        Serial.print(event.orientation.y, 4);
        Serial.print("\tZ: ");
        Serial.print(event.orientation.z, 4);
        
        displayCalStatus();

        // LPS25HB sensöründen yükseklik değerini oku
        float pressure = ps.readPressureMillibars();
        float altitude = ps.pressureToAltitudeMeters(pressure);
        Serial.print("\tAltitude: ");
        Serial.print(altitude);
        Serial.println(" m");

        if (event.orientation.z > 0 && event.orientation.y > 0 && altitude > 100) {
          digitalWrite(1, HIGH);
          digitalWrite(2, LOW);
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, HIGH);
        } else if (event.orientation.z < 0 && event.orientation.y > 0 && altitude > 100) {
          digitalWrite(1, LOW);
          digitalWrite(2, HIGH);
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, HIGH);
        } else if (event.orientation.z < 0 && event.orientation.y < 0 && altitude > 100) {
          digitalWrite(1, LOW);
          digitalWrite(2, LOW);
          digitalWrite(3, HIGH);
          digitalWrite(4, LOW);
          digitalWrite(5, HIGH);
        } else if (event.orientation.z > 0 && event.orientation.y < 0 && altitude > 100) {
          digitalWrite(1, LOW);
          digitalWrite(2, LOW);
          digitalWrite(3, LOW);
          digitalWrite(4, HIGH);
          digitalWrite(5, HIGH);
        } else if (event.orientation.z == 0 && event.orientation.y == 0 && altitude > 100) {
          digitalWrite(1, LOW);
          digitalWrite(2, LOW);
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, HIGH);
        }

        Serial.println("");

        // MPU6050 ivme verilerini oku ve işlem yap
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        float accelerationZ = az / 16384.0 - baseAccelerationZ;

        unsigned long currentTime = millis();
        float dt = (currentTime - previousTime) / 1000.0;

        if (fabs(accelerationZ) < ACCELERATION_DEADZONE) {
          accelerationZ = 0.0;
        }

        // Kalman filtresi uygulama
        float x_predict = x_est;
        float P_predict = P_est + Q;
        float K = P_predict / (P_predict + R);
        float x_update = x_predict + K * accelerationZ;
        float P_update = (1 - K) * P_predict;
        x_est = x_update;
        P_est = P_update;

        verticalVelocity += x_est * dt;

        previousTime = currentTime;

        velocityTotal = velocityTotal - velocityReadings[readIndex];
        velocityReadings[readIndex] = verticalVelocity;
        velocityTotal = velocityTotal + verticalVelocity;
        readIndex = (readIndex + 1) % FILTER_WINDOW;
        float filteredVelocity = velocityTotal / FILTER_WINDOW;

        // Hareket algılama ve hız sıfırlama
        if (fabs(filteredVelocity) < VELOCITY_THRESHOLD) {
          unsigned long currentResetTime = millis();
          if (currentResetTime - lastResetTime >= RESET_INTERVAL) {
            verticalVelocity = 0.0;
            lastResetTime = currentResetTime;
          }
        }

        // Yeni: Hareket algılama ve hareket durumu güncelleme
        if (fabs(accelerationZ) > ACCELERATION_DEADZONE) {
          motionCount++;
          if (motionCount >= MOTION_THRESHOLD_COUNT) {
            isMoving = true;
            motionCount = 0;
          }
        } else {
          motionCount = 0;
          isMoving = false;
        }

        // SD karta veri kaydetme
        File dataFile = SD.open("sensordata.txt", FILE_WRITE);
        if (dataFile) {
          dataFile.print("X: ");
          dataFile.print(event.orientation.x, 4);
          dataFile.print("\tY: ");
          dataFile.print(event.orientation.y, 4);
          dataFile.print("\tZ: ");
          dataFile.print(event.orientation.z, 4);
          dataFile.print("\tAltitude: ");
          dataFile.print(altitude);
          dataFile.print(" m\tVertical Velocity: ");
          dataFile.print(verticalVelocity, 4);
          dataFile.print(" m/s\tIs Moving: ");
          dataFile.println(isMoving);
          dataFile.close();
        } else {
          Serial.println("Veri dosyasını açarken hata oluştu!");
        }

        delay(BNO055_SAMPLERATE_DELAY_MS);
      }
    }
  }
}

void displaySensorDetails(void) {
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       "); Serial.println(sensor.name);
  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displaySensorStatus() {
  uint8_t system_status;     // Sistem durumu
  uint8_t self_test_result;  // Self test sonucu
  uint8_t system_error;      // Sistem hatası

  // getSystemStatus fonksiyonunu çağırın, ancak dönüş değeri yerine doğrudan argümanları kullanın
  bno.getSystemStatus(&system_status, &self_test_result, &system_error);

  Serial.print("System Status: ");
  Serial.println(system_status, HEX);
  Serial.print("Self Test: ");
  Serial.println(self_test_result, HEX);
  Serial.print("System Error: ");
  Serial.println(system_error, HEX);
}

void displayCalStatus() {
  uint8_t system;  // Sistem kalibrasyon durumu
  uint8_t gyro;    // Gyro kalibrasyon durumu
  uint8_t accel;   // Accelerometer kalibrasyon durumu
  uint8_t mag;     // Magnetometer kalibrasyon durumu

  // getCalibration fonksiyonuna dört argüman geçiriliyor
  bno.getCalibration(&system, &gyro, &accel, &mag);

  Serial.print("System Calibration: ");
  Serial.println(system);
  Serial.print("Gyro Calibration: ");
  Serial.println(gyro);
  Serial.print("Accelerometer Calibration: ");
  Serial.println(accel);
  Serial.print("Magnetometer Calibration: ");
  Serial.println(mag);
}
