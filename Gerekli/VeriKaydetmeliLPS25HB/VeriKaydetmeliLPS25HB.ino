#include <Wire.h>
#include <LPS.h>
#include <SD.h>
#include <SPI.h>

LPS ps;
const int chipSelect = BUILTIN_SDCARD;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Basınç sensörünü başlat
  if (!ps.init()) {
    Serial.println("Failed to autodetect pressure sensor!");
    while (1);
  }
  ps.enableDefault();

  // SD kartı başlat
  if (!SD.begin(chipSelect)) {
    Serial.println("SD kart başlatılamadı!");
    while (1);
  }
  Serial.println("SD kart başlatıldı.");
}

void loop() {
  // Basınç, irtifa ve sıcaklık verilerini oku
  float pressure = ps.readPressureMillibars();
  float altitude = ps.pressureToAltitudeMeters(pressure);
  float temperature = ps.readTemperatureC();

  // Verileri seri porta yazdır
  Serial.print("p: ");
  Serial.print(pressure);
  Serial.print(" mbar\ta: ");
  Serial.print(altitude);
  Serial.print(" m\tt: ");
  Serial.print(temperature);
  Serial.println(" deg C");

  // Verileri SD karta kaydet
  File dataFile = SD.open("data.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print("p: ");
    dataFile.print(pressure);
    dataFile.print(" mbar\ta: ");
    dataFile.print(altitude);
    dataFile.print(" m\tt: ");
    dataFile.print(temperature);
    dataFile.println(" deg C");
    dataFile.close();
  } else {
    Serial.println("Veri dosyası açma hatası!");
  }

  delay(100);
}
