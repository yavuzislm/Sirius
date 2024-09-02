#include <SPI.h> 
#include "nRF24L01.h"
#include "RF24.h"     // Modül ile ilgili kütüphaneleri ekliyoruz

int mesaj[1];            // mesaj isminde bir dizi tanımlıyoruz
RF24 verici(9, 10);      // Kütüphane tarafından kullanılacak pinleri tanımlıyoruz
const uint64_t kanal = 0xE8E8F0F0E1LL;  // Kanalı tanımlıyoruz  
int buton = 7;           // Butonun bağlı olduğu dijital pin

void setup(void)
{
  Serial.begin(9600);
  verici.begin();                  // NRF24L01 modülünü başlatıyoruz
  verici.openWritingPipe(kanal);   // Kanal ID'si tanımlanıyor
  verici.setPALevel(RF24_PA_MIN);  // Düşük güç seviyesinde çalıştırıyoruz
  pinMode(buton, INPUT);           // Buton pinini giriş olarak ayarlıyoruz
}

void loop(void)
{
  if (digitalRead(buton) == HIGH)
  { 
    Serial.println("1");  // Butonun basıldığı bilgisini seri porttan gönderiyoruz
    mesaj[0] = 123;       // Mesajı hazırlıyoruz
    verici.write(mesaj, sizeof(mesaj));  // Mesajı gönderiyoruz
  }
  else
  {
    Serial.println("0");
  }
}