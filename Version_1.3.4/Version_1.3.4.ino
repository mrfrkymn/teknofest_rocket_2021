//Pin Atamaları   3(gps TX) 4(gps RX) 5(Kurtarma-1) 6(Kurtarma-2) 7(Kurtarma-3) 9(lora RX)  11(lora TX)
//String myLocation = "https://www.google.com/maps/@" + String(gps.location.lat()) + String(",@") + String(gps.location.lng()) + String(",16z");  
//******************************************  Kütüphaneler  ************************************************************************

#include <SoftwareSerial.h>                                                     // Yazılımsal seri iletişim kütüphanesi
#include <SFE_BMP180.h>                                                         // Barometre kütüphanesi
#include <TinyGPS++.h>                                                          // GPS kütüphanesi
#include "LoRa_E32.h"                                                           // Kablosuz iletişim modülü kütüphanesi

//******************************************  Sensör,Modül,İletişim  ***************************************************************

SoftwareSerial lora_ss(9,11);//RX,TX                                            // Lora seri iletişim ağı 
SoftwareSerial gps_ss(4,3);  //RX,TX                                            // GPS seri iletişim ağı
LoRa_E32 lora(&lora_ss);                                                        // Lora nesnesi oluşturuldu
TinyGPSPlus gps;                                                                // GPS nesnesi oluşturuldu
SFE_BMP180 bmp;                                                                 // Barometre nesnesi oluşturuldu

//******************************************  Global Değişkenler  ******************************************************************

int flight_step = 0;                                                            // Uçuşun hangi aşamada olduğunu belirlediğimiz değişken
int mainRecoveryPin = 5;                                                        // Röleye bağlı olan, roket ana kurtarma sistemini ateşleyecek pin
const double baseline = 1013.25;                                                // Referans zemin basınç değeri(deniz seviyesi = 1013.25)
struct Signal {                                                                 // Kablosuz iletişim ile gönderilecek structure yapısı
  float dLon;                                                                   // Boylam değeri
  float dLat;                                                                   // Enlem değeri
  float dAlt;                                                                   // İrtifa değeri
  float dV;                                                                     // Hız değeri
  int  dRS = 0;                                                                 // Kurtarma sisteminin açılıp açılmadığını tutan değişken
} data;                                                                         // Signal yapısını içeren değişkenimiz

//******************************************  Setup Bölümü  ************************************************************************

void setup() {
  pinMode(mainRecoveryPin,OUTPUT);                                              // Pin çıkış olarak tanımlanır
  Serial.begin(115200);                                                         // Seri iletişim başlatıldı
  gps_ss.begin(9600);                                                           // GPS ile seri iletişim başlatıldı
  gps_ss.listen();//eğer gps çalışmazsa loop içine koy                          // GPS seri ağı dinleniyor
  lora.begin();                                                                 // Lora ile seri iletişim başlatıldı
  if (bmp.begin()){Serial.println("BMP180 Bağlanıldı");}                        // BMP ile I2C iletişim başaltıldı
  else            {Serial.println("BMP180 Bağlanılamadı\n\n");while(1);}        // BMP ile iletişim başarısız mesajı
}

//******************************************  Loop Bölümü  *************************************************************************

void loop() {
  unsigned long startTime;                                                      // Her döngü için başlangıç zamanını tutacak değişken
  float startAlt;                                                               // Her döngü için başlangıç yüksekliğini tutacak değişken
  switch(flight_step){                                                          // Uçuş süreci yönetimini gerçekleştirecek yapı
    
    case 0://*******************************************************************// Kalkış öncesi aşaması (Yaklaşık 2 saniyede bir yenileme)
      startTime = millis();                                                     // Süreç başlangıç zamanı
      startAlt = altData();                                                     // Süreç başlangıç yüksekliğ
      data.dAlt = startAlt                                                      // Veri, paket içine aktarılıyor
      if(data.dAlt > 50){flight_step = 1;break;}                                // Yükseklik 50 metreyi geçtiğinde birinci aşama başlıyor
      data.dLon = gps.location.lng();                                           // GPSden boylam değeri alınır ve veri paketine yerleştirilir
      data.dLat = gps.location.lat();                                           // GPSden enlem değeri alınır ve veri paketine yerleştirilir
      delay(500);
      data.dAlt = altData();                                                    // Yükseklik verisi alınır ve veri paketine yerleştirilir
      float velocity = (data.dAlt-startAlt) / ( (millis()-startTime) / 1000 );  // Yükseklik değişimi bölü zaman değişiminden hız bulunur.
      if(velocity < 1){velocity = 0;}                                           // Hata payı düşünülerek hız saniyede 1 metreden az ise hız sıfır yapılır
      data.dV   = velocity;                                                     // Hız verisi veri paketine yerleştirilir
      lora.sendFixedMessage(0, 2, 23, &data , sizeof(Signal) );                 // Veri paketi yer istasyonuna gönderilir
      Serial.println("Aşama sıfır döngü sonu");                                 // Döngü kontrolü sırasında kontrol ifadesi
      Serial.print("Döngü süresi: ");Serial.print(millis()-startTime);          // Döngü yenileme süresini veren ifade
      Serial.println("ms");Serial.println("**********************");            // Seri çıkış ifadeleri arası çizgiler
      break;
    case 1://*******************************************************************// Kalkış sonrası aşaması
      startTime = millis();                                                     // Süreç başlangıç zamanı
      startAlt = altData();                                                     // Süreç başlangıç yüksekliği
      data.dLon = gps.location.lng();                                           // GPSden boylam değeri alınır ve veri paketine yerleştirilir
      data.dLat = gps.location.lat();                                           // GPSden enlem değeri alınır ve veri paketine yerleştirilir
      delay(500);                                                               // Yer değişimi miktarının artması için ölçüm aralığına biraz bekleme ekliyoruz
      data.dAlt = altData();                                                    // Yükseklik verisi alınır ve veri paketine yerleştirilir
      data.dV   = (data.dAlt-startAlt) / ( (millis()-startTime) / 1000 );       // Yükseklik değişimi bölü zaman değişiminden hız bulunur.
      if(data.dAlt > 600 && data.dV < 20 ){flight_step = 2;}                    // Yükseklik 2km'yi aşmış ve hız 20 m/s'den yavaşsa uçuş aşama 2'ye geçilir.
      lora.sendFixedMessage(0, 2, 23, &data , sizeof(Signal) );                 // Veri paketi yer istasyonuna gönderilir
      break;
    case 2://*******************************************************************// Apogee noktasına yaklaşma
      startTime = millis();                                                     // Süreç başlangıç zamanı
      startAlt = altData();                                                     // Süreç başlangıç yüksekliği
      delay(100);                                                               // İki ölçüm arası kısa bir bekleme eklenir
      data.dV = (data.dAlt-startAlt) / ( (millis()-startTime) / 1000 );         // Kurtarma için ana veri olan hız hesaplanır
      if(data.dV < 5){flight_step = 3;}                                         // Hız saniyede 5 metrenin altına inmişse uçuş aşama 3'e geçilir.
      break;
    case 3://*******************************************************************// Apogee / yük kurtarma aşaması
      delay(2000);                                                              // Eğim sensörü çalıştırılamazsa tek yol oalrak kısa bir gecikme kullanabiliriz :(
      digitalWrite(mainRecoveryPin,HIGH);                                       // Paket kurtarma paraşütü rölesine güç verilir
      delay(1000);digitalWrite(mainRecoveryPin,LOW);                            // Yeterli güç verildikten sonra güç kesilir
      data.dRS = 1;                                                             // Faydalı yük kurtarma onayı
      lora.sendFixedMessage(0, 2, 23, &data , sizeof(Signal) );                 // Veri paketi yer istasyonuna gönderilir
      flight_step = 4;                                                          // Uçuş aşama 4'e geçilir
      break;
    case 4://*******************************************************************// Serbest düşüş / ana kurtarma aşaması   !!! İniş hızı kontrol et !!
      startTime = millis();                                                     // Süreç başlangıç zamanı
      startAlt = altData();                                                     // Süreç başlangıç yüksekliği
      data.dLon = gps.location.lng();                                           // GPSden boylam değeri alınır ve veri paketine yerleştirilir
      data.dLat = gps.location.lat();                                           // GPSden enlem değeri alınır ve veri paketine yerleştirilir
      data.dAlt = altData();                                                    // Yükseklik verisi alınır ve veri paketine yerleştirilir
      data.dV   = (data.dAlt-startAlt) / ( (millis()-startTime) / 1000 );       // Hız verisi veri paketine yerleştirilir
      if(data.dV < 3 && data.dV > -3 ){flight_step = 5;}                        // Hız 3 m/s'den yavaşsa uçuş aşama 5'e geçilir(yerde)
      lora.sendFixedMessage(0, 2, 23, &data , sizeof(Signal) );                 // Veri paketi yer istasyonuna gönderilir
      delay(2000);                                                              // Anlık veri önemi düştü, işlem sıklığı azaltılıyor.
      break;
    case 5://*******************************************************************// İniş / bekleme aşaması
      data.dLon = gps.location.lng();                                           // GPSden boylam değeri alınır ve veri paketine yerleştirilir
      data.dLat = gps.location.lat();                                           // GPSden enlem değeri alınır ve veri paketine yerleştirilir
      data.dV   = 0;                                                            // Hareket yok, hız sıfır
      data.dAlt = altData();                                                    // Yükseklik verisi alınır ve veri paketine yerleştirilir
      lora.sendFixedMessage(0, 2, 23, &data , sizeof(Signal) );                 // Veri paketi yer istasyonuna gönderilir
      delay(10000);                                                             // Gelen veriler birbirinin aynısı oalcaktır, işlem sıklığı daha da azaltıldı.
      break;
  }//***************************************************************************// Switch sonu, döngü tekrarına döner.

}

//******************************************  Fonksiyonlar  *****************************************************************************************************************************

float altData(){                                            // Barometre verilerine medyan filtresi uygulayarak veri dönen fonksiyon (200 ms)
  float dataList[5];                                        // Filtreleme için kullanıalcak verilerin listesi
  for(int i = 0 ; i < 5 ; i++ ){
    dataList[i] = bmp.altitude(getPressure(),baseline);
    delay(9);
  }
  int sz = 5; //sizeof(dataList)/sizeof(float);
  float biggest = 0;
  int loc = 0;

  for(int i = sz-1 ; i >= 0 ; i-- ){         //_______________________//
    for(int j = 0 ; j <= i ; j++ ){          //         _             //
      if(dataList[j] > biggest){             //     •__(.)<  (QUACK)  //
        biggest = dataList[j];               //      \___)            //
        loc = j;                             //_______________________//
      }
    }
    if( biggest != dataList[i] ){
    float temp = dataList[i];
    dataList[i] = biggest;
    dataList[loc] = temp;
    }
    biggest = 0;    
    loc = 0;
  }
  return dataList[2];
}
//******************************************O******************************************

double getPressure(){                                       // Barometreden basınç verisi almamızı sağlayan fonksiyon
char status;
double T,P,p0,a;
status = bmp.startTemperature();
if (status != 0) {
  delay(status); // Ölçüm tamamlanması için bekle
  status = bmp.getTemperature(T);
  if (status != 0){
    status = bmp.startPressure(3);//Basınç ölçümünü başlat
    if (status != 0){
      delay(status);// Ölçümünü tamamlanmasını bekle
      status = bmp.getPressure(P,T); //Tamamlanan basınç ölçümü Al :
      if (status != 0) //sıfıra eşit değilse{
        return(P); // Ölçüm birimi P saklanır
    }
    else Serial.println("Basınç ölçümünde hata alındı\n");
  }

  else Serial.println("Basınç Ölçümü başlatılamadı\n");
}
else Serial.println("Sıcaklık değeri alınamadı\n");
}

//***************************************************************************************************************************************************************************************
