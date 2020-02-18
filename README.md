# GPSnode + TTNmapper

### Testované GPS moduly
- NEO-6M - [obchod](https://arduino-shop.cz/arduino/1510-gps-neo-6m-gyneo6mv2-modul-s-antenou.html?gclid=EAIaIQobChMIl8ecrNm54wIVF-d3Ch2MSg4FEAQYASABEgKXpPD_BwE) | [návod](https://navody.arduino-shop.cz/navody-k-produktum/arduino-gps-modul-neo-6m.html)


Postup: Moteino Mega R4

1. Propoj GPS modul s vývojovou deskou

   <table style="width: 100%;">
   <tbody>
      <tr>
      <td style="font-size: 15px; padding: 10px;"><b>NEO modul</b></td>
      <td style="font-size: 15px; padding: 10px;"><b>-></b></td>
      <td style="font-size: 15px; padding: 10px;"><b>Arduino UNO</b></td>
      </tr>
      <tr>
         <td>VCC</td>
         <td>-></td>
         <td>3V3 / 5V</td>
      </tr>
      <tr>
         <td>GND</td>
         <td>-></td>
         <td>GND</td>
      </tr>
      <tr>
         <td>RX</td>
         <td>-></td>
         <td>3</td>
      </tr>
      <tr>
         <td>TX</td>
         <td>-></td>
         <td>2</td>
      </tr>
   </tbody>
   </table>


2. Stáhni knihovnu lmic, otevři vzorový sketch, vlož klíče pro komunikaci s NS a namapuj piny pro Moteino Mega R4
   
   ```
   // Pin mapping
   const lmic_pinmap lmic_pins = {
     .nss = 4,
     .rxtx = LMIC_UNUSED_PIN,
     .rst = 13,
     .dio = {2, 1, 0},
   };
   ```
   
3. Převeď užitečné zatížení GPS informací na pole bytů
   
   ```
   uint8_t txBuffer[9];

   uint32_t latitude = ([Tady bude metoda, která naplní latitude skutečnou hodnotou]) * 10000;
   uint32_t longitude = ([Tady bude metoda, která naplní longitude skutečnou hodnotou])* 10000;
   uint16_t altitude = ([Tady bude metoda, která naplní altitude skutečnou hodnotou]) * 10;
   uint8_t hdop = ([Tady bude metoda, která naplní hdop skutečnou hodnotou]) * 10;     

   txBuffer[0] = latitude >> 16;
   txBuffer[1] = latitude >> 8;
   txBuffer[2] = latitude;

   txBuffer[3] = longitude >> 16;
   txBuffer[4] = longitude >> 8;
   txBuffer[5] = longitude;

   txBuffer[6] = altitude >> 8;
   txBuffer[7] = altitude;

   txBuffer[8] = hdop;

   LMIC_setTxData2(1,txBuffer, sizeof(txBuffer), 0);  
   ```
   
4. V konzoli TTN vlož do "Payload Formats" vlastní dekodér
   
   ```
   function Decoder(bytes, port) {
     // Decode an uplink message from a buffer
     // (array) of bytes to an object of fields.
     var decoded = {};

     decoded.lat = (bytes[0]<<16) + (bytes[1]<<8)+ bytes[2];
     decoded.lat = decoded.lat / 10000.0 ;

     decoded.lon = (bytes[3]<<16) + (bytes[4]<<8) + bytes[5];
     decoded.lon = decoded.lon / 10000.0;

     decoded.alt = (bytes[6]<<8) + bytes[7];
     decoded.alt = decoded.alt / 10.0;

     decoded.hdop = bytes[8] / 10.0;

     return decoded;
   }
   ```
   
5. V konzoli TTN nastav v "Integrations" integraci s TTN Mapperem
6. Sleduj výsledky https://ttnmapper.org/advanced-maps/





## Coder

### Decoder



### Zapojení

<table style="width: 100%;">
<tbody>
   <tr>
   <td style="font-size: 15px; padding: 10px;"><b>NEO modul</b></td>
   <td style="font-size: 15px; padding: 10px;"><b>-></b></td>
   <td style="font-size: 15px; padding: 10px;"><b>Arduino UNO</b></td>
   </tr>
   <tr>
      <td>VCC</td>
      <td>-></td>
      <td>3V3 / 5V</td>
   </tr>
   <tr>
      <td>GND</td>
      <td>-></td>
      <td>GND</td>
   </tr>
   <tr>
      <td>RX</td>
      <td>-></td>
      <td>3</td>
   </tr>
   <tr>
      <td>TX</td>
      <td>-></td>
      <td>2</td>
   </tr>
</tbody>
</table>

### Arduino kód (bez použití knihovny)

Výstupem tohoto kódu jsou NMEA (National Marine Electronics Association) zprávy (surová data je nutné zpracovat).

```c
#include <SoftwareSerial.h>

// Choose two Arduino pins to use for software serial
int RXPin = 2;
int TXPin = 3;

//Default baud of NEO-6M is 9600
int GPSBaud = 9600;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(RXPin, TXPin);

void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);

  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBaud);
}

void loop()
{
  // Displays information when new sentence is available.
  while (gpsSerial.available() > 0)
    Serial.write(gpsSerial.read());
}
```
```
Výstup:
$GPGGA,143241.00,4940.07303,N,01820.56767,E,1,07,1.46,292.8,M,41.1,M,,*5C
$GPGSA,A,3,10,08,14,32,27,20,18,,,,,,3.40,1.46,3.07*01
$GPGSV,3,1,12,01,,,32,08,71,248,42,10,49,059,37,11,,,32*7B
$GPGSV,3,2,12,14,26,143,39,18,52,287,38,20,22,059,32,21,01,099,*73
$GPGSV,3,3,12,22,19,230,25,24,02,044,,27,49,166,39,32,39,123,39*72
$GPGLL,4940.07303,N,01820.56767,E,143241.00,A,A*68
$GPRMC,143242.00,A,4940.07302,N,01820.56778,E,0.082,,150719,,,A*7C
$GPVTG,,T,,M,0.082,N,0.151,K,A*2C
```

### Arduino kód (s použitím knihovny)

Výstupem tohoto kódu jsou zprávy NMEA formátovány do čitelné podoby. K tomu slouží knihovny, pro tyto potřeby určené.

#### Knihovna

Knihovna: např. [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus) 

Metody knihovny: 

```c
gps.location.lat() // Latitude in degrees (double)
gps.location.lng() // Longitude in degrees (double)
gps.location.rawLat().negative ? "-" : "+"
gps.location.rawLat().deg // Raw latitude in whole degrees
gps.location.rawLat().billionths// ... and billionths (u16/u32)
gps.location.rawLng().negative ? "-" : "+");
gps.location.rawLng().deg // Raw longitude in whole degrees
gps.location.rawLng().billionths// ... and billionths (u16/u32)
gps.date.value() // Raw date in DDMMYY format (u32)
gps.date.year() // Year (2000+) (u16)
gps.date.month() // Month (1-12) (u8)
gps.date.day() // Day (1-31) (u8)
gps.time.value() // Raw time in HHMMSSCC format (u32)
gps.time.hour() // Hour (0-23) (u8)
gps.time.minute() // Minute (0-59) (u8)
gps.time.second() // Second (0-59) (u8)
gps.time.centisecond() // 100ths of a second (0-99) (u8)
gps.speed.value() // Raw speed in 100ths of a knot (i32)
gps.speed.knots() // Speed in knots (double)
gps.speed.mph() // Speed in miles per hour (double)
gps.speed.mps() // Speed in meters per second (double)
gps.speed.kmph() // Speed in kilometers per hour (double)
gps.course.value() // Raw course in 100ths of a degree (i32)
gps.course.deg() // Course in degrees (double)
gps.altitude.value() // Raw altitude in centimeters (i32)
gps.altitude.meters() // Altitude in meters (double)
gps.altitude.miles() // Altitude in miles (double)
gps.altitude.kilometers() // Altitude in kilometers (double)
gps.altitude.feet() // Altitude in feet (double)
gps.satellites.value() // Number of satellites in use (u32)
gps.hdop.value() // Horizontal Dim. of Precision (100ths-i32)
```

použití knihovny:

```c
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Choose two Arduino pins to use for software serial
int RXPin = 2;
int TXPin = 3;

int GPSBaud = 9600;

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(RXPin, TXPin);

void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);

  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBaud);
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read()))
      displayInfo();

  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected");
    while(true);
  }
}

void displayInfo()
{
  if (gps.location.isValid())
  {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());
  }
  else
  {
    Serial.println("Location: Not Available");
  }
  
  Serial.print("Date: ");
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.println(gps.date.year());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.print("Time: ");
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(".");
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.println(gps.time.centisecond());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.println();
  Serial.println();
  delay(1000);
}
```
```
Výstup:
Latitude: 49.667858
Longitude: 18.343093
Altitude: 0.00
Satellites: 4
Speed: 0
Date: 7/15/2019
Time: 14:42:59.00
```

### Arduino kód (odeslaní dat do TTN ve formátu Cayenne LPP)

```c
float latitude = 0;
float longitude = 0;
float altitude = 0;
float satelittes = 0;
float speed = 0;
float timestamp = 0;

while (gpsSerial.available() > 0){
   if (gps.encode(gpsSerial.read())){

      if (gps.location.isValid()){

         latitude = gps.location.lat();
         longitude = gps.location.lng();
         altitude = gps.altitude.meters();
         altitude = gps.satellites.value();
         altitude = gps.altitude.meters();
       }
      
      if (gps.date.isValid() && gps.time.isValid()){

         uint32_t timestamp = lpp.convertToEpochtime(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());
       }
     }
   }
      
lpp.reset();
lpp.addGPSplus(1, latitude, longitude, altitude, satelittes, speed, timestamp);    
                                                   
LMIC_setTxData2(1,lpp.getBuffer(), lpp.getSize(), 0);  

Serial.println(F("Packet queued"));
```

Výstup
![alt text](https://github.com/davidvasicek/GPSModules/blob/master/ttn_output.png "Logo Title Text 1")

celý kód k dispozici zde: [ABP_TTN_GPS_MOTEINO.ino](https://github.com/davidvasicek/GPSModules/blob/master/ABP_TTN_GPS_MOTEINO.ino).

Upozornění: 
- V případě použití kombinací knihoven TinyGPS++ a LMIC je potřeba použít microprocesor disponující větší programovatelnou pamětí než 32KB (Samotný LMIC zabere cca 22KB)
- Při prvním spuštění GPS je nutné GPS modul umístit do otevřeného prostoru s přímým výhledem na oblohu. Zvlášť, když byl modul déle než 14dní bez napájení a baterie udržující paměť BBR (Battery Backed RAM) je vybitá. Pak je GPS tzv. "studený" a trvá mu mnohem déle zafixovat polohu. (při měření jsem prvních výsledků docílil až po 6-10 minutách). GPS modul je také osazen indikační LED diodou (Svítí konstantně - hledá satelity, Bliká (1Hz) - zafixovaná pozice)
