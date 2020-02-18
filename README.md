# GPSnode + TTNmapper

### Testované GPS moduly
- NEO-6M - [obchod](https://arduino-shop.cz/arduino/1510-gps-neo-6m-gyneo6mv2-modul-s-antenou.html?gclid=EAIaIQobChMIl8ecrNm54wIVF-d3Ch2MSg4FEAQYASABEgKXpPD_BwE) | [návod](https://navody.arduino-shop.cz/navody-k-produktum/arduino-gps-modul-neo-6m.html)


Postup: Moteino Mega R4

1. Propoj GPS modul s vývojovou deskou (Piny 10 a 11)

2. Stáhni knihovnu lmic, otevři vzorový sketch, vlož klíče pro komunikaci s NS a namapuj piny pro Moteino Mega R4
   
   ```c
   // Pin mapping
   const lmic_pinmap lmic_pins = {
     .nss = 4,
     .rxtx = LMIC_UNUSED_PIN,
     .rst = 13,
     .dio = {2, 1, 0},
   };
   ```
   
3. Převeď užitečné zatížení GPS informací na pole bytů
   
   ```c
   // Užitečné zatížení ponese 9bytů. 3byte pro latitude, 3byte pro longitude, 2byte pro altitude, 1byte pro hdop
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
   
   ```js
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
