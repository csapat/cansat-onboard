#include <Wire.h>
#include <SPI.h>
#include <NeoSWSerial.h>

#include <TinyGPS.h>
#include <LoRa.h>

// PINs
#define PIN_GPS_RX 4
#define PIN_GPS_TX 3
#define PIN_DAT_RX 5
#define PIN_DAT_TX 6

TinyGPS gps;
NeoSWSerial gpsSerial(PIN_GPS_RX, PIN_GPS_TX);
NeoSWSerial datSerial(PIN_DAT_RX, PIN_DAT_TX);

float lat, lon, alt = 0.0, course = 0.0;
unsigned long age;

void setup() {
	Serial.begin(9600);
	gpsSerial.begin(9600);
	datSerial.begin(9600);
	if (!LoRa.begin(433E6)) {
		Serial.println("Starting LoRa failed!");
	}
}

char dataStream[] = "";
unsigned long lastRadioBroadcast = 0;
unsigned int loopsSinceLastBroadcast = 0;
void loop() {
	gpsSerial.listen();
	int availableSerialChars = gpsSerial.available();
	while ((availableSerialChars--) > 0) {
		char c = gpsSerial.read();
		if (gps.encode(c)){
			gps.f_get_position(&lat, &lon, &age);
			//alt = gps.f_altitude();
			course = gps.course();
		}
	}
	
	datSerial.listen();
	availableSerialChars = datSerial.available();
	while ((availableSerialChars--) > 0) {
		char c = datSerial.read();
		if (c == '#') {
			unsigned long millisNow = millis();
			
			LoRa.beginPacket();
			LoRa.print(course, 5);
			LoRa.print(" ");
			LoRa.print(lat, 10);
			LoRa.print(" ");
			LoRa.print(lon, 10);
			LoRa.print(" ");
			
			/*
			alt
			dht_temp
			bmp_temp
			mpu_temp
			x_g
			y_g
			z_g
			mx
			my
			mz
			gx
			gy
			gz
			roll
			pitch
			yaw
			pressure
			humidity
			uv
			*/
			LoRa.print(dataStream);
			
			LoRa.print(" ");
			LoRa.print(1000*loopsSinceLastBroadcast/(millisNow-lastRadioBroadcast));
			LoRa.print("#");
			LoRa.endPacket();
			lastRadioBroadcast = millisNow;
			loopsSinceLastBroadcast = 0;
			dataStream = "";
		}else{
			dataStream += c;
			loopsSinceLastBroadcast++;
		}
	}
}
