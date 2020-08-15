#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU9250.h>
#include <LoRa.h>

//#include <SoftwareSerial.h>
#include <NeoSWSerial.h>
#include <TinyGPS.h>
#include <SdFat.h>

#define SEALEVELPRESSURE_HPA (1013.25)

// SD SELECT 6
// LORA SELECT 10

TinyGPS gps;
NeoSWSerial gpsSerial(4, 3);

float gps_latitude, gps_longtitude, gps_altitude, gps_course = 0.0;
unsigned long gps_age;

Adafruit_BME280 bme;

float bmeTemperature = 0;
float bmePressure = 0;
float bmeAltitude = 0;
float bmeHumidity = 0;

MPU9250 mpu = MPU9250();
float mpu_temperature = 0;

int gas_value = 0;

unsigned long millisLastRadioBroadcast = 0;

SdFat SD;
File myFile;

//#define sdCgipSelect 6
//#define LoRaChipSelect 10


void setup() {
	pinMode(5, OUTPUT);
	pinMode(10, OUTPUT);

	Serial.begin(9600);
	//while(!Serial);
	Serial.println("CANSAT#");

	bool bmeStatus;
	bmeStatus = bme.begin(0x76, &Wire);  
	if (!bmeStatus) {
		Serial.println("Could not find a valid BME280 at address 0x76!#");
	}
	setupMpuRanges();
	gpsSerial.begin(9600);

	openSD();
	if (!SD.begin(5)) {
		Serial.println("SD initialization failed!#");
	}

	myFile = SD.open("log.txt", FILE_WRITE);

	// if the file opened okay, write to it:
	if (myFile) {
		myFile.println("STARTING#");
		myFile.close();
	} else {
		Serial.println("error opening log.txt");
	}

	openLoRa();

	if (!LoRa.begin(433E6)) {
		Serial.println("Starting LoRa failed! #");
	}
}


void loop() { 
	readGps();
	
	readBme();
	
	readMpuAccel();
	readMpuGyro();
	readMpuTemperature();
	
	gas_value = analogRead(A1);

	//readMpuMagneto();
	Serial.print(bmeTemperature);
	Serial.print(" ");
	Serial.print(bmePressure);
	Serial.print(" ");
	Serial.print(bmeAltitude);
	Serial.print(" ");
	Serial.print(bmeHumidity);
	Serial.print(" ");
	Serial.print(mpu_temperature);
	Serial.print(" ");
	Serial.print(mpu.x_g, 5);
	Serial.print(" ");
	Serial.print(mpu.y_g, 5);
	Serial.print(" ");
	Serial.print(mpu.z_g, 5);
	Serial.print(" ");
	Serial.print(mpu.mx_t, 5);
	Serial.print(" ");
	Serial.print(mpu.my_t, 5);
	Serial.print(" ");
	Serial.print(mpu.mz_t, 5);
	Serial.print(" ");
	Serial.print(mpu.gx_d, 5);
	Serial.print(" ");
	Serial.print(mpu.gy_d, 5);
	Serial.print(" ");
	Serial.print(mpu.gz_d, 5);
	Serial.print(" ");
	Serial.print(gas_value);
	Serial.print(" ");
	Serial.print(gps_latitude);
	Serial.print(" ");
	Serial.print(gps_longtitude);
	Serial.print(" ");
	Serial.print(gps_altitude);
	Serial.print(" ");
	Serial.print(gps_course);
	Serial.print(" ");
	Serial.print("1.1");
	Serial.print(" ");
	Serial.print("-1");
	
	Serial.println("#");
	
	LoRa.beginPacket();
	LoRa.print(bmeTemperature);
	LoRa.print(" ");
	LoRa.print(bmePressure);
	LoRa.print(" ");
	LoRa.print(bmeAltitude);
	LoRa.print(" ");
	LoRa.print(bmeHumidity);
	LoRa.print(" ");
	LoRa.print(mpu_temperature);
	LoRa.print(" ");
	LoRa.print(mpu.x_g, 5);
	LoRa.print(" ");
	LoRa.print(mpu.y_g, 5);
	LoRa.print(" ");
	LoRa.print(mpu.z_g, 5);
	LoRa.print(" ");
	LoRa.print(mpu.mx_t, 5);
	LoRa.print(" ");
	LoRa.print(mpu.my_t, 5);
	LoRa.print(" ");
	LoRa.print(mpu.mz_t, 5);
	LoRa.print(" ");
	LoRa.print(mpu.gx_d, 5);
	LoRa.print(" ");
	LoRa.print(mpu.gy_d, 5);
	LoRa.print(" ");
	LoRa.print(mpu.gz_d, 5);
	LoRa.print(" ");
	LoRa.print(gas_value);
	LoRa.print(" ");
	LoRa.print(gps_latitude);
	LoRa.print(" ");
	LoRa.print(gps_longtitude);
	LoRa.print(" ");
	LoRa.print(gps_altitude);
	LoRa.print(" ");
	LoRa.print(gps_course);
	LoRa.print(" ");
	LoRa.print("1.1");
	LoRa.print(" ");
	LoRa.print("-1#");
	LoRa.endPacket();

	openSD();
	if (!SD.begin(5)) {
		Serial.println("SD initialization failed!#");
	}
	myFile = SD.open("log.txt", FILE_WRITE);
	if (myFile) {
		myFile.print(bmeTemperature);
		myFile.print(" ");
		myFile.print(bmePressure);
		myFile.print(" ");
		myFile.print(bmeAltitude);
		myFile.print(" ");
		myFile.print(bmeHumidity);
		myFile.print(" ");
		myFile.print(mpu_temperature);
		myFile.print(" ");
		myFile.print(mpu.x_g, 5);
		myFile.print(" ");
		myFile.print(mpu.y_g, 5);
		myFile.print(" ");
		myFile.print(mpu.z_g, 5);
		myFile.print(" ");
		myFile.print(mpu.mx_t, 0);
		myFile.print(" ");
		myFile.print(mpu.my_t, 0);
		myFile.print(" ");
		myFile.print(mpu.mz_t, 0);
		myFile.print(" ");
		myFile.print(mpu.gx_d, 5);
		myFile.print(" ");
		myFile.print(mpu.gy_d, 5);
		myFile.print(" ");
		myFile.print(mpu.gz_d, 5);
		myFile.print(" ");
		myFile.print(gas_value);
		myFile.print(" ");
		myFile.print(gps_latitude, 5);
		myFile.print(" ");
		myFile.print(gps_longtitude, 5);
		myFile.print(" ");
		myFile.print(gps_altitude, 5);
		myFile.print(" ");
		myFile.print(gps_course, 5);
		myFile.print(" ");
		myFile.print("1.1");
		myFile.print(" ");
		myFile.print("-1");

		myFile.println("#");

		myFile.close();
	} else {
		Serial.println("error opening log.txt");
	}

	openLoRa();

}

void openLoRa(){
	digitalWrite(5, HIGH);
	digitalWrite(10, LOW);
}

void openSD(){
	digitalWrite(10, HIGH);
	digitalWrite(5, LOW);
}

void readGps(){
	int availableSerialChars = gpsSerial.available();
	if (availableSerialChars) {
		for (int i=0; i < availableSerialChars; i++){
			char c = gpsSerial.read();
			//Serial.write(c);
			if (gps.encode(c)){
				gps.f_get_position(&gps_latitude, &gps_longtitude, &gps_age);
				gps_altitude = gps.f_altitude();
				gps_course = gps.course();
			}
		}
	}
}

void readBme() {
	bmeTemperature = bme.readTemperature(); // Celsius
	bmePressure = bme.readPressure(); // hPa
	bmeAltitude = bme.readAltitude(SEALEVELPRESSURE_HPA); // meters
	bmeHumidity = bme.readHumidity(); // %
}

void setupMpuRanges(){
	mpu.set_accel_range(RANGE_4G);
	mpu.set_gyro_range(RANGE_GYRO_250);
	mpu.set_mag_scale(SCALE_14_BITS);
	mpu.set_mag_speed(MAG_8_Hz);
}

void readMpuAccel(){
	// Accel
	//mpu.get_accel();
	mpu.get_accel_g();
	// values: mpu.x mpu.y mpu.z
	// in G: mpu.x_g mpu.y_g mpu.z_g
}

void readMpuGyro(){
	//mpu.get_gyro();
	mpu.get_gyro_d();
	// values: mpu.gx mpu.gy mpu.gz
	// in deg/s: mpu.gx_d mpu.gy_d mpu.gz_d
}

void readMpuMagneto(){
	// Magnetometer
	mpu.get_mag();
	mpu.get_mag_t();
	// values: mpu.mx mpu.my mpu.mz
	// in uT: mpu.mx_t mpu.my_t mpu.mz_t
}

void readMpuTemperature(){
	mpu_temperature = (((float) mpu.get_temp()) / 333.87 + 21.0);
}