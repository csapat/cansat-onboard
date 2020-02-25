#include <MPU9250.h> // MPU9250
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h> // Universal Adafruit Library
#include <Adafruit_BMP280.h> // BMP280
#include <dht.h> // DHT Humidity
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SPI.h>
#include <LoRa.h>
#include <MadgwickAHRS.h>
#include <PID_v1.h>

// PINs
#define PIN_DHT_HUMIDITY 7
#define PIN_GPS_RX 4
#define PIN_GPS_TX 3
#define PIN_UV_1 A0

dht DHT;
MPU9250 mpu = MPU9250();
Adafruit_BMP280 bmp;
TinyGPS gps;
SoftwareSerial gpsSerial(PIN_GPS_RX, PIN_GPS_TX);

float lat, lon, alt = 0.0;
unsigned long age;
float dht_temperature = 0;
float dht_humidity = 0;
float mpu_temperature = 0;
float bmp_temperature = 0;
float bmp_pressure = 0;
float bmp_altitude = 0;
float uv_1 = 0;

Madgwick madgwickFilter;
unsigned long microsPerReading, microsPrevious;

float roll, pitch, yaw;
double Kp=1, Ki=1, Kd=0.07;

double rollSetpoint, rollInput, rollOutput;
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, Kp, Ki, Kd, DIRECT);

int servoOutput;
int sampleRate = 2;
void setup() {
	Serial.begin(9600);
	uint8_t temp = mpu.begin();
	if (!bmp.begin()) Serial.println("Could not find a valid BMP280 sensor");
	gpsSerial.begin(9600);
	if (!LoRa.begin(433E6)) {
		Serial.println("Starting LoRa failed!");
	}
	mpu.set_accel_range(RANGE_4G);
	mpu.set_gyro_range(RANGE_GYRO_250);
	mpu.set_mag_scale(SCALE_14_BITS);
	mpu.set_mag_speed(MAG_8_Hz);
	
	rollInput = roll;
	rollSetpoint = 0;
	rollPID.SetOutputLimits(-90, 90);
	rollPID.SetMode(AUTOMATIC);
	
	madgwickFilter.begin(sampleRate);
	microsPerReading = 1000000 / sampleRate;
	microsPrevious = micros();
	rollPID.SetTunings(Kp, Ki, Kd);
}

void loop() {

	// Read DHT Humidity & Temperature

	//int chk = DHT.read11(PIN_DHT_HUMIDITY);
	//temp_dht_temperature = DHT.temperature;
	//temp_dht_humidity = DHT.humidity;
	//if (temp_dht_temperature!=-999.00) dht_temperature = temp_dht_temperature;
	//if (temp_dht_humidity!=-999.00) dht_humidity = temp_dht_humidity;

	// values: float dht_humidity, dht_temperature
	// END DHT

	// Read MPU9250 Gyro, Accelrometer, Magnetometer

	// Accel
	
	//mpu.get_accel();
	//mpu.get_accel_g();
	// values: mpu.x mpu.y mpu.z
	// in G: mpu.x_g mpu.y_g mpu.z_g

	// Gyro
	
	//mpu.get_gyro();
	//mpu.get_gyro_d();
	// values: mpu.gx mpu.gy mpu.gz
	// in deg/s: mpu.gx_d mpu.gy_d mpu.gz_d

	// Magnetometer
	//mpu.get_mag();
	//mpu.get_mag_t();
	// values: mpu.mx mpu.my mpu.mz
	// in uT: mpu.mx_t mpu.my_t mpu.mz_t

	// Temperature (MPU9250) (C)
	//mpu_temperature = (((float) mpu.get_temp()) / 333.87 + 21.0);

	// END MPU9250

	// ROLL PITCH YAW 
	rollInput = roll;
	
  	

	//if (microsNow - microsPrevious >= microsPerReading) {
		Serial.print("freq: ");
		float freq = 1.0/(((float)(micros()-microsPrevious))/1000000.0);
		microsPrevious = micros();
		Serial.println(freq, 10);
		//Serial.print(" Microsprev: ");
		//Serial.println(microsPrevious);
		mpu.get_accel_g();
		mpu.get_gyro_d();
		mpu.get_mag_t();

		// update the filter, which computes orientation
		madgwickFilter.begin(freq);
		madgwickFilter.update(mpu.gx_d, mpu.gy_d, mpu.gz_d, mpu.x_g, mpu.y_g, mpu.z_g, mpu.mx_t, mpu.my_t, mpu.mz_t);

		roll = madgwickFilter.getRoll();
		pitch = madgwickFilter.getPitch();
		yaw = madgwickFilter.getYaw();
		
		//rollPID.Compute();
		
	//} 

	// Read BMP280 (Temperature, Pressure, Altitude?) (C, Pa, m)

	//bmp_temperature = bmp.readTemperature();
	//bmp_pressure = bmp.readPressure();
	//bmp_altitude = bmp.readAltitude(1013.25);

	// END BMP280

	// UV Sensors
	//uv_1 = analogRead(A0);
	// END UV Sensors

	// GPS
	//bool newGpsData = false;
	/*
	for (unsigned long start = millis(); millis() - start < 500;){
		while (gpsSerial.available()){
			char c = gpsSerial.read();
			Serial.write(c);
			if (gps.encode(c)) newGpsData = true;
		}
	}
	*/

	/*
	int availableSerialChars = gpsSerial.available();
	if (availableSerialChars) {
		for (int i=0; i < availableSerialChars; i++){
			char c = gpsSerial.read();
			if (gps.encode(c)){
				gps.f_get_position(&lat, &lon, &age);
				alt = gps.f_altitude();
			}
		}
	}
	*/

	//Serial.print(lat, 10);
	//Serial.print(" ");
	//Serial.print(lon, 10);
	//Serial.print(" ");
	//Serial.println(alt);
	
	LoRa.beginPacket();
	LoRa.print(lat, 10);
	LoRa.print(" ");
	LoRa.print(lon, 10);
	LoRa.print(" ");
	LoRa.print(dht_temperature);
	LoRa.print(" ");
	LoRa.print(bmp_temperature);
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
	LoRa.print(roll, 5);
	LoRa.print(" ");
	LoRa.print(pitch, 5);
	LoRa.print(" ");
	LoRa.print(yaw, 5);
	LoRa.print("#");
	LoRa.endPacket();
	
}
