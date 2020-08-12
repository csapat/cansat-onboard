#include <Wire.h>
#include <SPI.h>
#include <NeoSWSerial.h>

#include <MPU9250.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <dht.h>
#include <MadgwickAHRS.h>

//PINs
#define PIN_DHT_HUMIDITY 7
#define PIN_UV A0
#define PIN_DAT_RX 5
#define PIN_DAT_TX 6

NeoSWSerial datSerial(PIN_DAT_RX, PIN_DAT_TX);
dht DHT;
MPU9250 mpu = MPU9250();
Adafruit_BMP280 bmp;
Madgwick madgwickFilter;

float dht_temperature = 0;
float dht_humidity = 0;
float bmp_temperature = 0;
float bmp_pressure = 0;
float bmp_altitude = 0;
float mpu_temperature = 0;
float uv = 0;
float roll, pitch, yaw;

int sampleRate = 50;

void setup() {
	Serial.begin(9600);
	datSerial.begin(9600);
	
	mpu.begin();
	mpu.set_accel_range(RANGE_4G);
	mpu.set_gyro_range(RANGE_GYRO_250);
	mpu.set_mag_scale(SCALE_14_BITS);
	mpu.set_mag_speed(MAG_8_Hz);
	
	madgwickFilter.begin(sampleRate);
}

void loop() {
	mpu.get_accel_g();
	mpu.get_gyro_d();
	mpu.get_mag_t();
	mpu_temperature = (((float) mpu.get_temp()) / 333.87 + 21.0);
	
	float gx_d_filtered = abs(mpu.gx_d+0.8)<0.5 ? 0 : (mpu.gx_d+0.8);
	float gy_d_filtered = abs(mpu.gy_d-0.45)<0.5 ? 0 : (mpu.gy_d-0.45);
	float gz_d_filtered = abs(mpu.gz_d+1.4)<0.5 ? 0 : (mpu.gz_d+1.4);
	
	madgwickFilter.updateIMU(gx_d_filtered, gy_d_filtered, gz_d_filtered, mpu.x_g, mpu.y_g, mpu.z_g);
	roll = madgwickFilter.getRoll();
	pitch = madgwickFilter.getPitch();
	yaw = madgwickFilter.getYaw();
	
	DHT.read11(PIN_DHT_HUMIDITY);
	float temp_dht_temperature = DHT.temperature;
	float temp_dht_humidity = DHT.humidity;
	if (temp_dht_temperature!=-999.00) dht_temperature = temp_dht_temperature;
	if (temp_dht_humidity!=-999.00) dht_humidity = temp_dht_humidity;

	bmp_temperature = bmp.readTemperature();
	bmp_pressure = bmp.readPressure();
	bmp_altitude = bmp.readAltitude(1013.25);

	uv = analogRead(PIN_UV);
	
	datSerial.print(bmp_altitude);
	datSerial.print(" ");
	datSerial.print(dht_temperature, 5);
	datSerial.print(" ");
	datSerial.print(bmp_temperature, 5);
	datSerial.print(" ");
	datSerial.print(mpu_temperature, 5);
	datSerial.print(" ");
	datSerial.print(mpu.x_g, 5);
	datSerial.print(" ");
	datSerial.print(mpu.y_g, 5);
	datSerial.print(" ");
	datSerial.print(mpu.z_g, 5);
	datSerial.print(" ");
	datSerial.print(mpu.mx_t, 5);
	datSerial.print(" ");
	datSerial.print(mpu.my_t, 5);
	datSerial.print(" ");
	datSerial.print(mpu.mz_t, 5);
	datSerial.print(" ");
	datSerial.print(gx_d_filtered, 5);
	datSerial.print(" ");
	datSerial.print(gy_d_filtered, 5);
	datSerial.print(" ");
	datSerial.print(gz_d_filtered, 5);
	datSerial.print(" ");
	datSerial.print(roll, 5);
	datSerial.print(" ");
	datSerial.print(pitch, 5);
	datSerial.print(" ");
	datSerial.print(yaw, 5);
	datSerial.print(" ");
	datSerial.print(bmp_pressure, 5);
	datSerial.print(" ");
	datSerial.print(dht_humidity, 5);
	datSerial.print(" ")
	datSerial.print(uv, 5);
	datSerial.print("#");
}