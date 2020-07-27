#include <MPU9250.h> // MPU9250
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h> // Universal Adafruit Library
#include <Adafruit_BMP280.h> // BMP280
#include <dht.h> // DHT Humidity
#include <NeoSWSerial.h>
//#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SPI.h>
#include <LoRa.h>
#include <MadgwickAHRS.h>
#include <PID_v1.h>
//#include <ServoTimer2.h>
#include <Servo.h>


// PINs
#define PIN_DHT_HUMIDITY 7
#define PIN_GPS_RX 4
#define PIN_GPS_TX 3
#define PIN_UV_1 A0

dht DHT;
MPU9250 mpu = MPU9250();
Adafruit_BMP280 bmp;
TinyGPS gps;
NeoSWSerial gpsSerial(PIN_GPS_RX, PIN_GPS_TX);

float lat, lon, alt, course = 0.0;
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
int sampleRate = 50;

Servo servo;

void setupMpuRanges(){
	mpu.set_accel_range(RANGE_4G);
	mpu.set_gyro_range(RANGE_GYRO_250);
	mpu.set_mag_scale(SCALE_14_BITS);
	mpu.set_mag_speed(MAG_8_Hz);
}

void setup(){
	Serial.begin(9600);
	uint8_t temp = mpu.begin();
	if (!bmp.begin()) Serial.println("Could not find a valid BMP280 sensor");
	gpsSerial.begin(9600);
	if (!LoRa.begin(433E6)) {
		Serial.println("Starting LoRa failed!");
	}
	
	rollInput = roll;
	rollSetpoint = 0;
	rollPID.SetOutputLimits(-90, 90);
	rollPID.SetMode(AUTOMATIC);
	
	madgwickFilter.begin(sampleRate);
	microsPerReading = 1000000 / sampleRate;
	//microsPrevious = micros();
	rollPID.SetTunings(Kp, Ki, Kd);
	readMpuMagneto();
	servo.attach(5);
}

void readDht(){
	// Read DHT Humidity & Temperature
	int chk = DHT.read11(PIN_DHT_HUMIDITY);
	float temp_dht_temperature = DHT.temperature;
	float temp_dht_humidity = DHT.humidity;
	if (temp_dht_temperature!=-999.00) dht_temperature = temp_dht_temperature;
	if (temp_dht_humidity!=-999.00) dht_humidity = temp_dht_humidity;
	// values: float dht_humidity, dht_temperature
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
	//mpu.get_mag();
	mpu.get_mag_t();
	// values: mpu.mx mpu.my mpu.mz
	// in uT: mpu.mx_t mpu.my_t mpu.mz_t
}

void readMpuTemperature(){
	mpu_temperature = (((float) mpu.get_temp()) / 333.87 + 21.0);
}

void readGps(){
	int availableSerialChars = gpsSerial.available();
	if (availableSerialChars) {
		for (int i=0; i < availableSerialChars; i++){
			char c = gpsSerial.read();
			Serial.write(c);
			if (gps.encode(c)){
				gps.f_get_position(&lat, &lon, &age);
				alt = gps.f_altitude();
				course = gps.course();
			}
		}
	}
}

void readBmp(){
	bmp_temperature = bmp.readTemperature();
	bmp_pressure = bmp.readPressure();
	//bmp_altitude = bmp.readAltitude(1013.25);
}

unsigned long millisAtStart = 0;
unsigned long millisLastRadioBroadcast = 0;
unsigned int loopsSinceLastBroadcast = 0;
void loop(){
	
	readGps();
	readMpuAccel();
	readMpuGyro();
	readMpuMagneto();
	
	float gx_d_filtered = abs(mpu.gx_d+0.8)<0.5 ? 0 : (mpu.gx_d+0.8);
	float gy_d_filtered = abs(mpu.gy_d-0.45)<0.5 ? 0 : (mpu.gy_d-0.45);
	float gz_d_filtered = abs(mpu.gz_d+1.4)<0.5 ? 0 : (mpu.gz_d+1.4);
	
	madgwickFilter.updateIMU(gx_d_filtered, gy_d_filtered, gz_d_filtered, mpu.x_g, mpu.y_g, mpu.z_g);
	//madgwickFilter.update(gx_d_filtered, gy_d_filtered, gz_d_filtered, mpu.x_g, mpu.y_g, mpu.z_g, mpu.mx_t, mpu.mz_t, mpu.my_t);
	roll = madgwickFilter.getRoll();
	pitch = madgwickFilter.getPitch();
	yaw = madgwickFilter.getYaw();


	rollInput = pitch;
	rollPID.Compute();
	Serial.print(roll);
	Serial.print(" ");
	Serial.println(((int) rollOutput)+90);
	servo.write(((int) rollOutput)+90);
	/*
	for (int pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
		// in steps of 1 degree
		servo.write(pos);              // tell servo to go to position in variable 'pos'
		delay(15);                       // waits 15ms for the servo to reach the position
	}
	for (int pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
		servo.write(pos);              // tell servo to go to position in variable 'pos'
		delay(15);                       // waits 15ms for the servo to reach the position
	}
	*/
	//Serial.print("  ");
	
	unsigned long millisNow = millis();
	//Serial.println(float(float(1000)/float(millisNow-millisAtStart)), 4);

	if (millisNow-millisLastRadioBroadcast>500){
		
		//Serial.print(loopsSinceLastBroadcast);
		//Serial.print(" loops in ");
		//Serial.print(millisNow-millisLastRadioBroadcast);
		//Serial.print(" ms ");
		//Serial.println((loopsSinceLastBroadcast*1000/(millisNow-millisLastRadioBroadcast)));
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
		LoRa.print(" ");
		LoRa.print(9);
		LoRa.print(" ");
		LoRa.print((loopsSinceLastBroadcast*1000/(millisNow-millisLastRadioBroadcast)));
		LoRa.print(" ");
		LoRa.print(gx_d_filtered, 5);
		//LoRa.print(mpu.gx_d, 5);
		LoRa.print(" ");
		LoRa.print(gy_d_filtered, 5);
		//LoRa.print(mpu.gy_d, 5);
		LoRa.print(" ");
		LoRa.print(gz_d_filtered, 5);
		//LoRa.print(mpu.gz_d, 5);
		LoRa.print(" ");
		LoRa.print(course, 5);
		LoRa.print("#");
		LoRa.endPacket();
		millisLastRadioBroadcast = millis();
		loopsSinceLastBroadcast = 0;
	} else {
		loopsSinceLastBroadcast++;
	}

	millisAtStart = millis();
}
