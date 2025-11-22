#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BMP085.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_ADXL345_U.h>
#include <SPort.h>
#include <SimpleKalmanFilter.h>
#include <math.h>
#include <TinyGPSPlus.h>


//This is a test change


/*************************************************** 
  This is an example for the BMP085 Barometric Pressure & Temp Sensor

  Designed specifically to work with the Adafruit BMP085 Breakout 
  ----> https://www.adafruit.com/products/391

  These pressure and temperature sensors use I2C to communicate, 2 pins
  are required to interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here

Adafruit_BMP085 bmp;

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);


sensors_event_t accel_event;
sensors_event_t mag_event;

int sensor_read_timeout = 100; 
long last_sensor_read = 0;
int log_write_timeout = 500;
long last_log_write = 0; 

//variables to store sensor data
float alt; 
float kf_alt;
float vsi;
float estimated_vsi;
float temp;
long pressure;
long SL_pressure; 
float accel_X, accel_Y, accel_Z;
float mag_X, mag_Y, mag_Z;
long time_tag; 
float heading;
float headingDegrees;



long sensor_time, read_time, t1, t2;

//KalmanFilter
SimpleKalmanFilter vsiFilter(5, 5, 0.05);
SimpleKalmanFilter altFilter(1, 1, 0.01);

unsigned long task_start = 0;
unsigned long task_timer = 0;

//gps object
TinyGPSPlus gps;


sportData send_latlon(CustomSPortSensor* sensor_gps_latlon) {
  sportData data;
  static bool send_lat = true;
  float lat = gps.location.lat();
  float lon = gps.location.lng();
  data.applicationId = 0x0800;
  uint32_t p = 0;

  if (send_lat) {
    p = (uint32_t)((lat < 0 ? -lat : lat) * 60 * 10000) & 0x3FFFFFFF;
    if(lat < 0) p |= 0x40000000;
    //Serial.print("sending lat ");
    //Serial.println(p, BIN);
  }
  if (!send_lat) {
    p = (uint32_t)((lon < 0 ? -lon : lon) * 60 * 10000) & 0x3FFFFFFF;
    p |= 0x80000000;
    if(lon < 0) p |= 0x40000000;
    //Serial.print("sending lon ");
    //Serial.println(p, BIN);
  }
  
  send_lat = !send_lat;

  data.value = p;

  return data;
}


//FRSKy sensors
SPortHub hub(0x12, D1);
SimpleSPortSensor sensor_alt(0x0100);
SimpleSPortSensor sensor_vario(0x0110);
CustomSPortSensor sensor_gps_latlon(send_latlon);
SimpleSPortSensor sensor_gps_alt(0x0820);
SimpleSPortSensor sensor_gps_spd(0x0830);
SimpleSPortSensor sensor_gps_cog(0x0840);
SimpleSPortSensor sensor_volts(0x0210);








void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void print_header(Stream &refSer) {
  String header = "";
  header += "millis,";
  header += "time_tag,";
  header += "Accel_X,";
  header += "Accel_Y,";
  header += "Accel_Z,";
  header += "Mag_X,";
  header += "Mag_Y,";
  header += "Mag_Z,";
  header += "Temp,";
  header += "Alt,";
  header += "kf_Alt,";
  header += "Pressure,";
  header += "Heading,";
  header += "vsi,";
  header += "estimated_vsi,";

  header += "gps_lat,";
  header += "gps_lon,";
  header += "gps_alt,";
  header += "gps_knots,";
  header += "gps_cog,";



  header += "year,";
  header += "month,";
  header += "day,";
  header += "hour,";
  header += "min,";
  header += "sec,";
  header += "centisec,";
  refSer.println(header);
}

void print_data(Stream &refSer) {
  refSer.print(millis());refSer.print(",");
  refSer.print(time_tag);refSer.print(",");
  refSer.print(accel_X,5); refSer.print(", ");
  refSer.print(accel_Y,5); refSer.print(", ");
  refSer.print(accel_Z,5); refSer.print(", ");
  refSer.print(mag_X,5); refSer.print(", ");
  refSer.print(mag_Y,5); refSer.print(", ");
  refSer.print(mag_Z,5); refSer.print(", ");
  refSer.print(temp,5); refSer.print(", ");
  refSer.print(alt,5); refSer.print(", ");
  refSer.print(kf_alt,5); refSer.print(", ");
  refSer.print(pressure,5); refSer.print(", ");
  refSer.print(headingDegrees,5);refSer.print(", ");
  refSer.print(vsi,5);refSer.print(", ");
  refSer.print(estimated_vsi,5);refSer.print(", ");
  refSer.print(gps.location.lat(),10);refSer.print(", ");
  refSer.print(gps.location.lng(),10);refSer.print(", ");
  refSer.print(gps.altitude.meters() * 100);refSer.print(", ");
  refSer.print(gps.speed.knots(),5);refSer.print(", ");
  refSer.print(gps.course.deg() * 100,3);refSer.print(", ");
  refSer.print(gps.date.year());refSer.print(", ");
  refSer.print(gps.date.month());refSer.print(", ");
  refSer.print(gps.date.day());refSer.print(", ");
  refSer.print(gps.time.hour());refSer.print(", ");
  refSer.print(gps.time.minute());refSer.print(", ");
  refSer.print(gps.time.second());refSer.print(", ");
  refSer.print(gps.time.centisecond());



    
  refSer.println("");
}

float calc_vario(long time, float alt) {
  static long last_time;
  static float last_alt;
  static float last_vsi;
  static float filter = 0.8;

  long delta_t = time - last_time;
  

  estimated_vsi = (alt - last_alt)/(float)delta_t*1000;
  
  //estimated_vsi = vsiFilter.updateEstimate(vsi);


  //vsi = last_vsi*filter +vsi*(1-filter);  
  //last_vsi = vsi;    
  //Serial.print(time); Serial.print("\t"); Serial.print(alt);
  //Serial.print("\t"); Serial.print(delta_t);
  //Serial.print("\t");Serial.print(vsi);
  //Serial.print("\t");Serial.println(estimated_vsi);

  last_time = time;
  last_alt = alt;
  return estimated_vsi;
}

void update_sensors() {
  sensor_time = millis();

  temp = bmp.readTemperature();
  pressure = bmp.readPressure();
  alt = bmp.readAltitude();
  kf_alt = altFilter.updateEstimate(alt);
  SL_pressure = bmp.readSealevelPressure();
  estimated_vsi = calc_vario(sensor_time, kf_alt);
  
  accel.getEvent(&accel_event);
  accel_X = accel_event.acceleration.x;
  accel_Y = accel_event.acceleration.y;
  accel_Z = accel_event.acceleration.z;

  mag.getEvent(&mag_event);
  mag_X = mag_event.magnetic.x;
  mag_Y = mag_event.magnetic.y;
  mag_Z = mag_event.magnetic.z;

  float heading = atan2(mag_event.magnetic.y, mag_event.magnetic.x);
  float headingDegrees = heading * 180/M_PI; 
  
  read_time = millis() - sensor_time;
  time_tag = millis();
  //Serial.print("Reading sensors took "); Serial.print(read_time); Serial.println("ms.");
}

  
void setup() {
  Serial.begin(115200);
  Serial1.begin(38400, SERIAL_8N1, D7, D6); // Serial port for logger
  //Serial1.begin(57600, SERIAL_8N1, D8, D9,true);
  Serial.print("Hello World!");
  //start the frsky sensors
  hub.registerSensor(sensor_alt);
  hub.registerSensor(sensor_vario);
  hub.registerSensor(sensor_gps_latlon);
  hub.registerSensor(sensor_gps_alt);
  hub.registerSensor(sensor_gps_spd);
  hub.registerSensor(sensor_gps_cog);
  hub.registerSensor(sensor_volts);
  hub.begin();
  
  delay(2000);
  if (!bmp.begin()) {
	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	while (1) {}
  }

  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }

    if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  accel.setRange(ADXL345_RANGE_16_G);
    
  /* Display some basic information on this sensor */
  //displaySensorDetails();
  //print_header(Serial);
  
  //Print file header to log file.
  print_header(Serial1);
  //print_header(Serial);
  
}


//gps test function
void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }
}



void loop() {
  if (millis() > last_sensor_read + sensor_read_timeout) {
    task_start = micros();
    update_sensors(); 
    task_timer = micros() - task_start;
    //Serial.print("sensor update took: ");
    //Serial.println(task_timer);
    last_sensor_read = millis();
  }

    
  
  
  //Send Data to Logger
  
  if (millis() > last_log_write + log_write_timeout) {
    task_start = micros();
    //write to the SD Card
    //Serial.print("Writing to SD Card");
    print_data(Serial1);
    //print_data(Serial);
    last_log_write = millis();
    task_timer = micros() - task_start;
    //Serial.print(" took: ");
    //Serial.println(task_timer);
  }
  
  //print_data(Serial);
  //delay(10);

  while (Serial1.available()) {
    //Serial.print("Bytes available!");
    //Serial.print((char)Serial1.read());
    gps.encode(Serial1.read());
    sensor_gps_alt.value = gps.altitude.meters() * 100;
    sensor_gps_spd.value = gps.speed.knots();//gps.speed.mps() * 100;
    sensor_gps_cog.value = gps.course.deg() * 100;
    //if (gps.encode(Serial1.read()))
      //displayInfo();
    
  }//Serial.println(",");


  //displayInfo();
  sensor_alt.value = alt*100;
  sensor_vario.value = estimated_vsi*100;
  sensor_volts.value = 8.23 *100;
  
  hub.handle();
































































































































































































































































































































































































































}