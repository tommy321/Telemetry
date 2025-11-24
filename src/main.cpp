#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP5xx.h>

#include <Arduino.h>
#include <string.h>

//#include <Adafruit_BMP085.h>
//#include <Adafruit_HMC5883_U.h>
//#include <Adafruit_ADXL345_U.h>

#include <SPort.h>
#include <SimpleKalmanFilter.h>
#include <math.h>
#include <TinyGPSPlus.h>

//stuff for the SD Card
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "sd_card_test.h"



#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP5xx bmp; // Create BMP5xx object
bmp5xx_powermode_t desiredMode = BMP5XX_POWERMODE_NORMAL; // Cache desired power mode


/*Sutff for the old sensors
Adafruit_BMP085 bmp;

// Assign a unique ID to this sensor at the same time 
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
*/

//We don't have accel or mag anymore
//sensors_event_t accel_event;
//sensors_event_t mag_event;

int sensor_read_timeout = 1000;//20; 
long last_sensor_read = 0;
int log_write_timeout = 1000;//100;
long last_log_write = 0; 

//variables to store sensor data
float alt; 
float kf_alt;
float vsi;
float estimated_vsi;
float temp;
long pressure;
long SL_pressure; 
//float accel_X, accel_Y, accel_Z;
//float mag_X, mag_Y, mag_Z;
long time_tag; 
//float heading;
//float headingDegrees;



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







/* //mag sensor is gone
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
}*/


void print_header(Stream &refSer) {
  String header = "";
  header += "millis,";
  header += "time_tag,";
  header += "Temp,";
  header += "Pressure,";
  header += "Alt,";

  /*
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
  */
  refSer.println(header);
}

void print_data(Stream &refSer) {
  refSer.print(millis());refSer.print(",");
  refSer.print(time_tag);refSer.print(",");
  refSer.print(temp,5); refSer.print(", ");
  refSer.print(pressure,5); refSer.print(", ");
  refSer.print(alt,5); refSer.print(", ");
  /*
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
  */


    
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

/* will need to rewrite this
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
}*/

void update_bmp581() {
  if (bmp.performReading()) {
    temp = bmp.temperature;
    pressure = bmp.pressure;
    alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  }

}


void setup_bmp581() {
  //start the BMP581
  // Try to initialize the sensor
  // For I2C mode (default):
  if (!bmp.begin(BMP5XX_DEFAULT_ADDRESS, &Wire)) {
  // For SPI mode (uncomment the line below and comment out the I2C line above):
  // if (!bmp.begin(BMP5XX_CS_PIN, &SPI)) {
    Serial.println(F("Could not find a valid BMP5xx sensor, check wiring!"));
    while (1) delay(10);
  }
  Serial.println(F("BMP5xx found!"));
  Serial.println();
// Demonstrate all setter functions with range documentation
  Serial.println(F("=== Setting Up Sensor Configuration ==="));
  
  /* Temperature Oversampling Settings:
   * BMP5XX_OVERSAMPLING_1X   - 1x oversampling (fastest, least accurate)
   * BMP5XX_OVERSAMPLING_2X   - 2x oversampling  
   * BMP5XX_OVERSAMPLING_4X   - 4x oversampling
   * BMP5XX_OVERSAMPLING_8X   - 8x oversampling
   * BMP5XX_OVERSAMPLING_16X  - 16x oversampling
   * BMP5XX_OVERSAMPLING_32X  - 32x oversampling
   * BMP5XX_OVERSAMPLING_64X  - 64x oversampling
   * BMP5XX_OVERSAMPLING_128X - 128x oversampling (slowest, most accurate)
   */
  Serial.println(F("Setting temperature oversampling to 2X..."));
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);

  /* Pressure Oversampling Settings (same options as temperature):
   * Higher oversampling = better accuracy but slower readings
   * Recommended: 16X for good balance of speed/accuracy
   */
  Serial.println(F("Setting pressure oversampling to 16X..."));
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);

  /* IIR Filter Coefficient Settings:
   * BMP5XX_IIR_FILTER_BYPASS   - No filtering (fastest response)
   * BMP5XX_IIR_FILTER_COEFF_1  - Light filtering
   * BMP5XX_IIR_FILTER_COEFF_3  - Medium filtering
   * BMP5XX_IIR_FILTER_COEFF_7  - More filtering
   * BMP5XX_IIR_FILTER_COEFF_15 - Heavy filtering
   * BMP5XX_IIR_FILTER_COEFF_31 - Very heavy filtering
   * BMP5XX_IIR_FILTER_COEFF_63 - Maximum filtering
   * BMP5XX_IIR_FILTER_COEFF_127- Maximum filtering (slowest response)
   */
  Serial.println(F("Setting IIR filter to coefficient 3..."));
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);

  /* Output Data Rate Settings (Hz):
   * BMP5XX_ODR_240_HZ, BMP5XX_ODR_218_5_HZ, BMP5XX_ODR_199_1_HZ
   * BMP5XX_ODR_179_2_HZ, BMP5XX_ODR_160_HZ, BMP5XX_ODR_149_3_HZ
   * BMP5XX_ODR_140_HZ, BMP5XX_ODR_129_8_HZ, BMP5XX_ODR_120_HZ
   * BMP5XX_ODR_110_1_HZ, BMP5XX_ODR_100_2_HZ, BMP5XX_ODR_89_6_HZ
   * BMP5XX_ODR_80_HZ, BMP5XX_ODR_70_HZ, BMP5XX_ODR_60_HZ, BMP5XX_ODR_50_HZ
   * BMP5XX_ODR_45_HZ, BMP5XX_ODR_40_HZ, BMP5XX_ODR_35_HZ, BMP5XX_ODR_30_HZ
   * BMP5XX_ODR_25_HZ, BMP5XX_ODR_20_HZ, BMP5XX_ODR_15_HZ, BMP5XX_ODR_10_HZ
   * BMP5XX_ODR_05_HZ, BMP5XX_ODR_04_HZ, BMP5XX_ODR_03_HZ, BMP5XX_ODR_02_HZ
   * BMP5XX_ODR_01_HZ, BMP5XX_ODR_0_5_HZ, BMP5XX_ODR_0_250_HZ, BMP5XX_ODR_0_125_HZ
   */
  Serial.println(F("Setting output data rate to 50 Hz..."));
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);

  /* Power Mode Settings:
   * BMP5XX_POWERMODE_STANDBY     - Standby mode (no measurements)
   * BMP5XX_POWERMODE_NORMAL      - Normal mode (periodic measurements)
   * BMP5XX_POWERMODE_FORCED      - Forced mode (single measurement then standby)
   * BMP5XX_POWERMODE_CONTINUOUS  - Continuous mode (fastest measurements)
   * BMP5XX_POWERMODE_DEEP_STANDBY - Deep standby (lowest power)
   */
  Serial.println(F("Setting power mode to normal..."));
  desiredMode = BMP5XX_POWERMODE_NORMAL;
  bmp.setPowerMode(desiredMode);

  /* Enable/Disable Pressure Measurement:
   * true  - Enable pressure measurement (default)
   * false - Disable pressure measurement (temperature only)
   */
  Serial.println(F("Enabling pressure measurement..."));
  bmp.enablePressure(true);

  /* Interrupt Configuration:
   * BMP5XX_INTERRUPT_PULSED / BMP5XX_INTERRUPT_LATCHED - Interrupt mode
   * BMP5XX_INTERRUPT_ACTIVE_LOW / BMP5XX_INTERRUPT_ACTIVE_HIGH - Interrupt polarity  
   * BMP5XX_INTERRUPT_PUSH_PULL / BMP5XX_INTERRUPT_OPEN_DRAIN - Interrupt drive
   * BMP5XX_INTERRUPT_DATA_READY, BMP5XX_INTERRUPT_FIFO_FULL, etc. - Interrupt sources (can combine with |)
   */
  Serial.println(F("Configuring interrupt pin with data ready source..."));
bmp.configureInterrupt(BMP5XX_INTERRUPT_LATCHED,
                       BMP5XX_INTERRUPT_ACTIVE_HIGH, BMP5XX_INTERRUPT_PUSH_PULL, BMP5XX_INTERRUPT_DATA_READY, false);

  Serial.println();
  Serial.println(F("=== Current Sensor Configuration ==="));
  
  // Pretty print temperature oversampling inline
  Serial.print(F("Temperature Oversampling: "));
  switch(bmp.getTemperatureOversampling()) {
    case BMP5XX_OVERSAMPLING_1X:   Serial.println(F("1X")); break;
    case BMP5XX_OVERSAMPLING_2X:   Serial.println(F("2X")); break;
    case BMP5XX_OVERSAMPLING_4X:   Serial.println(F("4X")); break;
    case BMP5XX_OVERSAMPLING_8X:   Serial.println(F("8X")); break;
    case BMP5XX_OVERSAMPLING_16X:  Serial.println(F("16X")); break;
    case BMP5XX_OVERSAMPLING_32X:  Serial.println(F("32X")); break;
    case BMP5XX_OVERSAMPLING_64X:  Serial.println(F("64X")); break;
    case BMP5XX_OVERSAMPLING_128X: Serial.println(F("128X")); break;
    default: Serial.println(F("Unknown")); break;
  }
  
  // Pretty print pressure oversampling inline
  Serial.print(F("Pressure Oversampling: "));
  switch(bmp.getPressureOversampling()) {
    case BMP5XX_OVERSAMPLING_1X:   Serial.println(F("1X")); break;
    case BMP5XX_OVERSAMPLING_2X:   Serial.println(F("2X")); break;
    case BMP5XX_OVERSAMPLING_4X:   Serial.println(F("4X")); break;
    case BMP5XX_OVERSAMPLING_8X:   Serial.println(F("8X")); break;
    case BMP5XX_OVERSAMPLING_16X:  Serial.println(F("16X")); break;
    case BMP5XX_OVERSAMPLING_32X:  Serial.println(F("32X")); break;
    case BMP5XX_OVERSAMPLING_64X:  Serial.println(F("64X")); break;
    case BMP5XX_OVERSAMPLING_128X: Serial.println(F("128X")); break;
    default: Serial.println(F("Unknown")); break;
  }
  
  // Pretty print IIR filter coefficient inline
  Serial.print(F("IIR Filter Coefficient: "));
  switch(bmp.getIIRFilterCoeff()) {
    case BMP5XX_IIR_FILTER_BYPASS:   Serial.println(F("Bypass (No filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_1:  Serial.println(F("1 (Light filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_3:  Serial.println(F("3 (Medium filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_7:  Serial.println(F("7 (More filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_15: Serial.println(F("15 (Heavy filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_31: Serial.println(F("31 (Very heavy filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_63: Serial.println(F("63 (Maximum filtering)")); break;
    case BMP5XX_IIR_FILTER_COEFF_127:Serial.println(F("127 (Maximum filtering)")); break;
    default: Serial.println(F("Unknown")); break;
  }
  
  // Pretty print output data rate inline
  Serial.print(F("Output Data Rate: "));
  switch(bmp.getOutputDataRate()) {
    case BMP5XX_ODR_240_HZ:   Serial.println(F("240 Hz")); break;
    case BMP5XX_ODR_218_5_HZ: Serial.println(F("218.5 Hz")); break;
    case BMP5XX_ODR_199_1_HZ: Serial.println(F("199.1 Hz")); break;
    case BMP5XX_ODR_179_2_HZ: Serial.println(F("179.2 Hz")); break;
    case BMP5XX_ODR_160_HZ:   Serial.println(F("160 Hz")); break;
    case BMP5XX_ODR_149_3_HZ: Serial.println(F("149.3 Hz")); break;
    case BMP5XX_ODR_140_HZ:   Serial.println(F("140 Hz")); break;
    case BMP5XX_ODR_129_8_HZ: Serial.println(F("129.8 Hz")); break;
    case BMP5XX_ODR_120_HZ:   Serial.println(F("120 Hz")); break;
    case BMP5XX_ODR_110_1_HZ: Serial.println(F("110.1 Hz")); break;
    case BMP5XX_ODR_100_2_HZ: Serial.println(F("100.2 Hz")); break;
    case BMP5XX_ODR_89_6_HZ:  Serial.println(F("89.6 Hz")); break;
    case BMP5XX_ODR_80_HZ:    Serial.println(F("80 Hz")); break;
    case BMP5XX_ODR_70_HZ:    Serial.println(F("70 Hz")); break;
    case BMP5XX_ODR_60_HZ:    Serial.println(F("60 Hz")); break;
    case BMP5XX_ODR_50_HZ:    Serial.println(F("50 Hz")); break;
    case BMP5XX_ODR_45_HZ:    Serial.println(F("45 Hz")); break;
    case BMP5XX_ODR_40_HZ:    Serial.println(F("40 Hz")); break;
    case BMP5XX_ODR_35_HZ:    Serial.println(F("35 Hz")); break;
    case BMP5XX_ODR_30_HZ:    Serial.println(F("30 Hz")); break;
    case BMP5XX_ODR_25_HZ:    Serial.println(F("25 Hz")); break;
    case BMP5XX_ODR_20_HZ:    Serial.println(F("20 Hz")); break;
    case BMP5XX_ODR_15_HZ:    Serial.println(F("15 Hz")); break;
    case BMP5XX_ODR_10_HZ:    Serial.println(F("10 Hz")); break;
    case BMP5XX_ODR_05_HZ:    Serial.println(F("5 Hz")); break;
    case BMP5XX_ODR_04_HZ:    Serial.println(F("4 Hz")); break;
    case BMP5XX_ODR_03_HZ:    Serial.println(F("3 Hz")); break;
    case BMP5XX_ODR_02_HZ:    Serial.println(F("2 Hz")); break;
    case BMP5XX_ODR_01_HZ:    Serial.println(F("1 Hz")); break;
    case BMP5XX_ODR_0_5_HZ:   Serial.println(F("0.5 Hz")); break;
    case BMP5XX_ODR_0_250_HZ: Serial.println(F("0.25 Hz")); break;
    case BMP5XX_ODR_0_125_HZ: Serial.println(F("0.125 Hz")); break;
    default: Serial.println(F("Unknown")); break;
  }
  
  // Pretty print power mode inline
  Serial.print(F("Power Mode: "));
  switch(bmp.getPowerMode()) {
    case BMP5XX_POWERMODE_STANDBY:     Serial.println(F("Standby")); break;
    case BMP5XX_POWERMODE_NORMAL:      Serial.println(F("Normal")); break;
    case BMP5XX_POWERMODE_FORCED:      Serial.println(F("Forced")); break;
    case BMP5XX_POWERMODE_CONTINUOUS:  Serial.println(F("Continuous")); break;
    case BMP5XX_POWERMODE_DEEP_STANDBY:Serial.println(F("Deep Standby")); break;
    default: Serial.println(F("Unknown")); break;
  }
  
  Serial.println();
}


std::string get_next_logfilename(fs::FS &fs, const char * dirname, uint8_t levels) {

  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  //listDir(SD, "/", 0);
  
  File root = fs.open(dirname);
  //if (~root) {
  // Serial.println("Failed to Open Directory");
  //}
  //if(!root.isDirectory()){
  //  Serial.println("Not a directory");
  //  
  //}

  File file = root.openNextFile();
  int lognumber = 0;  //if we have no existing logs we'll start at zero
  while(file){
    //check if the first three letters of the filename are "LOG"
    std::string filename = file.name();
    if (filename.length() >= 3) {
      //Serial.println(filename.substr(0,3).c_str());
      if (filename.substr(0, 3) == "LOG") {
        //Serial.print(filename.c_str());
        //Serial.print(" ");
        //Serial.print(filename.find("LOG"));
        //Serial.print(" ");
        //Serial.print(filename.find("."));

        //get the number out of the file name
        std::string filenumber = filename.substr(3, 5);
        //Serial.print(" ");
        //Serial.println(std::stoi(filenumber));
        if (lognumber < std::stoi(filenumber)) {
          lognumber = std::stoi(filenumber);
        }
        
      }
    }
    file = root.openNextFile();
  }
  lognumber++; //this is the next open log number
  std::string lognumberstring = "00000";
  lognumberstring.append(std::to_string(lognumber)); //add the new lognumber
  lognumberstring = lognumberstring.substr(lognumberstring.length()-5, 5);
  std::string new_filename = "/LOG";
  new_filename.append(lognumberstring);
  new_filename.append(".TXT");
  return new_filename;

}
  
void setup() {
  Serial.begin(115200);
  //don't have logger anymore
  //Serial1.begin(38400, SERIAL_8N1, D7, D6); // Serial port for logger
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
  
  //delay(2000);
  
  //start the BMP581:
  setup_bmp581();


  /* we don't have the bmp anymore
  if (!bmp.begin()) {
	  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	  while (1) {}
  }*/

  /* don't have a mag or accelerometer anymore 
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections 
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }

    if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections 
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  } 
  accel.setRange(ADXL345_RANGE_16_G);
  */

  


  /* Display some basic information on this sensor */
  //displaySensorDetails();
  //print_header(Serial);
  
  //Print file header to log file.
  //print_header(Serial1);

  //test the SD card
  //test_SD_card();

  //get the lastest file name
  
  if(!SD.begin(D0)){
    Serial.println("Card Mount Failed");
    
  }
  std::string new_filename = get_next_logfilename(SD, "/", 0);
  Serial.println(new_filename.c_str());
  writeFile(SD, new_filename.c_str(), "Test content");
  print_header(Serial);
  
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
    //update_sensors(); 
    update_bmp581();
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
    //print_data(Serial1);
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