// Сохранение сырых данных от акселя, гиро и мага на SD карту
// Плата - XIAO SAMD21
// IMU - Invensense MPU9250

#include <SparkFunMPU9250-DMP.h>
#include <SD.h>
#include <SPI.h>

const int chipSelectPin = 3;

#define DEBUG 1  // использовать Serial для вывода сообщений

// текущие значения от акселя, гиро и мага
int ax = 0, ay = 0, az = 0;
int gx = 0, gy = 0, gz = 0;
int mx = 0, my = 0, mz = 0;

// смещения для калибровки акселя
int ax_offset = 100;
int ay_offset = 150;
int az_offset = 300;
int ax_cal_avg = 0;
int ay_cal_avg = 0;
int az_cal_avg = 0;

// смещения и коэфф.для калибровки мага
int mx_offset = 200;
int my_offset = 500;
int mz_offset = -140;
float mx_scale = 1.01;
float my_scale = 0.94;
float mz_scale = 1.05;

//uint32_t delt_t = 0;                           // used to control display output rate
uint32_t count = 0, sumCount = 0;                // used to control display output rate
float deltat = 0.0f, sum = 0.0f;                 // integration interval for both filter schemes
uint32_t lastUpdate = 0;      // used to calculate integration interval
uint32_t Now = 0; // used to calculate integration interval

MPU9250_DMP imu;
File f;
char rows[100][80];

void setup() 
{
  
  #ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Starting");
  #endif

  delay(3000);

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelectPin)) {
    #ifdef DEBUG
    Serial.println("Card failed, or not present");
    #endif
    blinkLED(3, 500, 500);;
    while(1);
  }
  Serial.println("card initialized.");

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS) {
    #ifdef DEBUG
    Serial.println("Unable to communicate with MPU-9250");
    #endif
    blinkLED(3, 500, 500);
    while(1);
  }

  // запрещаем генерацию прерываний 
  imu.enableInterrupt(0);

  // будем использовать все сенсоры
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  // Use setGyroFSR() and setaccFSR() to configure the
  // gyroscope and accerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(1000); // Set gyro to 1000 dps
  // acc options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(4); // Set acc to +/-4g
  // Note: the MPU-9250's magnetometer FSR is set at 
  // +/- 4912 uT (micro-tesla's)

  // setLPF() can be used to set the digital low-pass filter
  // of the accerometer and gyroscope.
  // Can be any of the following: 188, 98, 42, 20, 10, 5
  // (values are in Hz).
  imu.setLPF(42); // Set LPF corner frequency to 42Hz

  // The sample rate of the acc/gyro can be set using
  // setSampleRate. Acceptable values range from 4Hz to 1kHz
  imu.setSampleRate(100); // Set sample rate to 100Hz

  // Likewise, the compass (magnetometer) sample rate can be
  // set using the setCompassSampleRate() function.
  // This value can range between: 1-100Hz
  imu.setCompassSampleRate(100); // Set mag rate to 100Hz

}

void loop() 
{

  // ждем появления новых данных в FIFO
  while(!imu.dataReady()); 

  // Call update() to update the imu objects sensor data.
  // You can specify which sensors to update by combining
  // UPDATE_acc, UPDATE_GYRO, UPDATE_COMPASS, and/or
  // UPDATE_TEMPERATURE.
  // (The update function defaults to acc, gyro, compass,
  //  so you don't have to specify these values.)
  if (imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS) != INV_SUCCESS) {
    // сигнализируем 
    #ifdef DEBUG
    Serial.println("MPU update sensors failed");
    #endif
    blinkLED(3, 500, 500);
  }

  // получаем значения от акселя
  ax = imu.ax - ax_offset;
  ay = imu.ay - ay_offset; 
  az = imu.az - az_offset;
  // средние значения для калибровки
  //ax_cal_avg = (ax_cal_avg * 299 + imu.ax - ax_offset) / 300;
  //ay_cal_avg = (ay_cal_avg * 299 + imu.ay - ay_offset) / 300;
  //az_cal_avg = (az_cal_avg * 299 + imu.az - az_offset) / 300;

  // получаем значения от гиро
  gx = imu.gx;
  gy = imu.gy; 
  gz = imu.gz;
  
  // получаем значения от мага
  mx = (int)((imu.mx - mx_offset) * mx_scale);
  my = (int)((imu.my - my_offset) * my_scale); 
  mz = (int)((imu.mz - mz_offset) * mz_scale);

  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  sum += deltat; // sum for averaging filter update rate
  //sumCount++;

  sprintf(rows[sumCount++ % 100], "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", 
          ax, ay, az, gx, gy, gz, mx, my, mz, (int)(deltat * 1000000));

  if (sumCount % 100 == 0) {
    f = SD.open("test2.txt", FILE_WRITE);
    if (f) {
      for (int i = 0; i < 100; i ++) {
        f.println((String)rows[i]);
        Serial.println(rows[i]);
      }
      f.close();
    }
  }
  
  #ifdef DEBUG
  //Serial.println((String)imu.ax + "\t" + imu.ay + "\t" +imu.az + "\t");
  //Serial.println((String)ax_cal_avg + "\t" + ay_cal_avg + "\t" + az_cal_avg);
  //Serial.println((String)(imu.ax - ax_offset) + "\t" + (imu.ay - ay_offset)+ "\t" + (imu.az - az_offset));
  //Serial.print((String)imu.gx + "\t" + imu.gy + "\t" +imu.gz + "\t");
  //Serial.println((String)imu.mx + "\t" + imu.my + "\t" +imu.mz);
  //Serial.println((String)ax + "\t" + ay + "\t" + az);
  //Serial.print("\t");
  //Serial.println((String)gx + "\t" + gy + "\t" + gz);
  //Serial.println((String)mx + "\t" + my + "\t" + mz);
  //Serial.println((String)qn[0] + "\t" + qn[1] + "\t" + qn[2] + "\t" + qn[3]);
  //Serial.println(deltat);

  //Serial.println((String)ax_c + "\t" + ay_c + "\t" + az_c);
  //Serial.println((String)ay + "\t" + ay_c + "\t" + ay_avg);

  #endif
}

// мигание светодиодом и стоп-сигналом
void blinkLED(int count, int delay_on, int delay_off) {

  for (int i = 0; i < count; i++) {
    digitalWrite(LED_BUILTIN, LOW);   // включаем
    delay(delay_on);
    digitalWrite(LED_BUILTIN, HIGH);  // выключаем
    delay(delay_off);
  }
}
