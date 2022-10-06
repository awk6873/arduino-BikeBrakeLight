// Сохранение сырых данных от акселя, гиро и мага на SD карту
// Плата - XIAO SAMD21
// IMU - Invensense MPU9250

#include <SparkFunMPU9250-DMP.h>
#include "quaternionFilters.h"
#include "helper_3dmath.h"
#include <SD.h>
#include <SPI.h>

const int chipSelectPin = 3;

#define DEBUG 1  // использовать Serial для вывода сообщений

// частота ФНЧ для акселя и гиро (188, 98, 42, 20, 10, 5 Гц)
#define IMU_LPF 42
// частота выборки для акселя, гиро и мага (от 4 Гц to 1 кГц)
#define IMU_SAMPLE_RATE 76

// макс.диапазон значений (full scale range) для гиро +/- 250, 500, 1000, 2000 dps
#define GYRO_FSR 1000
// для акселя +/- 2, 4, 8, or 16 g
#define ACCEL_FSR 4
// для магнитометра - константа +/- 4912 uT (micro-tesla)

// множитель перевода значений в целые
#define INT_MULTIPLIER 10000

// текущие значения от акселя, гиро и мага
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float mx = 0, my = 0, mz = 0;
// значения от акселя после компенсации гравитации
float ax_c = 0, ay_c = 0, az_c = 0;

// текущие значения для кватерниона
float *qn;

// смещения для калибровки акселя
int ax_offset = -100;
int ay_offset = 200;
int az_offset = -100;
// смещения для калибровки гиро
int gx_offset = 33;
int gy_offset = 40;
int gz_offset = 12;
// смещения и коэфф.для калибровки мага
int mx_offset = 420;
int my_offset = 180;
int mz_offset = -180;
float mx_scale = 1.0;
float my_scale = 1.5;
float mz_scale = 0.80;

//uint32_t delt_t = 0;                           // used to control display output rate
uint32_t count = 0, sumCount = 0;                // used to control display output rate
float deltat = 0.0f, sum = 0.0f;                 // integration interval for both filter schemes
uint32_t lastUpdate = 0;      // used to calculate integration interval
uint32_t Now = 0; // used to calculate integration interval

MPU9250_DMP imu;
File f;
char rows[300][80];

void setup() 
{
  delay(5000);
    
  #ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Starting");
  #endif

  // кнопка для регистрации торможения
  pinMode(0, INPUT_PULLUP);
  pinMode(1, OUTPUT);
  digitalWrite(1, LOW);

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

  // запускаем MPU
  MPU_init();

}

void loop() 
{

  // ждем появления новых данных в FIFO
  while(!imu.dataReady());

  if (imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS) != INV_SUCCESS) {
    // сигнализируем 
    #ifdef DEBUG
    Serial.println("MPU update sensors failed");
    #endif
    #ifdef OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("MPU update sensors failed");
    display.display();
    #endif
    blinkLED(3, 500, 500);
  }

  // получаем значения от акселя
  ax = imu.calcAccel(imu.ax - ax_offset);
  ay = imu.calcAccel(imu.ay - ay_offset); 
  az = imu.calcAccel(imu.az - az_offset);

  // получаем значения от гиро
  gx = imu.calcGyro(imu.gx - gx_offset);
  gy = imu.calcGyro(imu.gy - gy_offset); 
  gz = imu.calcGyro(imu.gz - gz_offset);

  // получаем значения от мага
  mx = imu.calcMag((imu.mx - mx_offset) * mx_scale);
  my = imu.calcMag((imu.my - my_offset) * my_scale); 
  mz = imu.calcMag((imu.mz - mz_offset) * mz_scale);

  // параметр для фильтра Mahony
  //float Kp = 120 * analogRead(A0) / 1023.0;
  //setMahonyKp(Kp);

  // параметр для фильтра Madgwick
  float beta = 2; //5.0 * analogRead(A0) / 1023.0;
  setMadgwickBeta(beta);

  // вычисляем кватернион положения относительно Земли
//  MahonyQuaternionUpdate(
//  ax,                ay,                  az,
//  gx * DEG_TO_RAD,   gy * DEG_TO_RAD,     gz * DEG_TO_RAD,
//  my,                mx,                  -mz,
//  deltat);

  MadgwickQuaternionUpdate(
    ax,                ay,                  az,
    gx * DEG_TO_RAD,   gy * DEG_TO_RAD,     gz * DEG_TO_RAD,
    my,                mx,                  -mz,
    deltat
  );

  qn = (float *)getQ();
  Quaternion q1(qn[0], qn[1], qn[2], qn[3]);       // кватернион
  Quaternion q1_(qn[0], -qn[1], -qn[2], -qn[3]);   // обратный кватернион

  // компенсируем гравитацию для акселя
  VectorFloat v_acc(ax, ay, az);     // вектор для акселя
  v_acc = v_acc.getRotated(&q1);     // поворачиваем его в систему координат Земли
  v_acc.z -= 1.0;                    // вычитаем гравитацию из значения на оси Z
  v_acc = v_acc.getRotated(&q1_);    // поворачиваем обратно в систему координат вело

  // значения для акселя без влияния гравитации
  ax_c = v_acc.x;
  ay_c = v_acc.y;
  az_c = v_acc.z;

  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  sum += deltat; // sum for averaging filter update rate

  int brake_state = digitalRead(0)?0:1;
  if (brake_state == 1)
    // тормоз нажат
    digitalWrite(LED_BUILTIN, LOW);
  else
    digitalWrite(LED_BUILTIN, HIGH);
    
  sprintf(rows[sumCount++ % 300], "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d",
          (int)(ax * INT_MULTIPLIER),   (int)(ay * INT_MULTIPLIER),   (int)(az * INT_MULTIPLIER), 
          (int)(ax_c * INT_MULTIPLIER), (int)(ay_c * INT_MULTIPLIER), (int)(az_c * INT_MULTIPLIER),
          (int)(gx * INT_MULTIPLIER),   (int)(gy * INT_MULTIPLIER),   (int)(gz * INT_MULTIPLIER), 
          brake_state);

  if (sumCount % 300 == 0) {
    f = SD.open("mpu_qn.txt", FILE_WRITE);
    if (!f) {
      Serial.println("Error opening file");
      while(1);
    }
    for (int i = 0; i < 300; i ++) {
      f.println((String)rows[i]);
      Serial.println(rows[i]);
    }
    f.close();
  }
  
  #ifdef DEBUG
  //Serial.print((String)ax + "\t" + ay + "\t" + az + "\t");
  //Serial.println((String)ax_cal_avg + "\t" + ay_cal_avg + "\t" + az_cal_avg);
  //Serial.println((String)(imu.ax - ax_offset) + "\t" + (imu.ay - ay_offset)+ "\t" + (imu.az - az_offset));
  //Serial.print((String)(imu.gx - gx_offset) + "\t" + (imu.gy - gy_offset)+ "\t" + (imu.gz - gz_offset));
  //Serial.print("\t");
  //Serial.println((String)gx + "\t" + gy + "\t" + gz + "\t");
  //Serial.println((String)mx + "\t" + my + "\t" + mz);
  //Serial.print((String)ax + "\t" + ay + "\t" + az);
  //Serial.print("\t");
  //Serial.println((String)imu.gx + "\t" + imu.gy + "\t" + imu.gz);
  //Serial.println((String)mx + "\t" + my + "\t" + mz);
  //Serial.println((String)qn[0] + "\t" + qn[1] + "\t" + qn[2] + "\t" + qn[3]);
  //Serial.println((int)(deltat * 1000000));

  //Serial.println((String)(ax_c * INT_MULTIPLIER) + "\t" + (String)(ay_c * INT_MULTIPLIER) + "\t" + (String)(az_c * INT_MULTIPLIER));
  //Serial.println((String)ay + "\t" + ay_c + "\t" + ay_avg);

  #endif
}

// инициализация MPU
int MPU_init(){

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS) {
    #ifdef DEBUG
    Serial.println("Error init IMU");
    #endif
    #ifdef OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Error init IMU");
    display.display();
    #endif
    blinkLED(3, 300, 300);
    return INV_ERROR;
  }

  // запрещаем генерацию прерываний 
  imu.enableInterrupt(0);

  // будем использовать все сенсоры
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  // частота выборки для акселя и гиро
  imu.setSampleRate(IMU_SAMPLE_RATE); 
  // для мага
  imu.setCompassSampleRate(IMU_SAMPLE_RATE);

  // частота ФНЧ для акселя и гиро
  imu.setLPF(IMU_LPF); 

  // масштаб значений для гиро
  imu.setGyroFSR(GYRO_FSR);
  // для акселя
  imu.setAccelFSR(ACCEL_FSR);

  // установленные значения LPF и SampleRate
  #ifdef DEBUG
  Serial.print("IMU LPF: ");
  Serial.println(imu.getLPF());
  Serial.print("IMU sample rate: ");
  Serial.println(imu.getSampleRate());
  Serial.print("Accel FSR: ");
  Serial.println(imu.getAccelFSR());
  Serial.print("Gyro FSR: ");
  Serial.println(imu.getGyroFSR());
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
