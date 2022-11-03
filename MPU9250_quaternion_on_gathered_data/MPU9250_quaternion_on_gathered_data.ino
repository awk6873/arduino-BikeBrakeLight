// Управление стоп-сигналом для велосипеда по значениям ускорения от акселерометра при торможении
// Эмуляция MPU на массиве собранных данных от акселя, гиро и мага 
// Плата - XIAO SAMD21
// IMU - Invensense MPU9250

#include <SparkFunMPU9250-DMP.h>
#include "quaternionFilters.h"
#include "helper_3dmath.h"
#include <SD.h>
#include <SPI.h>

const int chipSelectPin = 3;

#define STOP_LED_PIN 2    // pin для управления ключом стоп-сигнала

#define OLED 1            // использовать i2c OLED дисплей SSD1306 для вывода сообщений
#ifdef OLED
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #define OLED_ADDRESS 0x3c  // i2с адрес
  #define OLED_WIDTH 128     // разрешение экрана
  #define OLED_HEIGHT 64   
  #define OLED_RESET -1      //   QT-PY / XIAO
  Adafruit_SSD1306 display = Adafruit_SSD1306(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
#endif

#define DEBUG 1  // использовать Serial для вывода сообщений

// порог значений на осях акселя для определения неактивности, G
#define INACTIVITY_ACCEL_THRESHOLD 0.1
// продолжительность неактивности от акселя перед переходом в режим SLEEP, мс
#define INACTIVITY_BEFORE_SLEEP 60000
// отметка времени последней активности от акселя
uint32_t accel_last_activity_ts;
// порог ускорений для пробуждения акселя, mG
#define ACCEL_WAKE_ON_THRESHOLD 100
// продолжительность применения ускорений, мс
#define ACCEL_WAKE_ON_DURATION 1
// частота выборки для акселя в режиме Low Power
#define ACCEL_WAKE_ON_FREQ 2    // 2 - 1.95 Гц, 4 - 3.91 Гц ?

// текущие значения от акселя, гиро и мага
int imu_ax = 0, imu_ay = 0, imu_az = 0;
int imu_gx = 0, imu_gy = 0, imu_gz = 0;
int imu_mx = 0, imu_my = 0, imu_mz = 0;

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float mx = 0, my = 0, mz = 0;
// значения от акселя после компенсации гравитации
float ax_c = 0, ay_c = 0, az_c = 0;
// усредненное значение
float ay_avg = 0;

// смещения для калибровки акселя
int ax_offset = -100;
int ay_offset = 70;
int az_offset = -125;
int ax_cal_avg = 0;
int ay_cal_avg = 0;
int az_cal_avg = 0;

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
uint32_t deltat_orig;
uint32_t lastUpdate = 0;      // used to calculate integration interval
uint32_t Now = 0; // used to calculate integration interval

float *qn;   //[4];
char qn_str[4][10];

MPU9250_DMP imu;
File f;
char row[200];

void setup() 
{
  
  #ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Starting");
  #endif

  // режимы pin
  pinMode(STOP_LED_PIN, OUTPUT);

  // OLED дисплей SSD1306
  #ifdef OLED
  delay(250); 
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  display.setTextSize(4); 
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.println("Starting");
  display.display();
  #endif

  delay(10000);

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS) {
    #ifdef DEBUG
    Serial.println("Unable to communicate with MPU-9250");
    #endif
    blinkLED(3, 500, 500);
    while(1);
  }

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

  // будем читать данные из файла с SD карты
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

  f = SD.open("MPU_DMP.TXT");
  
}

void loop() 
{

  // если добрались до конца файла, стоп
  while(!f.available());

  for (int i = 0; i < sizeof(row); i++) {
    row[i] = f.read();
    if (row[i] == '\r') {
      // считываем символ перевода строки
      f.read();
      row[i + 1] = '\0';
      break;
    }
  }
  //Serial.println(String(row));
  sscanf(row, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", 
              &imu_ax, &imu_ay, &imu_az, &imu_gx, &imu_gy, &imu_gz, &imu_mx, &imu_my, &imu_mz, 
              /*qn_str[0], qn_str[1], qn_str[2], qn_str[3], */ &deltat_orig);
  delayMicroseconds(deltat_orig);
 
  // получаем значения от акселя
  ax = imu.calcAccel(imu_ax);
  ay = imu.calcAccel(imu_ay); 
  az = imu.calcAccel(imu_az);

  // получаем значения от гиро
  gx = imu.calcGyro(imu_gx);
  gy = imu.calcGyro(imu_gy); 
  gz = imu.calcGyro(imu_gz);
  
  // получаем значения от мага
  mx = imu.calcMag(imu_mx);
  my = imu.calcMag(imu_my); 
  mz = imu.calcMag(imu_mz);

  // получаем значения для кватерниона
//  qn[0] = atof(qn_str[0]);
//  qn[1] = atof(qn_str[1]);
//  qn[2] = atof(qn_str[2]);
//  qn[3] = atof(qn_str[3]);
  
  deltat = deltat_orig / 1000000.0f; // set integration time by time elapsed since last filter update
  sum += deltat; // sum for averaging filter update rate
  sumCount++;

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
  v_acc = v_acc.getRotated(&q1);   // поворачиваем его в систему координат Земли
  v_acc.z -= 1;                      // вычитаем гравитацию из значения на оси Z
  v_acc = v_acc.getRotated(&q1_);  // поворачиваем обратно в систему координат вело

  // значения для акселя без влияния гравитации
  ax_c = v_acc.x;
  ay_c = v_acc.y;
  az_c = v_acc.z; 

  // пороги включения и выключения стоп-сигнала
  //float accel_brake_upper_threshold = -0.2 * analogRead(A0) / 1023.0;
  //float accel_brake_lower_threshold = accel_brake_upper_threshold * 0.7;
  float accel_brake_upper_threshold = -0.05;
  float accel_brake_lower_threshold = -0.03;

  // параметр для фильтра Mahony
  //float Kp = 120 * analogRead(A0) / 1023.0;
  //setMahonyKp(Kp);

  // параметр для фильтра Madgwick
  float beta = 5.0 * analogRead(A0) / 1023.0;
  setMadgwickBeta(beta);
  
  #ifdef DEBUG
  //Serial.println((String)imu.ax + "\t" + imu.ay + "\t" +imu.az + "\t");
  //Serial.println((String)ax_cal_avg + "\t" + ay_cal_avg + "\t" + az_cal_avg);
  //Serial.println((String)(imu.ax - ax_offset) + "\t" + (imu.ay - ay_offset)+ "\t" + (imu.az - az_offset));
  //Serial.print((String)imu.gx + "\t" + imu.gy + "\t" +imu.gz + "\t");
  //Serial.println((String)imu.mx + "\t" + imu.my + "\t" +imu.mz);
  //Serial.print((String)imu_ax + "\t" + imu_ay + "\t" + imu_az + "\t");
  //Serial.print((String)imu_gx + "\t" + imu_gy + "\t" + imu_gz + "\t");
  //Serial.print((String)imu_mx + "\t" + imu_my + "\t" + imu_mz + "\t");
  //Serial.println((String)deltat_orig + "\t" + deltat);
  //Serial.println((String)qn_str[0] + "\t" + qn_str[1] + "\t" + qn_str[2] + "\t" + qn_str[3]);
  //Serial.print((String)qn[0] + "\t" + qn[1] + "\t" + qn[2] + "\t" + qn[3] + "\t");

  //Serial.println((String)ax + "\t" + ay + "\t" + az);
  Serial.println((String)ay + "\t" + ay_c + "\t" + ay_avg);

/*
  Serial.print("X: \t");
  Serial.print(accel_filter_input[0], 10);
  Serial.print("\t");
  Serial.print("Y: \t");
  Serial.print(accel_filter_output[0], 10);
  Serial.print("\t");
  Serial.print(accel_filter_output[1], 10);
  Serial.print("\t");
  Serial.print(accel_filter_output[2], 10);
  Serial.print("\t");
  Serial.println(accel_filter_output[3], 10);
*/
  #endif
  
  #ifdef OLED
  if (sumCount % 10 == 0) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println((String)((ay_avg < 0)?"-":"+") + (String)abs(ay_avg));
    display.println(beta);
    display.display();
  }
  #endif

  if (ay_avg <= accel_brake_upper_threshold) {  
    digitalWrite(LED_BUILTIN, LOW);     // включаем св.диод на плате
    digitalWrite(STOP_LED_PIN, HIGH);   // и стоп-сигнал 
  }
  else 
    if (ay_avg >= accel_brake_lower_threshold) {
      digitalWrite(LED_BUILTIN, HIGH);     // вЫключаем св.диод на плате
      digitalWrite(STOP_LED_PIN, LOW);     // и стоп-сигнал      
    }

}

// заглушка обработчика прерываний от MPU
void dummy() {
}

// мигание светодиодом и стоп-сигналом
void blinkLED(int count, int delay_on, int delay_off) {

  for (int i = 0; i < count; i++) {
    digitalWrite(LED_BUILTIN, LOW);   // включаем
    digitalWrite(STOP_LED_PIN, HIGH);
    delay(delay_on);
    digitalWrite(LED_BUILTIN, HIGH);  // выключаем
    digitalWrite(STOP_LED_PIN, LOW);
    delay(delay_off);
  }
}
