// Управление стоп-сигналом для велосипеда по значениям ускорения от акселерометра при торможении
// Плата - XIAO SAMD21
// IMU - Invensense MPU9250

#include <SparkFunMPU9250-DMP.h>
#include <ArduinoLowPower.h>
#include "quaternionFilters.h"
#include "helper_3dmath.h"

#define INTERRUPT_PIN 3   // pin для прерываний от MPU
#define STOP_LED_PIN 2    // pin для управления ключом стоп-сигнала

#define OLED 1 // использовать i2c OLED дисплей для вывода сообщений
#ifdef OLED
  #include <Adafruit_GFX.h>
  // выбор типа дисплея
  #define OLED_TYPE SSD1306  
  #if OLED_TYPE == SSD1306
    #include <Adafruit_SSD1306.h>
    #define OLED_ADDRESS 0x3c  // i2с адрес
    #define OLED_WIDTH 128   // разрешение экрана
    #define OLED_HEIGHT 64   
    #define OLED_RESET -1      //   QT-PY / XIAO
    Adafruit_SSD1306 display = Adafruit_SSD1306(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
  #elif
    #include <Adafruit_SH110X.h>
    #define OLED_ADDRESS 0x3c  // i2с адрес
    #define OLED_WIDTH 128   // разрешение экрана
    #define OLED_HEIGHT 64   
    #define OLED_RESET -1      //   QT-PY / XIAO
    Adafruit_SH1106G display = Adafruit_SH1106G(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
  #endif
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
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float mx = 0, my = 0, mz = 0;
// значения от акселя после компенсации гравитации
float ax_c = 0, ay_c = 0, az_c = 0;

//uint32_t delt_t = 0;                           // used to control display output rate
uint32_t count = 0, sumCount = 0;                // used to control display output rate
float deltat = 0.0f, sum = 0.0f;                 // integration interval for both filter schemes
uint32_t lastUpdate = 0;      // used to calculate integration interval
uint32_t Now = 0; // used to calculate integration interval

float *qn;

// фильтр Баттерворта 3-го порядка
// y(n) = b0*x(n) + b1*x(n-1) + b2*x(n-2) + b3*x(n-3) - a1*y(n-1) - a2*y(n-2) - a3*y(n-3))
// Расчет: https://www.meme.net.au/butterworth.html

// фильтр для ускорения
#define ACCEL_FILTER_COEF_COUNT 4
// Частота выборки: 100Hz, частота среза: 1Hz (3 dB)
//float accell_filter_coef_b[ACCEL_FILTER_COEF_COUNT] = {0.00002915, 0.00008744, 0.00008744, 0.00002915};
//float accell_filter_coef_a[ACCEL_FILTER_COEF_COUNT] = {1.0000, -2.8744, 2.7565, -0.8819};
// Частота выборки: 100Hz, частота среза: 2Hz (3 dB)
//float accell_filter_coef_b[ACCEL_FILTER_COEF_COUNT] = {0.00021961, 0.00065882, 0.00065882, 0.00021961};
//float accell_filter_coef_a[ACCEL_FILTER_COEF_COUNT] = {1.0000, -2.7488, 2.5282, -0.7776};
// Частота выборки: 100Hz, частота среза: 4Hz (3 dB)
float accell_filter_coef_b[ACCEL_FILTER_COEF_COUNT] = {0.0015669, 0.00470072, 0.00470072, 0.0015669};
float accell_filter_coef_a[ACCEL_FILTER_COEF_COUNT] = {1.0000, -2.498444, 2.115114, -0.6040692};

float accell_filter_input[ACCEL_FILTER_COEF_COUNT] = {0, 0, 0, 0};
float accell_filter_output[ACCEL_FILTER_COEF_COUNT] = {0, 0, 0, 0};

// фильтр для компенсации склонов/подъемов
// Частота выборки: 100Hz, частота среза: 0.1Hz (3 dB)
#define GRAVITY_FILTER_COEF_COUNT 3
float gravity_filter_coef_b[GRAVITY_FILTER_COEF_COUNT] = {9.8259e-06, 1.9652e-05, 9.8259e-06};
float gravity_filter_coef_a[GRAVITY_FILTER_COEF_COUNT] = {1.00000, -1.99111, 0.99115};
// Частота выборки: 100Hz, частота среза: 1Hz (3 dB)
//#define GRAVITY_FILTER_COEF_COUNT 4
//float gravity_filter_coef_b[GRAVITY_FILTER_COEF_COUNT] = {0.00002915, 0.00008744, 0.00008744, 0.00002915};
//float gravity_filter_coef_a[GRAVITY_FILTER_COEF_COUNT] = {1.0000, -2.8744, 2.7565, -0.8819};

float gravity_filter_input[GRAVITY_FILTER_COEF_COUNT] = {0, 0, 0};
float gravity_filter_output[GRAVITY_FILTER_COEF_COUNT] = {0, 0, 0};

MPU9250_DMP imu;

void setup() 
{
  
  #ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Starting");
  #endif

  // режимы pin
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(STOP_LED_PIN, OUTPUT);

  // OLED дисплей
  #ifdef OLED
  delay(250); 
  #if OLED_TYPE == SSD1306
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  display.setTextSize(4); 
  display.setTextColor(SSD1306_WHITE);
  #elif
  display.begin(OLED_ADDRESS, true);
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  #endif
  display.clearDisplay();
  display.println("Starting");
  display.display();
  #endif

  delay(3000);

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      #ifdef DEBUG
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println();
      #endif

      #ifdef OLED
      display.setCursor(0, 0);
      display.println("Unable to communicate with MPU-9250");
      display.display();
      #endif

      delay(5000);
    }
  }

  // запрещаем генерацию прерываний 
  imu.enableInterrupt(0);

  // Use setSensors to turn on or off MPU-9250 sensors.
  // Any of the following defines can be combined:
  // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
  // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
  // Enable all sensors:
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  // Use setGyroFSR() and setAccelFSR() to configure the
  // gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(1000); // Set gyro to 1000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(4); // Set accel to +/-4g
  // Note: the MPU-9250's magnetometer FSR is set at 
  // +/- 4912 uT (micro-tesla's)

  // setLPF() can be used to set the digital low-pass filter
  // of the accelerometer and gyroscope.
  // Can be any of the following: 188, 98, 42, 20, 10, 5
  // (values are in Hz).
  imu.setLPF(50); // Set LPF corner frequency to 50Hz

  // The sample rate of the accel/gyro can be set using
  // setSampleRate. Acceptable values range from 4Hz to 1kHz
  imu.setSampleRate(100); // Set sample rate to 100Hz

  // Likewise, the compass (magnetometer) sample rate can be
  // set using the setCompassSampleRate() function.
  // This value can range between: 1-100Hz
  imu.setCompassSampleRate(100); // Set mag rate to 100Hz

  // разрешаем генерацию прерываний
  imu.enableInterrupt(0);

  // подвешиваем обработчик по спадающему фронту
  LowPower.attachInterruptWakeup(INTERRUPT_PIN, dummy, FALLING);

}

void loop() 
{

  // ждем появления новых данных в FIFO
  while(!imu.dataReady()); 

  // Call update() to update the imu objects sensor data.
  // You can specify which sensors to update by combining
  // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
  // UPDATE_TEMPERATURE.
  // (The update function defaults to accel, gyro, compass,
  //  so you don't have to specify these values.)
  if (imu.update(UPDATE_ACCEL | UPDATE_GYRO /*| UPDATE_COMPASS */) != INV_SUCCESS) {
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
  ax = imu.calcAccel(imu.ax);
  ay = imu.calcAccel(imu.ay); 
  az = imu.calcAccel(imu.az);

  // получаем значения от гиро
  gx = imu.calcGyro(imu.gx);
  gy = imu.calcGyro(imu.gy); 
  gz = imu.calcGyro(imu.gz);
  
  // получаем значения от мага
  mx = imu.calcMag(imu.mx);
  my = imu.calcMag(imu.my); 
  mz = imu.calcMag(imu.mz);

  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  sum += deltat; // sum for averaging filter update rate
  sumCount++;
/*
  // вычисляем кватернион положения относительно Земли
  MahonyQuaternionUpdate(
  ax,                ay,                  az,
  gx * DEG_TO_RAD,   gy * DEG_TO_RAD,     gz * DEG_TO_RAD,
  my,                mx,                 -mz,
  deltat);
*/
/*
  MadgwickQuaternionUpdate(ax,                ay,                  az,
  gx * DEG_TO_RAD,   gy * DEG_TO_RAD,     gz * DEG_TO_RAD,
  my,                mx,                 -mz,
  deltat);
  */
  /*
  qn = (float *)getQ();
  Quaternion q1(qn[0], qn[1], qn[2], qn[3]);       // кватернион
  Quaternion q1_(qn[0], -qn[1], -qn[2], -qn[3]);   // обратный кватернион

  // компенсируем гравитацию для акселя
  VectorFloat v_accel(ax, ay, az);     // вектор для акселя
  v_accel = v_accel.getRotated(&q1);   // поворачиваем его в систему координат Земли
  v_accel.z -= 1;                      // вычитаем гравитацию из значения на оси Z
  v_accel = v_accel.getRotated(&q1_);  // поворачиваем обратно в систему координат вело

  // значения для акселя без влияния гравитации
  ax_c = v_accel.x;
  ay_c = v_accel.y;
  az_c = v_accel.z;
*/
  // отфильтрованное значение ускорения по оси Y
  // вариант с фильтром по скользящему среднему
  float ay_avg = (ay_avg * 3.0 + ay_c) / 4.0;

/*
  // вариант с фильтром Баттерворта
  // сдвигаем сэмплы
  for (int i = ACCEL_FILTER_COEF_COUNT - 1; i > 0; i--) {
    accell_filter_input[i] = accell_filter_input[i - 1];
    accell_filter_output[i] = accell_filter_output[i - 1];
  }
  accell_filter_input[0] = ay_c;
  //accell_filter_input[0] = ay;
  
  // вычисляем текущее значение
  accell_filter_output[0] = accell_filter_coef_b[0] * accell_filter_input[0]
                          + accell_filter_coef_b[1] * accell_filter_input[1]
                          + accell_filter_coef_b[2] * accell_filter_input[2]
                          + accell_filter_coef_b[3] * accell_filter_input[3]
                          - accell_filter_coef_a[1] * accell_filter_output[1]
                          - accell_filter_coef_a[2] * accell_filter_output[2]
                          - accell_filter_coef_a[3] * accell_filter_output[3];
  float ay_avg = accell_filter_output[0];
*/
/*
  // сдвигаем сэмплы
  for (int i = GRAVITY_FILTER_COEF_COUNT - 1; i > 0; i--) {
    gravity_filter_input[i] = gravity_filter_input[i - 1];
    gravity_filter_output[i] = gravity_filter_output[i - 1];
  }
  // accell_filter_input[0] = ay_c;
  gravity_filter_input[0] = ay;
  
  // вычисляем текущее значение
  gravity_filter_output[0] = gravity_filter_coef_b[0] * gravity_filter_input[0]
                           + gravity_filter_coef_b[1] * gravity_filter_input[1]
                           + gravity_filter_coef_b[2] * gravity_filter_input[2]
                           - gravity_filter_coef_a[1] * gravity_filter_output[1]
                           - gravity_filter_coef_a[2] * gravity_filter_output[2];

  // компенсируем склоны/подъемы
  ay_avg -= gravity_filter_output[0];
*/ 

  // пороги включения и выключения стоп-сигнала
  float accel_brake_upper_threshold = -0.2 * analogRead(A0) / 1023.0;
  float accel_brake_lower_threshold = accel_brake_upper_threshold * 0.7;
  
  #ifdef DEBUG
  //Serial.print((String)ax + "\t" + ay + "\t" + az);
  //Serial.print("\t");
  // Serial.println((String)gx + "\t" + gy + "\t" + gz);
  // Serial.println((String)mx + "\t" + my + "\t" + mz);
  Serial.println((String)qn[0] + "\t" + qn[1] + "\t" + qn[2] + "\t" + qn[3] + "\t" + deltat);

  //Serial.println((String)ax_c + "\t" + ay_c + "\t" + az_c);
  //Serial.println((String)ay + "\t" + ay_c + "\t" + ay_avg);

/*
  Serial.print("X: \t");
  Serial.print(accell_filter_input[0], 10);
  Serial.print("\t");
  Serial.print("Y: \t");
  Serial.print(accell_filter_output[0], 10);
  Serial.print("\t");
  Serial.print(accell_filter_output[1], 10);
  Serial.print("\t");
  Serial.print(accell_filter_output[2], 10);
  Serial.print("\t");
  Serial.println(accell_filter_output[3], 10);
*/
  #endif
  
  #ifdef OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println((String)((ay_avg < 0)?"-":"+") + (String)abs(ay_avg));
  display.println(accel_brake_upper_threshold);
  display.display();
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

  // проверяем, есть ли активность на акселе
  if (abs(ax) >= INACTIVITY_ACCEL_THRESHOLD || abs(ay) >= INACTIVITY_ACCEL_THRESHOLD || abs(az) >= INACTIVITY_ACCEL_THRESHOLD) {
    // активность есть, запоминаем отметку времени
    accel_last_activity_ts = millis();
  }
  else {
    // активности нет

    if ((millis() - accel_last_activity_ts) > INACTIVITY_BEFORE_SLEEP) {
      // истек период макс.неактивности
    
      // переводим MPU в режим Wake On Motion с указанием:
      // порога ускорения для просыпания, 
      // продолжительности применения ускорения в мс,
      // частоты выборок 
      if (mpu_lp_motion_interrupt(ACCEL_WAKE_ON_THRESHOLD, ACCEL_WAKE_ON_DURATION, ACCEL_WAKE_ON_FREQ) != INV_SUCCESS) {
        // ошибка
        #ifdef DEBUG
        Serial.println(F("IMU LP W-o-M mode failed"));
        #endif
        #ifdef OLED
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("IMU LP W-o-M mode failed");
        display.display();
        #endif
        blinkLED(1, 1000, 500);
      }

      // сигнализируем о переходе в режим сна
      #ifdef DEBUG
      Serial.println("Going to sleep mode");
      #endif
      #ifdef OLED
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Sleep mode");
      display.display();
      #endif
      blinkLED(1, 500, 500);
      
      // переводим CPU в режим сна
      LowPower.sleep();

      // сигнализируем о выходе из режима сна
      blinkLED(1, 500, 500);

      // перезапускаем MPU в обычном режиме 
      if (imu.begin() != INV_SUCCESS) { 
        Serial.println("Error reinit IMU");
        digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (LOW is the voltage level)
        #ifdef OLED
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Error reinit IMU");
        display.display();
        #endif
        blinkLED(3, 500, 500);
      }

      // запрещаем генерацию прерываний
      imu.enableInterrupt(0);
      
      imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

      // Use setGyroFSR() and setAccelFSR() to configure the
      // gyroscope and accelerometer full scale ranges.
      // Gyro options are +/- 250, 500, 1000, or 2000 dps
      imu.setGyroFSR(1000); // Set gyro to 1000 dps
      // Accel options are +/- 2, 4, 8, or 16 g
      imu.setAccelFSR(4); // Set accel to +/-4g
      // Note: the MPU-9250's magnetometer FSR is set at 
      // +/- 4912 uT (micro-tesla's)
      
      // setLPF() can be used to set the digital low-pass filter
      // of the accelerometer and gyroscope.
      // Can be any of the following: 188, 98, 42, 20, 10, 5
      // (values are in Hz).
      imu.setLPF(50); // Set LPF corner frequency to 50Hz
      
      // The sample rate of the accel/gyro can be set using
      // setSampleRate. Acceptable values range from 4Hz to 1kHz
      imu.setSampleRate(100); // Set sample rate to 100Hz
      
      // Likewise, the compass (magnetometer) sample rate can be
      // set using the setCompassSampleRate() function.
      // This value can range between: 1-100Hz
      imu.setCompassSampleRate(100); // Set mag rate to 100Hz
    }
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
