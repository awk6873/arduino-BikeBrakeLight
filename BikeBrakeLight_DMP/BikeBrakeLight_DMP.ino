// Управление стоп-сигналом для велосипеда по значениям ускорения от акселерометра при торможении
// Плата - XIAO SAMD21
// IMU - Invensense MPU9250

#include <SparkFunMPU9250-DMP.h>
#include <ArduinoLowPower.h>
#include "quaternionFilters.h"
#include "helper_3dmath.h"

#define INTERRUPT_PIN 3   // pin для прерываний от MPU
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
// частота выборки для акселя в режиме Low Power, Гц
#define ACCEL_WAKE_ON_FREQ 2    // 2 - 1.95 Гц, 4 - 3.91 Гц ?
// частота выборки для режима DMP, Гц
#define DMP_FIFO_RATE 50

// текущие значения от акселя, гиро и мага
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

uint32_t cycle_count = 0;                    // used to control display output rate
float delta_t = 0.0f, delta_t_avg = 0.0f;    // integration interval for both filter schemes
uint32_t last_update_t = 0;                  // used to calculate integration interval
uint32_t Now = 0;                            

float qn[4];

// фильтр Баттерворта 3-го порядка
// y(n) = b0*x(n) + b1*x(n-1) + b2*x(n-2) + b3*x(n-3) - a1*y(n-1) - a2*y(n-2) - a3*y(n-3))
// Расчет: https://www.meme.net.au/butterworth.html

// ФНЧ для ускорения
#define ACCEL_FILTER_COEF_COUNT 4
// Частота выборки: 100Hz, частота среза: 1Hz (3 dB)
//float accel_filter_coef_b[ACCEL_FILTER_COEF_COUNT] = {2.91464947e-5, 8.74394842e-5, 8.74394842e-5, 2.91464947e-5};
//float accel_filter_coef_a[ACCEL_FILTER_COEF_COUNT] = {1.0000, -2.87435692, 2.75648322, -0.88189312};
// Частота выборки: 100Hz, частота среза: 2Hz (3 dB)
//float accel_filter_coef_b[ACCEL_FILTER_COEF_COUNT] = {0.00021961, 0.00065882, 0.00065882, 0.00021961};
//float accel_filter_coef_a[ACCEL_FILTER_COEF_COUNT] = {1.0000, -2.74883592, 2.52823137, -0.77763860};
// Частота выборки: 100Hz, частота среза: 4Hz (3 dB)
//float accel_filter_coef_b[ACCEL_FILTER_COEF_COUNT] = {0.0015669, 0.00470072, 0.00470072, 0.0015669};
//float accel_filter_coef_a[ACCEL_FILTER_COEF_COUNT] = {1.0000, -2.498444, 2.115114, -0.6040692};

// Частота выборки: 20Hz, частота среза: 1Hz (3 dB)
//float accel_filter_coef_b[ACCEL_FILTER_COEF_COUNT] = {0.00289855, 0.00869565, 0.00869565,  0.00289855};
//float accel_filter_coef_a[ACCEL_FILTER_COEF_COUNT] = {1.0,       -2.37391304, 1.92753623, -0.53043478};
// Частота выборки: 20Hz, частота среза: 2Hz (3 dB)
//float accel_filter_coef_b[ACCEL_FILTER_COEF_COUNT] = {0.01818181, 0.05454545, 0.05454545,  0.01818181};
//float accel_filter_coef_a[ACCEL_FILTER_COEF_COUNT] = {1.0,       -1.76363636, 1.18181818, -0.27272727};

// Частота выборки: 10Hz, частота среза: 2Hz (3 dB)
//float accel_filter_coef_b[ACCEL_FILTER_COEF_COUNT] = {0.09853188, 0.29559563, 0.29559563,  0.09853188};
//float accel_filter_coef_a[ACCEL_FILTER_COEF_COUNT] = {1.0,       -0.57719972, 0.42181496, -0.05626170};
// Частота выборки: 10Hz, частота среза: 1Hz (3 dB)
float accel_filter_coef_b[ACCEL_FILTER_COEF_COUNT] = {0.01809889, 0.05429668, 0.05429668,  0.01809889};
float accel_filter_coef_a[ACCEL_FILTER_COEF_COUNT] = {1.0,       -1.76004488, 1.18288931, -0.27805328};

float ay_filter_input[ACCEL_FILTER_COEF_COUNT] = {0, 0, 0, 0};
float ay_filter_output[ACCEL_FILTER_COEF_COUNT] = {0, 0, 0, 0};

// ФВЧ для ускорения
#define ACCEL_HPF_FILTER_COEF_COUNT 4
// Частота выборки: 20Нz, частота среза 0.5Hz (3dB)
float accel_hpf_filter_coef_b[ACCEL_HPF_FILTER_COEF_COUNT] = {0.85470085, -2.56410256, 2.56410256, -0.85470085};
float accel_hpf_filter_coef_a[ACCEL_HPF_FILTER_COEF_COUNT] = {1.0,        -2.68717948, 2.42051282, -0.72991452};
// Частота выборки: 100Нz, частота среза 0.5Hz (3dB)
//float accel_hpf_filter_coef_b[ACCEL_HPF_FILTER_COEF_COUNT] = {0.96899225, -2.90697674, 2.90697674, -0.96899225};
//float accel_hpf_filter_coef_a[ACCEL_HPF_FILTER_COEF_COUNT] = {1.0,        -2.93701550, 2.87596899, -0.93895349};
// Частота выборки: 100Нz, частота среза 1Hz (3dB)
//float accel_hpf_filter_coef_b[ACCEL_HPF_FILTER_COEF_COUNT] = {0.93896714, -2.81690141, 2.81690141, -0.93896713};
//float accel_hpf_filter_coef_a[ACCEL_HPF_FILTER_COEF_COUNT] = {1.0,        -2.87417840, 2.75586854, -0.88169014};

float ay_hpf_filter_input[ACCEL_HPF_FILTER_COEF_COUNT] = {0, 0, 0, 0};
float ay_hpf_filter_output[ACCEL_HPF_FILTER_COEF_COUNT] = {0, 0, 0, 0};

// ФНЧ для компенсации склонов/подъемов
// Частота выборки: 100Hz, частота среза: 0.1Hz (3 dB)
//#define GRAVITY_FILTER_COEF_COUNT 3
//float gravity_filter_coef_b[GRAVITY_FILTER_COEF_COUNT] = {9.8259e-06, 1.9652e-05, 9.8259e-06};
//float gravity_filter_coef_a[GRAVITY_FILTER_COEF_COUNT] = {1.00000, -1.99111, 0.99115};
// Частота выборки: 100Hz, частота среза: 1Hz (3 dB)
//#define GRAVITY_FILTER_COEF_COUNT 4
//float gravity_filter_coef_b[GRAVITY_FILTER_COEF_COUNT] = {0.00002915, 0.00008744, 0.00008744, 0.00002915};
//float gravity_filter_coef_a[GRAVITY_FILTER_COEF_COUNT] = {1.0000, -2.8744, 2.7565, -0.8819};

// Частота выборки: 20Hz, частота среза: 0.5Hz (3 dB)
//#define GRAVITY_FILTER_COEF_COUNT 4
//float gravity_filter_coef_b[GRAVITY_FILTER_COEF_COUNT] = {4.16666666e-4, 0.00125, 0.00125, 4.16666666e-4};
//float gravity_filter_coef_a[GRAVITY_FILTER_COEF_COUNT] = {1.0, -2.68666666, 2.42, -0.73};

//float ay_g_filter_input[GRAVITY_FILTER_COEF_COUNT] = {0, 0, 0, 0};
//float ay_g_filter_output[GRAVITY_FILTER_COEF_COUNT] = {0, 0, 0, 0};
//float az_g_filter_input[GRAVITY_FILTER_COEF_COUNT] = {0, 0, 0, 0};
//float az_g_filter_output[GRAVITY_FILTER_COEF_COUNT] = {0, 0, 0, 0};

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

  delay(3000);

  // запускаем MPU
  MPU_init();

  // генерация прерываний спадающим импульсом
  imu.setIntLevel(INT_ACTIVE_LOW);
  imu.setIntLatched(INT_50US_PULSE);
  imu.enableInterrupt(1);

  // подвешиваем обработчик по спадающему фронту
  LowPower.attachInterruptWakeup(INTERRUPT_PIN, dummy, FALLING);

}

void loop() 
{

  // ждем появления новых данных в FIFO
  while(!imu.fifoAvailable()); 

  // Call update() to update the imu objects sensor data.
  // You can specify which sensors to update by combining
  // UPDATE_ACC, UPDATE_GYRO, UPDATE_COMPASS, and/or
  // UPDATE_TEMPERATURE.
  // (The update function defaults to acc, gyro, compass,
  //  so you don't have to specify these values.)
  if (imu.dmpUpdateFifo() != INV_SUCCESS) {
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
  // средние значения для калибровки
  //ax_cal_avg = (ax_cal_avg * 99 + imu.ax - ax_offset) / 100;
  //ay_cal_avg = (ay_cal_avg * 99 + imu.ay - ay_offset) / 100;
  //az_cal_avg = (az_cal_avg * 99 + imu.az - az_offset) / 100;

  // получаем кватернион
  qn[0] = imu.calcQuat(imu.qw);
  qn[1] = imu.calcQuat(imu.qx);
  qn[2] = imu.calcQuat(imu.qy);
  qn[3] = imu.calcQuat(imu.qz);

  // вычисляем дельту t
  Now = micros();
  delta_t = ((Now - last_update_t) / 1000000.0f);
  if (delta_t < 0)
    // при переполнения micros() используем среднее значение дельты t 
    delta_t = delta_t_avg;
  last_update_t = Now;
  delta_t_avg = (delta_t_avg + delta_t) / 2.0;
  cycle_count++;

  Quaternion q1(qn[0], qn[1], qn[2], qn[3]);       // кватернион
  Quaternion q1_(qn[0], -qn[1], -qn[2], -qn[3]);   // обратный кватернион

  // компенсируем гравитацию для акселя
  VectorFloat v_acc(ax, ay, az);     // вектор для акселя
  v_acc = v_acc.getRotated(&q1);     // поворачиваем его в систему координат Земли
  v_acc.z -= 1;                      // вычитаем гравитацию из значения на оси Z
  v_acc = v_acc.getRotated(&q1_);    // поворачиваем обратно в систему координат вело

  // значения для акселя без влияния гравитации
  ax_c = v_acc.x;
  ay_c = v_acc.y;
  az_c = v_acc.z;

  // отфильтрованное значение ускорения по оси Y
  // вариант с фильтром по скользящему среднему
  //float avg_coef = 50.0 * analogRead(A0) / 1023.0;
  //float avg_coef = 20;
  //ay_avg = (ay_avg * (avg_coef - 1) + ay_c) / avg_coef;
  // вариант с фильтром Баттерворта
  // ФНЧ
  ay_avg = butterworth_filter(ay_c, ACCEL_FILTER_COEF_COUNT, accel_filter_coef_b, accel_filter_coef_a, 
                              ay_filter_input, ay_filter_output);
  // ФВЧ
  //ay_avg = butterworth_filter(ay_avg, ACCEL_HPF_FILTER_COEF_COUNT, accel_hpf_filter_coef_b, accel_hpf_filter_coef_a, 
  //                            ay_hpf_filter_input, ay_hpf_filter_output); 

  // пороги включения и выключения стоп-сигнала
  //float accel_brake_upper_threshold = -0.2 * analogRead(A0) / 1023.0;
  //float accel_brake_lower_threshold = accel_brake_upper_threshold * 0.7;
  float accel_brake_upper_threshold = -0.05;
  float accel_brake_lower_threshold = -0.03;

  // параметр для фильтра Mahony
  //float Kp = 120 * analogRead(A0) / 1023.0;
  //setMahonyKp(Kp);

  // параметр для фильтра Madgwick
  //float beta = 5.0 * analogRead(A0) / 1023.0;
  //setMadgwickBeta(beta);
  
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
  Serial.print((String)qn[0] + "\t" + qn[1] + "\t" + qn[2] + "\t" + qn[3]  + "\t");

  Serial.println((String)ax_c + "\t" + ay_c + "\t" + az_c);
  //Serial.println((String)ay + "\t" + ay_c + "\t" + ay_avg);

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
  if (cycle_count % 5 == 0) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println((String)((ay_avg < 0)?"-":"+") + (String)abs(ay_avg));
    display.println((String)((ay_c < 0)?"-":"+") + (String)abs(ay_c));
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

  // проверяем, есть ли активность на акселе
  if (abs(ax_c) >= INACTIVITY_ACCEL_THRESHOLD || abs(ay_c) >= INACTIVITY_ACCEL_THRESHOLD || abs(az_c) >= INACTIVITY_ACCEL_THRESHOLD /*|| 1 == 1 */) {
    // активность есть, запоминаем отметку времени
    accel_last_activity_ts = millis();
  }
  else {
    // активности нет

    // вычисляем продолжительность неактивности
    float accel_inactivity_t = millis() - accel_last_activity_ts;
    if (accel_inactivity_t < 0) 
      // обработка переполнения millis()
      accel_inactivity_t = 0; 
      
    if (accel_inactivity_t > INACTIVITY_BEFORE_SLEEP) {
      // истек период макс.неактивности
    
      // переводим MPU в режим Wake On Motion с указанием:
      // порога ускорения для просыпания, 
      // продолжительности применения ускорения в мс,
      // частоты выборок 
      if (mpu_lp_motion_interrupt(ACCEL_WAKE_ON_THRESHOLD, ACCEL_WAKE_ON_DURATION, ACCEL_WAKE_ON_FREQ) != INV_SUCCESS) {
        // ошибка
        #ifdef DEBUG
        Serial.println("IMU LP WoM mode failed");
        #endif
        #ifdef OLED
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("IMU LP WoM mode failed");
        display.display();
        #endif
        blinkLED(3, 300, 300);
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
      #ifdef DEBUG
      Serial.println("Waking up");
      #endif
      #ifdef OLED
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Wake up");
      display.display();
      #endif
      blinkLED(2, 300, 200);

      // перезапускаем MPU
      MPU_init();

      // начинаем заново отсчет неактивности
      accel_last_activity_ts = millis();
    }
  }
}

// инициализация MPU
int MPU_init(){

  // перезапускаем MPU в обычном режиме 
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

  if (imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT |       // Enable 6-axis quat
                   DMP_FEATURE_GYRO_CAL |             // Use gyro calibration
                   DMP_FEATURE_SEND_CAL_GYRO |        // Send cal'd gyro values
                   DMP_FEATURE_SEND_RAW_ACCEL,
                   DMP_FIFO_RATE) != INV_SUCCESS) {   // Set DMP FIFO rate 
    #ifdef DEBUG
    Serial.println("Error DMP mode on");
    #endif
    #ifdef OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Error DMP mode on");
    display.display();
    #endif
    blinkLED(3, 300, 300);
    return INV_ERROR;
  }

  imu.setGyroFSR(1000); // Set gyro to 1000 dps
  imu.setAccelFSR(4);   // Set acc to +/-4g
  //imu.setLPF(5);
  //imu.setSampleRate(10);
  
  // установленные значения LPF и SampleRate
  #ifdef DEBUG
  Serial.println((String)imu.getLPF() + "\t" + imu.getSampleRate() + "\t" + imu.getCompassSampleRate());
  #endif
}

// фильтр Баттерворта
// y(n) = b0*x(n) + b1*x(n-1) + b2*x(n-2) + b3*x(n-3) - a1*y(n-1) - a2*y(n-2) - a3*y(n-3))
float butterworth_filter(float in, int n, float *coef_b, float *coef_a, float *ff_buf, float *fb_buf) {

  // сдвигаем сэмплы
  for (int i = n - 1; i > 0; i--) {
    ff_buf[i] = ff_buf[i - 1];
    fb_buf[i] = fb_buf[i - 1];
  }
  ff_buf[0] = in;
  fb_buf[0] = 0;

  // вычисляем выходное значение
  for (int i = 0; i < n; i++) {
    fb_buf[0] += ff_buf[i] * coef_b[i];
    if (i > 0) 
      fb_buf[0] -= fb_buf[i] * coef_a[i]; 
  }

  return fb_buf[0];
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
