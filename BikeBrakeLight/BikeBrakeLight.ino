// Управление стоп-сигналом для велосипеда по значениям ускорения от акселерометра при торможении
// Плата - XIAO SAMD21
// IMU - Invensense MPU9250

#include <SparkFunMPU9250-DMP.h>
//#include <util/arduino_mpu9250_i2c.h>
//#define i2c_write(a, b, c, d) arduino_i2c_write(a, b, c, d)
#include <ArduinoLowPower.h>
#include "quaternionFilters.h"
#include "helper_3dmath.h"
#include "neuton.h"

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

// частота ФНЧ для акселя и гиро (188, 98, 42, 20, 10, 5 Гц)
#define IMU_LPF 42
// частота выборки для акселя, гиро и мага (от 4 Гц to 1 кГц)
#define IMU_SAMPLE_RATE 75

// макс.диапазон значений (full scale range) для гиро +/- 250, 500, 1000, 2000 dps
#define GYRO_FSR 1000
// для акселя +/- 2, 4, 8, or 16 g
#define ACCEL_FSR 4
// для магнитометра - константа +/- 4912 uT (micro-tesla)

// порог включения стоп-сигнала (вероятность предсказания класса события от модели)
#define BRAKE_INFERENCE_THRESHOLD 0.2
// порог значений на осях акселя для определения неактивности, G
#define INACTIVITY_ACCEL_THRESHOLD 0.1
// продолжительность неактивности от акселя перед переходом в режим SLEEP, мс
#define INACTIVITY_BEFORE_SLEEP 600000
// отметка времени последней активности от акселя
uint32_t accel_last_activity_ts;
// порог ускорений для пробуждения акселя, mG
#define ACCEL_WAKE_ON_THRESHOLD 100
// продолжительность применения ускорений, мс
#define ACCEL_WAKE_ON_DURATION 1
// частота выборки для акселя в режиме Low Power
#define ACCEL_WAKE_ON_FREQ 2    // 2 - 1.95 Гц, 4 - 3.91 Гц ?

// размер окна для усреднения значения от датчика освещенности, кол-во сэмплов за 10 сек
#define LIGHT_AVG_WINDOW_SIZE 10 * IMU_SAMPLE_RATE

// текущие значения от акселя, гиро и мага
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float mx = 0, my = 0, mz = 0;
// значения от акселя после компенсации гравитации
float ax_c = 0, ay_c = 0, az_c = 0;
// отфильтрованные значения
float ax_avg = 0, ay_avg = 0, az_avg = 0;

// смещения для калибровки акселя
int ax_offset = -100;
int ay_offset = 70;
int az_offset = -125;

// смещения и коэфф.для калибровки мага
int mx_offset = 420;
int my_offset = 180;
int mz_offset = -180;
float mx_scale = 1.0;
float my_scale = 1.5;
float mz_scale = 0.80;

// ФВЧ для ускорения
#define ACCEL_HPF_FILTER_COEF_COUNT 4
// Частота выборки: 20Нz, частота среза 0.5Hz (3dB)
//float accel_hpf_filter_coef_b[ACCEL_HPF_FILTER_COEF_COUNT] = {0.85470085, -2.56410256, 2.56410256, -0.85470085};
//float accel_hpf_filter_coef_a[ACCEL_HPF_FILTER_COEF_COUNT] = {1.0,        -2.68717948, 2.42051282, -0.72991452};
// Частота выборки: 100Нz, частота среза 0.5Hz (3dB)
float accel_hpf_filter_coef_b[ACCEL_HPF_FILTER_COEF_COUNT] = {0.96899225, -2.90697674, 2.90697674, -0.96899225};
float accel_hpf_filter_coef_a[ACCEL_HPF_FILTER_COEF_COUNT] = {1.0,        -2.93701550, 2.87596899, -0.93895349};
// Частота выборки: 100Нz, частота среза 1Hz (3dB)
//float accel_hpf_filter_coef_b[ACCEL_HPF_FILTER_COEF_COUNT] = {0.93896714, -2.81690141, 2.81690141, -0.93896713};
//float accel_hpf_filter_coef_a[ACCEL_HPF_FILTER_COEF_COUNT] = {1.0,        -2.87417840, 2.75586854, -0.88169014};

float ax_hpf_filter_input[ACCEL_HPF_FILTER_COEF_COUNT] = {0, 0, 0, 0};
float ax_hpf_filter_output[ACCEL_HPF_FILTER_COEF_COUNT] = {0, 0, 0, 0};
float ay_hpf_filter_input[ACCEL_HPF_FILTER_COEF_COUNT] = {0, 0, 0, 0};
float ay_hpf_filter_output[ACCEL_HPF_FILTER_COEF_COUNT] = {0, 0, 0, 0};
float az_hpf_filter_input[ACCEL_HPF_FILTER_COEF_COUNT] = {0, 0, 0, 0};
float az_hpf_filter_output[ACCEL_HPF_FILTER_COEF_COUNT] = {0, 0, 0, 0};

uint32_t count = 0, sumCount = 0;  // used to control display output rate
float deltat = 0.0f, sum = 0.0f;   // integration interval for both filter schemes
uint32_t lastUpdate = 0;           // used to calculate integration interval
uint32_t Now = 0;                  // used to calculate integration interval

float *qn;

short int model_inputs[100][6]; 
int model_window_size;
int num_samples = 0;
uint16_t class_index;
float* model_outputs;

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

  #ifdef DEBUG
  delay(10000);
  #endif
  
  // свойства модели
  model_window_size = neuton_model_window_size();
  Serial.print("Model window size: ");
  Serial.println(model_window_size);

  // инициализация IMU
  MPU_init();

  // подвешиваем обработчик прерываний (по спадающему фронту)
  LowPower.attachInterruptWakeup(INTERRUPT_PIN, dummy, FALLING);
}

void loop() 
{

  // ждем появления новых данных в FIFO
  while(!imu.dataReady()); 

  // обновляем данные
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
  gx = imu.calcGyro(imu.gx);
  gy = imu.calcGyro(imu.gy); 
  gz = imu.calcGyro(imu.gz);
  
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
    ax,                -ay,                 -az,
    gx * DEG_TO_RAD,   -gy * DEG_TO_RAD,    -gz * DEG_TO_RAD,
    my,                -mx,                 mz,
    deltat
  );

  qn = (float *)getQ();
  Quaternion q1(qn[0], qn[1], qn[2], qn[3]);       // кватернион
  Quaternion q1_(qn[0], -qn[1], -qn[2], -qn[3]);   // обратный кватернион

  // компенсируем гравитацию для акселя
  VectorFloat v_acc(ax, ay, az);     // вектор для акселя
  v_acc = v_acc.getRotated(&q1);   // поворачиваем его в систему координат Земли
  v_acc.z -= 8192;                      // вычитаем гравитацию из значения на оси Z
  v_acc = v_acc.getRotated(&q1_);  // поворачиваем обратно в систему координат вело

  // значения для акселя без влияния гравитации
  ax_c = v_acc.x;
  ay_c = v_acc.y;
  az_c = v_acc.z;

/*
  // ФВЧ для акселя
  ax_avg = butterworth_filter(ax, ACCEL_HPF_FILTER_COEF_COUNT, accel_hpf_filter_coef_b, accel_hpf_filter_coef_a, 
                              ax_hpf_filter_input, ax_hpf_filter_output); 
  ay_avg = butterworth_filter(ay, ACCEL_HPF_FILTER_COEF_COUNT, accel_hpf_filter_coef_b, accel_hpf_filter_coef_a, 
                              ay_hpf_filter_input, ay_hpf_filter_output);
  az_avg = butterworth_filter(az, ACCEL_HPF_FILTER_COEF_COUNT, accel_hpf_filter_coef_b, accel_hpf_filter_coef_a, 
                              az_hpf_filter_input, az_hpf_filter_output);
*/

  // сдвигаем предыдущие значения
  for (int i = model_window_size - 1; i > 0; i--) {
    model_inputs[i][0] = model_inputs[i - 1][0];
    model_inputs[i][1] = model_inputs[i - 1][1];
    model_inputs[i][2] = model_inputs[i - 1][2];
    model_inputs[i][3] = model_inputs[i - 1][3];
    model_inputs[i][4] = model_inputs[i - 1][4];
    model_inputs[i][5] = model_inputs[i - 1][5];
  }
  // самые свежие - в элемент [0]
  model_inputs[0][0] = ax_c; 
  model_inputs[0][1] = ay_c;
  model_inputs[0][2] = az_c; 
  model_inputs[0][3] = gx; 
  model_inputs[0][4] = gy;
  model_inputs[0][5] = gz;
  
  if (num_samples == model_window_size) {
    // набрали нужное для модели кол-во сэмплов, отправляем их на вход
    for (int i = model_window_size - 1; i >= 0; i--)
      neuton_model_set_inputs(model_inputs[i]);
    // запускаем модель
    neuton_model_run_inference(&class_index, &model_outputs);
  }
  else
    // считаем сэмплы
    num_samples++;

  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  sum += deltat; // sum for averaging filter update rate
  sumCount++;

  #ifdef DEBUG
  Serial.println((String)ax + "\t" + ay + "\t" + az + "\t");
  //Serial.println((String)ax_avg + "\t" + ay_avg + "\t" + az_avg);
  //Serial.println((String)(imu.ax - ax_offset) + "\t" + (imu.ay - ay_offset)+ "\t" + (imu.az - az_offset));
  //Serial.print((String)imu.gx + "\t" + imu.gy + "\t" +imu.gz + "\t");
  //Serial.println((String)imu.mx + "\t" + imu.my + "\t" +imu.mz);
  //Serial.println((String)ax + "\t" + ay + "\t" + az);
  //Serial.print("\t");
  //Serial.println((String)gx + "\t" + gy + "\t" + gz);
  //Serial.println((String)mx + "\t" + my + "\t" + mz);
  //Serial.println((String)qn[0] + "\t" + qn[1] + "\t" + qn[2] + "\t" + qn[3]);

  //Serial.println((String)ax_c + "\t" + ay_c + "\t" + az_c);
  //Serial.println((String)ax + "\t" + ax_avg);
  //Serial.println((String)model_outputs[0] + "\t" + model_outputs[1] + "\t" + deltat);
  #endif
  
  #ifdef OLED
  if (sumCount % 10 == 0) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(model_outputs[1]);
    display.display();
  }
  #endif

  if (model_outputs[1] >= BRAKE_INFERENCE_THRESHOLD) {  
    digitalWrite(LED_BUILTIN, LOW);     // включаем св.диод на плате
    digitalWrite(STOP_LED_PIN, HIGH);   // и стоп-сигнал 
    // переделать на промежуток свечения
  }
  else {
    digitalWrite(LED_BUILTIN, HIGH);    // ВЫключаем св.диод на плате
    digitalWrite(STOP_LED_PIN, LOW);    // и стоп-сигнал 
  }

/*
  // проверяем, есть ли активность на акселе
  if (abs(ax_c) >= INACTIVITY_ACCEL_THRESHOLD || abs(ay_c) >= INACTIVITY_ACCEL_THRESHOLD || abs(az_c) >= INACTIVITY_ACCEL_THRESHOLD) {
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
        Serial.println(F("IMU LP WoM mode failed"));
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
      blinkLED(2, 300, 200);
      
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
      blinkLED(1, 500, 500);

      // перезапускаем MPU
      MPU_init();

      // начинаем заново отсчет неактивности
      accel_last_activity_ts = millis();
    }
  }
  */
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

  // генерация прерываний спадающим импульсом
  imu.setIntLevel(INT_ACTIVE_LOW);
  imu.setIntLatched(INT_50US_PULSE);
  imu.enableInterrupt(1);
  
  // установленные значения LPF и SampleRate
  #ifdef DEBUG
  Serial.println("IMU LPF: " + (String)imu.getLPF() + "\tIMU sample rate: " + imu.getSampleRate());
  byte data;
  mpu_read_reg(0x19, &data);
  Serial.println(data, BIN);
  mpu_read_reg(0x1A, &data);
  Serial.println(data, BIN);
  mpu_read_reg(0x1B, &data);
  Serial.println(data, BIN);
  mpu_read_reg(0x1C, &data);
  Serial.println(data, BIN);
  mpu_read_reg(0x1D, &data);
  Serial.println(data, BIN);
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

/*
int mpu_write_reg(unsigned char reg, unsigned char data)
{
    return i2c_write(0x68, reg, 1, &data);
}
*/
