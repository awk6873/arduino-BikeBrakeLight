// Управление стоп-сигналом для велосипеда по значениям ускорения от акселерометра при торможении
// Эмуляция MPU на массиве собранных данных от акселя и гиро 
// Плата - XIAO SAMD21
// IMU - Invensense MPU9250

#include <SparkFunMPU9250-DMP.h>
#include <neuton.h>
#include <SD.h>
#include <SPI.h>

const int chipSelectPin = 3;

#define STOP_LED_PIN 2    // pin для управления ключом стоп-сигнала

//#define OLED 1            // использовать i2c OLED дисплей SSD1306 для вывода сообщений
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
int imu_ax_c = 0, imu_ay_c = 0, imu_az_c = 0;
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

int brake;

neuton_input_t model_inputs[100][9]; 
int model_window_size;
int model_inputs_num;
int num_samples = 0;

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

  // инициализируем модель
  neuton_nn_setup();
  // кол-во сэмплов в окне данных
  model_window_size = neuton_nn_input_window_size();
  // кол-во переменных в сэмпле
  model_inputs_num = neuton_nn_uniq_inputs_num();
  Serial.print("Model window size: ");
  Serial.println(model_window_size);
  Serial.print("Model inputs num: ");
  Serial.println(model_inputs_num);

  // будем читать данные из файла с SD карты
  Serial.println("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelectPin)) {
    #ifdef DEBUG
    Serial.println("Card failed, or not present");
    #endif
    blinkLED(3, 500, 500);;
    while(1);
  }
  Serial.println("card initialized.");

  f = SD.open("mpu_lbl.tsv"); 
  Serial.println(f.available());
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
              &imu_ax, &imu_ay, &imu_az, &imu_ax_c, &imu_ay_c, &imu_az_c, &imu_gx, &imu_gy, &imu_gz, 
              &brake);
  delayMicroseconds(13000);

  // сдвигаем предыдущие значения
  for (int i = model_window_size - 1; i > 0; i--) {
    model_inputs[i][0] = model_inputs[i - 1][0];
    model_inputs[i][1] = model_inputs[i - 1][1];
    model_inputs[i][2] = model_inputs[i - 1][2];
    model_inputs[i][3] = model_inputs[i - 1][3];
    model_inputs[i][4] = model_inputs[i - 1][4];
    model_inputs[i][5] = model_inputs[i - 1][5];
	  model_inputs[i][6] = model_inputs[i - 1][6];
    model_inputs[i][7] = model_inputs[i - 1][7];
    model_inputs[i][8] = model_inputs[i - 1][8];
  }
  // самые свежие - в элемент [0]
  model_inputs[0][0] = imu_ax; 
  model_inputs[0][1] = imu_ay;
  model_inputs[0][2] = imu_az;
  model_inputs[0][3] = imu_ax_c; 
  model_inputs[0][4] = imu_ay_c;
  model_inputs[0][5] = imu_az_c;
  model_inputs[0][6] = imu_gx; 
  model_inputs[0][7] = imu_gy;
  model_inputs[0][8] = imu_gz;

  neuton_inference_input_t* p_input;

  if (num_samples < model_window_size - 1)
    // накапливаем нужное кол-во сэмплов
    num_samples++;
  else {
    // отправляем окно данных на вход модели
    for (int i = model_window_size - 1; i >= 0; i--) {
      Serial.println(i);
      p_input = neuton_nn_feed_inputs(model_inputs[i], model_inputs_num);
      if (p_input != NULL)
        //make inference
        break;
      Serial.println(i);
    }

    neuton_u16_t predicted_target;
    const neuton_output_t* probabilities;
    neuton_nn_run_inference(p_input, &predicted_target, &probabilities);

    Serial.print(predicted_target);
    Serial.print("\t");
    Serial.print(probabilities[0]);
    Serial.print("\t");
    Serial.print(probabilities[1]);
    Serial.print("\t");
    Serial.print(brake);
    Serial.print("\t");
    Serial.println(deltat, 7);
  } 
  uint16_t index;
  float* outputs;
  /*  
  if (neuton_model_run_inference(&index, &outputs) == 0) {
    //  code for handling prediction result
     Serial.print(outputs[0]);
     Serial.print("\t");
     Serial.print(outputs[1]);
     Serial.print("\t");
     Serial.print(brake);
     Serial.print("\t");
     Serial.println(deltat, 7);
  }
  */

  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  //deltat = deltat_orig / 1000000.0f; // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  sum += deltat; // sum for averaging filter update rate
  sumCount++;
  
  #ifdef DEBUG
  //Serial.println((String)imu.ax + "\t" + imu.ay + "\t" +imu.az + "\t");
  //Serial.println((String)ax_cal_avg + "\t" + ay_cal_avg + "\t" + az_cal_avg);
  //Serial.println((String)(imu.ax - ax_offset) + "\t" + (imu.ay - ay_offset)+ "\t" + (imu.az - az_offset));
  //Serial.print((String)imu.gx + "\t" + imu.gy + "\t" +imu.gz + "\t");
  //Serial.println((String)imu.mx + "\t" + imu.my + "\t" +imu.mz);
  //Serial.println((String)imu_ax + "\t" + imu_ay + "\t" + imu_az + "\t");
  //Serial.print((String)imu_gx + "\t" + imu_gy + "\t" + imu_gz + "\t");
  //Serial.print((String)imu_mx + "\t" + imu_my + "\t" + imu_mz + "\t");
  //Serial.println((String)deltat_orig + "\t" + deltat);
  //Serial.println((String)qn_str[0] + "\t" + qn_str[1] + "\t" + qn_str[2] + "\t" + qn_str[3]);
  //Serial.print((String)qn[0] + "\t" + qn[1] + "\t" + qn[2] + "\t" + qn[3] + "\t");

  //Serial.println((String)ax + "\t" + ay + "\t" + az);
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
  
/*  
  #ifdef OLED
  if (sumCount % 10 == 0) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println((String)((ay_avg < 0)?"-":"+") + (String)abs(ay_avg));
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
*/

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
