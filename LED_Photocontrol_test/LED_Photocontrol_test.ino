// pin'ы для датчика освещенности
#define LIGHT_SENS_OUT_PIN A0
#define LIGHT_SENS_VDD_PIN 1
// коэфф.масштабирования для целочисленных вычислений
#define LIGHT_SENS_SCALE 100L
// кол-во замеров освещенности для усреднения за 5 сек
#define LIGHT_SENS_AVG_CNT (IMU_SAMPLE_RATE * 5)
// пороговые значения для включения и отключения габаритных огней
#define RUNNING_LIGHT_THREASHOLD_ON 750 * LIGHT_SENS_SCALE
#define RUNNING_LIGHT_THREASHOLD_OFF 200 * LIGHT_SENS_SCALE
// уровень ШИМ габаритных огней
#define RUNNING_LIGHT_PWM 10

// pin для управления ключом стоп-сигнала
#define STOP_LED_PIN 2    // pin для управления ключом стоп-сигнала
// running light

uint32_t light_sens_val;
uint32_t light_sens_val_avg = 100;

void setup() {

  Serial.begin(115200);
  
  // режимы pin
  pinMode(LIGHT_SENS_OUT_PIN, INPUT);
  pinMode(LIGHT_SENS_VDD_PIN, OUTPUT);
  pinMode(STOP_LED_PIN, OUTPUT);

  // питание на датчик освещенности
  digitalWrite(LIGHT_SENS_VDD_PIN, HIGH);

}

void loop() {

  // выключаем габариты и стоп-сигнал для устранения засветки фото-датчика
  //analogWrite(STOP_LED_PIN, 0);
  delay(15);
  // измеряем уровень освещенности
  light_sens_val = analogRead(LIGHT_SENS_OUT_PIN) * LIGHT_SENS_SCALE;

  // усредняем значение уровня освещенности по нескольким замерам
  light_sens_val_avg = (light_sens_val_avg * (LIGHT_SENS_AVG_CNT - 1) + light_sens_val) / LIGHT_SENS_AVG_CNT;

  Serial.print(light_sens_val);
  Serial.print("\t");
  Serial.print(light_sens_val_avg);
  Serial.print("\t");
  //Serial.print(light_sens_val_avg * (LIGHT_SENS_AVG_CNT - 1));
  //Serial.print("\t");
  //Serial.print(light_sens_val_avg * (LIGHT_SENS_AVG_CNT - 1) + light_sens_val);
  //Serial.print("\t");

  if (light_sens_val_avg > RUNNING_LIGHT_THREASHOLD_ON) {
    // освещенность низкая, включаем габариты
    analogWrite(STOP_LED_PIN, RUNNING_LIGHT_PWM);
    Serial.print(1);
  }
  if (light_sens_val_avg < RUNNING_LIGHT_THREASHOLD_OFF) {
    // освещенность высокая, вЫключаем габариты
    analogWrite(STOP_LED_PIN, 0);
    Serial.print(0);
  } 

  Serial.println();
  //delay(200);


  //analogWrite(STOP_LED_PIN, 255);
  //delay(1000);

  //analogWrite(STOP_LED_PIN, 0);
  //delay(1000);
}
