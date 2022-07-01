#include <SparkFunMPU9250-DMP.h>
#include "ArduinoLowPower.h"

#define INTERRUPT_PIN 2


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
#define ACCEL_WAKE_ON_FREQ 2    // 2 соответствует 2.5 Гц

// текущие значения для акселя, приведенные к G
float ax = 0, ay = 0, az = 0;

MPU9250_DMP imu;

void setup() 
{
  Serial.begin(115200);
  delay(5000);
  Serial.println("Starting");

  // pin для прерываний от MPU
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(5000);
    }
  }

  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL| // Use gyro calibration
               DMP_FEATURE_SEND_RAW_ACCEL,
              1); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive

  noInterrupts();
  
  // The interrupt level can either be active-high or low.
  // Configure as active-low, since we'll be using the pin's
  // internal pull-up resistor.
  // Options are INT_ACTIVE_LOW or INT_ACTIVE_HIGH
  imu.setIntLevel(INT_ACTIVE_LOW);

  // The interrupt can be set to latch until data has
  // been read, or to work as a 50us pulse.
  // Use latching method -- we'll read from the sensor
  // as soon as we see the pin go LOW.
  // Options are INT_LATCHED or INT_50US_PULSE
  //imu.setIntLatched(INT_LATCHED);
  imu.setIntLatched(INT_50US_PULSE);
  
  // Use enableInterrupt() to configure the MPU-9250's 
  // interrupt output as a "data ready" indicator.
  imu.enableInterrupt();

  // подвешиваем обработчик по спадающему фронту
  LowPower.attachInterruptWakeup(INTERRUPT_PIN, dummy, LOW);

  interrupts();
}

void loop() 
{
  // сигнализируем 
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (LOW is the voltage level)
  delay(200);                       // wait for a second
  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage HIGH
  delay(1000);

  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
      // сигнализируем 
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (LOW is the voltage level)
  delay(200);                       // wait for a second
  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage HIGH
  delay(1000);
  
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() != INV_SUCCESS) {

        // сигнализируем 
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (LOW is the voltage level)
  delay(200);                       // wait for a second
  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage HIGH
  delay(1000);
      Serial.println("MPU update FIFO failed");
    }
  }

  // получаем значения от акселя
  ax = imu.calcAccel(imu.ax);
  ay = imu.calcAccel(imu.ay); 
  az = imu.calcAccel(imu.az);

  Serial.print(ax);
  Serial.print(ay);
  Serial.println(az);

  // проверяем, есть ли активность на акселе
  if (ax >= INACTIVITY_ACCEL_THRESHOLD || ay >= INACTIVITY_ACCEL_THRESHOLD /* || az >= INACTIVITY_ACCEL_THRESHOLD*/) {
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
      if (mpu_lp_motion_interrupt(ACCEL_WAKE_ON_THRESHOLD, ACCEL_WAKE_ON_DURATION, ACCEL_WAKE_ON_FREQ) != INV_SUCCESS) 
        // ошибка
        Serial.println(F("IMU set up failed. Please check installed IMU IC."));

      // сигнализируем о переходе в режим сна
      digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (LOW is the voltage level)
      delay(3000);                       // wait for a second
      digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage HIGH
  
      // переводим CPU в режим сна
      LowPower.sleep();

      delay(100);
      
      // перезапуск DMP 
      if (imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL| // Use gyro calibration
               DMP_FEATURE_SEND_RAW_ACCEL,
              1) != INV_SUCCESS) { // Set DMP FIFO rate to 10 Hz
      // DMP_FEATURE_LP_QUAT can also be used. It uses the 
      // accelerometer in low-power mode to estimate quat's.
      // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
      digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (LOW is the voltage level)
      delay(5000);                       // wait for a second
      digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage HIGH
              }
    }
  }
}

// заглушка обработчика прерываний от MPU
void dummy() {
}
