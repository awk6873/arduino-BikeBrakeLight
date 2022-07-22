#include <SparkFunMPU9250-DMP.h>
#include <ArduinoLowPower.h>

#define INTERRUPT_PIN 1

#define LCD 1 // использовать i2c OLED для вывода сообщений
#ifdef LCD
  #include <Adafruit_GFX.h>
  #include <Adafruit_SH110X.h>
  #define OLED_ADDRESS 0x3c  // i2с адрес
  #define SCREEN_WIDTH 128   // разрешение экрана
  #define SCREEN_HEIGHT 64   
  #define OLED_RESET -1      //   QT-PY / XIAO
  Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif

#define DEBUG 1  // использовать Serial для вывода сообщений

// порог значений на осях акселя для определения неактивности, G
#define INACTIVITY_ACCEL_THRESHOLD 0.1
// продолжительность неактивности от акселя перед переходом в режим SLEEP, мс
#define INACTIVITY_BEFORE_SLEEP 10000
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

MPU9250_DMP imu;

void setup() 
{
  Serial.begin(115200);
  delay(5000);
  Serial.println("Starting");

  // pin для прерываний от MPU
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);

  #ifdef LCD
  delay(250); 
  display.begin(OLED_ADDRESS, true); // OLED дисплей
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.clearDisplay();
  display.display();
  #endif

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();

      #ifdef LCD
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
  imu.setLPF(5); // Set LPF corner frequency to 5Hz

  // The sample rate of the accel/gyro can be set using
  // setSampleRate. Acceptable values range from 4Hz to 1kHz
  imu.setSampleRate(10); // Set sample rate to 10Hz

  // Likewise, the compass (magnetometer) sample rate can be
  // set using the setCompassSampleRate() function.
  // This value can range between: 1-100Hz
  imu.setCompassSampleRate(10); // Set mag rate to 10Hz

  //
  imu.enableInterrupt(0);

  // подвешиваем обработчик по спадающему фронту
  LowPower.attachInterruptWakeup(INTERRUPT_PIN, dummy, FALLING);

}

void loop() 
{

  // Check for new data in the FIFO
  if ( imu.dataReady() )
  { 
    delay(10); // если не ждать 10 мс, .update(...) выдает ошибку от мага
    // Call update() to update the imu objects sensor data.
    // You can specify which sensors to update by combining
    // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
    // UPDATE_TEMPERATURE.
    // (The update function defaults to accel, gyro, compass,
    //  so you don't have to specify these values.)
    if (imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS) != INV_SUCCESS) {
        // сигнализируем 
        digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (LOW is the voltage level)
        delay(2000);                       // wait for a second
        digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage HIGH
        delay(1000);
        Serial.println("MPU update sensors failed");
    }
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

  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.print("\t");

  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.print(gz);
  Serial.print("\t");
  
  Serial.print(mx);
  Serial.print("\t");
  Serial.print(my);
  Serial.print("\t");
  Serial.println(mz);

  #ifdef LCD
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println((String)ax + " " + ay + " " + az);
  display.println((String)gx + " " + gy + " " + gz);
  display.println((String)mx + " " + my + " " + mz);
  display.display();
  #endif

  // проверяем, есть ли активность на акселе
  if (abs(ax) >= INACTIVITY_ACCEL_THRESHOLD || abs(ay) >= INACTIVITY_ACCEL_THRESHOLD /* || az >= INACTIVITY_ACCEL_THRESHOLD*/) {
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
        Serial.println(F("IMU LP wake-on-move mode failed"));

      // сигнализируем о переходе в режим сна
      Serial.println("Going to sleep mode");
      digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (LOW is the voltage level)
      delay(1000);                       // wait for a second
      digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage HIGH

      #ifdef LCD
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Sleep mode");
      display.display();
      #endif
      
      // переводим CPU в режим сна
      LowPower.sleep();

      delay(100);

      // сигнализируем о выходе из режима сна
      digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (LOW is the voltage level)
      delay(500);                       // wait for a second
      digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage HIGH

      // перезапуск MPU 
      if (imu.begin() != INV_SUCCESS) { 
        Serial.println("Error reinit IMU");
        digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (LOW is the voltage level)
        #ifdef LCD
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Error reinit IMU");
        display.display();
        #endif
        delay(5000);                       // wait for a second
        digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage HIGH
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
      imu.setLPF(5); // Set LPF corner frequency to 5Hz
      
      // The sample rate of the accel/gyro can be set using
      // setSampleRate. Acceptable values range from 4Hz to 1kHz
      imu.setSampleRate(10); // Set sample rate to 10Hz
      
      // Likewise, the compass (magnetometer) sample rate can be
      // set using the setCompassSampleRate() function.
      // This value can range between: 1-100Hz
      imu.setCompassSampleRate(10); // Set mag rate to 10Hz
      

    }
  }
 
}

// заглушка обработчика прерываний от MPU
void dummy() {
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (LOW is the voltage level)
  delay(10);                       // wait for a second
  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage HIGH
  delay(30); 
}
