/************************************************************
MPU9250_DMP_Quaternion
 Quaternion example for MPU-9250 DMP Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library
The MPU-9250's digital motion processor (DMP) can calculate
four unit quaternions, which can be used to represent the
rotation of an object.
This exmaple demonstrates how to configure the DMP to 
calculate quaternions, and prints them out to the serial
monitor. It also calculates pitch, roll, and yaw from those
values.
Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0
Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/
#include <SparkFunMPU9250-DMP.h>
#include "ArduinoLowPower.h"

#define INTERRUPT_PIN 1

MPU9250_DMP imu;

void setup() 
{
  Serial.begin(115200);
  Serial.println("Starting");

  // Set pin 7 as INPUT_PULLUP to avoid spurious wakeup
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  // Attach a wakeup interrupt on pin 8, calling repetitionsIncrease when the device is woken up
  LowPower.attachInterruptWakeup(INTERRUPT_PIN, dummy, LOW);

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
               DMP_FEATURE_GYRO_CAL | // Use gyro calibration
               DMP_FEATURE_SEND_CAL_GYRO | // Send cal'd gyro values
               DMP_FEATURE_SEND_RAW_ACCEL,
              1); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive


  // Use enableInterrupt() to configure the MPU-9250's 
  // interrupt output as a "data ready" indicator.
  imu.enableInterrupt();

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

}

void loop() 
{

  static long ts_delta = 0;
  static long ts = 0;
   
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      //imu.computeEulerAngles();
      //printIMUData();
      
      ts_delta = millis() - ts;
      ts = millis();
      Serial.println(ts_delta);
    }
  }

/*
    float q[4] = {imu.calcQuat(imu.qw), imu.calcQuat(imu.qx), imu.calcQuat(imu.qy), imu.calcQuat(imu.qz)};
    float v[4] = {0, imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az)};
    float v1_[4];
    float v1[4];
    float v2[4];
    float v3_[4];
    float v3[4];
    float q1[4] = {q[0], -q[1], -q[2], -q[3]};
    float u[4] = {0.0, 0.0, 0.0, 1.0};
    float v3s = 0;
    
    multiply_quaternion(q, v, v1_);
    multiply_quaternion(v1_, q1, v1);
    v2[0] = v1[0] - u[0], v2[1] = v1[1] - u[1], v2[2] = v1[2] - u[2], v2[3] = v1[3] - u[3];
    multiply_quaternion(q1, v2, v3_);
    multiply_quaternion(v3_, q, v3);

    // v' = q . v . q*
    // v' = (w,x,y,z) * (0,ax,ay,az) * (w,-x,-y,-z);  q* - сопряженный кватернион (w,-x,-y,-z)
    // v'' = v' - u,    u = (0,0,g)
    // v''' = q-1 . v'' . q-1*,     q-1 - обратный кватернион, для нормированного равен сопряженному кватерниону


/*
    Serial.print("Accel:    ");
    Serial.print(v[1]);
    Serial.print("\t");
    Serial.print(v[2]);
    Serial.print("\t");
    Serial.print(v[3]);
    Serial.println();

    Serial.print("Accel':   ");
    Serial.print(v1[1]);
    Serial.print("\t");
    Serial.print(v1[2]);
    Serial.print("\t");
    Serial.print(v1[3]);
    Serial.println();

    Serial.print("Accel'':  ");
    Serial.print(v2[1]);
    Serial.print("\t");
    Serial.print(v2[2]);
    Serial.print("\t");
    Serial.print(v2[3]);
    Serial.println();

    Serial.print("Accel''': "); */
/*
//    Serial.print(v3[1]);
//    Serial.print("\t");
    Serial.print(v3[2]);
    Serial.print("\t");
*/
/*
    v3s = (v3s + v3[2]) / 2.0;
    Serial.print(v3s);
    Serial.print("\t");
    Serial.print(v3[3]);
    Serial.println();

    if (v3s < -0.3) {
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (LOW is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage HIGH
 }
 */   
    //delay(500);


  //static long ts_delta = millis() - ts;
  //Serial.println(ts_delta);


  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (LOW is the voltage level)
  delay(100);                       // wait for a second
  digitalWrite(LED_BUILTIN, HIGH);
  // Triggers an infinite sleep (the device will be woken up only by the registered wakeup sources)
  // The power consumption of the chip will drop consistently
  LowPower.sleep();
}

void printIMUData(void)
{  
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);

/*
  Serial.println("Q: " + String(q0, 4) + ", " +
                    String(q1, 4) + ", " + String(q2, 4) + 
                    ", " + String(q3, 4));
  Serial.println("R/P/Y: " + String(imu.roll) + ", "
            + String(imu.pitch) + ", " + String(imu.yaw));
  Serial.println("Time: " + String(imu.time) + " ms");
  Serial.println();
*/

 // Serial.print(q0, 4);
 // Serial.print(",");
 // Serial.print(q1, 4);
 // Serial.print(",");
 // Serial.print(q2, 4);
 // Serial.print(",");
 // Serial.println(q3, 4);

  float ax = imu.calcAccel(imu.ax);
  float ay = imu.calcAccel(imu.ay);
  float az = imu.calcAccel(imu.az);

  static float ays = 0;

  ays = (ays + ay) / 2.0;
//  Serial.print(ax);
//  Serial.print(",");
  Serial.print(ay);
  Serial.print(",");
  Serial.println(ays);
//  Serial.println(az);

  if (ays < -0.3) {
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (LOW is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage HIGH
 }
}

// процедура умножения кватернионов
void multiply_quaternion(float qa[4], float qb[4], float *q) { 

  q[0] = qa[0]*qb[0] - qa[1]*qb[1] - qa[2]*qb[2] - qa[3]*qb[3];
  q[1] = qa[0]*qb[1] + qb[0]*qa[1] + qa[2]*qb[3] - qb[2]*qa[3];
  q[2] = qa[0]*qb[2] + qb[0]*qa[2] + qa[3]*qb[1] - qb[3]*qa[1];
  q[3] = qa[0]*qb[3] + qb[0]*qa[3] + qa[1]*qb[2] - qb[1]*qa[2];
}

void dummy() {
}
