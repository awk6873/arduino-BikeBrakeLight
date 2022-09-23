/*
 "compass_cal.pde"
 (c) 2019 lingib
 https://www.instructables.com/member/lingib/instructables/
 Last update: 28 November 2019
 
 --------
 ABOUT
 --------
 This program calculates hard-iron and soft-iron offsets that are required to calibrating 
 the magnetometer inside an MPU-9250 accelerometer|gyro|magnetometer package.
 
 Temperature compensation is not applied.
 
 --------
 NOTES:
 --------
 (1)
 All data values are maintained at 100% in the array[][]
 All data points are scaled to fit the 3D display
 Change the arrayLength in code-line 63 if you need greater accuracy.
 
 (2)
 An arrayLength of 12000 provides sufficient data points for creating a sphere but slows everything down.
 An arrayLength 0f 6000 provides sufficient time to create six circles each of 1000 data points.
 
 (3)
 Orientate the MPU-9250 sensor with the Y-axis pointing downwards before starting then follow the onscreen instructions.
 
 ----------
 COPYRIGHT
 ----------
 This is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This software is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License. If 
 not, see <http://www.gnu.org/licenses/>.
 */

/*
*/

import processing.serial.*;
Serial myPort;                                    // create Serial port instance

PrintWriter rotateXdown;                          // create Printwriter instance
PrintWriter rotateXup;                            // create Printwriter instance
PrintWriter rotateXall;                           // create Printwriter instance
PrintWriter rotateYdown;                          // create Printwriter instance
PrintWriter rotateYup;                            // create Printwriter instance
PrintWriter rotateYall;                           // create Printwriter instance
PrintWriter rotateZdown;                          // create Printwriter instance
PrintWriter rotateZup;                            // create Printwriter instance
PrintWriter rotateZall;                           // create Printwriter instance
PrintWriter rotateXYZ;                            // create Printwriter instance

//final int arrayLength = 12000;                  // use this value if you intend to plot a shere
final int arrayLength = 6000;                     // use this value if you intend to plot circles
float [][] array = new float [arrayLength+1][3];  // 6000 rows of 3 columns (xPos, yPos, zPos)

final int arrayLimit1 = int((float)arrayLength*1/6);
final int arrayLimit2 = int((float)arrayLength*2/6);
final int arrayLimit3 = int((float)arrayLength*3/6);
final int arrayLimit4 = int((float)arrayLength*4/6);
final int arrayLimit5 = int((float)arrayLength*5/6);

int arraySegment = 0;                             // which array[][] data segment are we talking about

int drawIndex = 0;                                // used by the draw() loop to scan the array 60 times per second                        
int captureIndex = 0;                             // used by the serialEvent() routine which captures the data at a slower rate

String inputString = "";                          //for incoming data 


float xPos;                                       // scratch pads for raw array[][] data
float yPos;
float zPos;

float xPrev;                                      // scratch pads for validating the incoming data.
float yPrev;
float zPrev;
float xyzLimit = 10;                              // data range for validation

float xMax = -32768.0;                            // Dummy raw data extremes
float yMax = -32768.0; 
float zMax = -32768.0; 
float xMin =  32767.0; 
float yMin =  32767.0; 
float zMin =  32767.0; 

float xOffset;                                    // hard-iron offsets
float yOffset;
float zOffset;

float xChord;                                     // scratchPads for calculating soft-iron scale factors
float yChord;
float zChord;
float avgChord;

float xScale = 1.0;                               // soft-iron scale factors
float yScale = 1.0;
float zScale = 1.0;

// ----- flags
boolean connected = false;                        // used for synching data
boolean dataValid = true;                        // used to eliminate false readings
boolean offsetsValid = false;                     // used during data aquisition & 3D display
boolean pauseFlag = false;                        // used during data aquisition
boolean scanComplete = false;                     // used during data aquisition
boolean debug = true;

// ==========================
// setup()
// ==========================
void setup()
{
  // ----- define drawing area size
  size(600, 600, P3D);                            // P3D allows rotation about the Z axis
  background(0);                                  // initial background color

  // ----- create 10 data files in the sketch directory
  rotateXdown = createWriter("rotateXdown.csv"); 
  rotateXup = createWriter("rotateXup.csv"); 
  rotateXall = createWriter("rotateXall.csv");
  rotateYdown = createWriter("rotateYdown.csv"); 
  rotateYup = createWriter("rotateYup.csv"); 
  rotateYall = createWriter("rotateYall.csv");
  rotateZdown = createWriter("rotateZdown.csv"); 
  rotateZup = createWriter("rotateZup.csv"); 
  rotateZall = createWriter("rotateZall.csv");
  rotateXYZ = createWriter("rotateXYZ.csv"); 

  // ----- configure the serial port
  printArray(Serial.list()); 
  myPort = new Serial(this, Serial.list()[2], 115200);
  myPort.bufferUntil('\n');                      // serialEvent() won't trigger until buffer has "\n"
  myPort.clear();
}

// ==========================
// draw()
// ==========================
void draw() {
  background(0);
  for (drawIndex = 10; drawIndex < arrayLength; drawIndex++) {    // drawIndex=10 skips any stray 9999 sync values
    plotData(drawIndex);
  }
}

// =======================
// serial event  (called with each Arduino data string)
// =======================
void serialEvent(Serial myPort) {

  // ----- wait for a line-feed
  inputString = myPort.readStringUntil('\n');

  // ----- validate input data
  if (inputString != null) 
  {

    // ----- establish connection
    inputString = trim(inputString);                   // remove leading/trailing whitespace
    if (connected == false) {
      if (inputString.equals("S")) {
        connected = true;                              // connection established
        pauseFlag = true;                              // spacebar must be pressed to continue
        println("");
        println("Place your compass on a flat surface with the RED arrows DOWN");
        println("Press the spacebar to continue ...");
      }
    } else {
      if (pauseFlag == false) {

        // ----- request data
        if (connected && !inputString.equals("S")) {
          inputString = trim(inputString);               // remove leading/trailing whitespace

          // ----- process the data
          String[] str = split(inputString, ",");        // convert inputString into string array
          xPos = float(str[0]);
          yPos = float(str[1]);
          zPos = float(str[2]);

          // ----- debug
          if (debug == true) {
            println(inputString);
            //print(xPos); 
            //print("\t");
            //print(yPos); 
            //print("\t");
            //println(zPos);
          }

          // ----- validate the data
          /*
           Extraneous data points sometime appear.
           Data points do not tend to change much from one reading to the next.
           This routine flags any data point that changes by more than 10 counts
           Three zeros not possible
           */
          if (abs(int(xPos - xPrev))> xyzLimit) dataValid = false;
          if (abs(int(yPos - yPrev))> xyzLimit) dataValid = false;
          if (abs(int(zPos - zPrev))> xyzLimit) dataValid = false;
          if ((int(xPos) == 0) && (int(yPos) == 0) && (int(zPos) == 0)) dataValid = false;
          xPrev = xPos;
          yPrev = yPos;
          zPrev = zPos;

          // ----- store data set in array[][]
          if (dataValid == true) {
            dataValid = false;
            array[captureIndex][0] = xPos;
            array[captureIndex][1] = yPos;
            array[captureIndex][2] = zPos;

            // ----- point to next storage location
            captureIndex++;
          }

          // ----- do we need to change axis
          if ((captureIndex == arrayLimit1) && (arraySegment == 0)) {
            pauseFlag = true;                                             // this flag is cleared in "keyPressed()"
            arraySegment++; 

            println("Change axis ... RED arrows UP");
            println("Press the spacebar to continue ...");
          }

          if ((captureIndex == arrayLimit2) && (arraySegment == 1)) {
            pauseFlag = true;                                             // this flag is cleared in "keyPressed()"
            arraySegment++;

            println("Change axis ... GREEN arrows DOWN");
            println("Press the spacebar to continue ...");
          }

          if ((captureIndex == arrayLimit3) && (arraySegment == 2)) {
            pauseFlag = true;                                             // this flag is cleared in "keyPressed()"
            arraySegment++;

            println("Change axis ... GREEN arrows UP");
            println("Press the spacebar to continue ...");
          }

          if ((captureIndex == arrayLimit4) && (arraySegment == 3)) {
            pauseFlag = true;                                             // this flag is cleared in "keyPressed()"
            arraySegment++;

            println("Change axis ... BLUE arrows DOWN");
            println("Press the spacebar to continue ...");
          }

          if ((captureIndex == arrayLimit5) && (arraySegment == 4)) {
            pauseFlag = true;                                             // this flag is cleared in "keyPressed()"
            arraySegment++;

            println("Change axis ... BLUE arrows UP");
            println("Press the spacebar to continue ...");
          }

          if (captureIndex == arrayLength) {
            scanComplete = true;
            pauseFlag = true;

            println("Scan complete"); 
            println(""); 

            calculateOffsets();                                           // calculate hard-iron offsets and soft-iron scale-factors
            createCSV();                                                  // create CSV data files in the sketch folder
          }

          // ----- send data requests until the array[][] is full
          if ((pauseFlag == false) && (scanComplete == false)) {         // data requests stop when scanComplete flag set 
            dataValid = true;
            myPort.clear();                                              //clear the receive buffer
            myPort.write("S");
          }
        }
      }
    }
  }
}

// ==========================
// keyPressed()
// ==========================
void keyPressed()
{
  //println(keyCode);                                 // uncomment this line to find your spacebar keycode

  // ----- look for spacebar
  if ((pauseFlag == true) && (scanComplete == false)) { 
    if (key == 32) {                                  // spacebar keyCode=32
      println("Rotate about the new axis ...");
      println("");
      delay(1000);
      pauseFlag = false;
      myPort.write("S");                              // (S)end more data
    }
  }

  // ----- toggle offsets on|off
  if (scanComplete) {
    if ((key == 'o')||(key == 'O')) {
      offsetsValid = !offsetsValid;
    }
  }
}

// ==========================
// plot3D(int scanRow)
// ==========================
void plotData(int scanRow) {

  // ----- locals
  float axesSF = 0.7;                           // 3D axes (S)cale (F)actor
  //float dataSF = 0.5;                         // data (S)cale (F)actor
  float dataSF = 0.3;                           // data (S)cale (F)actor

  pushMatrix();  

  translate(width/2, width/2, -width/2);        // set origin to cube center
  scale(axesSF);                                // reduce axis sizes

  if (scanComplete == true) {
    // ----- rotate box using mouse
    rotateX(map(mouseX, 0, height, 0, PI));    // change viewing angle
    rotateY(map(mouseY, 0, height, 0, PI));    // change viewing angle
    rotateZ(radians(0));
  } else {    
    // ----- fixed view while capturing data
    rotateX(radians(20));                      // change viewing angle
    rotateY(radians(-30));                     // change viewing angle
    rotateZ(radians(-20));
  }

  // ----- draw XYZ axes
  stroke(255, 0, 0);                           // X-axis (red)  
  line(-width/2, 0, 0, width/2, 0, 0);
  stroke(0, 255, 0);                           // y-axis (green)
  line(0, -width/2, 0, 0, width/2, 0);
  stroke(0, 0, 255);                           // z-axis (blue)
  line(0, 0, -width/2, 0, 0, width/2);

  // ----- draw wire frame
  stroke(64);
  //  line(x1,y1,z1  x2,y2,z3);
  line(-width/2, 0, -width/2, width/2, 0, -width/2);                  // XY plane (Z center)
  line(-width/2, 0, width/2, width/2, 0, width/2);
  line(-width/2, 0, -width/2, -width/2, 0, width/2);
  line(width/2, 0, -width/2, width/2, 0, width/2);

  line(-width/2, -width/2, -width/2, width/2, -width/2, -width/2);    // XY plane (-Z)
  line(-width/2, -width/2, +width/2, width/2, -width/2, width/2);
  line(-width/2, -width/2, -width/2, -width/2, -width/2, width/2);
  line(width/2, -width/2, -width/2, width/2, -width/2, width/2);
  line( 0, -width/2, -width/2, 0, -width/2, width/2);
  line(-width/2, -width/2, 0, width/2, -width/2, 0); 


  line(-width/2, width/2, width/2, width/2, width/2, width/2);        // XY plane (+Z)
  line(-width/2, width/2, 0, width/2, width/2, 0);
  line(-width/2, width/2, -width/2, width/2, width/2, -width/2);
  line(-width/2, width/2, -width/2, -width/2, width/2, width/2);
  line( 0, width/2, -width/2, 0, width/2, width/2);
  line(+width/2, width/2, -width/2, width/2, width/2, width/2);

  line(-width/2, width/2, -width/2, -width/2, -width/2, -width/2);    // YZ plane (verticals)
  line(-width/2, width/2, 0, -width/2, -width/2, 0);
  line(-width/2, width/2, width/2, -width/2, -width/2, width/2);
  line(0, width/2, -width/2, 0, -width/2, -width/2);
  line(0, width/2, width/2, 0, -width/2, width/2);
  line(width/2, width/2, -width/2, width/2, -width/2, -width/2);
  line(width/2, width/2, 0, width/2, -width/2, 0);
  line(width/2, width/2, width/2, width/2, -width/2, width/2);

  // ----- apply data corrections
  if (offsetsValid == false) {

    // ----- Read raw data (no offsets applied)
    xPos = array[scanRow][0] * dataSF;                              //  dataSF reduces the size of the 3D balls
    yPos = array[scanRow][1] * dataSF;                              //  dataSF reduces the size of the 3D balls
    zPos = array[scanRow][2] * dataSF;                              //  dataSF reduces the size of the 3D balls
  } else {

    // ----- read raw data and apply the  offsets
    xPos = (array[scanRow][0] - xOffset) * xScale * dataSF;  
    yPos = (array[scanRow][1] - yOffset) * yScale * dataSF;  
    zPos = (array[scanRow][2] - zOffset) * zScale * dataSF;
  } 

  // ----- display the three 3D-spheres
  noStroke();                                                       // no wire-frame
  lights();                                                         // 3D shadows

  // ----- rotation about X-axis (YZ plane
  if (scanRow <arrayLimit2) {
    fill(255, 0, 0);                                                // red dots
    translate(xPos, yPos, zPos);
  }

  // ----- rotation about the Y-axis (XZ plane)
  if ((scanRow >= arrayLimit2) && (scanRow <arrayLimit4)) {
    fill(0, 255, 0);                                                // green dots
    translate(xPos, yPos, zPos);
  }

  // ----- rotation about Z-axis (XY plane)
  if ((scanRow >= arrayLimit4) && (scanRow < arrayLength)) {
    fill(0, 0, 255);                                                // blue dots 
    translate(xPos, yPos, zPos);
  }

  sphere(4);                                                        // draw the sphere
  noLights();
  popMatrix();
}

// ==========================
// calculateOffsets()
// ==========================
void calculateOffsets() {

  // ----- search array for max|min values
  for (int i=10; i<arrayLength; i++) {

    // ----- find maximums
    if (array[i][0] > xMax) xMax = array[i][0];
    if (array[i][1] > yMax) yMax = array[i][1];
    if (array[i][2] > zMax) zMax = array[i][2];

    // ----- find minimums
    if (array[i][0] < xMin) xMin = array[i][0];
    if (array[i][1] < yMin) yMin = array[i][1];
    if (array[i][2] < zMin) zMin = array[i][2];
  }

  // ----- calculate the hard-iron offsets 
  xOffset = (xMax + xMin)/2.0;
  yOffset = (yMax + yMin)/2.0;
  zOffset = (zMax + zMin)/2.0;

  // ----- calculate the soft-iron offsets
  xChord = (xMax - xMin)/2.0;
  yChord = (yMax - yMin)/2.0;
  zChord = (zMax - zMin)/2.0;
  avgChord = (xChord + yChord + zChord)/3.0;

  xScale = avgChord/xChord;
  yScale = avgChord/yChord;
  zScale = avgChord/zChord; 

  // ----- print the results
  println("----------------");
  println(" Max|min values ");
  println("----------------");
  print("xMax: "); 
  print(xMax); 
  print("\t");         
  print("xMin: "); 
  println(xMin); 

  print("yMax: "); 
  print(yMax); 
  print("\t");         
  print("yMin: "); 
  println(yMin); 

  print("zMax: "); 
  print(zMax); 
  print("\t");         
  print("zMin: "); 
  println(zMin); 
  println("");

  println("----------");
  println(" Offsets");
  println("----------");  
  print("xOffset: "); 
  println(xOffset);        
  print("yOffset: "); 
  println(yOffset);       
  print("zOffset: "); 
  println(zOffset);
  println("");

  println("---------------");
  println(" Scale Factors");
  println("---------------");
  print("xScale: "); 
  println(xScale);  
  print("yScale: ");  
  println(yScale);  
  print("zScale: "); 
  println(zScale);
  println("");

  println("----------------------------------------");
  println("Copy&Paste the following code into your ");
  println("Arduino header then delete the old code.");
  println("----------------------------------------");
  println("");
  println("float");
  print("Mag_x_offset = ");
  print(xOffset);
  println(",");
  print("Mag_y_offset = "); 
  print(yOffset);
  println(",");
  print("Mag_z_offset = "); 
  print(zOffset);
  println(",");
  print("Mag_x_scale = "); 
  print(xScale);
  println(",");
  print("Mag_y_scale = "); 
  print(yScale);
  println(",");
  print("Mag_z_scale = "); 
  print(zScale);
  println(";");
}

// ==========================
// createCSV()
// ==========================
void createCSV() {

  for (int i=10; i<arrayLimit1; i++) {
    rotateXdown.print(array[i][0]);                       // write magX data to file
    rotateXdown.print(",");                               // write comma delimiter
    rotateXdown.print(array[i][1]);                       // write magY data to file
    rotateXdown.print(",");                               // write comma delimiter
    rotateXdown.println(array[i][2]);                     // write magZ data to file
  }
  rotateXdown.flush();
  rotateXdown.close();
  delay(100);

  for (int i=arrayLimit1; i<arrayLimit2; i++) {
    rotateXup.print(array[i][0]);                       // write magX data to file
    rotateXup.print(",");                               // write comma delimiter
    rotateXup.print(array[i][1]);                       // write magY data to file
    rotateXup.print(",");                               // write comma delimiter
    rotateXup.println(array[i][2]);                     // write magZ data to file
  }
  rotateXup.flush();
  rotateXup.close();
  delay(100);

  for (int i=10; i<arrayLimit2; i++) {
    rotateXall.print(array[i][0]);                       // write magX data to file
    rotateXall.print(",");                               // write comma delimiter
    rotateXall.print(array[i][1]);                       // write magY data to file
    rotateXall.print(",");                               // write comma delimiter
    rotateXall.println(array[i][2]);                     // write magZ data to file
  }
  rotateXall.flush();
  rotateXall.close();
  delay(100);

  for (int i=arrayLimit2; i<arrayLimit3; i++) {
    rotateYdown.print(array[i][0]);                       // write magX data to file
    rotateYdown.print(",");                               // write comma delimiter
    rotateYdown.print(array[i][1]);                       // write magY data to file
    rotateYdown.print(",");                               // write comma delimiter
    rotateYdown.println(array[i][2]);                     // write magZ data to file
  }
  rotateYdown.flush();
  rotateYdown.close();
  delay(100);

  for (int i=arrayLimit3; i<arrayLimit4; i++) {
    rotateYup.print(array[i][0]);                       // write magX data to file
    rotateYup.print(",");                               // write comma delimiter
    rotateYup.print(array[i][1]);                       // write magY data to file
    rotateYup.print(",");                               // write comma delimiter
    rotateYup.println(array[i][2]);                     // write magZ data to file
  }
  rotateYup.flush();
  rotateYup.close();
  delay(100);

  for (int i=arrayLimit2; i<arrayLimit4; i++) {
    rotateYall.print(array[i][0]);                       // write magX data to file
    rotateYall.print(",");                               // write comma delimiter
    rotateYall.print(array[i][1]);                       // write magY data to file
    rotateYall.print(",");                               // write comma delimiter
    rotateYall.println(array[i][2]);                     // write magZ data to file
  }
  rotateYall.flush();
  rotateYall.close();
  delay(100);

  for (int i=arrayLimit4; i<arrayLimit5; i++) {
    rotateZdown.print(array[i][0]);                       // write magX data to file
    rotateZdown.print(",");                               // write comma delimiter
    rotateZdown.print(array[i][1]);                       // write magY data to file
    rotateZdown.print(",");                               // write comma delimiter
    rotateZdown.println(array[i][2]);                     // write magZ data to file
  }
  rotateZdown.flush();
  rotateZdown.close();
  delay(100);

  for (int i=arrayLimit5; i<arrayLength; i++) {
    rotateZup.print(array[i][0]);                       // write magX data to file
    rotateZup.print(",");                               // write comma delimiter
    rotateZup.print(array[i][1]);                       // write magY data to file
    rotateZup.print(",");                               // write comma delimiter
    rotateZup.println(array[i][2]);                     // write magZ data to file
  }
  rotateZup.flush();
  rotateZup.close();
  delay(100);

  for (int i=arrayLimit4; i<arrayLength; i++) {
    rotateZall.print(array[i][0]);                       // write magX data to file
    rotateZall.print(",");                               // write comma delimiter
    rotateZall.print(array[i][1]);                       // write magY data to file
    rotateZall.print(",");                               // write comma delimiter
    rotateZall.println(array[i][2]);                     // write magZ data to file
  }
  rotateZall.flush();
  rotateZall.close();
  delay(100);

  for (int i=10; i<arrayLength; i++) {
    rotateXYZ.print(array[i][0]);                       // write magX data to file
    rotateXYZ.print(",");                               // write comma delimiter
    rotateXYZ.print(array[i][1]);                       // write magY data to file
    rotateXYZ.print(",");                               // write comma delimiter
    rotateXYZ.println(array[i][2]);                     // write magZ data to file
  }
  rotateXYZ.flush();
  rotateXYZ.close();
  delay(100);

  //exit();                                             // Stop the program
}
