// Wes Gates
#include <Servo.h>

/// Timekeeping ///
signed long tms = 0;
float t = 0;
const float tdel = 100; // Delay time in milliseconds
float d = 2.5; // Defines movement time in seconds for the Minimum Jerk Trajectory

/// Servo Properties ///
const int p1min = 779;  // min writeMicroseconds command
const int p1max = 2191; // max writeMicroseconds command
//const float s1 = 10.25; // Calibration slope (microseconds/degree)
const float s1 = 9.774; // Original Calibration Slope (bad calibration)
const float b1 = 1280;  // Calibration Bias (microseconds)
float p1 = b1;            // Starting (home) position
float th1 = 0; float th1r = 0;

const int p2min = 805;
const int p2max = 2249;
const float s2 = 10.24;
const float b2 = 1090;
float p2 = b2;
float th2 = 0; float th2r = 0;

const int p3min = 792;
const int p3max = 2250;
const float s3 = 10.227;
const float b3 = 1710;
float p3 = b3;
float th3 = 0; float th3r = 0;

//const int p4min = 800;
const int p4min = 1210; // Avoids sticking hands together
const int p4max = 2230;
const float b4 = 1670;
const float s4 = 10.294;
float p4 = 2100; float pclose = 1370; float popen = 2000;
float th4 = 0; float th4r = 0;

/// Positions ///
// Home Position: Positions the robot to be mostly out of the camera's way
float x0 = 190; float y0 = 100; float z0 = 150;

// Initial Position: Updated after every performMovements()
float xi = x0; float yi = y0; float zi = z0;

// Final Position: xf, yf, zf
float xf = x0; float yf = y0; float zf = z0;
float xf_offset = 0; // Vary offset for PM1 based on target location

// Trash location: Where the robot will move stuff to for dropoff
const float xt1 = 20; const float yt1 = 20; const float zt1 = 120;
const float xt2 = 20; const float yt2 = 20; const float zt2 = 40;

/// Reading Position Values as Strings ///
const int numStrings = 3; // The number of strings to receive
String receivedStrings[numStrings];
float receivedFloat[numStrings];

/// Robot Geometry for Inverse Kinematics ///
float r1 = 0; float rb = 0;
float alpha = 0; float beta = 0; float gamma = 0;
float th1d = 0; float th2d = 0; float th3d = 0;

// Define servo objects
Servo servo1; Servo servo2; Servo servo3; Servo servo4;

void setup() {
  // Attach each servo 1-4 to pins 5,6, 10, and 11, respectively
  servo1.attach(5, p1min, p1max); // servo 1
  servo2.attach(6, p2min, p2max);
  servo3.attach(10, p3min, p3max);
  servo4.attach(11, p4min, p4max);

  // Move all servos to their home position
  servo1.writeMicroseconds(1277);
  servo2.writeMicroseconds(805);
  servo3.writeMicroseconds(1855);
  servo4.writeMicroseconds(popen);

  // Start serial communication
  Serial.begin(115200);
  Serial.setTimeout(3000); // Set a timeout for reading strings
  delay(tdel);
}


void loop() {
  // Detach servos, clear serial buffer, and wait for a data packet
  servo1.detach(); servo2.detach(); servo3.detach(); servo4.detach();
  clearSerialBuffer();

  int receivedCount = 0;
  while (receivedCount < numStrings) {
    if (Serial.available()) {
      // Read the string until a newline character is encountered; then send the received string back to MATLAB

      // Floats
      receivedFloat[receivedCount] = Serial.parseFloat();
      Serial.print(receivedFloat[receivedCount]);
      Serial.print("\n");

      receivedCount++;
      Serial.flush();
      delay(100); // To DEBUG: Increase delay to provide enough time to input coordinate strings to the serial monitor
    }
  }

  // Reattach servos and resume initial positions
  servo1.attach(5, p1min, p1max);   servo2.attach(6, p2min, p2max);
  servo3.attach(10, p3min, p3max);  servo4.attach(11, p4min, p4max);
  servo1.writeMicroseconds(1277);   servo2.writeMicroseconds(805);
  servo3.writeMicroseconds(1855);   servo4.writeMicroseconds(popen);

  // Convert the received strings to float values
  xf = receivedFloat[0];
  yf = receivedFloat[1];
  zf = receivedFloat[2];
  zf = 50; // Keeps aproach low without causing collision with ground.

  //PM1: Stop just behind object (xa, ya, za)
  float xa, ya; // Adusted approach vector
  approachVector(xa,ya,     xf,yf,zf);
  d = scale_d(xi, yi, zi, xa, ya, zf); // Scales d to be faster for short movements
  float xd, yd, zd, th1, th2, th3; // Returns the target location and all joint angles
  performMovements(xd, yd, zd, th1, th2, th3,
                   xi, yi, zi, xa, ya, zf, d);
  // Redefine the initial position as the final (target) position.
  xi = xa;
  yi = ya;
  zi = zf;

  //PM2: Advance to object centroid (xf, yf, zf)
  float xc, yc; // Corrected centroid vector for elongated bounding boxes
  correctedVector(xc,yc,    xf, yf, zf);
  
  d = scale_d(xi, yi, zi, xc, yc, zf);
  
  performMovements(xd, yd, zd, th1, th2, th3,
                   xi, yi, zi, xc, yc, zf, d);
  xi = xc;
  yi = yc;
  zi = zf;

  //Close Gripper
  moveGripper();

  //PM3.1: Move just above trash location (xt1, yt1, zt1)
  d = scale_d(xi, yi, zi, xt1, yt1, zt1);
  performMovements(xd, yd, zd, th1, th2, th3,
                   xi, yi, zi, xt1, yt1, zt1, d);
  xi = xt1;
  yi = yt1;
  zi = zt1;

  //PM3.2: Place object gently on target (trash) (xt2, yt2, zt2)
  d = scale_d(xi, yi, zi, xt2, yt2, zt2);
  performMovements(xd, yd, zd, th1, th2, th3,
                   xi, yi, zi, xt2, yt2, zt2, d);
  xi = xt2;
  yi = yt2;
  zi = zt2;
  
  // Open Gripper
  moveGripper();

  //PM4: Return to Home Position (x0, y0, z0)
  d = scale_d(xi, yi, zi, x0, y0, z0);
  performMovements(xd, yd, zd, th1, th2, th3,
                   xi, yi, zi, x0, y0, z0, d);
  xi = x0;
  yi = y0;
  zi = z0;

  clearSerialBuffer();
  Serial.print("done"); // Tells MATLAB a movement sequence has been completed
  Serial.print("\n"); // Needed in order to read the string
  return;

}

////////////////////////////////////////////////////////////////////////////////////
//*** Sequential Movement function
// Not seeing movement? Try changing xf, yf, zf.
void performMovements(float &xd, float &yd, float &zd, float &th1, float &th2, float &th3, float xi, float yi, float zi, float xf, float yf, float zf, float d) {
  unsigned long startTime = millis();

  for (unsigned long tms = millis(); tms - startTime <= d * 1000; tms = millis()) {

    unsigned long relativeTime = tms - startTime;
    float t_d = (float)relativeTime / (d * 1000); // Reduces division computation within the Minimum Jerk Trajectory

    float xd, yd, zd; // Return the MJT for each axis
    minimumJerkTrajectory(xd, yd, zd,
                          xi, yi, zi, xf, yf, zf, t_d);

    // Call the inverse kinematics function to compute joint angles
    float th1, th2, th3; // Return the joint angles
    computeJointAngles(th1, th2, th3,
                       xd, yd, zd);

    // Move the servos to the calculated positions
    moveServos(th1, th2, th3);

    ////////////////////////////////////////////////////////////////////////////////////////////
    //        // Position of the end-effector and calculated joint angles (Inverse Kinematics)
    //        Serial.print(tms * .001); Serial.print("\t");
    //        Serial.print(xd); Serial.print("\t");
    //        Serial.print(yd); Serial.print("\t");
    //        Serial.print(zd); Serial.print("\t");
    //
    //        Serial.print(th1); Serial.print("\t");
    //        Serial.print(th2); Serial.print("\t");
    //        Serial.print(th3); Serial.print("\t");
    //
    //        Serial.println();
    ////////////////////////////////////////////////////////////////////////////////////////////

    // Add a small delay (if needed) to manage the loop's execution speed
//    delay(10);
  }

}
////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////
//*** Minimum Jerk Trajectory Function
void minimumJerkTrajectory(float &xd, float &yd, float &zd, float xi, float yi, float zi, float xf, float yf, float zf, float t_d) {
  xd = xi + (xf - xi) * ( 10 * pow(t_d, 3) - 15 * pow(t_d, 4) + 6 * pow(t_d, 5));
  yd = yi + (yf - yi) * ( 10 * pow(t_d, 3) - 15 * pow(t_d, 4) + 6 * pow(t_d, 5));
  zd = zi + (zf - zi) * ( 10 * pow(t_d, 3) - 15 * pow(t_d, 4) + 6 * pow(t_d, 5));
}
////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////
// *** Compute Joint Angles using Inverse Kinematics *** //
void computeJointAngles(float &th1, float &th2, float &th3, float xd, float yd, float zd) {
  float r1, rb, alpha, beta, gamma;

  r1 = sqrt(pow((xd - 190), 2) + pow(yd, 2));
  rb = sqrt(pow((zd - 60), 2) + pow((r1 - 60), 2));

  alpha = degrees(atan2(sqrt(abs(pow(80, 2) - pow(rb / 2, 2))) , (rb / 2)));
  beta = degrees(atan2((zd - 60), (r1 - 60)));
  gamma = 180 - 2 * alpha;

  th1 = degrees(atan((190 - xd) / yd));
  th2 = 90 - alpha - beta;
  th3 = -th2 - (90 - gamma);
}
////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////
void moveServos(float th1, float th2, float th3) {
  float p1, p2, p3;

  p1 = s1 * th1 + b1;
  p1 = constrain(p1, p1min, p1max);

  p2 = s2 * th2 + b2;
  p2 = constrain(p2, p2min, p2max);

  p3 = s3 * th3 + b3;
  p3 = constrain(p3, p3min, p3max);

  servo1.writeMicroseconds(p1);
  servo2.writeMicroseconds(p2);
  servo3.writeMicroseconds(p3);
}
////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////
void moveGripper() {
  static bool isopen = false;

  if (isopen != true) {
    servo4.writeMicroseconds(pclose);
    isopen = true;
  }
  else {
    servo4.writeMicroseconds(popen);
    isopen = false;
  }
}

////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
float scale_d(float xi, float yi, float zi, float x, float y, float z) {
  // Calculate the Euclidean distance between the two points
  float distance = sqrt(pow(x - xi, 2) + pow(y - yi, 2) + pow(z - zi, 2));

  // Scale the distance according to your requirements
  //  float d = distance * .03; // Replace 'scaleFactor' with your desired scaling factor
  float d = distance * .03; // Replace 'scaleFactor' with your desired scaling factor

  return d;
}
////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////
void clearSerialBuffer() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}
////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////
// *** 
void approachVector(float &xa, float &ya, float xf, float yf, float zf) {
  float th, P, p;
  float delp = 40; // Defines approach distance

  th = atan2(xf-190,yf);
  P = sqrt(pow(xf-190,2)+pow(yf,2));
  p = P - delp;

  ya = p * cos(th);
  xa = p * sin(th) + 190;
}

////////////////////////////////////////////////////////////////////////////////////
// *** Attempts to correct the x-position error. This error is a result of the 
// depth-data not being captured due to the orientation of the camera.
void correctedVector(float &xc, float &yc, float xf, float yf, float zf) {
  float th, P, p;
  
  // For every 5mm away from center, increase the perpendicular grasping lunge distance by .5mm
  float delp = abs(xf - 190) / 10; 

  th = atan2(xf-190,yf);
  P = sqrt(pow(xf-190,2)+pow(yf,2));
  p = P + delp; // Increase perpendicular grasping lunge distance

  xc = p * sin(th) + 190;  
  yc = p * cos(th); 

//  Serial.print(xc);
//  Serial.print("\t");
//  Serial.print(yc);
//  Serial.print("\t");
//  Serial.print(p);
}

////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// Code for verifying that minimumJerkTrajectory & computeJointAngles work
//  // Apply MJT to smooth motion
//  float xd, yd, zd; // List the returned values of the function
//  minimumJerkTrajectory(xd, yd, zd, xi, yi, zi, xf, yf, zf, t_d);
//
//  // Use Inverse Kinematics to compute joint angles
//  float th1, th2, th3; // The returned angles wanted from the function
//  computeJointAngles(th1, th2, th3, xd, yd, zd);
////////////////////////////////////////////////////////////////////////////////////
