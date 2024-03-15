/////////////////////////
// Trajectory Planning //
/////////////////////////

#include <Servo.h>
#include <math.h>
#include <stdlib.h>

//////////////////////////////////////////////////////////////////////
// Change values below to specify start and finish cartesian points  //
//////////////////////////////////////////////////////////////////////

// starting X, Y, Z Values (Change these values to starting position) 
float x_start = 0;
float y_start = 170;
float z_start = -100;

// finishing X, Y, Z Values (Change these values to starting position) 
float x_finish = 120;
float y_finish = 90;
float z_finish = 90;

///////////////////////////////////////////////////////////////////////////////
// Dont change any values below as they define robot constants and variables  //
///////////////////////////////////////////////////////////////////////////////

// These will be the current [x,y,z] values used to calculate theta at each time (t)
int x = x_start;
int y = y_start;
int z = z_start;

// Arm Servo pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 4
#define GripperPin 9

// Servo Objects
Servo Joint1;
Servo Joint2;
Servo Joint3;
Servo Gripper;

// Link lengths (mm)
float l1 = 90;
float l2 = 170;

// Declare thetas which will be the joint angles
float theta_1;
float theta_2;
float theta_3;

// Gripper Joint Angles
int GripperOpen = 150; // Open gripper
int GripperClose = 60; // Close gripper

// Joint Angle Offsets
int Joint1Offset = 10;
int Joint2Offset = 15;
int Joint3Offset = -14;

// Trajectory time and granularity (and therefore total number of points generated)
int tf = 5;
float granularity = 0.1;
float num_points = 50;

// Declare array that stores the projected trajectory.
float *x_points;
float *y_points;
float *z_points;

void calculate_theta_1()
{
theta_1 = acos(x / sqrt(pow(x, 2) + pow(y, 2))) * (180 / PI);
}

void calculate_theta_2()
{
float phi = atan(-1*z / sqrt(pow(x, 2) + pow(y, 2))) * (180 / PI);
float beta_top = pow(l1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2) - pow(l2, 2);
float beta_bottom = (2 * l1 * sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)));
float beta = acos((beta_top / beta_bottom)) * (180 / PI);
theta_2 = phi + beta;
}

void calculate_theta_3()
{
theta_3 = acos(((pow(x, 2) + pow(y, 2) + pow(z, 2) - pow(l1, 2) - pow(l2, 2)) / (2 * l1 * l2))) * (180 / PI);
}

float* calculate_projected_trajectory(int u_start, int u_finish, int tf, int num_points)
{
 float *arr = (float*)malloc(num_points * sizeof(float));

 float a0 = u_start;
 float a2 = (3 / pow(tf, 2)) * (u_finish - u_start);
 float a3 = (-1 * (2 / pow(tf, 3))) * (u_finish - u_start);

 int counter = 0;
 for (float t = granularity; t <= tf; t = t + granularity) {
  float pos = a0 + a2*pow(t, 2) + a3*pow(t, 3);
  arr[counter] = pos;
  counter++;
 }

 return arr;
}

void setup()
{
 // Setup and write initial joint positions
 Serial.begin(9600);
 Joint1.attach(Joint1Pin);
 Joint2.attach(Joint2Pin);
 Joint3.attach(Joint3Pin);
 Gripper.attach(GripperPin);

 calculate_theta_1();
 calculate_theta_2();
 calculate_theta_3();
 
 Joint1.write(theta_1+Joint1Offset); 
 Joint2.write(theta_2+Joint2Offset); 
 Joint3.write(theta_3+Joint3Offset); 
 Gripper.write(GripperOpen);

 // Pre-calculate the projected trajectory between the two points
 x_points = calculate_projected_trajectory(x_start, x_finish, tf, (int) num_points);
 y_points = calculate_projected_trajectory(y_start, y_finish, tf, (int) num_points);
 z_points = calculate_projected_trajectory(z_start, z_finish, tf, (int) num_points);

 // Wait the 10 seconds as required.
 delay(10000);
}

void loop()
{
 //////////////////////
 // Forward Movement //
 //////////////////////

 // 5s with 0.1s granularity therefore delay of 100ms between each movement.
 for (int i = 0; i < num_points; i++)
 {
  x = x_points[i];
  y = y_points[i];
  z = z_points[i];

  calculate_theta_1();
  calculate_theta_2();
  calculate_theta_3();
 
  Joint1.write(theta_1+Joint1Offset); 
  Joint2.write(theta_2+Joint2Offset); 
  Joint3.write(theta_3+Joint3Offset); 

  delay(100);
 }

 delay(10000); // Wait 10 seconds at point

 //////////////////////
 // Reverse Movement //
 //////////////////////
 
 // 5s with 0.1s granularity therefore delay of 100ms between each movement.
 for (int i = num_points - 1; i >= 0; i--)
 {
  x = x_points[i];
  y = y_points[i];
  z = z_points[i];

  calculate_theta_1();
  calculate_theta_2();
  calculate_theta_3();
 
  Joint1.write(theta_1+Joint1Offset); 
  Joint2.write(theta_2+Joint2Offset); 
  Joint3.write(theta_3+Joint3Offset); 

  delay(100);
 }

 delay(10000); // Wait 10 seconds at point
}