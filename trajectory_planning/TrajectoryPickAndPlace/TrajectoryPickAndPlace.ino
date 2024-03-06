/////////////////////////
// Trajectory Planning //
/////////////////////////

#include <Servo.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////
// Change values below to specify start and finish cartesian points //
//////////////////////////////////////////////////////////////////////

// starting X, Y, Z Values (Change these values to starting position)
float x_start = 160;
float y_start = 0;
float z_start = -50;

// mid X, Y, Z Values (Change these values to starting position)
float x_finish = 160;
float y_finish = 0;
float z_finish = 50;

// final X, Y, Z Values (Change these values to starting position)
float x_finish_2 = 160;
float y_finish_2 = -100;
float z_finish_2 = -50;

///////////////////////////////////////////////////////////////////////////////
// Dont change any values below as they define robot constants and variables //
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
float l1 = 100;
float l2 = 170;

// Declare thetas which will be the joint angles
float theta_1;
float theta_2;
float theta_3;

// Gripper Joint Angles
int GripperOpen = 60; // Open gripper
int GripperClose = 150; // Close gripper

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

float *x_points_2;
float *y_points_2;
float *z_points_2;

void calculate_theta_1()
{
theta_1 = acos(y / sqrt(pow(x, 2) + pow(y, 2))) * (180 / PI);
}

void calculate_theta_2()
{
float phi = atan(z / sqrt(pow(x, 2) + pow(y, 2))) * (180 / PI);
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

void forward_trajectory_movement(float* x_arr, float* y_arr, float* z_arr)
{
 // 5s with 0.1s granularity therefore delay of 100ms between each movement.
 for (int i = 0; i < num_points; i++)
 {
  x = x_arr[i];
  y = y_arr[i];
  z = z_arr[i];

  calculate_theta_1();
  calculate_theta_2();
  calculate_theta_3();
  Joint1.write(theta_1+Joint1Offset);
  Joint2.write(theta_2+Joint2Offset);
  Joint3.write(theta_3+Joint3Offset);

  delay(100);
 }
}

void reverse_trajectory_movement(float* x_arr, float* y_arr, float* z_arr)
{
 for (int i = num_points - 1; i >= 0; i--)
 {
  x = x_arr[i];
  y = y_arr[i];
  z = z_arr[i];

  calculate_theta_1();
  calculate_theta_2();
  calculate_theta_3();
  Joint1.write(theta_1+Joint1Offset);
  Joint2.write(theta_2+Joint2Offset);
  Joint3.write(theta_3+Joint3Offset);

  delay(100);
 }
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

x_points_2 = calculate_projected_trajectory(x_finish, x_finish_2, tf, (int) num_points);
y_points_2 = calculate_projected_trajectory(y_finish, y_finish_2, tf, (int) num_points);
z_points_2 = calculate_projected_trajectory(z_finish, z_finish_2, tf, (int) num_points);

// Wait the 10 seconds as required.
Serial.println("Delay 10 sec");
delay(5000);
}

void loop()
{
//////////////////////
// Forward Movement //
//////////////////////
Gripper.write(GripperClose);

Serial.println("Start forward movement");
forward_trajectory_movement(x_points, y_points, z_points);
Serial.println("Forward halfway point");
forward_trajectory_movement(x_points_2, y_points_2, z_points_2);
Serial.println("End forward movement");

delay(1000);
Gripper.write(GripperOpen);
delay(1000);
Gripper.write(GripperClose);
delay(1000);

//////////////////////
// Reverse Movement //
//////////////////////

Serial.println("Start reverse movement");
reverse_trajectory_movement(x_points_2, y_points_2, z_points_2);
Serial.println("Reverse halfway point");
reverse_trajectory_movement(x_points, y_points, z_points);
Serial.println("End reverse movement");

delay(1000);
Gripper.write(GripperOpen);
delay(1000);
Gripper.write(GripperClose);
delay(1000);
}