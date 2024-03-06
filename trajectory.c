#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* Change the starting [x,y,z], final [x,y,z] position and the time (tf) */

// starting X, Y, Z Values (Change these values to starting position) 
float x_start = 90;
float y_start = 90;
float z_start = 90;

// starting X, Y, Z Values (Change these values to starting position) 
float x_finish = 120;
float y_finish = 90;
float z_finish = 90;

// Time of projection
int tf = 5;

/* Don't change these values as they are constants or defined by the values above */

// These will be the [x,y,z] values used to calculate theta at each point
int x;
int y;
int z;

// Link lengths (mm)
float l1 = 90;
float l2 = 170;

// Declare thetas which will be the joint angles
float theta_1;
float theta_2;
float theta_3;

// Trajectory point granularity and num points
float granularity = 0.1;
float num_points;

// Declare array for storing trajectory projection
float *x_points;
float *y_points;
float *z_points;

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

int main()
{
 // Set the initial values
 x = x_start;
 y = y_start;
 z = z_start;
 num_points = tf / granularity;

 x_points = calculate_projected_trajectory(x_start, x_finish, tf, (int) num_points);
 y_points = calculate_projected_trajectory(y_start, y_finish, tf, (int) num_points);
 z_points = calculate_projected_trajectory(z_start, z_finish, tf, (int) num_points);

 for (int i = 0; i < 50; i++) {
    printf("%d - X: %f, Y: %f, Z: %f\n", i, x_points[i], y_points[i], z_points[i]);
 }

 printf("\n");

 for (int i = 49; i >= 0; i--) {
    printf("%d - X: %f, Y: %f, Z: %f\n", i, x_points[i], y_points[i], z_points[i]);
 }
}