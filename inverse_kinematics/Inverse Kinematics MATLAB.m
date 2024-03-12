% Finding the angles for robot

% Set Parameters
x = 90;
y = -60;
z = 97;
L1 = 90;
L2 = 170;

% Calculate theta3
theta3 = acos((x^2+y^2+z^2-L1^2-L2^2)/(2*L1*L2))*(180/pi);

% Calculate theta2
phi = atan(z/sqrt(x^2+y^2))*(180/pi);
beta = (acos((L1^2 + x^2 + y^2 + z^2 - L2^2)/(2*L1*sqrt(x^2 +y^2 +z^2))))*(180/pi);
theta2 = phi + beta;

% Calculate theta1
theta1 = acos(y/sqrt(x^2 + y^2))*(180/pi);




