% Set Parameters
x = -60;
y = 60;
z = -60;
L1 = 100;
L2 = 170;

% Calculate theta3
theta3 = acos((x^2+y^2+z^2-L1^2-L2^2)/(2*L1*L2))*(180/pi);
theta3_not = -theta3;

% Calculate theta2
phi = atan(-z/sqrt(x^2+y^2))*(180/pi);
beta = (acos((L1^2 + x^2 + y^2 + (z)^2 - L2^2)/(2*L1*sqrt(x^2 +y^2 +(z)^2))))*(180/pi);
theta2 = phi + beta;
theta2_not = phi - beta;

% Calculate theta1
theta1 = acos(x/sqrt(x^2 + y^2))*(180/pi);
theta1_not = theta1 + 180;


% Finding the angles for robot

% Case 1: Elbow up, non-inversed base
case1_1 = theta1;
case1_2 = theta2;
case1_3 = theta3;

% Case 2: Elbow down, non-inversed base
case2_1 = theta1;
case2_2 = theta2_not;
case2_3 = theta3_not;

% Case 3: Elbow up, Inversed base
case3_1 = theta1_not;
case3_2 = 180 - theta2;
case3_3 = theta3_not;

% Case 4: Elbow down, inversed base
case4_1 = theta1_not;
case4_2 = 180 - theta2_not;
case4_3 = theta3;



