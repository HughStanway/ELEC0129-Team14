clc
L1=97
L2=175
a0=0
a1=0
a2=97
alpha0=0
alpha1=pi/2
alpha2=-pi
d1=0
d2=0
d3=0
theta1=(90/360)*2*pi
theta2=(90/360)*2*pi
theta3=(90/360)*2*pi
T1=[cos(theta1),-sin(theta1),0,a0;cos(alpha0)*sin(theta1),cos(alpha0)*cos(theta1),-sin(alpha0),-sin(alpha0)*d1;sin(alpha0)*sin(theta1),sin(alpha0)*cos(theta1),cos(alpha0),cos(alpha0)*d1;0,0,0,1]
T2=[cos(theta2),-sin(theta2),0,a1;cos(alpha1)*sin(theta2),cos(alpha1)*cos(theta2),-sin(alpha1),-sin(alpha1)*d2;sin(alpha1)*sin(theta2),sin(alpha1)*cos(theta2),cos(alpha1),cos(alpha2)*d2;0,0,0,1]
T3=[cos(theta3),-sin(theta3),0,a2;cos(alpha2)*sin(theta3),cos(alpha2)*cos(theta3),-sin(alpha2),-sin(alpha2)*d3;sin(alpha2)*sin(theta3),sin(alpha2)*cos(theta3),cos(alpha2),cos(alpha2)*d3;0,0,0,1] 
T_final=T1*T2*T3
T_END_EFFOCTOR=[L2*sin(theta3),0,L2*cos(theta3),1]
T_position=T_final.*T_END_EFFOCTOR