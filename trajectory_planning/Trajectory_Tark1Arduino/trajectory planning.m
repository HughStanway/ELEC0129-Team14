%boundary condition
x0=0
xf=90
y0=0
yf=90
z0=0
zf=90
tf=5
%time array
t=0:0.1:tf
t=t'
%function x
X0=x0
X1=0
X2=3/tf^2*(xf-x0)
X3=-2/tf^3*(xf-x0)
Xcubic=X0*ones(length(t),1)+X1*t+X2*t.^2+X3*t.^3
Xdotcubit=X1+2*X2*t+3*X3*t.^2
XDoublecubic=2*X2+6*X3*t
figure,sgtitle('cubic')
subplot(1,3,1),plot(t,Xcubic),title('position')
subplot(1,3,2),plot(t,Xdotcubit),title('Velocity')
subplot(1,3,3),plot(t,XDoublecubic),title('acceleration')
%Function y
Y0=y0
Y1=0
Y2=3/tf^2*(yf-y0)
Y3=-2/tf^3*(yf-y0)
Ycubic=Y0*ones(length(t),1)+Y1*t+Y2*t.^2+Y3*t.^3
Ydotcubit=Y1+2*Y2*t+3*Y3*t.^2
YDoublecubic=2*Y2+6*Y3*t
figure,sgtitle('cubic')
subplot(1,3,1),plot(t,Ycubic),title('position')
subplot(1,3,2),plot(t,Ydotcubit),title('Velocity')
subplot(1,3,3),plot(t,YDoublecubic),title('acceleration')
%Function z
Z0=z0
Z1=0
Z2=3/tf^2*(zf-z0)
Z3=-2/tf^3*(zf-z0)
Zcubic=Z0*ones(length(t),1)+Z1*t+Z2*t.^2+Z3*t.^3
Zdotcubit=Z1+2*Z2*t+3*Z3*t.^2
ZDoublecubic=2*Z2+6*Z3*t
figure,sgtitle('cubic')
subplot(1,3,1),plot(t,Zcubic),title('position')
subplot(1,3,2),plot(t,Zdotcubit),title('Velocity')
subplot(1,3,3),plot(t,ZDoublecubic),title('acceleration')

