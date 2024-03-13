%boundary condition
x0_1=0
xf_1=90

x0_2=90
xf_2=0

%time array
tf=5
t=0:0.1:tf
t=t'

%function x for forward movement
X0=x0_1
X1=0
X2=3/tf^2*(xf_1-x0_1)
X3=-2/tf^3*(xf_1-x0_1)
Xcubic_f=X0*ones(length(t),1)+X1*t+X2*t.^2+X3*t.^3
Xdotcubit_f=X1+2*X2*t+3*X3*t.^2
XDoublecubic_f=2*X2+6*X3*t
figure,sgtitle('cubic')

%function x for reverse movement
X0=x0_2
X1=0
X2=3/tf^2*(xf_2-x0_2)
X3=-2/tf^3*(xf_2-x0_2)
Xcubic_r=X0*ones(length(t),1)+X1*t+X2*t.^2+X3*t.^3
Xdotcubit_r=X1+2*X2*t+3*X3*t.^2
XDoublecubic_r=2*X2+6*X3*t
figure,sgtitle('cubic')

% Merged graph
tmerge = 0:0.1:20.1
vector_90_reverse = 90 * ones(100, 1);
vector_0_reverse = 0 * ones(100, 1);
combined_Xcubic = [Xcubic_f; vector_90_reverse; Xcubic_r];
combined_Xcubic
combined_Xdotcubuc = [Xdotcubit_f; vector_0_reverse; Xdotcubit_r];
combined_Xdotcubuc
combined_XDoublecubuc = [XDoublecubic_f; vector_0_reverse; XDoublecubic_r];
combined_XDoublecubuc

subplot(1,3,1),plot(tmerge,combined_Xcubic),title('position')
subplot(1,3,2),plot(tmerge,combined_Xdotcubuc),title('Velocity')
subplot(1,3,3),plot(tmerge,combined_XDoublecubuc),title('acceleration')
