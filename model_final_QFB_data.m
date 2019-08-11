clear all
%**************************%
%-Properties of the Bodies-%
%**************************%

% B1
m1 = 420 ; % mass (kg)
W_1 = 1 ; % body width (m)
L_1 = 1 ; % body length (m)
H_1 = 1.4 ; % body height (m)
I1x = m1*(1/12)*((W_1)^2+(H_1)^2) ;
I1y = m1*(1/12)*((L_1)^2+(H_1)^2) ;
I1z = m1*(1/12)*((W_1)^2+(L_1)^2) ;
I1 = [I1x,0,0;0,I1y,0;0,0,I1z] ; % moment of inertia (kg-m2)

% B2
m2 = 5 ; % mass (kg)
W_2 = 0.04 ; % body width (m)
L_2 = 0.5 ; % body length (m)
H_2 = 0.5 ; % body height (m)
I2x = m2*(1/12)*((W_2)^2+(H_2)^2) ;
I2y = m2*(1/12)*((L_2)^2+(H_2)^2) ;
I2z = m2*(1/12)*((W_2)^2+(L_2)^2) ;
I2 = [I2x,0,0;0,I2y,0;0,0,I2z] ; % moment of inertia (kg-m2)

% B3
m3 = 10 ; % mass (kg)
W_3 = 0.02 ; % body width (m)
L_3 = 1.0 ; % body length (m)
H_3 = 1.1 ; % body height (m)
I3x = m3*(1/12)*((W_3)^2+(H_3)^2) ;
I3y = m3*(1/12)*((L_3)^2+(H_3)^2) ;
I3z = m3*(1/12)*((W_3)^2+(L_3)^2) ;
I3 = [I3x,0,0;0,I3y,0;0,0,I3z] ; % moment of inertia (kg-m2)

% B4
m4 = 10 ; % mass (kg)
W_4 = 0.02 ; % body width (m)
L_4 = 1.0 ; % body length (m)
H_4 = 1.1 ; % body height (m)
I4x = m4*(1/12)*((W_4)^2+(H_4)^2) ;
I4y = m4*(1/12)*((L_4)^2+(H_4)^2) ;
I4z = m4*(1/12)*((W_4)^2+(L_4)^2) ;
I4 = [I4x,0,0;0,I4y,0;0,0,I4z] ; % moment of inertia (kg-m2)

% B5
m5 = 5 ; % mass (kg)
W_5 = 0.04 ; % body width (m)
L_5 = 0.5 ; % body length (m)
H_5 = 0.5 ; % body height (m)
I5x = m5*(1/12)*((W_5)^2+(H_5)^2) ;
I5y = m5*(1/12)*((L_5)^2+(H_5)^2) ;
I5z = m5*(1/12)*((W_5)^2+(L_5)^2) ;
I5 = [I5x,0,0;0,I5y,0;0,0,I5z] ; % moment of inertia (kg-m2)

% B6
m6 = 10 ; % mass (kg)
W_6 = 0.02 ; % body width (m)
L_6 = 1.0 ; % body length (m)
H_6 = 1.1 ; % body height (m)
I6x = m6*(1/12)*((W_6)^2+(H_6)^2) ;
I6y = m6*(1/12)*((L_6)^2+(H_6)^2) ;
I6z = m6*(1/12)*((W_6)^2+(L_6)^2) ;
I6 = [I6x,0,0;0,I6y,0;0,0,I6z] ; % moment of inertia (kg-m2)

% B7
m7 = 10 ; % mass (kg)
W_7 = 0.02 ; % body width (m)
L_7 = 1.0 ; % body length (m)
H_7 = 1.1 ; % body height (m)
I7x = m7*(1/12)*((W_7)^2+(H_7)^2) ;
I7y = m7*(1/12)*((L_7)^2+(H_7)^2) ;
I7z = m7*(1/12)*((W_7)^2+(L_7)^2) ;
I7 = [I7x,0,0;0,I7y,0;0,0,I7z] ; % moment of inertia (kg-m2)

% position vectors
r12 = [-L_1/2; 0; -(H_1/2-H_3/2)] ; 
r15 = [L_1/2; 0; -(H_1/2-H_5/2)] ;
r21 = [L_2/2; 0; 0] ;
r23 = [-L_2/2; 0; 0] ;
r32 = [L_3/2; 0; 0] ;
r34 = [-L_3/2; 0; 0] ;
r43 = [L_4/2; 0; 0] ;
r51 = [-L_5/2; 0; 0] ;
r56 = [L_5/2; 0; 0] ;
r65 = [-L_6/2; 0; 0] ;
r67 = [L_6/2; 0; 0] ;
r76 = [-L_7/2; 0; 0] ;

%**********************************%
%-Calculate Total Mass and Inertia-%
%**********************************%

% calculate the position of each body (assuming joint angles = 0)
r02 = r12 - r21 ;
r03 = r02 + r23 - r32 ;
r04 = r03 + r34 - r43 ;
r05 = r15 - r51 ;
r06 = r05 + r56 - r65 ;
r07 = r06 + r67 - r76 ;

I_Tot_x = I1x + (I2x + m2*(r02(2)^2+r02(3)^2)) +...
    (I3x + m3*(r03(2)^2+r03(3)^2)) + (I4x + m4*(r04(2)^2+r04(3)^2)) +...
    (I5x + m5*(r05(2)^2+r05(3)^2)) + (I6x + m6*(r06(2)^2+r06(3)^2)) +...
    (I7x + m7*(r07(2)^2+r07(3)^2)) ;

I_Tot_y = I1y + (I2y + m2*(r02(1)^2+r02(3)^2)) +...
    (I3y + m3*(r03(1)^2+r03(3)^2)) + (I4y + m4*(r04(1)^2+r04(3)^2)) +...
    (I5y + m5*(r05(1)^2+r05(3)^2)) + (I6y + m6*(r06(1)^2+r06(3)^2)) +...
    (I7y + m7*(r07(1)^2+r07(3)^2)) ;

I_Tot_z = I1z + (I2z + m2*(r02(2)^2+r02(1)^2)) +...
    (I3z + m3*(r03(2)^2+r03(1)^2)) + (I4z + m4*(r04(2)^2+r04(1)^2)) +...
    (I5z + m5*(r05(2)^2+r05(1)^2)) + (I6z + m6*(r06(2)^2+r06(1)^2)) +...
    (I7z + m7*(r07(2)^2+r07(1)^2)) ;

I_Tot = [I_Tot_x,0,0;0,I_Tot_y,0;0,0,I_Tot_z] ; % total moment of inertia
m_Tot = m1 + m2 + m3 + m4 + m5 + m6 + m7 ; % total mass - 4 panels & 2 yokes

%********************%
%-Initial Conditions-%
%********************%

w_0 = [0,0,0]; %deg/s
eul_0 = [0,0,0]; %deg

%***************************%
%-Reaction Wheel Properties-%
%***************************%

% RW Coefficients
R = 4.0 ; % Ohm
Kv = 0.04 ; % V/(rad/s)
Km = 0.150 ; % Nm/A
B = 0.159*10^-5 ; % Nm/(rad/s)
Jw = 0.07500 ; % kg-m2
I_max = 1.80 ; % current limit [A]
V_max = 50 ; % voltage limit [V]
omg_max = 2500*2*pi/60 ; % angular velocity limit [rad/s]
T_max = 0.240 ; % torque limit [Nm]

% Initial Conditions
omg_w_0 = 1000*2*pi/60 ; % (rad/s)

% Calculare gain from settling time
ts = 0.05 ; % sec
K = R*(6/ts - B/Jw - (Kv*Km)/(Jw*R)) ;

% Different Reaction Wheel MOIs
Jw1 = Jw ;
Jw2 = Jw*1.010 ;
Jw3 = Jw*0.990 ;
Jw4 = Jw*1.005 ;

% RW Cluster Alignment Matrix
E = [   sqrt(2/3), -sqrt(2/3),  0,          0        ;
        -sqrt(1/3), -sqrt(1/3), sqrt(1/3),  sqrt(1/3);
        0,          0,         -sqrt(2/3),  sqrt(2/3)];

%*********************************************%    
%-Quaternion Feedback Controller Coefficients-%
%*********************************************%
Kp = 9 ;
Kd = 52.5 ;