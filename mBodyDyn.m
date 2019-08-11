function...
    [w1_d,v1_d,Th_dd]...
    = mBodyDyn...
    (F1,T1,w1,v1,Th,Th_d)

%% 
%MBODYDYN function houses the equations of motions derived for the
% spacecraft with six rigid body appendages representing the solar panels
% and yokes. These appendages are connected to the body via spherical
% joints. The bodies in the system are numbered as
%   B1 : main body
%   B2 : yoke on array 1
%   B3 : solar panel #1 on array 1
%   B4 : solar panel #2 on array 1
%   B5 : yoke on array 2
%   B3 : solar panel #1 on array 2
%   B4 : solar panel #2 on array 2
% These bodies are connected via joints:
%   G1 : spherical joint connecting B1 and B2
%   G2 : spherical joint connecting B2 and B3
%   G3 : spherical joint connecting B3 and B4
%   G4 : spherical joint connecting B1 and B5
%   G5 : spherical joint connecting B5 and B6
%   G6 : spherical joint connecting B6 and B7
% the frame in which a vector quantity is represented is given in
% brackets []. The units are given in parantheses.
%
% INPUTS:
%   F1   : External forces applied on B1 [B1] (Nm)
%   T1   : External torque applied on B1 [B1] (Nm)
%   Th   : Vector containing angular position of joints [Bo] (rad)
%   Th_d : Vector containing angular veloctiy of joints [Bo] (rad/s)
% OUTPUTS:
%   w1_d : angular accel. of B1 [B1] (rad/s)
%   Th_dd: Vector containing angular accel of joints [Bo] (rad/s2)

%% match variables with inputs

% w1 = zeros(3,1) ;
% w2 = zeros(3,1) ;
% w3 = zeros(3,1) ;
% w4 = zeros(3,1) ;
% w5 = zeros(3,1) ;
% w6 = zeros(3,1) ;
% w7 = zeros(3,1) ;

% w1(:,1) = w1(1:3,1) ;
% w2(:,1) = w(4:6,1) ;
% w3(:,1) = w(7:9,1) ;
% w4(:,1) = w(10:12,1) ;
% w5(:,1) = w(13:15,1) ;
% w6(:,1) = w(16:18,1) ;
% w7(:,1) = w(19:21,1) ;

%% Moment of inertia, mass & other geometric information

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

%% position vectors
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

%% transformation matrices

C_21 = [cos(Th(1)),   sin(Th(1)),   0;
        -sin(Th(1)),  cos(Th(1)),   0;
        0,          0,          1] ; % transformatiom from B1 to B2 
    
C_32 = [cos(Th(2)),   sin(Th(2)),   0;
        -sin(Th(2)),  cos(Th(2)),   0;
        0,          0,          1] ; % transformatiom from B2 to B3

C_43 = [cos(Th(3)),   sin(Th(3)),   0;
        -sin(Th(3)),  cos(Th(3)),   0;
        0,          0,          1] ; % transformatiom from B3 to B4
    
C_51 = [cos(Th(4)),   sin(Th(4)),   0;
        -sin(Th(4)),  cos(Th(4)),   0;
        0,          0,          1] ; % transformatiom from B1 to B5
    
C_65 = [cos(Th(5)),   sin(Th(5)),   0;
        -sin(Th(5)),  cos(Th(5)),   0;
        0,          0,          1] ; % transformatiom from B5 to B6

C_76 = [cos(Th(6)),   sin(Th(6)),   0;
        -sin(Th(6)),  cos(Th(6)),   0;
        0,          0,          1] ; % transformatiom from B5 to B7     
    
C_12 = C_21' ; % transformatiom from B2 to B1
    
C_23 = C_32' ; % transformatiom from B3 to B2

C_34 = C_43' ; % transformatiom from B4 to B3

C_15 = C_51' ; % transformatiom from B5 to B1

C_56 = C_65' ; % transformatiom from B6 to B5

C_67 = C_76' ; % transformatiom from B7 to B6

%% Intermediate terms

u1 = [0; 0; Th_d(1)] ;
u2 = [0; 0; Th_d(2)] ;
u3 = [0; 0; Th_d(3)] ;
u4 = [0; 0; Th_d(4)] ;
u5 = [0; 0; Th_d(5)] ;
u6 = [0; 0; Th_d(6)] ;

w2 = C_21*w1 + u1 ;
w3 = C_32*w2 + u2 ;
w4 = C_43*w3 + u3 ;
w5 = C_51*w1 + u4 ;
w6 = C_65*w5 + u5 ;
w7 = C_76*w6 + u6 ;

%% Assumptions

% no body forces are being applied to the bodies
% F1 = zeros(3,1) ;
F2 = zeros(3,1) ;
F3 = zeros(3,1) ;
F4 = zeros(3,1) ;
F5 = zeros(3,1) ;
F6 = zeros(3,1) ;
F7 = zeros(3,1) ;

% no body torques are being applied to the bodies (except B1)
T2 = zeros(3,1) ;
T3 = zeros(3,1) ;
T4 = zeros(3,1) ;
T5 = zeros(3,1) ;
T6 = zeros(3,1) ;
T7 = zeros(3,1) ;

%% Equations of motion

Zm = zeros(3,3) ;
Zv = zeros(3,1) ;

% contstructing the system matrix

A_11 = [...
    I1, Zm, Zm, Zm, Zm, Zm, Zm;
    Zm, I2, Zm, Zm, Zm, Zm, Zm;
    Zm, Zm, I3, Zm, Zm, Zm, Zm;
    Zm, Zm, Zm, I4, Zm, Zm, Zm;
    Zm, Zm, Zm, Zm, I5, Zm, Zm;
    Zm, Zm, Zm, Zm, Zm, I6, Zm;
    Zm, Zm, Zm, Zm, Zm, Zm, I7...
    ];

A_12 = zeros(21,21) ;

A_13 = [...
    -skew(r12)*C_12, Zm, Zm, -skew(r15)*C_15, Zm, Zm ;
    skew(r21), -skew(r23)*C_23, Zm, Zm, Zm, Zm ;
    Zm, skew(r32), -skew(r34)*C_34, Zm, Zm, Zm ;
    Zm, Zm, skew(r43), Zm, Zm, Zm ;
    Zm, Zm, Zm, skew(r51), -skew(r56)*C_56, Zm ;
    Zm, Zm, Zm, Zm, skew(r65), -skew(r67)*C_67 ;
    Zm, Zm, Zm, Zm, Zm, skew(r76)...
    ];

A_14 = [...
    -C_12, Zm, Zm, -C_15, Zm, Zm ;
    eye(3), -C_23, Zm, Zm, Zm, Zm ;
    Zm, eye(3), -C_34, Zm, Zm, Zm ;
    Zm, Zm, eye(3), Zm, Zm, Zm ;
    Zm, Zm, Zm, eye(3), -C_56, Zm ;
    Zm, Zm, Zm, Zm, eye(3), -C_67 ;
    Zm, Zm, Zm, Zm, Zm, eye(3)...
    ] ;
    
A_15 = zeros(21,6) ;

A_21 = zeros(21,21) ;

A_22 = [...
    m1*eye(3), Zm, Zm, Zm, Zm, Zm, Zm ;
    Zm, m2*eye(3), Zm, Zm, Zm, Zm, Zm ;
    Zm, Zm, m3*eye(3), Zm, Zm, Zm, Zm ;
    Zm, Zm, Zm, m4*eye(3), Zm, Zm, Zm ;
    Zm, Zm, Zm, Zm, m5*eye(3), Zm, Zm ;
    Zm, Zm, Zm, Zm, Zm, m6*eye(3), Zm ;
    Zm, Zm, Zm, Zm, Zm, Zm, m7*eye(3)...
    ] ;

A_23 = [...
    -C_12, Zm, Zm, -C_15, Zm, Zm ;
    eye(3), -C_23, Zm, Zm, Zm, Zm ;
    Zm, eye(3), -C_34, Zm, Zm, Zm ;
    Zm, Zm, eye(3), Zm, Zm, Zm ;
    Zm, Zm, Zm, eye(3), -C_56, Zm ;
    Zm, Zm, Zm, Zm, eye(3), -C_67 ;
    Zm, Zm, Zm, Zm, Zm, eye(3)...
    ] ;

A_24 = zeros(21,18) ;

A_25 = zeros(21,6) ;

A_31 = [...
    -skew(r12), C_12*skew(r21), Zm, Zm, Zm, Zm, Zm ;
    Zm, -skew(r23), C_23*skew(r32), Zm, Zm, Zm, Zm ;
    Zm, Zm, -skew(r34), C_34*skew(r43), Zm, Zm, Zm ;
    -skew(r15), Zm, Zm, Zm, C_15*skew(r51), Zm, Zm ;
    Zm, Zm, Zm, Zm, -skew(r56), C_56*skew(r65), Zm ;
    Zm, Zm, Zm, Zm, Zm, -skew(r67), C_67*skew(r76)...
    ] ;

A_32 = [...
    eye(3), -C_12, Zm, Zm, Zm, Zm, Zm ;
    Zm, eye(3), -C_23, Zm, Zm, Zm, Zm ;
    Zm, Zm, eye(3), -C_34, Zm, Zm, Zm ;
    eye(3), Zm, Zm, Zm, -C_15, Zm, Zm ;
    Zm, Zm, Zm, Zm, eye(3), -C_56, Zm ;
    Zm, Zm, Zm, Zm, Zm, eye(3), -C_67...
    ] ; 

A_33 = zeros(18,18) ;

A_34 = zeros(18,18) ;

A_35 = zeros(18,6) ;

A_41 = [...
    -[1 0 0]*C_21, [1 0 0], Zv', Zv', Zv', Zv', Zv' ;
    -[0 1 0]*C_21, [0 1 0], Zv', Zv', Zv', Zv', Zv' ;
    -[0 0 1]*C_21, [0 0 1], Zv', Zv', Zv', Zv', Zv' ;
    Zv', -[1 0 0]*C_32, [1 0 0], Zv', Zv', Zv', Zv' ;
    Zv', -[0 1 0]*C_32, [0 1 0], Zv', Zv', Zv', Zv' ;
    Zv', -[0 0 1]*C_32, [0 0 1], Zv', Zv', Zv', Zv' ;
    Zv', Zv', -[1 0 0]*C_43, [1 0 0], Zv', Zv', Zv' ;
    Zv', Zv', -[0 1 0]*C_43, [0 1 0], Zv', Zv', Zv' ;
    Zv', Zv', -[0 0 1]*C_43, [0 0 1], Zv', Zv', Zv' ;
    -[1 0 0]*C_51, Zv', Zv', Zv', [1 0 0], Zv', Zv' ;
    -[0 1 0]*C_51, Zv', Zv', Zv', [0 1 0], Zv', Zv' ;
    -[0 0 1]*C_51, Zv', Zv', Zv', [0 0 1], Zv', Zv' ;
    Zv', Zv', Zv', Zv', -[1 0 0]*C_65, [1 0 0], Zv' ;
    Zv', Zv', Zv', Zv', -[0 1 0]*C_65, [0 1 0], Zv' ;
    Zv', Zv', Zv', Zv', -[0 0 1]*C_65, [0 0 1], Zv' ;
    Zv', Zv', Zv', Zv', Zv', -[1 0 0]*C_76, [1 0 0] ;
    Zv', Zv', Zv', Zv', Zv', -[0 1 0]*C_76, [0 1 0] ;
    Zv', Zv', Zv', Zv', Zv', -[0 0 1]*C_76, [0 0 1]...
    ] ;

A_42 = zeros(18,21) ;

A_43 = zeros(18,18) ;

A_44 = zeros(18,18) ;

A_45 = [...
    zeros(1,6) ;
    zeros(1,6) ;
    -1, 0, 0, 0, 0, 0 ;
    zeros(1,6) ;
    zeros(1,6) ;
    0, -1, 0, 0, 0, 0 ;
    zeros(1,6) ;
    zeros(1,6) ;
    0, 0, -1, 0, 0, 0 ;
    zeros(1,6) ;
    zeros(1,6) ;
    0, 0, 0, -1, 0, 0 ;
    zeros(1,6) ;
    zeros(1,6) ;
    0, 0, 0, 0, -1, 0 ;
    zeros(1,6) ;
    zeros(1,6) ;
    0, 0, 0, 0, 0, -1 ...
    ] ;

A_51 = zeros(6,21) ;

A_52 = zeros(6,21) ;

A_53 = zeros(6,18) ;

A_54 = [...
    [0 0 1], Zv', Zv', Zv', Zv', Zv' ;
    Zv', [0 0 1], Zv', Zv', Zv', Zv' ;
    Zv', Zv', [0 0 1], Zv', Zv', Zv' ;
    Zv', Zv', Zv', [0 0 1], Zv', Zv' ;
    Zv', Zv', Zv', Zv', [0 0 1], Zv' ;
    Zv', Zv', Zv', Zv', Zv', [0 0 1]...
    ] ;

A_55 = zeros(6,6) ;
    
A = [...
    A_11, A_12, A_13, A_14, A_15 ;
    A_21, A_22, A_23, A_24, A_25 ;
    A_31, A_32, A_33, A_34, A_35 ;
    A_41, A_42, A_43, A_44, A_45 ;
    A_51, A_52, A_53, A_54, A_55...
    ] ;

% constructing the input vector

U_11 = [...
    T1 - skew(w1)*(I1*w1) ;
    T2 - skew(w2)*(I2*w2) ;
    T3 - skew(w3)*(I3*w3) ;
    T4 - skew(w4)*(I4*w4) ; 
    T5 - skew(w5)*(I5*w5) ;
    T6 - skew(w6)*(I6*w6) ;
    T7 - skew(w7)*(I7*w7)...
    ] ;

U_21 = [...
    F1 ;
    F2 ;
    F3 ;
    F4 ;
    F5 ;
    F6 ;
    F7...
    ] ;

U_31 = [...
    C_12*skew(w2)*(skew(w2)*r21) - skew(w1)*(skew(w1)*r12) ;
    C_23*skew(w3)*(skew(w3)*r32) - skew(w2)*(skew(w2)*r23) ;
    C_34*skew(w4)*(skew(w4)*r43) - skew(w3)*(skew(w3)*r34) ;
    C_15*skew(w5)*(skew(w5)*r51) - skew(w1)*(skew(w1)*r15) ;
    C_56*skew(w6)*(skew(w6)*r65) - skew(w5)*(skew(w5)*r56) ;
    C_67*skew(w2)*(skew(w7)*r76) - skew(w6)*(skew(w6)*r67)...
    ] ;

U_41 = [...
    skew(C_21*w1)*u1 ;
    skew(C_32*w2)*u2 ;
    skew(C_43*w3)*u3 ;
    skew(C_51*w1)*u4 ;
    skew(C_65*w5)*u5 ;
    skew(C_76*w6)*u6...
    ] ;

U_51 = [...
    nlSpring(Th(1),Th_d(1)) ;
    nlSpring(Th(2),Th_d(2)) ;
    nlSpring(Th(3),Th_d(3)) ;
    nlSpring(Th(4),Th_d(4)) ;
    nlSpring(Th(5),Th_d(5)) ;
    nlSpring(Th(6),Th_d(6))...
    ] ;

U = [...
    U_11 ;
    U_21 ;
    U_31 ;
    U_41 ;
    U_51...
    ] ;

% obtaining the output vector

X = inv(A)*U ;

%% Outputs

w1_d = zeros(3,1);
w1_d(:,1) = X(1:3,1);

v1_d = zeros(3,1);
v1_d(:,1) = X(22:24,1);

Th_dd = zeros(6,1) ;
Th_dd(:,1) = X(79:84,1) ;

%% Functions

% skew symmetric matrix generation
function [a] = skew(b)
a = [   0,      -b(3),  b(2);
        b(3),   0,      -b(1);
        -b(2),  b(1),   0       ];

function [T] = nlSpring(Th,Th_d)

% Linear spring and damper coefficients
k1 = 2 ; % (Nm/rad)
k2 = 100 ; % (Nm/rad)
b = 0 ; % (Nm-s/rad)

if Th >= -(pi/180)*(1/4) && Th <= (pi/180)*(1/4)
    T = Th*k1 + Th_d*b ;
else
    T = Th*k2 + Th_d*b;
end
