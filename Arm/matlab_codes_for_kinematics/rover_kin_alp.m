clc
clear
syms d0 d1 d2 d3 d4 real
syms q0 q1 q2 q3 q4 q5 q6 real
syms q0dot q1dot q2dot q3dot q4dot q5dot q6dot real

% Transformation matrices
T1 = [ RotateYaw(q0) [0;0;0] ; 0 0 0 1];
T2 = [ RotatePitch(q1) [0;0;0] ; 0 0 0 1];    
T3 = [ RotatePitch(q2) [0;0;d0] ; 0 0 0 1]; 
T4 = [ RotatePitch(q3) [0;0;d1] ; 0 0 0 1]; 
T5 = [ RotateYaw(q4) [0;0;d2] ; 0 0 0 1];
T6 = [ RotatePitch(q5) [0;0;d3]  ; 0 0 0 1];
T7 = [ RotateYaw(q6)  [0;0;0]  ; 0 0 0 1];
T8 = [ eye(3)          [0;0;d4] ; 0 0 0 1]; 

T = T1*T2*T3*T4*T5*T6*T7*T8;

P1 = simplify(T1*T2);
P2 = simplify(P1*T3);
P3 = simplify(P2*T4);
P4 = simplify(P3*T5);
P5 = simplify(P4*T6);
P6 = simplify(P5*T7);
P7 = simplify(P6*T8);

%Forward için (Bunlara simulastona aldým direkt
P1fwd = [P1(1,4) P1(2,4) P1(3,4)];
P2fwd = [P2(1,4) P2(2,4) P2(3,4)];
P3fwd = [P3(1,4) P3(2,4) P3(3,4)];
P4fwd = [P4(1,4) P4(2,4) P4(3,4)];
P5fwd = [P5(1,4) P5(2,4) P5(3,4)];
P6fwd = [P6(1,4) P6(2,4) P6(3,4)];
P7fwd = [P7(1,4) P7(2,4) P7(3,4)];

T = simplify(T);

% Getting end efector coordinates from transformation matrix.
Xe = T(1,4);
Ye = T(2,4);
Ze = T(3,4);

R = T(1:3,1:3);

% Contructing angular velocity matrix
SSM = simplify(doDiff(R)*transpose(R)); 

Wbx = SSM(3,2);
Wby = SSM(1,3);
Wbz = SSM(2,1);

%Building Jacobian
J = sym(zeros(4));
J(1,1) = diff(Xe,q0);
J(1,2) = diff(Xe,q1);
J(1,3) = diff(Xe,q2);
J(1,4) = diff(Xe,q3);
J(1,5) = diff(Xe,q4);
J(1,6) = diff(Xe,q5);
J(1,7) = diff(Xe,q6);

J(2,1) = diff(Ye,q0);
J(2,2) = diff(Ye,q1);
J(2,3) = diff(Ye,q2);
J(2,4) = diff(Ye,q3);
J(2,5) = diff(Ye,q4);
J(2,6) = diff(Ye,q5);
J(2,7) = diff(Ye,q6);

J(3,1) = diff(Ze,q0);
J(3,2) = diff(Ze,q1);
J(3,3) = diff(Ze,q2);
J(3,4) = diff(Ze,q3);
J(3,5) = diff(Ze,q4);
J(3,6) = diff(Ze,q5);
J(3,7) = diff(Ze,q6);

J(4,1) = diff(Wbx,q0dot);
J(4,2) = diff(Wbx,q1dot);
J(4,3) = diff(Wbx,q2dot);
J(4,4) = diff(Wbx,q3dot);
J(4,5) = diff(Wbx,q4dot);
J(4,6) = diff(Wbx,q5dot);
J(4,7) = diff(Wbx,q6dot);

J(5,1) = diff(Wby,q0dot);
J(5,2) = diff(Wby,q1dot);
J(5,3) = diff(Wby,q2dot);
J(5,4) = diff(Wby,q3dot);
J(5,5) = diff(Wby,q4dot);
J(5,6) = diff(Wby,q5dot);
J(5,7) = diff(Wby,q6dot);

J(6,1) = diff(Wbz,q0dot);
J(6,2) = diff(Wbz,q1dot);
J(6,3) = diff(Wbz,q2dot);
J(6,4) = diff(Wbz,q3dot);
J(6,5) = diff(Wbz,q4dot);
J(6,6) = diff(Wbz,q5dot);
J(6,7) = diff(Wbz,q6dot);

J(7,1) = 0;
J(7,2) = 0;
J(7,3) = 0;
J(7,4) = 0;
J(7,5) = 0;
J(7,6) = 0;
J(7,7) = 1;

J = simplify(J);
Jinv = simplify(inv(J));
