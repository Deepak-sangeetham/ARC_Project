clear;

%% parameters

m1 = 6;
m2 = 4;
l1 = 0.6;
l2 = 0.4;
g = 9.81;
r1 = 0.3;
r2 = 0.2;
I1 = 0.43;
I2 = 0.05;
mp = 0.5;

% max estimation error of parameters
r1_error = 0.3;
r2_error = 0.25;
I1_error = 0.48;
I2_error = 0.14;
mp_error = 2;

L = [11 0; 0 11];

%% dynamics of 2R manipulator

% M matrix: mass matrix of the robot
m11 = @(q1,q2) I1 + I2 + m1*r1^2 + m2*(l1^2 + r2^2 + 2*l1*r2*cos(q2)) + mp*(l1^2 + l2^2 + 2*l1*l2*cos(q2));
m12 = @(q1,q2) I2 + m2*r2*(r2+l1*cos(q2)) + mp*(l2^2 + l1*l2*cos(q2));
m22 = @(q1,q2) I2 + m2*r2^2 + mp*l2^2;

m22 = @(q1, q2) [m11(q1, q2) m12(q1, q2); m12(q1, q2) m22(q1, q2)];

% C matrix: centripetal & Coriolis forces
c11 = @(q1, q2, q1_dot, q2_dot) -l1 * (m2 * r2 + mp*l2) * sin(q2) * q2_dot;
c12 = @(q1, q2, q1_dot, q2_dot) -l1 * (m2 * r2 + mp*l2) * sin(q2) * (q2_dot + q1_dot);
c21 = @(q1, q2, q1_dot, q2_dot) -l1 * (m2 * r2 + mp*l2) * sin(q2) * q1_dot;
c22 = @(q1, q2, q1_dot, q2_dot) 0;

C = @(q1, q2, q1_dot, q2_dot) [c11(q1, q2, q1_dot, q2_dot) c12(q1, q2, q1_dot, q2_dot); c21(q1, q2, q1_dot, q2_dot) c22(q1, q2, q1_dot, q2_dot)];

% g array: gravitational forces
g1 = @(q1, q2) (m2 * r2 + mp*l2) * g * cos(q1 + q2) + (m2 * l1 + m1 * r1 + mp*l1) * g * cos(q1);
g2 = @(q1, q2) (m2 * r2 + mp*l2) * g * cos(q1 + q2);

g = @(q1, q2) [g1(q1, q2); g2(q1, q2)];

%% maximum value of matrices

% M maximum error
m11_error = @(q1, q2) m1 * r1_error + m2 * (r2_error + 2 * l1 * cos(q2) * r2_error) + ...
            mp_error * (l1^2 + l2^2 + 2 * l1 * l2 * cos(q2)) + I1_error + I2_error;
m12_error = @(q1, q2) m2 * (r2_error + l1 * cos(q2) * r2_error) + ...
            l2 * mp_error * (l2 + l1 * cos(q2)) + I2_error;
m22_error = @(q1, q2) ...
              m2 * r2_error + l2^2 * mp_error + I2_error;

M_max = @(q1, q2) ...
         [m11_error(q1, q2) m12_error(q1, q2); ...
          m12_error(q1, q2) m22_error(q1, q2)];

% C maximum error
c11 = @(q1, q2, q1_dot, q2_dot) ...
        -q2_dot;
    
c12 = @(q1, q2, q1_dot, q2_dot) ...
        -(q2_dot + q1_dot);
    
c21 = @(q1, q2, q1_dot, q2_dot) ...
        q1_dot;
    
C_max = @(q1, q2, q1_dot, q2_dot) ...
          (m2 * r2_error + mp_error * l2) * (l1 * sin(q2)) * ... 
          [c11(q1, q2, q1_dot, q2_dot) c12(q1, q2, q1_dot, q2_dot); ...
           c21(q1, q2, q1_dot, q2_dot) 0];
      
% g maximum error
g1_error = @(q1, q2) ...
            (m2 * r2_error + mp_error * l2) * g * cos(q1 + q2) + ... 
            (mp_error * l1 + m1 * r1_error + m2*l1) * g * cos(q1);

g2_error = @(q1, q2) ...
            (m2 * r2_error + mp_error * l2) * g * cos(q1 + q2);

G_max = @(q1, q2) [g1_error(q1, q2); 
                    g2_error(q1, q2)];
%% controller

% e, e_dot (e = q - qd)
e = @(q, qd) q - qd;
e_dot = @(q_dot, qd_dot) q_dot - qd_dot;

% sliding surface of the system
s = @(q, q_dot, qd, qd_dot) e_dot(q_dot, qd_dot) + L * e(q, qd);

%% desired path ( a circle)

qd = @(t) [0.8* sin(0.2*pi*t); ...
           0.8 * cos(0.2*pi*t)];
   
qd_dot = @(t) [0.16 * pi * cos(0.2*pi*t); ...
               -0.16 * pi * sin(0.2*pi*t)];
    
qd_ddot = @(t) [-0.032 * pi * sin(0.2*pi*t); ...
                -0.032 * pi * cos(0.2*pi*t)];

%% control law
K = [K0, K1, K2];
alpha = [5,5,5];
r = K0 + K1*norm(s) + K2*norm(s)^2;
u = @(q, q_dot, qd, qd_dot, qd_ddot)- Delta * s - r*sign(s);

 dK = [alpha[0] * (abs(s) - K[0]),
          alpha[1] * (abs(s) - K[1]),
          alpha[2] * (abs(s) - K[2])]
