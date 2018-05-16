import ETS3.*


% clear enviorment  -------------------------------------------------------
clear;
clc;

% set all parameters SI values  -------------------------------------------
a_tp0 = 0.101;
d_tp1 = 0;
a_tp2 = 0.145;
a_tp3 = 0.122;
a_tp4 = 0.135;

m_tp1 = (1);
m_tp2 = (1);
m_tp3 = (1);
m_tp4 = (1);

I_tp1 = eye(3);
I_tp2 = eye(3);
I_tp3 = eye(3);
I_tp4 = eye(3);

%a_pp0 = a_tp0;
%d_pp0 = 0.470;
%d_pp1 = d_tp1;
%a_pp2 = a_tp2;
%a_pp3 = 0.260;


% configure DH model ------------------------------------------------------
%gdzie dodac odsuniecie l_5 z pracy przejsciowej?

syms theta_tp1 theta_tp2 theta_tp3 theta_tp4 real
syms theta_tp1_dot theta_tp2_dot theta_tp3_dot theta_tp4_dot real
%syms theta_pp1 theta_pp2 theta_pp3

theta = [ theta_tp1 theta_tp2 theta_tp3 theta_tp4 ];
theta_dot = [ theta_tp1_dot theta_tp2_dot theta_tp3_dot theta_tp4_dot ];

tp_dh_parameters = [    %sym(-pi/2)  a_tp0   0       sym(-pi/2);%its only base transform
                        sym(-pi/2)  0       d_tp1   theta_tp1;
                        0           a_tp2   0       sym(-pi/2) + theta_tp2;
                        0           a_tp3   0       theta_tp3;
                        0           a_tp4   0       theta_tp4;    ];
%pp_dh_parameters = [    sym(-pi/2)  a_pp0   d_pp0   sym(-pi/2);
%                        sym(-pi/2)  0       d_pp1   theta_pp1;
%                        0           a_pp2   0       sym(-pi/2) + theta_pp2;
%                        0           a_pp3   0       theta_pp3;    ];

            

% calculate ---------------------------------------------------------------

% Kinematics: End Frame Homogeneous Transformations
T = simplify(calculate_transformation_matrix(tp_dh_parameters));

% Kinematics: End Frame Position
t = get_translation_matrix(T);

% Kinematics: Linear Velocity Jacobian
J_v = simplify(calculate_jacobian_v(t, theta));

% Kinematics: Angular Velocity Jacobian
J_w = simplify(calculate_jacobian_w(tp_dh_parameters));

% Kinematics: Combined Velocity Jacobian
J = [ J_v;
      J_w ]




















