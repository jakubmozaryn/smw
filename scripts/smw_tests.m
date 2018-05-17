import ETS3.*


% clear enviorment  -------------------------------------------------------
clear;
clc;

% test case models
syms a0 a1 a2 d0 d1 d2 q0 q1 q2 real

% test case 1
rrr_q = [ q0 q1 q2 ];
rrr_dh_parameters = [   0  a0   0   q0;
                        0  a1   0   q1; 
                        0  a2   0   q2; ];
                    
rrr_robot = SerialLink( [   Revolute('a', a0)
                            Revolute('a', a1)
                            Revolute('a', a2)
                        ], 'name', 'simple');
         
rrr_T = [ 
    cos(q0 + q1 + q2), -sin(q0 + q1 + q2), 0, a1*cos(q0 + q1) + a0*cos(q0) + a2*cos(q0 + q1 + q2);
    sin(q0 + q1 + q2),  cos(q0 + q1 + q2), 0, a1*sin(q0 + q1) + a0*sin(q0) + a2*sin(q0 + q1 + q2);
                    0,                  0, 1,                                                   0;
                    0,                  0, 0,                                                   1;];
                    
rrr_J = [ 
  - a1*sin(q0 + q1) - a0*sin(q0) - a2*sin(q0 + q1 + q2), - a1*sin(q0 + q1) - a2*sin(q0 + q1 + q2), -a2*sin(q0 + q1 + q2);
    a1*cos(q0 + q1) + a0*cos(q0) + a2*cos(q0 + q1 + q2),   a1*cos(q0 + q1) + a2*cos(q0 + q1 + q2),  a2*cos(q0 + q1 + q2);
                                                      0,                                        0,                     0;
                                                      0,                                        0,                     0;
                                                      0,                                        0,                     0;
                                                      1,                                        1,                     1;];                      

% test case 2
rrr2_q = [ q0 q1 q2 ];
rrr2_dh_parameters = [   
    sym(pi/2),  0, d0, q0;
            0, a1,  0, q1; 
            0, a2, d2, q2; ];
                    
rrr2_robot = SerialLink( [   
    Revolute('alpha', sym(pi/2), 'd', d0) 
    Revolute('a', a1)
    Revolute('a', a2, 'd', d2)
    ], 'name', 'simple');
         
rrr2_T = [ 
    cos(q0)*cos(q1 + q2), -cos(q0)*sin(q1 + q2),  sin(q0), a1*cos(q0)*cos(q1) + a2*cos(q0)*cos(q1 + q2) + d2*sin(q0);
    sin(q0)*cos(q1 + q2), -sin(q0)*sin(q1 + q2), -cos(q0), a1*sin(q0)*cos(q1) + a2*sin(q0)*cos(q1 + q2) - d2*cos(q0);
            sin(q1 + q2),          cos(q1 + q2),        0,                         a1*sin(q1) + a2*sin(q1 + q2) + d0;
                       0,                     0,        0,                                                         1;];
                    
rrr2_J = [ 
   d2*cos(q0) - a1*cos(q1)*sin(q0) - a2*sin(q0)*cos(q1 + q2), -cos(q0)*(a2*sin(q1 + q2) + a1*sin(q1)), -a2*sin(q1 + q2)*cos(q0);
   d2*sin(q0) + a1*cos(q0)*cos(q1) + a2*cos(q0)*cos(q1 + q2), -sin(q0)*(a2*sin(q1 + q2) + a1*sin(q1)), -a2*sin(q1 + q2)*sin(q0);
                                                           0,  sin(q0)*(a1*sin(q0)*cos(q1) + a2*sin(q0)*cos(q1 + q2) - cos(q0)*d2) + cos(q0)*(a1*cos(q0)*cos(q1) + a2*cos(q0)*cos(q1 + q2) + sin(q0)*d2),          sin(q0)*a2*sin(q0)*cos(q1 + q2) + cos(q0)*a2*cos(q0)*cos(q1 + q2);
                                                           0,                                 sin(q0),                  sin(q0);
                                                           0,                                -cos(q0),                 -cos(q0);
                                                           1,                                       0,                        0;];                 
         
% q = rrr_q;
% dh_parameters = rrr_dh_parameters;
% robot = rrr_robot;
% T_ideal = rrr_T;
% J_ideal = rrr_J;
q = rrr2_q;
dh_parameters = rrr2_dh_parameters;
robot = rrr2_robot;
T_ideal = rrr2_T;
J_ideal = rrr2_J;

test_raw_calculations(q, dh_parameters, T_ideal, J_ideal);
test_ETS3_calculations(q, robot, T_ideal, J_ideal);  


function test_raw_calculations(q, dh_parameters, T_ideal, J_ideal)   
    fprintf('test_raw_calculations\n'); 
    % Kinematics: End Frame Homogeneous Transformations
    T = calculate_transformation_matrix(dh_parameters);
    compare_end_frame_transform(T, T_ideal);
    
    % Kinematics Test 1: End Frame Position
    t = get_translation_matrix(T);
    t_ideal = get_translation_matrix(T_ideal);
    compare_end_frame_position(t, t_ideal);

    % Kinematics: Linear Velocity Jacobian
    J_v = simplify(calculate_jacobian_v(t, q));

    % Kinematics: Angular Velocity Jacobian
    J_w = simplify(calculate_jacobian_w(dh_parameters));

    % Kinematics: Combined Velocity Jacobian
    J = [ J_v;
          J_w ];
    compare_velocity_jacobian(J, J_ideal);
end

function test_ETS3_calculations(q, robot, T_ideal, J_ideal)  
    fprintf('test_ETS3_calculations\n');
    % Kinematics: End Frame Homogeneous Transformations
    T_obj = robot.fkine(q);
    t = T_obj.t;
    R = [T_obj.n, T_obj.o, T_obj.a];
    T = [           R, t;
          zeros(1, 3), 1;   ];
    compare_end_frame_transform(T, T_ideal);
   
    % Kinematics Test 1: End Frame Position
    t_ideal = get_translation_matrix(T_ideal);
    compare_end_frame_position(t, t_ideal);

    % Kinematics: Linear Velocity Jacobian
    %J_v_toolbox = simplify(robot.jacob0(theta, 'trans'))
    J_v = jacobian(t, q);
%     compare_linear_velocity_jacobian(J_v, J_v_toolbox);

    % Kinematics: Angular Velocity Jacobian
    J_w = robot.jacobe(q, 'rot');
%     compare_angular_velocity_jacobian(J_w, J_w_toolbox, 'jacobe');

    % J_w_toolbox = robot.jacob0(theta, 'rot');
%     J_w_toolbox = robot.jacob0(q, 'rot');
%     compare_angular_velocity_jacobian(J_w, J_w_toolbox, 'jacob0');

    % Kinematics: Combined Velocity Jacobian
    
    J = [ J_v;
          J_w ];
    % J_toolbox = robot.jacob0(q)
    compare_velocity_jacobian(J, J_ideal);
end

function compare_matrices(M1, M2)
%     simplify(expand(M1))
%     simplify(expand(M2))
    if(isequal(simplify(expand(M1)), simplify(expand(M2))))
        fprintf('SUCCESS!\n');
    else
        fprintf('FAILED!\n');
    end
end

function compare_end_frame_position(t1, t2)
    fprintf('Position comparision ');
    compare_matrices(t1, t2);
end

function compare_end_frame_transform(T1, T2)
    fprintf('Transform comparision ');
    compare_matrices(T1, T2);
end

function compare_linear_velocity_jacobian(J1_v, J2_v)
    fprintf('Jacobian v comparision ');
    compare_matrices(J1_v, J2_v);
end

function compare_angular_velocity_jacobian(J1_w, J2_w, note)
    fprintf('Jacobian w (%s) comparision ', note);
    compare_matrices(J1_w, J2_w);
end

function compare_velocity_jacobian(J1, J2)
    fprintf('Jacobian comparision ');
    compare_matrices(J1, J2);
end



% Robotics Toolbox test
% 
% robot_robotics = create_robot(tp_dh_parameters);
% showdetails(robot_robotics);        
% 
% function robot = create_robot(dh_parameters)
%     robot = robotics.RigidBodyTree;
%     
%     prev_body_name = 'base';
%     for i = 1:size(dh_parameters, 1)
%         body_name = strcat('body',i);
%         joint_name = strcat('jnt',i);
%         body = robotics.RigidBody(body_name);
%         joint = robotics.Joint(joint_name,'revolute');
% 
%         setFixedTransform(joint, dh_parameters(i,:),'dh');
%         body.Joint = joint;
% 
%         addBody(robot, body, prev_body_name)
%         prev_body_name = body_name;
%     end
% end
