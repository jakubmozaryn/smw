function A = calculate_transform_from_dh_parameters(alpha_a_d_theta_vector)
    alpha = alpha_a_d_theta_vector(1, 1);
    a = alpha_a_d_theta_vector(1, 2);
    d = alpha_a_d_theta_vector(1, 3);
    theta = alpha_a_d_theta_vector(1, 4);
    A = [   cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
            sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
            0           sin(alpha)             cos(alpha)            d;
            0           0                      0                     1              ];
end