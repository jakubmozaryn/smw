function J_v = calculate_jacobian_v(translation_matrix, theta)
%    T_0_tp1 = get_transformation_matrix(tp_dh_parameters, 1);
%    T_0_tp2 = get_transformation_matrix(tp_dh_parameters, 2);
%    T_0_tp3 = get_transformation_matrix(tp_dh_parameters, 3);
%    T_0_tp4 = get_transformation_matrix(tp_dh_parameters, 4);

%    t_0_tp4 = get_translation_matrix(T_0_tp4);
%    J_vtp1 = get_jacobian_v_element(t_0_tp4, theta, 1);%diff(t_0_tp4, theta_tp1);
%    J_vtp2 = get_jacobian_v_element(t_0_tp4, theta, 2);%diff(t_0_tp4, theta_tp2);
%    J_vtp3 = get_jacobian_v_element(t_0_tp4, theta, 3);%diff(t_0_tp4, theta_tp3);
%    J_vtp4 = get_jacobian_v_element(t_0_tp4, theta, 4);%diff(t_0_tp4, theta_tp4);
%    J_vtp = [J_vtp1 J_vtp2 J_vtp3 J_vtp4];

    J_v = [];
    for i = 1:size(theta, 2)
        J_v = [J_v diff(translation_matrix, theta(1, i))];
    end
end