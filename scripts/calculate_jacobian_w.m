function J_w = calculate_jacobian_w(dh_parameters)
%    k = [0 0 1]';
%    R_0_tp1 = get_rotation_matrix(T_0_tp1);
%    R_0_tp2 = get_rotation_matrix(T_0_tp2);
%    R_0_tp3 = get_rotation_matrix(T_0_tp3);
%    z_tp0 = k;
%    z_tp1 = R_0_tp1 * k;
%    z_tp2 = R_0_tp2 * k;
%    z_tp3 = R_0_tp3 * k;
%    J_wtp = [z_tp0 z_tp1 z_tp2 z_tp3];
    k = [0 0 1]';
    J_w = k;
    T = eye(4);
    for i = 1:size(dh_parameters, 1)-1
        T = T * calculate_transform_from_dh_parameters(dh_parameters(i, :));
        R = get_rotation_matrix(T);
        J_w = [J_w R*k];
    end
    %transform to base?
    %J_w = R * J_w;
end