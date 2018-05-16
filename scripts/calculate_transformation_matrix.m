function T = calculate_transformation_matrix(dh_parameters)
%    A_tp0 = get_transform_from_dh_parameters(tp_dh_parameters(1, :));
%    A_tp1 = get_transform_from_dh_parameters(tp_dh_parameters(2, :));
%    A_tp2 = get_transform_from_dh_parameters(tp_dh_parameters(3, :));
%    A_tp3 = get_transform_from_dh_parameters(tp_dh_parameters(4, :));
%    A_tp4 = get_transform_from_dh_parameters(tp_dh_parameters(5, :));
    
%    T_0_tp1 = A_tp0 * A_tp1
%    T_0_tp2 = A_tp0 * A_tp1 * A_tp2
%    T_0_tp3 = A_tp0 * A_tp1 * A_tp2 * A_tp3
%    T_0_tp4 = A_tp0 * A_tp1 * A_tp2 * A_tp3 * A_tp4

    T = eye(4);
    for i = 1:size(dh_parameters, 1)
        T = T * calculate_transform_from_dh_parameters(dh_parameters(i, :));
    end
end