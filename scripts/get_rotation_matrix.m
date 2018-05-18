function R = get_rotation_matrix(transformation_matrix)
%    R_0_tp1 = T_0_tp1(1:3, 1:3);
%    R_0_tp2 = T_0_tp2(1:3, 1:3);
%    R_0_tp3 = T_0_tp3(1:3, 1:3);

    R = transformation_matrix(1:3, 1:3);
end