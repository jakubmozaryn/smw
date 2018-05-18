function D_i = get_dynamic_matrix(m, I, J_vi, J_wi, R_0_i)
    D_i = get_dynamic_element(m_i, I_i, J_vi, J_wi, R_0_i);
    for index = 2:i
        m_i = m(index);
        %J_vi = get_jacobian_v(dh_parameters, theta, i)
        %D_i = D_i + (m_i * (J_vi' * J_vi)) + (J_wi' * (R_0_i * I_i * R_0_i') * J_wi);
    end
end