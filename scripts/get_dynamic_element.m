function D_i = get_dynamic_element(m_i, I_i, J_vi, J_wi, R_0_i)
    D_i = (m_i * (J_vi' * J_vi)) + (J_wi' * (R_0_i * I_i * R_0_i') * J_wi);
end