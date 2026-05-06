function q = CRS_IK(p_des, q_init)

q = fsolve(@(q) CRS_FK(q) - p_des, q_init);

end