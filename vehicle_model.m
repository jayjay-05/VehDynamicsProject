function dx= vehicle_model( x, C_a_f, C_a_r, a, b, m ,u , I_z, angle_delta)

A11 = - (C_a_f + C_a_r)/(m*u);
A12 = (- (a*C_a_f - b*C_a_r)/(m*u)) -u;
A21 = - (a*C_a_f-b*C_a_r)/(I_z*u);
A22 = - (power(a,2)*C_a_f + power(b,2)*C_a_r)/(I_z*u);

B1 = C_a_f/m;
B2 = (a*C_a_f)/I_z; 

A = [A11, A12 ; A21, A22];
B = [B1 ; B2];

dx = A*x + B*angle_delta;
