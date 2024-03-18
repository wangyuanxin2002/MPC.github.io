syms X x_dot Y y_dot phi phi_dot Ccf Ccr I lf lr delta_f m Clf Clr Sf Sr Fx;


eq4 = ((Ccr+Ccf*cos(delta_f))*y_dot/m/x_dot+(Ccf*lf*cos(delta_f)-Ccr*lr)*phi_dot/m/x_dot-Ccf*cos(delta_f)*delta_f/m+sin(delta_f)*Fx/m-phi_dot*x_dot);%vy
eq2 = (-(Ccf*sin(delta_f))*y_dot/m/x_dot-(Ccf*lf*sin(delta_f))*phi_dot/m/x_dot+Ccf*sin(delta_f)*delta_f/m+Fx*cos(delta_f)/m+phi_dot*y_dot); % u1 =vx
eq5 = phi_dot;
eq6 = ((Ccf*lf*cos(delta_f)-Ccr*lr)*y_dot/I/x_dot+(Ccf*lf^2*cos(delta_f)+Ccr*lr^2)*phi_dot/I/x_dot-Ccf*lf*cos(delta_f)*delta_f/I+lf*sin(delta_f)*Fx/I);
eq3 = (x_dot*sin(phi)+y_dot*cos(phi)); % Y 
eq1 = (x_dot*cos(phi)-y_dot*sin(phi)); % X


vars = [X; x_dot; Y; y_dot; phi; phi_dot]; 
vars_need = [X; x_dot; Y; y_dot; phi; phi_dot; Ccf; Ccr; I; lf; lr; delta_f; m; Clf; Clr; Sf; Sr; Fx];

A0 = jacobian([eq1; eq2; eq3; eq4; eq5; eq6], vars)

matlabFunction(A0, 'file', 'genA_1_by_MPC_threedof_forge0312', 'vars', vars_need);
