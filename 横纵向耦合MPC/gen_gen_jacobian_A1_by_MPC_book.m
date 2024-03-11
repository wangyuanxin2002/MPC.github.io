syms X x_dot Y y_dot phi phi_dot Ccf Ccr I lf lr delta_f m Clf Clr Sf Sr;


eq1 = (-m*x_dot*phi_dot +2*(Ccf*(delta_f - (y_dot+lf*phi_dot)/x_dot) + Ccr*(lr*phi_dot-y_dot)/x_dot))/m;%((Ccr+Ccf*cos(delta_f))*y_dot/m/x_dot+(Ccf*lf*cos(delta_f)-Ccr*lr)*phi_dot/m/x_dot-Ccf*cos(delta_f)*delta_f/m+sin(delta_f)*Fx/m-phi_dot*x_dot);
eq2 = (m*y_dot*phi_dot + 2*(Clf*Sf +Ccf*delta_f*(delta_f - (y_dot+lf*phi_dot)/x_dot) + Clf*Sr))/m;%(-(Ccf*sin(delta_f))*y_dot/m/x_dot-(Ccf*lf*sin(delta_f))*phi_dot/m/x_dot+Ccf*sin(delta_f)*delta_f/m+Fx*cos(delta_f)/m+phi_dot*y_dot); % u1 =vx
eq3 = phi_dot;
eq4 = (2*lf*Ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)-2*lr*Ccr*(lr*phi_dot-y_dot)/x_dot)/I;%( 2*(Ccf*lf+Ccr*lr)*y_dot/I/x_dot + 2*(Ccf*lf^2-Ccr*lr*lf)*phi_dot/I/x_dot - 2*Ccf*lf*delta_f/I ); %
eq5 = (x_dot*sin(phi)+y_dot*cos(phi)); % Y
eq6 = (x_dot*cos(phi)-y_dot*sin(phi)); % X


vars = [y_dot;  x_dot; phi; phi_dot; Y; X];
vars_need = [X; x_dot; Y; y_dot; phi; phi_dot; Ccf; Ccr; I; lf; lr; delta_f; m; ];

A0 = jacobian([eq1; eq2; eq3; eq4; eq5; eq6], vars)

matlabFunction(A0, 'file', 'genA_1_by_MPC_bool', 'vars', vars_need);
