syms X x_dot Y y_dot phi phi_dot Ccf Ccr I lf lr delta_f m Clf Clr Sf Sr;
%原 X; u1; Y; v; phi; w 123456
%y.  x. fai fai. y x   4 2 5 6 3 1

eq1 = (-m*x_dot*phi_dot +2*(Ccf*(delta_f - (y_dot+lf*phi_dot)/x_dot) + Ccr*(lr*phi_dot-y_dot)/x_dot))/m;%((Ccr+Ccf*cos(delta_f))*y_dot/m/x_dot+(Ccf*lf*cos(delta_f)-Ccr*lr)*phi_dot/m/x_dot-Ccf*cos(delta_f)*delta_f/m+sin(delta_f)*Fx/m-phi_dot*x_dot);
eq2 = (m*y_dot*phi_dot + 2*(Clf*Sf +Ccf*delta_f*(delta_f - (y_dot+lf*phi_dot)/x_dot) + Clf*Sr))/m;%(-(Ccf*sin(delta_f))*y_dot/m/x_dot-(Ccf*lf*sin(delta_f))*phi_dot/m/x_dot+Ccf*sin(delta_f)*delta_f/m+Fx*cos(delta_f)/m+phi_dot*y_dot); % u1 =vx
eq3 = phi_dot;
eq4 = (2*lf*Ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)-2*lr*Ccr*(lr*phi_dot-y_dot)/x_dot)/I;%( 2*(Ccf*lf+Ccr*lr)*y_dot/I/x_dot + 2*(Ccf*lf^2-Ccr*lr*lf)*phi_dot/I/x_dot - 2*Ccf*lf*delta_f/I ); %
eq5 = (x_dot*sin(phi)+y_dot*cos(phi)); % Y
eq6 = (x_dot*cos(phi)-y_dot*sin(phi)); % X

vars = delta_f; %[Fx;delta]  %[delta]
vars_need = [X; x_dot; Y; y_dot; phi; phi_dot; Ccf; Ccr; I; lf; lr; delta_f; m; Clf; Clr; Sf; Sr];

B0 = jacobian([eq1; eq2; eq3; eq4; eq5; eq6], vars)

matlabFunction(B0, 'file', 'genB_1_by_MPC_bool', 'vars', vars_need);

% eq1 = (u1*cos(phi)-v*sin(phi)); % X
% eq2 = (-(Cf*sin(delta))*v/m/u1-(Cf*Lf*sin(delta))*w/m/u1+Cf*sin(delta)*delta/m+Fx*cos(delta)/m+w*v); % u1 =vx
% eq3 = (u1*sin(phi)+v*cos(phi)); % Y 
% eq4 = ((Cr+Cf*cos(delta))*v/m/u1+(Cf*Lf*cos(delta)-Cr*Lr)*w/m/u1-Cf*cos(delta)*delta/m+sin(delta)*Fx/m-w*u1);vy
% eq5 = w;
% eq6 = ((Cf*Lf*cos(delta)-Cr*Lr)*v/Iz/u1+(Cf*Lf^2*cos(delta)+Cr*Lr^2)*w/Iz/u1-Cf*Lf*cos(delta)*delta/Iz+Lf*sin(delta)*Fx/Iz);
