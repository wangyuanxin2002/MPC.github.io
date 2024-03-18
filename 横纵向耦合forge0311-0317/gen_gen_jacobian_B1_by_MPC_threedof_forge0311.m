syms X x_dot Y y_dot phi phi_dot Cclf Ccr I lf lr delta_f m Clf Clr Sf Sr Fx;
%原 X; u1; Y; v; phi; phi_dot 123456
%y.  x. fai fai. y x   4 2 5 6 3 1

% eq1 = (-m*x_dot*phi_dot +2*(Ccf*(delta_f - (y_dot+lf*phi_dot)/x_dot) + Ccr*(lr*phi_dot-y_dot)/x_dot))/m;%((Ccr+Ccf*cos(delta_f))*y_dot/m/x_dot+(Ccf*lf*cos(delta_f)-Ccr*lr)*phi_dot/m/x_dot-Ccf*cos(delta_f)*delta_f/m+sin(delta_f)*Fx/m-phi_dot*x_dot);
% eq2 = (m*y_dot*phi_dot + 2*(Clf*Sf +Ccf*delta_f*(delta_f - (y_dot+lf*phi_dot)/x_dot) + Clf*Sr))/m;%(-(Ccf*sin(delta_f))*y_dot/m/x_dot-(Ccf*lf*sin(delta_f))*phi_dot/m/x_dot+Ccf*sin(delta_f)*delta_f/m+Fx*cos(delta_f)/m+phi_dot*y_dot); % u1 =vx
% eq3 = phi_dot;
% eq4 = ( 2*(Ccf*lf+Ccr*lr)*y_dot/I/x_dot + 2*(Ccf*lf^2-Ccr*lr*lf)*phi_dot/I/x_dot - 2*Ccf*lf*delta_f/I ); %
% eq5 = (x_dot*sin(phi)+y_dot*cos(phi)); % Y
% eq6 = (x_dot*cos(phi)-y_dot*sin(phi)); % X

eq4 = ((Ccr+Ccf*cos(delta_f))*y_dot/m/x_dot+(Ccf*lf*cos(delta_f)-Ccr*lr)*phi_dot/m/x_dot-Ccf*cos(delta_f)*delta_f/m+sin(delta_f)*Fx/m-phi_dot*x_dot);%vy
eq2 = (-(Ccf*sin(delta_f))*y_dot/m/x_dot-(Ccf*lf*sin(delta_f))*phi_dot/m/x_dot+Ccf*sin(delta_f)*delta_f/m+Fx*cos(delta_f)/m+phi_dot*y_dot); % u1 =vx
eq5 = phi_dot;
eq6 = ((Ccf*lf*cos(delta_f)-Ccr*lr)*y_dot/I/x_dot+(Ccf*lf^2*cos(delta_f)+Ccr*lr^2)*phi_dot/I/x_dot-Ccf*lf*cos(delta_f)*delta_f/I+lf*sin(delta_f)*Fx/I);
eq3 = (x_dot*sin(phi)+y_dot*cos(phi)); % Y 
eq1 = (x_dot*cos(phi)-y_dot*sin(phi)); % X

vars = [Fx; delta_f];%delta_f; %[Fx;delta]  %[delta]
vars_need = [X; x_dot; Y; y_dot; phi; phi_dot; Ccf; Ccr; I; lf; lr; delta_f; m; Clf; Clr; Sf; Sr; Fx];

B0 = jacobian([eq1; eq2; eq3; eq4; eq5; eq6], vars)

matlabFunction(B0, 'file', 'genB_1_by_MPC_threedof_forge0312', 'vars', vars_need);

% 
% eq1 = (u1*cos(phi)-v*sin(phi)); % X
% eq2 = (-(Cf*sin(delta))*v/m/u1-(Cf*lf*sin(delta))*phi_dot/m/u1+Cf*sin(delta)*delta/m+Fx*cos(delta)/m+phi_dot*v); % u1 =vx
% eq3 = (u1*sin(phi)+v*cos(phi)); % Y 
% eq4 = ((Ccr+Cf*cos(delta))*v/m/u1+(Cf*lf*cos(delta)-Ccr*lr)*phi_dot/m/u1-Cf*cos(delta)*delta/m+sin(delta)*Fx/m-phi_dot*u1);vy
% eq5 = phi_dot;
% eq6 = ((Cf*lf*cos(delta)-Ccr*lr)*v/I/u1+(Cf*lf^2*cos(delta)+Ccr*lr^2)*phi_dot/I/u1-Cf*lf*cos(delta)*delta/I+lf*sin(delta)*Fx/I);
