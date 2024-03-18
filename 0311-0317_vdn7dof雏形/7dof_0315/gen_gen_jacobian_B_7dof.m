syms X x_dot Y y_dot phi1 phi2 Ccf Ccr I lf lr delta_f m Clf Clr Sf Sr Fx;
n = 2;
phi = sym('phi', [n, 1]);
%原 X; u1; Y; v; phi; phi(2) 123456
%y.  x. fai fai. y x   4 2 5 6 3 1

% eq1 = (-m*x_dot*phi(2) +2*(Ccf*(delta_f - (y_dot+lf*phi(2))/x_dot) + Ccr*(lr*phi(2)-y_dot)/x_dot))/m;%((Ccr+Ccf*cos(delta_f))*y_dot/m/x_dot+(Ccf*lf*cos(delta_f)-Ccr*lr)*phi(2)/m/x_dot-Ccf*cos(delta_f)*delta_f/m+sin(delta_f)*Fx/m-phi(2)*x_dot);
% eq2 = (m*y_dot*phi(2) + 2*(Clf*Sf +Ccf*delta_f*(delta_f - (y_dot+lf*phi(2))/x_dot) + Clf*Sr))/m;%(-(Ccf*sin(delta_f))*y_dot/m/x_dot-(Ccf*lf*sin(delta_f))*phi(2)/m/x_dot+Ccf*sin(delta_f)*delta_f/m+Fx*cos(delta_f)/m+phi(2)*y_dot); % u1 =vx
% eq3 = phi(2);
% eq4 = ( 2*(Ccf*lf+Ccr*lr)*y_dot/I/x_dot + 2*(Ccf*lf^2-Ccr*lr*lf)*phi(2)/I/x_dot - 2*Ccf*lf*delta_f/I ); %
% eq5 = (x_dot*sin(phi)+y_dot*cos(phi)); % Y
% eq6 = (x_dot*cos(phi)-y_dot*sin(phi)); % X

% eq4 = ((Ccr+Ccf*cos(delta_f))*y_dot/m/x_dot+(Ccf*lf*cos(delta_f)-Ccr*lr)*phi(2)/m/x_dot-Ccf*cos(delta_f)*delta_f/m+sin(delta_f)*Fx/m-phi(2)*x_dot);%vy
% eq2 = (-(Ccf*sin(delta_f))*y_dot/m/x_dot-(Ccf*lf*sin(delta_f))*phi(2)/m/x_dot+Ccf*sin(delta_f)*delta_f/m+Fx*cos(delta_f)/m+phi(2)*y_dot); % u1 =vx
% eq5 = phi(2);
% eq6 = ((Ccf*lf*cos(delta_f)-Ccr*lr)*y_dot/I/x_dot+(Ccf*lf^2*cos(delta_f)+Ccr*lr^2)*phi(2)/I/x_dot-Ccf*lf*cos(delta_f)*delta_f/I+lf*sin(delta_f)*Fx/I);
% eq3 = (x_dot*sin(phi)+y_dot*cos(phi)); % Y 
% eq1 = (x_dot*cos(phi)-y_dot*sin(phi)); % X
eq4 = ((Ccr+Ccf*cos(delta_f))*y_dot/m/x_dot+(Ccf*lf*cos(delta_f)-Ccr*lr)*phi(2)/m/x_dot-Ccf*cos(delta_f)*delta_f/m+sin(delta_f)*Fx/m-phi(2)*x_dot); % vy
eq2 = (-(Ccf*sin(delta_f))*y_dot/m/x_dot-(Ccf*lf*sin(delta_f))*phi(2)/m/x_dot+Ccf*sin(delta_f)*delta_f/m+Fx*cos(delta_f)/m+phi(2)*y_dot); % u1 = vx
eq5 = phi(2);
eq6 = ((Ccf*lf*cos(delta_f)-Ccr*lr)*y_dot/I/x_dot+(Ccf*lf^2*cos(delta_f)+Ccr*lr^2)*phi(2)/I/x_dot-Ccf*lf*cos(delta_f)*delta_f/I+lf*sin(delta_f)*Fx/I);
eq3 = (x_dot*sin(phi(1))+y_dot*cos(phi(1))); % Y 
eq1 = (x_dot*cos(phi(1))-y_dot*sin(phi(1))); % X
eq2 = phi;

syms Tw1 Tw2 Tw3 Tw4

% 替换Tw变量
subs([eq1; eq2; eq3; eq4; eq5; eq6], [phi(1), phi(2)], [phi1, phi2]);


vars = [Fx; delta_f; phi];%delta_f; %[Fx;delta]  %[delta]
vars_need = [X; x_dot; Y; y_dot; phi1; phi2; Ccf; Ccr; I; lf; lr; delta_f; m; Clf; Clr; Sf; Sr; Fx];

B0 = jacobian([eq1; eq2; eq3; eq4; eq5; eq6], vars)

matlabFunction(B0, 'file', 'genB_7dof_0316', 'vars', vars_need);

% 
% eq1 = (u1*cos(phi)-v*sin(phi)); % X
% eq2 = (-(Cf*sin(delta))*v/m/u1-(Cf*lf*sin(delta))*phi(2)/m/u1+Cf*sin(delta)*delta/m+Fx*cos(delta)/m+phi(2)*v); % u1 =vx
% eq3 = (u1*sin(phi)+v*cos(phi)); % Y 
% eq4 = ((Ccr+Cf*cos(delta))*v/m/u1+(Cf*lf*cos(delta)-Ccr*lr)*phi(2)/m/u1-Cf*cos(delta)*delta/m+sin(delta)*Fx/m-phi(2)*u1);vy
% eq5 = phi(2);
% eq6 = ((Cf*lf*cos(delta)-Ccr*lr)*v/I/u1+(Cf*lf^2*cos(delta)+Ccr*lr^2)*phi(2)/I/u1-Cf*lf*cos(delta)*delta/I+lf*sin(delta)*Fx/I);
