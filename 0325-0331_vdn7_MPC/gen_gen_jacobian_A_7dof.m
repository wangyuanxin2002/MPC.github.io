clc
clear
syms dXb dYb dYaw domegaFr domegaFl domegaRr domegaRl Xb Yb Yaw ...
    Tw1 Tw2 Tw3 Tw4 steer ddXbc ddYbc
%X x_dot Y y_dot phi phi_dot Ccf Ccr I lf lr delta_f m Clf Clr Sf Sr Fx ;
global pusr
%------------%
%     常量    %
%------------%
pusr.HalfCdArouDrag=0.0313828;   %200.85/(80^2)=0.0313828
pusr.HalfCdArouFrontDown=0.0193125; %123.6/(80^2)=0.0193125 From VI-grade simulation results
pusr.HalfCdArouRearDown=0.0331875; %212.4/(80^2)=0.0331875
pusr.lr  = 1.133;
pusr.lf  = 1.597;
pusr.l   = 2.730;
pusr.wf  = 1.58859;
pusr.wr  = 1.53785;
pusr.hg  = 0.2891;
pusr.hd  = 0.17;
pusr.x   = [1.597 1.597 -1.133 -1.133];
pusr.y   = [-1.58859/2 1.58859/2 -1.53785/2 1.53785/2];
pusr.mt  = 529;

pusr.Jzz = 711.3; % 
pusr.Jwy = [0.5 0.5 0.5 0.5];
pusr.Rt  =[0.275 0.275 0.285 0.285];

%-----------------------------------------------------------------------------------%
%                                     State vriables                                %
%-----------------------------------------------------------------------------------%
% dXb       = Xin(:,1); %Xin的第一列是Xb（X_body）的速度（为什么是第一列而不是第一个？速度有很多个？）
% dYb       = Xin(:,2); %
% dYaw      = Xin(:,3); %
% domegaFr  = Xin(:,4); %
% domegaFl  = Xin(:,5); %
% domegaRr  = Xin(:,6); %
% domegaRl  = Xin(:,7); % 
% s         = Xin(:,8);
% n         = Xin(:,9);
% chi       = Xin(:,10);
% Xb        = Xin(:,11); %
% Yb        = Xin(:,12); %
% Yaw       = Xin(:,13); %


omega     = [domegaFr domegaFl domegaRr domegaRl];
% curvi     = interp1(pusr.slength,pusr.Curv,s);
% ds        = (dXb.*cos(chi)-dYb.*sin(chi))./(1-n.*curvi);
% dn        = dXb.*sin(chi)+dYb.*cos(chi);
% dchi      = dYaw-curvi.*ds;
%-----------------------------------------------------------------------------------%
%                             Control                          %
%-----------------------------------------------------------------------------------%
% Tw        = u(1:4);
% steer     = u(5:8);
%-----------------------------------------------------------------------------------%
%                                    Aerodynamics%                                  %空气动力学
%-----------------------------------------------------------------------------------%
Fw                 = pusr.HalfCdArouDrag*(dXb.^2)*(3.6.^2);
FrontDownForceDrag = 0.5*pusr.HalfCdArouFrontDown*(dXb.^2)*(3.6.^2);
RearDownForceDrag  = 0.5*pusr.HalfCdArouRearDown*(dXb.^2)*(3.6.^2);

%-----------------------------------------------------------------------------------%
%                                    Normal load                                    %
%-----------------------------------------------------------------------------------%
% ddYbc = ddYbc*0;
% ddXbc = ddXbc*0;

Ftemp1  = pusr.lr*pusr.mt*9.8*0.5/pusr.l; % 0.5*前轴重 左右相同
Ftemp2  = pusr.hg*pusr.mt*ddXbc*0.5/pusr.l; % ax加速度导致的前后轴载荷转移
Ftemp3  = pusr.lr*pusr.hg*pusr.mt*ddYbc/(pusr.l*pusr.wf); %
Ftemp4  = pusr.hg*pusr.hg*pusr.mt*ddYbc.*ddXbc/(pusr.l*pusr.wf*9.8); %
Ftemp5  = 0.5*FrontDownForceDrag; %
Ftemp6  = 0.5*pusr.hd*Fw/pusr.l; % 
Ftemp7  = (FrontDownForceDrag*pusr.hg*pusr.l-pusr.hg*pusr.hd*Fw).*ddYbc... 
          /(pusr.l*pusr.wf*9.8); %
      
Ftemp8  = pusr.lf*pusr.mt*9.8*0.5/pusr.l; % 0.5*后轴重 左右相同
Ftemp9  = pusr.lf*pusr.hg*pusr.mt*ddYbc/(pusr.l*pusr.wr); % ay*重/轴距等 左右不同
Ftemp10 = pusr.hg*pusr.hg*pusr.mt*ddYbc.*ddXbc/(pusr.l*pusr.wr*9.8); %
Ftemp11 = 0.5*RearDownForceDrag; % 阻力
Ftemp12 = (RearDownForceDrag*pusr.hg*pusr.l+pusr.hg*pusr.hd*Fw).*ddYbc... 
          /(pusr.l*pusr.wr*9.8); % 阻力导致的左右偏差
Fzfl    = Ftemp1-Ftemp2-Ftemp3+Ftemp4+Ftemp5-Ftemp6-Ftemp7; %
Fzfr    = Ftemp1-Ftemp2+Ftemp3-Ftemp4+Ftemp5-Ftemp6+Ftemp7; %
Fzrl    = Ftemp8+Ftemp2-Ftemp9-Ftemp10+Ftemp11+Ftemp6-Ftemp12; %
Fzrr    = Ftemp8+Ftemp2+Ftemp9+Ftemp10+Ftemp11+Ftemp6+Ftemp12; %
Fz      = [Fzfr Fzfl Fzrr Fzrl]; %

%-----------------------------------------------------------------------------------%
%                                     Tire slip                                     %
%-----------------------------------------------------------------------------------%
Vbx   = repmat(dXb,size(pusr.y))-repmat(pusr.y,size(dXb)).*dYaw;
Vby   = repmat(dYb,size(pusr.x))+repmat(pusr.x,size(dYb)).*dYaw;

Vwx   = Vbx.*cos(steer)+Vby.*sin(steer); 
Vwy   =-Vbx.*sin(steer)+Vby.*cos(steer); 

kappa = (omega.*repmat(pusr.Rt,size(dXb))-Vwx)./Vwx;
alpha = -steer+atan(Vby./Vbx);
% 
% kappa(kappa>1)=1;
% kappa(kappa<-1)=-1;
% 
% alpha(alpha>1.57)=1.57;alpha
% alpha(alpha<-1.57)=-1.57;
%-----------------------------------------------------------------------------------%
%                                     Tire model                                    %
%-----------------------------------------------------------------------------------%
% [Ftx, Fty] = MfTire(Fz,kappa,alpha(4),pusr.gamma,Vwx);

B = 10;
C = 1.9;
D = 1;
E = 0.97;
Ftxfr = (Fzfr.*D.*sin(C.*atan(B.*kappa(1) - E.*(B.*kappa(1) - atan(B.*kappa(1))))));%
Ftxfl = (Fzfl.*D.*sin(C.*atan(B.*kappa(2) - E.*(B.*kappa(2) - atan(B.*kappa(2))))));%
Ftxrr = (Fzrr.*D.*sin(C.*atan(B.*kappa(3) - E.*(B.*kappa(3) - atan(B.*kappa(3))))));%
Ftxrl = (Fzrl.*D.*sin(C.*atan(B.*kappa(4) - E.*(B.*kappa(4) - atan(B.*kappa(4))))));%

Ftyfr = (-Fzfr.*D.*sin(C.*atan(B.*alpha(1) - E.*(B.*alpha(1) - atan(B.*alpha(1))))));%
Ftyfl = (-Fzfl.*D.*sin(C.*atan(B.*alpha(2) - E.*(B.*alpha(2) - atan(B.*alpha(2))))));%
Ftyrr = (-Fzrr.*D.*sin(C.*atan(B.*alpha(3) - E.*(B.*alpha(3) - atan(B.*alpha(3))))));%
Ftyrl = (-Fzrl.*D.*sin(C.*atan(B.*alpha(4) - E.*(B.*alpha(4) - atan(B.*alpha(4))))));%

Fx  = (Ftxfr+Ftxfl).*cos(steer)-(Ftyfr+Ftyfl).*sin(steer) + Ftxrr+Ftxrl -Fw;

Fy  = (Ftxfr+Ftxfl).*sin(steer)+(Ftyfr+Ftyfl).*cos(steer) + Ftyrr+Ftyrl;
Tyaw = Ftxfr*sin(steer)* pusr.x(1) + Ftxfl*sin(steer)* pusr.x(2) +Ftyfr*cos(steer)* pusr.x(1) + ...
    Ftyfl*cos(steer)* pusr.x(2) + Ftyrr* pusr.x(3)+Ftyrl* pusr.x(4) ...
     - ( (Ftxfr*pusr.y(1)+Ftxfl*pusr.y(2)).*cos(steer)-(Ftyfr*pusr.y(1)+Ftyfl*pusr.y(2)).*sin(steer) + Ftxrr*pusr.y(3)+Ftxrl*pusr.y(4) );

%-----------------------------------------------------------------------------------%
%                        Derivetives of the 7 dof vehicle model                     %
%-----------------------------------------------------------------------------------%
% ddXbc  = Fx/pusr.mt; % ddXbc作用是保存上一轮车辆的加速度，这个只能从外面传进来。。然后ddXbc在外面要更新一次。
% ddYbc  = Fy/pusr.mt;
ddXb       = Fx/pusr.mt ; % +dYaw.*dYb; % eq2
ddYb       = Fy/pusr.mt ; % -dYaw.*dXb; % eq4
ddYaw      = Tyaw/pusr.Jzz; % eq6
ddomegaFr    = (Tw1-Ftxfr.*pusr.Rt(1))./pusr.Jwy(1); % eq7
ddomegaFl    = (Tw2-Ftxfl.*pusr.Rt(2))./pusr.Jwy(2); % eq8
ddomegaRr    = (Tw3-Ftxrr.*pusr.Rt(3))./pusr.Jwy(3); % eq9
ddomegaRl    = (Tw4-Ftxrl.*pusr.Rt(4))./pusr.Jwy(4); % eq10

dgXb       = dXb.*cos(Yaw)-dYb.*sin(Yaw); %  % eq1 dgXb用于输出。dXb用于计算
dgYb       = dXb.*sin(Yaw)+dYb.*cos(Yaw); % % eq3
dgYaw   = dYaw; % eq5
Xout       = [dgXb; ddXb; dgYb; ddYb; dgYaw; ddYaw; ddomegaFr; ddomegaFl; ddomegaRr; ddomegaRl];

vars = [Xb; dXb; Yb; dYb; Yaw; dYaw; domegaFr; domegaFl; domegaRr; domegaRl]; 
vars_need = [dXb; dYb; dYaw; domegaFr; domegaFl; domegaRr; domegaRl; Xb; Yb; Yaw; Tw1; Tw2; Tw3; Tw4; steer; ddXbc; ddYbc ];

A0 = jacobian(Xout, vars)

matlabFunction(A0, 'file', 'genA_7dof_0320', 'vars', vars_need);
