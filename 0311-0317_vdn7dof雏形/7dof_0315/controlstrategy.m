function u = controlstrategy(t,Xin)
%-----------------------------------------------------------------------------------%
% Author: HUILONG YU, hlyubit@gmail.com.
% Date  : 09/11/2016
% Copyright (C) 2016 HUILONG YU. All Rights Reserved.
%-----------------------------------------------------------------------------------%

global pusr 
%-----------------------------------------------------------------------------------%
%                                     State vriables                                %
%-----------------------------------------------------------------------------------%
dXb       = Xin(:,1); %
dYb       = Xin(:,2); %
dYaw      = Xin(:,3); %
domegaFr  = Xin(:,4); %
domegaFl  = Xin(:,5); %
domegaRr  = Xin(:,6); %
domegaRl  = Xin(:,7); % 
s         = Xin(:,8);
n         = Xin(:,9);
chi       = Xin(:,10);
Xb        = Xin(:,11); %
Yb        = Xin(:,12); %
Yaw       = Xin(:,13); %
omega     = [domegaFr domegaFl domegaRr domegaRl];
curvi     = interp1(pusr.slength,pusr.Curv,s);
ds        = (dXb.*cos(chi)-dYb.*sin(chi))./(1-n.*curvi);
dn        = dXb.*sin(chi)+dYb.*cos(chi);
dchi      = dYaw-curvi.*ds;
%-----------------------------------------------------------------------------------%
%                                 Control variables                                 %
%-----------------------------------------------------------------------------------%

Nm           = 30*7.97*0.5*(Xin(:,6)+Xin(:,7))/pi;%N_motor 电机扭矩
Nm(Nm>18842) = 18842;
Tmax         = 7.97*interp1(pusr.Nm, pusr.Tm, Nm,'linear','extrap');
kpd          = 1e4;
Tt           = kpd*(20-dXb);  %前向推力，用了一个比例控制器kpd
Tt(Tt>Tmax)  = Tmax;  %前向推力的饱和限制
Tt(Tt<-2500) = -2500;
Tw           = [0*Tt-2 0*Tt-2 0.5*Tt 0.5*Tt]*(Tt>=0)...
             + [0.25*Tt 0.25*Tt 0.25*Tt 0.25*Tt]*(Tt<0); %转向力矩的，没看懂怎么算的
YawRef       = interp1(pusr.slength,pusr.theta,s+ds*1);  %期望横摆角
ref          = YawRef-Yaw; %与期望横摆角的偏差
stemp1       = -3e-2*n;     %看不懂
stemp2       = -5e-3*dn;        
stemp3       =  5e-1*ref;
usteer       = stemp1+stemp2+stemp3;  
usteer       = min(usteer,0.9)*(usteer>0)+max(usteer,-0.9)*(usteer<0);
u            = [Tw usteer usteer 0 0];
pusr.Td      = [Tt 0 0 0];
pusr.Nd      = [Nm 0 0 0];
nlog         = [stemp1 stemp2 stemp3]


% u = [];

end