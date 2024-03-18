function [Xout] = vdn7(t,Xin,u)
%-----------------------------------------------------------------------------------%
% Author: HUILONG YU, hlyubit@gmail.com.
% Date  : 09/11/2016
% Copyright (C) 2016 HUILONG YU. All Rights Reserved.
%-----------------------------------------------------------------------------------%
global pusr auxdata

%-----------------------------------------------------------------------------------%
%                                     State vriables                                %
%-----------------------------------------------------------------------------------%
dXb       = Xin(:,1); %Xin的第一列是Xb（X_body）的速度（为什么是第一列而不是第一个？速度有很多个？）
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
%                             Control                          %
%-----------------------------------------------------------------------------------%
Tw        = u(1:4);
steer     = u(5:8);
%-----------------------------------------------------------------------------------%
%                                    Aerodynamics%                                  %空气动力学
%-----------------------------------------------------------------------------------%
Fw                 = pusr.HalfCdArouDrag*(dXb.^2)*(3.6.^2);
FrontDownForceDrag = 0.5*pusr.HalfCdArouFrontDown*(dXb.^2)*(3.6.^2);
RearDownForceDrag  = 0.5*pusr.HalfCdArouRearDown*(dXb.^2)*(3.6.^2);

%-----------------------------------------------------------------------------------%
%                                    Normal load                                    %
%-----------------------------------------------------------------------------------%
% pusr.ddYb = pusr.ddYb*0;
% pusr.ddXb = pusr.ddXb*0;

Ftemp1  = pusr.lr*pusr.mt*9.8*0.5/pusr.l;
Ftemp2  = pusr.hg*pusr.mt*pusr.ddXb*0.5/pusr.l;
Ftemp3  = pusr.lr*pusr.hg*pusr.mt*pusr.ddYb/(pusr.l*pusr.wf);
Ftemp4  = pusr.hg*pusr.hg*pusr.mt*pusr.ddYb.*pusr.ddXb/(pusr.l*pusr.wf*9.8);
Ftemp5  = 0.5*FrontDownForceDrag;
Ftemp6  = 0.5*pusr.hd*Fw/pusr.l;
Ftemp7  = (FrontDownForceDrag*pusr.hg*pusr.l-pusr.hg*pusr.hd*Fw).*pusr.ddYb... 
          /(pusr.l*pusr.wf*9.8);
      
Ftemp8  = pusr.lf*pusr.mt*9.8*0.5/pusr.l;
Ftemp9  = pusr.lf*pusr.hg*pusr.mt*pusr.ddYb/(pusr.l*pusr.wr);
Ftemp10 = pusr.hg*pusr.hg*pusr.mt*pusr.ddYb.*pusr.ddXb/(pusr.l*pusr.wr*9.8);
Ftemp11 = 0.5*RearDownForceDrag;
Ftemp12 = (RearDownForceDrag*pusr.hg*pusr.l+pusr.hg*pusr.hd*Fw).*pusr.ddYb... 
          /(pusr.l*pusr.wr*9.8);
Fzfl    = Ftemp1-Ftemp2-Ftemp3+Ftemp4+Ftemp5-Ftemp6-Ftemp7;
Fzfr    = Ftemp1-Ftemp2+Ftemp3-Ftemp4+Ftemp5-Ftemp6+Ftemp7;
Fzrl    = Ftemp8+Ftemp2-Ftemp9-Ftemp10+Ftemp11+Ftemp6-Ftemp12;
Fzrr    = Ftemp8+Ftemp2+Ftemp9+Ftemp10+Ftemp11+Ftemp6+Ftemp12;
Fz      = [Fzfr Fzfl Fzrr Fzrl];

%-----------------------------------------------------------------------------------%
%                                     Tire slip                                     %
%-----------------------------------------------------------------------------------%
Vbx   = repmat(dXb,size(pusr.y))-repmat(pusr.y,size(dXb)).*dYaw; %根据车辆轮距轴距、车速角速度信息求得车轮速度
Vby   = repmat(dYb,size(pusr.x))+repmat(pusr.x,size(dYb)).*dYaw; %根据车辆轮距轴距、车速角速度信息求得车轮速度

Vwx   = Vbx.*cos(steer)+Vby.*sin(steer); %刚刚求得的中间量加入方向盘转角
Vwy   = -Vbx.*sin(steer)+Vby.*cos(steer); %求得真正的车轮速度

kappa = (omega.*repmat(pusr.Rt,size(dXb))-Vwx)./Vwx;
alpha = -steer+atan(Vby./Vbx);

kappa(kappa>1)=1;
kappa(kappa<-1)=-1;

alpha(alpha>1.57)=1.57;
alpha(alpha<-1.57)=-1.57;
%-----------------------------------------------------------------------------------%
%                                     Tire model                                    %
%-----------------------------------------------------------------------------------%
% [Ftx, Fty] = MfTire(Fz,kappa,alpha,pusr.gamma,Vwx);

B = 10;
C = 1.9;
D = 1;
E = 0.97;
Ftx = (Fz.*D.*sin(C.*atan(B.*kappa - E.*(B.*kappa - atan(B.*kappa)))));%
Fty = (-Fz.*D.*sin(C.*atan(B.*alpha - E.*(B.*alpha - atan(B.*alpha)))));%

Fx         = sum(Ftx.*cos(steer)-Fty.*sin(steer),2)-Fw;
Fy         = sum(Ftx.*sin(steer)+Fty.*cos(steer),2);
Tyaw = sum((Ftx.*sin(steer)+Fty.*cos(steer)).*repmat(pusr.x,size(dYb)))...
     - sum((Ftx.*cos(steer)-Fty.*sin(steer)).*repmat(pusr.y,size(dYb))); 

%-----------------------------------------------------------------------------------%
%                        Derivetives of the 7 dof vehicle model                     %
%-----------------------------------------------------------------------------------%
pusr.ddXb  = Fx/pusr.mt;
pusr.ddYb  = Fy/pusr.mt;
ddXb       = Fx/pusr.mt+dYaw.*dYb;
ddYb       = Fy/pusr.mt-dYaw.*dXb;
ddYaw      = Tyaw/pusr.Jzz;
ddomega    = (Tw-Ftx.*repmat(pusr.Rt,size(dXb)))./repmat(pusr.Jwy,size(dYb));
dgXb       = dXb.*cos(Yaw)-dYb.*sin(Yaw); %
dgYb       = dXb.*sin(Yaw)+dYb.*cos(Yaw); %
Xout       = [ddXb ddYb ddYaw ddomega  ds dn dchi dgXb dgYb dYaw];
auxdata    = [kappa, alpha, Fz, Ftx, Fty, pusr.Td, pusr.Nd, steer, Fx, Fy, Tyaw];
end