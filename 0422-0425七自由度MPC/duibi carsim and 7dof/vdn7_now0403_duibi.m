function Xout = vdn7_now0403 (Xin,u)
global pusr auxdata
Xin = Xin;
%         1 %X 
%         1 % xdot
%         0 % Y
%         0 % Ydot
%         0  %phi
%         0 %dphi

% 50/3.6 0 0 0 0 10*160.2230*pi/30/3.6 10*160.2230*pi/30/3.6 10*160.2230*pi/30/3.6 10*160.2230*pi/30/3.6

Xb      = Xin(:,1); %Xin的第一列是Xb（X_body）的速度（为什么是第一列而不是第一个？速度有很多个？）
dXb     = Xin(:,2); %
if dXb==0
    dXb = dXb+0.001;
end
Yb      = Xin(:,3); %
dYb     = Xin(:,4); %
Yaw     = Xin(:,5); %
dYaw    = Xin(:,6); %
domegaFr    = Xin(:,7); % 
domegaFl    = Xin(:,8); %
domegaRr    = Xin(:,9); %
domegaRl    = Xin(:,10); %
% if domegaFr==0
%     domegaFr = domegaFr;
% end
% if domegaFl==0
%     domegaFl = domegaFl;
% end
% if domegaRr==0
%     domegaRr = domegaRr;
% end
% if domegaRl==0
%     domegaRl = domegaRl;
% end

% Tw1        = u(1);
% Tw2        = u(2);
% Tw3        = u(3);
% Tw4        = u(4);
Tw        = u(1:4);
steer     = u(5);
global pusr
    %------------%
    %     常量    %
    %------------%
    pusr.HalfCdArouDrag=0.03;   %200.85/(80^2)=0.0313828
    pusr.HalfCdArouFrontDown=0.015; %123.6/(80^2)=0.0193125 From VI-grade simulation results
    pusr.HalfCdArouRearDown=0.03; %212.4/(80^2)=0.0331875
    pusr.lr  = 1.260;
    pusr.lf  = 1.260;
    pusr.l   = 2.600;
    pusr.wf  = 1.040;
    pusr.wr  = 1.040;
    pusr.hg  = 0.54;
    pusr.hd  = 0.17;
    pusr.x   = [1.260 1.260 -1.260 -1.260];
    pusr.y   = [-1.040/2 1.040/2 -1.040/2 1.040/2];
    pusr.mt  = 1333;
    
    pusr.Jzz = 1343; % 
    pusr.Jwy = [20 20 20 20];
    pusr.Rt  =[0.298 0.298 0.298 0.298];
    
    omega     = [domegaFr domegaFl domegaRr domegaRl];
    eeee = 0;
    if dXb > 1
        eeee = -1500;
    elseif dXb< -1
        eeee = 1500;
    else
        eeee = 1500*sqrt(dXb);
    end
    Fw                 = pusr.HalfCdArouDrag*(dXb.^2)*(3.6.^2) ;%- eeee;
    FrontDownForceDrag = 0.5*pusr.HalfCdArouFrontDown*(dXb.^2)*(3.6.^2);
    RearDownForceDrag  = 0.5*pusr.HalfCdArouRearDown*(dXb.^2)*(3.6.^2);
    
    %-----------------------------------------------------------------------------------%
    %                                    Normal load                                    %
    %-----------------------------------------------------------------------------------%
%     ddYbc = ddYbc*0;
%     ddXbc = ddXbc*0;
    
    Ftemp1  = pusr.lr*pusr.mt*9.8*0.5/pusr.l; % 0.5*前轴重 左右相同
    Ftemp2  = pusr.hg*pusr.mt*pusr.ddXb*0.5/pusr.l; % ax加速度导致的前后轴载荷转移
    Ftemp3  = pusr.lr*pusr.hg*pusr.mt*pusr.ddYb/(pusr.l*pusr.wf); % ay加速度导致的左右载荷转移
    Ftemp4  = pusr.hg*pusr.hg*pusr.mt*pusr.ddYb.*pusr.ddXb/(pusr.l*pusr.wf*9.8); % 可以忽略
    Ftemp5  = 0.5*FrontDownForceDrag; %
    Ftemp6  = 0.5*pusr.hd*Fw/pusr.l; % 
    Ftemp7  = (FrontDownForceDrag*pusr.hg*pusr.l-pusr.hg*pusr.hd*Fw).*pusr.ddYb... 
              /(pusr.l*pusr.wf*9.8); %
          
    Ftemp8  = pusr.lf*pusr.mt*9.8*0.5/pusr.l; % 0.5*后轴重 左右相同
    Ftemp9  = pusr.lf*pusr.hg*pusr.mt*pusr.ddYb/(pusr.l*pusr.wr); % ay*重/轴距等 左右不同
    Ftemp10 = pusr.hg*pusr.hg*pusr.mt*pusr.ddYb.*pusr.ddXb/(pusr.l*pusr.wr*9.8); %
    Ftemp11 = 0.5*RearDownForceDrag; % 阻力
    Ftemp12 = (RearDownForceDrag*pusr.hg*pusr.l+pusr.hg*pusr.hd*Fw).*pusr.ddYb... 
              /(pusr.l*pusr.wr*9.8); % 阻力导致的左右偏差
    Fzfl    = Ftemp1-Ftemp2-Ftemp3+Ftemp4+Ftemp5-Ftemp6-Ftemp7; %
    Fzfr    = Ftemp1-Ftemp2+Ftemp3-Ftemp4+Ftemp5-Ftemp6+Ftemp7; %
    Fzrl    = Ftemp8+Ftemp2-Ftemp9-Ftemp10+Ftemp11+Ftemp6-Ftemp12; %
    Fzrr    = Ftemp8+Ftemp2+Ftemp9+Ftemp10+Ftemp11+Ftemp6+Ftemp12; %
    Fz      = [Fzfr Fzfl Fzrr Fzrl]; % 到这应该都对
%     Fz      = [Ftemp1 Ftemp1 Ftemp8 Ftemp8]; % 到这应该都对
   
    %-----------------------------------------------------------------------------------%
    %                                      Tire slip                                     %
    %-----------------------------------------------------------------------------------%
    Vbx   = repmat(dXb,size(pusr.y))-repmat(pusr.y,size(dXb)).*dYaw;
    Vby   = repmat(dYb,size(pusr.x))+repmat(pusr.x,size(dYb)).*dYaw;
    
    Vwx   = Vbx.*cos([steer,steer,0,0])+Vby.*sin([steer,steer,0,0]); 
    Vwy   =-Vbx.*sin([steer,steer,0,0])+Vby.*cos([steer,steer,0,0]); 
    
    kappa = (omega.*repmat(pusr.Rt,size(dXb))-Vwx)./Vwx;
    alpha = -[steer,steer,0,0]+atan(Vby./Vbx);
    alpha = alpha.*180./pi;
    % 
%     kappa(kappa>1)=1;
%     kappa(kappa<-1)=-1;
%     
%     alpha(alpha>1.57)=1.57;%alpha
%     alpha(alpha<-1.57)=-1.57;
    %-----------------------------------------------------------------------------------%
    %                                     Tire model                                    %
    %-----------------------------------------------------------------------------------%
    % [Ftx, Fty] = MfTire(Fz,kappa,alpha(4),pusr.gamma,Vwx);
    
%     B = 22.62;%10;
%     C = 1.4;%1.9;
%     D = 0.5;%0.5;
%     E = 0.549;%0.97;
% 
%     B1 = 0.1813;%10;
%     C1 = 1.415;%1.9;
%     D1 = 0.5;%0.5;
%     E1 = -0.1848;%0.97;
% 
    B = 0.1813;
    C = 1.4;
    D = 1;
    E = -0.18;
% % 
%     B = 5;
%     C = 1.4;
%     D = 1;
%     E = 0.97;

    Bx = 10.62;%10;
    Cx = 1.4;%1.9;
    Dx = 1;%0.5;
    Ex = 0.549;%0.97;

%     Ftxfr = (Fzfr.*D.*sin(C.*atan(B.*kappa(1) - E.*(B.*kappa(1) - atan(B.*kappa(1))))));%
%     Ftxfl = (Fzfl.*D.*sin(C.*atan(B.*kappa(2) - E.*(B.*kappa(2) - atan(B.*kappa(2))))));%
%     Ftxrr = (Fzrr.*D.*sin(C.*atan(B.*kappa(3) - E.*(B.*kappa(3) - atan(B.*kappa(3))))));%
%     Ftxrl = (Fzrl.*D.*sin(C.*atan(B.*kappa(4) - E.*(B.*kappa(4) - atan(B.*kappa(4))))));%
%     
%     Ftyfr = (-Fzfr.*D1.*sin(C1.*atan(B1.*alpha(1) - E1.*(B1.*alpha(1) - atan(B1.*alpha(1))))));%
%     Ftyfl = (-Fzfl.*D1.*sin(C1.*atan(B1.*alpha(2) - E1.*(B1.*alpha(2) - atan(B1.*alpha(2))))));%
%     Ftyrr = (-Fzrr.*D1.*sin(C1.*atan(B1.*alpha(3) - E1.*(B1.*alpha(3) - atan(B1.*alpha(3))))));%
%     Ftyrl = (-Fzrl.*D1.*sin(C1.*atan(B1.*alpha(4) - E1.*(B1.*alpha(4) - atan(B1.*alpha(4))))));%
    
      
%     Ftx = [Ftxfr, Ftxfl, Ftxrr, Ftxrl];
    Ftx = (Fz.*Dx.*sin(Cx.*atan(Bx.*kappa - Ex.*(Bx.*kappa - atan(Bx.*kappa)))));%
%     Fty = [Ftyfr, Ftyfl, Ftyrr, Ftyrl];
    Fty = (-Fz.*D.*sin(C.*atan(B.*alpha - E.*(B.*alpha - atan(B.*alpha)))));%
%     Fx  = (Ftxfr+Ftxfl).*cos(steer)-(Ftyfr+Ftyfl).*sin(steer) + Ftxrr+Ftxrl -Fw;
    
%     Fy  = (Ftxfr+Ftxfl).*sin(steer)+(Ftyfr+Ftyfl).*cos(steer) + Ftyrr+Ftyrl;
%     Tyaw = Ftxfr*sin(steer)* pusr.x(1) + Ftxfl*sin(steer)* pusr.x(2) +Ftyfr*cos(steer)* pusr.x(1) + ...
%         Ftyfl*cos(steer)* pusr.x(2) + Ftyrr* pusr.x(3)+Ftyrl* pusr.x(4) ...
%          - ( (Ftxfr*pusr.y(1)+Ftxfl*pusr.y(2)).*cos(steer)-(Ftyfr*pusr.y(1)+Ftyfl*pusr.y(2)).*sin(steer) + Ftxrr*pusr.y(3)+Ftxrl*pusr.y(4) );

    Fx         = sum(Ftx.*cos([steer,steer,0,0])-Fty.*sin([steer,steer,0,0]),2)-Fw;
    Fy         = sum(Ftx.*sin([steer,steer,0,0])+Fty.*cos([steer,steer,0,0]),2);
    Tyaw = sum((Ftx.*sin([steer,steer,0,0])+Fty.*cos([steer,steer,0,0])).*repmat(pusr.x,size(dYb)))...
         - sum((Ftx.*cos([steer,steer,0,0])-Fty.*sin([steer,steer,0,0])).*repmat(pusr.y,size(dYb))); 

    %-----------------------------------------------------------------------------------%
    %                        Derivetives of the 7 dof vehicle model                     %
    %-----------------------------------------------------------------------------------%
    pusr.ddXb  = Fx/pusr.mt; % ddXbc作用是保存上一轮车辆的加速度，这个只能从外面传进来。。然后ddXbc在外面要更新一次。
    pusr.ddYb  = Fy/pusr.mt;
    state_k1=zeros(10,1);
    state_k1(2,1)       = Fx/pusr.mt ; % +dYaw.*dYb; % eq2 ddXb
    state_k1(4,1)       = Fy/pusr.mt ; % -dYaw.*dXb; % eq4 ddYb
    state_k1(6,1)     = Tyaw/pusr.Jzz; % eq6 ddYaw 

    ddomega    = (Tw-Ftx.*repmat(pusr.Rt,size(dXb)))./repmat(pusr.Jwy,size(dYb));

    state_k1(7,1)   = ddomega(1);%(Tw1-Ftxfr.*pusr.Rt(1))./pusr.Jwy(1); % eq7 ddomegaFr 右前
    state_k1(8,1)   = ddomega(2);% (Tw2-Ftxfl.*pusr.Rt(2))./pusr.Jwy(2); % eq8 ddomegaFl 左前
    state_k1(9,1)   = ddomega(3);% (Tw3-Ftxrr.*pusr.Rt(3))./pusr.Jwy(3); % eq9 ddomegaRr 右后
    state_k1(10,1)   = ddomega(4);% (Tw4-Ftxrl.*pusr.Rt(4))./pusr.Jwy(4); % eq10 ddomegaRl左后
    
    state_k1(1,1)     = dXb.*cos(Yaw)-dYb.*sin(Yaw); %  % eq1 dgXb用于输出。dXb用于计算 dgXb
    state_k1(3,1)     = dXb.*sin(Yaw)+dYb.*cos(Yaw); % % eq3  dgYb
    state_k1(5,1)     = dYaw; % eq5  dgYaw
%     Xout  = state_k1';%[dgXb; dd yXb; dgYb; ddYb; dgYaw; ddYaw; ddomegaFr; ddomegaFl; ddomegaRr; ddomegaRl]
pusr.ddXb  = Fx/pusr.mt;
pusr.ddYb  = Fy/pusr.mt;
ddXb       = Fx/pusr.mt+dYaw.*dYb;
ddYb       = Fy/pusr.mt-dYaw.*dXb;
ddYaw      = Tyaw/pusr.Jzz;
ddomega    = (Tw-Ftx.*repmat(pusr.Rt,size(dXb)))./repmat(pusr.Jwy,size(dYb));
dgXb       = dXb.*cos(Yaw)-dYb.*sin(Yaw); %
dgYb       = dXb.*sin(Yaw)+dYb.*cos(Yaw); %
Xout       = [dgXb ddXb dgYb ddYb dYaw ddYaw ddomega];

auxdata    = [kappa, alpha, Fz, Ftx, Fty];
