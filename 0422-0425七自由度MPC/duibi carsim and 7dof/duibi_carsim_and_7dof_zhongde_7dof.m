% clc
% clear
% close all
global Mu Cf Cr m g Iz Lf Lr L Fxtmax Fyfmax Fyrmax Ts ...
    Nu Np Nc Statelog Contollog RefSt RefCtr umax umin pusr auxdata
auxdata = zeros(1,100)
Mu = 0.9;
Cf = -Mu*127000;
Cr = -Mu*130000;
m = 1110;
g = 9.8;
Iz = 4600;
Lf = 1.040;
Lr = 1.560;
L = Lr+Lf;
Fxtmax = Mu*m*g;
Fyfmax = Mu*m*g*Lr/L;
Fyrmax = Mu*m*g*Lf/L;
%-----------------------------------------------------------------------------------%
%               MPC controller parameters
%-----------------------------------------------------------------------------------%
Ts = 0.01;%仿真步长
tf = 20;%仿真时间
    pusr.ddXb  = 0;
    pusr.ddYb = 0 ;

Statelog = zeros(10,tf/Ts+1);
Contollog = zeros(5,tf/Ts+1);
auxdatalog = zeros(20,tf/Ts+1);
Fx = Contollog(1,:);
Steer_rad = Contollog(5,:);
Ucur = Statelog(2,:);
Ycur = Statelog(3,:);
t = 0:Ts:tf;
% figure('Renderer', 'painters', 'Position', [600 50 600 900])%[left bottom width height]
% subplot(5,2,1);h1 = plot(t,Fx,'r','LineWidth',2);grid on;
% legend('Longitudinal Force Control','Location','northeast');
% xlim([0,tf-1]);ylim([15,30]);
% 
subplot(4,2,1);h2 = plot(t,Ucur,'k','LineWidth',2);grid on;
legend('longitudinal velocity','Location','southeast');
xlim([0,tf-1]);ylim([-10,20]);

% subplot(5,2,3);h3 = plot(t,Steer_rad,'k','LineWidth',2);grid on;
% legend('Steer Command (rad)','Location','southeast')
% xlim([0,tf-1]);ylim([0,0.4]);

subplot(4,2,2);h4 = plot(t,Ycur,'r','LineWidth',2);grid on;
legend('Current Y Coord','Location','southeast');
% xlim([0,tf-1]);ylim([0,40]);

subplot(4,2,3);h5 = plot(Statelog(1,:),Ycur,'r','LineWidth',2);grid on;
legend('Current X-Y','Location','southeast');
% xlim([-20,20]);ylim([-10,10]);

subplot(4,2,4);
h6 = plot(t,auxdata(:,1),'r','LineWidth',2);
grid on;legend('Kappa front right','Location','southeast');
% xlim([-20,20]);ylim([-10,10]);

subplot(4,2,5);h7 = plot(t,auxdata(:,2),'r','LineWidth',2);grid on;
legend('Kappa front left','Location','southeast');
% xlim([-20,20]);ylim([-10,10]);

subplot(4,2,6);h8 = plot(t,auxdata(:,3),'r','LineWidth',2);grid on;
legend('alpha rear right','Location','southeast');
% xlim([-20,20]);ylim([-10,10]);

subplot(4,2,7);h9 = plot(t,auxdata(:,4),'r','LineWidth',2);grid on;
legend('alpha rear left','Location','southeast');
% xlim([-20,20]);ylim([-10,10]);
% 
% subplot(5,2,8);h8 = plot(t,auxdata(:,5),'r','LineWidth',2);grid on;
% legend('alpha front right','Location','southeast');
% % xlim([-20,20]);ylim([-10,10]);
% 
% subplot(5,2,9);h9 = plot(t,auxdata(:,7),'r','LineWidth',2);grid on;
% legend('alpha rear right','Location','southeast');
% % xlim([-20,20]);ylim([-10,10]);
plotCtr = 0;
PlotFreq = 100;

% State = [1 5/3.6 0 0 0 0 160.2230*pi/30/3.6 160.2230*pi/30/3.6 160.2230*pi/30/3.6 160.2230*pi/30/3.6];%;0;0;0

% State = [1 5 0 0 0 0 160.2230*pi/30 160.2230*pi/30 160.2230*pi/30 160.2230*pi/30];%;0;0;0
State = [0 35/3.6 0 0 0 0 7*160.2230*pi/30/3.6 7*160.2230*pi/30/3.6 7*160.2230*pi/30/3.6 7*160.2230*pi/30/3.6]; %这是从35km/h出发的
State = [0 80/3.6 0 0 0 0 16*160.2230*pi/30/3.6 16*160.2230*pi/30/3.6 16*160.2230*pi/30/3.6 16*160.2230*pi/30/3.6]; %这是从35km/h出发的
% State = State*0.95;
%         1 %X 
%         1 % xdot
%         0 % Y
%         0 % Ydot
%         0  %phi
%         0 %dphi
%         160.2230;160.2230;160.2230;160.2230;];
for i =1:tf/Ts+1 %1e-2
    cccct = 15;
    sssst = sin((i-1)*Ts*pi/2)*pi/180-0.0000;%需要乘0.9以及减0.0002
%     if i > 2/Ts+1
%         sssst = 11*pi/180;%1*i*Ts*pi/180; %每秒1°
%         cccct = 60;
%     end
% 
%     if i > 4/Ts+1
%         cccct = 80;
%     end
  Ctr = [cccct cccct cccct cccct sssst ];%0.3 0 0
    State=runge_kutta4_0403_duibi(@vdn7_now0403_duibi,State,Ctr,Ts);%
    Ctr = Ctr(1:5);
    Statelog(:,i) = State;
    Contollog(:,i) = Ctr;
    auxdatalog(:,i) = auxdata;
    plotCtr = plotCtr+1;
%     t
    if plotCtr >PlotFreq
        plotCtr = 0;
% %         set(h1,'XData',t(1:i),'YData',Contollog(1,1:i));
%         set(h2,'XData',t(1:i),'YData',Statelog(2,1:i));s
% %         set(h3,'XData',t(1:i),'YData',Contollog(5,1:i));
%         set(h4,'XData',t(1:i),'YData',Statelog(3,1:i));
%         set(h5,'XData',Statelog(1,1:i),'YData',Statelog(3,1:i));
%         set(h6,'XData',t(1:i),'YData',auxdatalog(1,1:i));
%         set(h7,'XData',t(1:i),'YData',auxdatalog(2,1:i));
%         set(h8,'XData',t(1:i),'YData',auxdatalog(3,1:i));
%         set(h9,'XData',t(1:i),'YData',auxdatalog(4,1:i));
        drawnow;
    end
end

