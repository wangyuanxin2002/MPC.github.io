function Lapsim_ME780_demo
clc
close all
clear all
%-----------------------------------------------------------------------------------%
% Author: HUILONG YU, hlyubit@gmail.com.
% Date  : 09/11/2016
% Copyright (C) 2016 HUILONG YU. All Rights Reserved.
%-----------------------------------------------------------------------------------%
% Ref:1) Yu, H., Castelli-Dezza, F., & Cheli, F. (2017). Optimal powertrain design and 
% control of a 2-IWD electric race car. 2017 International Conference of Electrical 
% and Electronic Technologies for Automotive,(1), 1¨C7. 
% 2):Yu, H., Castelli-Dezza, F., & Cheli, F. (2017) Optimal powertrain design and 
% control of an electric race car, 25th International Symposium on Dynamics of 
% Vehicles on Roads and Tracks (IAVSD 2017), 14-18 August 2017, Rockhampton,
% Queensland, Australia

global Ftir Rtir pusr auxdata
tic
ManeuverSet='PathFollow';
DataSaveName=[ManeuverSet datestr(now,5) datestr(now,7)];
mkdir(DataSaveName);

pusr.ptype = 4;
tf=200;
dt=0.01;
pusr.tlog=0;
pusr.tsd=0;

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

% Front_Tire_Path = 'avon_ombra_front_f308.tir';
% Rear_Tire_Path  = 'avon_ombra_rear_f308.tir';
% [Ftir,Rtir]     = ImportData(Front_Tire_Path,Rear_Tire_Path);
% pusr.gamma      = [-3.1*pi/180 3.1*pi/180 -2.3*pi/180 2.3*pi/180];

load('NurburgringLengthVsCur3.mat');
pusr.slength = slength';
pusr.Curv    = Curv';
pusr.theta   = theta;

load('VIbest0915.mat');
pusr.xcoord  = xcoord;
pusr.ycoord  = ycoord;

MOTOR=load('motor2wd.mat');
pusr.Nm  = MOTOR.MotorV;
pusr.Tm  = MOTOR.MotorT;
pusr.ddXb  = 0;
pusr.ddYb  = 0;
pusr.kappalog=zeros(tf/dt,4);
pusr.alphalog=zeros(tf/dt,4);
pusr.Fzlog=zeros(tf/dt,4);
pusr.Ftzlog=zeros(tf/dt,4);
pusr.Ftlog=zeros(tf/dt,8);
pusr.Tdlog=zeros(tf/dt,4);
pusr.Steerlog=zeros(tf/dt,4);
pusr.Xlog=zeros(tf/dt,13);
pusr.dXlog=zeros(tf/dt,13);
pusr.ttlog=zeros(tf/dt,1);
pusr.TrackPlot=1;
pusr.TireYPlot=1;
pusr.VehicleBodyPlot=1;
pusr.TirePlot=1;
pusr.DriverinputPlot=1;

v0 = 3;
w0 = [v0 v0 v0 v0]./pusr.Rt;
ind = 500;
Xin=[v0 0 0 w0 pusr.slength(ind) 0 0 pusr.xcoord(ind) pusr.ycoord(ind) pusr.theta(ind)];
plotcontrol;
i=0;
for t= 0:dt:tf
    i = i+1;
    pusr.tlog = i;
    t
    u = controlstrategy(t,Xin);
    tic
    Xout=runge_kutta4(@vdn7,Xin,u,t,dt);%
    toc
    Xin = Xout; 
%-----------------------------------------------------------------------------------%
%                                     datalog                                    %
%-----------------------------------------------------------------------------------%
    pusr.kappalog(i,:) = auxdata(:,1:4);
    pusr.alphalog(i,:) = auxdata(:,5:8);
    pusr.Fzlog(i,:)    = auxdata(:,9:12);
    pusr.Ftlog(i,:)    = auxdata(:,13:20); 
    pusr.Tdlog(i,:)    = auxdata(:,21:24); 
    pusr.Ndlog(i,:)    = auxdata(:,25:28); 
    pusr.Steerlog(i,:) = auxdata(:,29:32); 
    pusr.Flog(i,:)     = auxdata(:,33:35); 
    pusr.Xlog(i,:)     = Xin;
    pusr.dXlog(i,:)    = Xout;
    pusr.ttlog(i,:)    = t;   
    if mod(i,100)==0  %plot sample time
       plot7dof(t,Xin,Xout)
    end    
    if  Xin(:,8)>4501
        break
    end

end

save([DataSaveName '\',DataSaveName]) 
h = get(0,'children');
for i = 1:length(h)
saveas(h(i),[DataSaveName '\',DataSaveName h(i).Name],'fig');
end
toc
end

