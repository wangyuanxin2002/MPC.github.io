function [y]=runge_kutta4(ufunc,Xin,u,t,h)% ode45
%-----------------------------------------------------------------------------------%
% Author: HUILONG YU, hlyubit@gmail.com.
% Date  : 09/11/2016
% Copyright (C) 2016 HUILONG YU. All Rights Reserved.
%-----------------------------------------------------------------------------------%
k1=ufunc(t,Xin,u);
k2=ufunc(t+h/2,Xin+h*k1/2,u);
k3=ufunc(t+h/2,Xin+h*k2/2,u);
k4=ufunc(t+h,Xin+h*k3,u);

y=Xin+h*(k1+2*k2+2*k3+k4)/6; 
end