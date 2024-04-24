function y =runge_kutta4_0403(ufunc,Xin,u,h)
k1 = ufunc(Xin,u);
k2 = ufunc(Xin+h/2*k1,u);
k3 = ufunc(Xin+h/2*k2,u);
k4 = ufunc(Xin+h*k3,u);
y = Xin+h/6*(k1+2*k2+2*k3+k4);
end