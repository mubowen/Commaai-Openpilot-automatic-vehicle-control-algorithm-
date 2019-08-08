function result=rungekutta(initial,t,h,func)
% 4th order Runge kutta

k1=func*t*h;
k2=k1*h/2+func*(h/2+t);
k3=k2*h/2+func*(h/2+t);
k4=h*k3+func*(t+h);

result=initial+h/6*(k1+2*k2+2*k3+k4);