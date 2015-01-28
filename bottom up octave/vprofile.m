close all;
clear all;
system clear;

global p_max
global p_min
global a_max
global v_max
global t14
global t12
global t23
global t34
global t45
global t1
global t2
global t3
global t4
global t5
global a12
global a34
global a45
global p1
global p2
global p3
global p4
global p5
global v1
global v23
global v4
global v5


p_max = 24;
p_min = -24;
v_max = 4;
a_max = 3;

t_start = 0;
t_end = 5.4;
p_start = 0;
p_end = 0;
v_start = 4;
v_end = 4;

%t_start = 0;
%t_end = 1.1;
%p_start = 0;
%p_end = .5;
%v_start = 2;
%v_end = -1;

t_start = 0;
t_end = 4;
p_start = 4;
p_end = 0;
v_start = 4;
v_end = 4;

t_start = 0;
t_end = 5;
p_start = 0;
p_end = 10;
v_start = 0;
v_end = 1;

%([4 6], [4 2], [0 .5], [4 -1], .5) % success!

setTrapezoidProfileDurationBottomUp(p_start, v_start, p_end, v_end, t_end)

%time = [0:.025:(t5+.5)];
%pos = zeros(1,length(time));
%vel = zeros(1,length(time));
%acc = zeros(1,length(time));
%
%i=1;
%
%for t=time
%[a, v, p] = posVelAcc(t);
%pos(i)=p;
%vel(i)=v;
%acc(i)=a;
%i = i+1;
%end

%figure(1);
%subplot(3,1,1);
%plot(time, pos, 'b', [t1-.25,t5+.5], 0, '');
%xlabel ('time [s]');
%ylabel ('position [rad]');
%subplot(3,1,2);
%plot([t1-.25,t5+.5], [v_max,v_max], 'r', [t1-.25,t5+.5], [-v_max,-v_max], 'r', time, vel, 'b');
%xlabel ('time [s]');
%ylabel ('velocity [rad/s]');
%subplot(3,1,3);
%plot([t1-.25,t5+.5], [a_max,a_max], 'r', [t1-.25,t5+.5], [-a_max,-a_max], 'r', time, acc, 'b');
%xlabel ('time [s]');
%ylabel ('acceleration [rad/s/s]');

