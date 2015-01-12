clear t
close all

t = trajectory;
t.setVerbose(1);
t.setDrawPlots(1);

t.setPMinMax(0, 0);
t.setAMax(3);
t.setVMax(4);

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

t.calculate(p_start, v_start, p_end, v_end, t_end)