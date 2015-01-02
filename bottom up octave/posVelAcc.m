function [a, v, p] = posVelAcc(t)
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

a=0;
v=0;
p=0;

if(t < t1)
	% Oh, really?
	a = 0;
	v = 0;
	p = 0;
elseif(t < t2)
	temp_t = t-t1;
	a = a12;
	v = v1 + a12*temp_t;
	p = p1 + v1*temp_t + a12*temp_t*temp_t/2;
elseif(t < t3)
	temp_t = t-t2;
	a = 0;
	v = v23;
	p = p2 + v23*temp_t;
elseif(t < t4)
	temp_t = t-t3;
	a = a34;
	v = v23 + a34*temp_t;
	p = p3 + v23*temp_t + a34*temp_t*temp_t/2;
elseif(t < t5)
	temp_t = t-t4;
	a = a45;
	v = v4 + a45*temp_t;
	p = p4 + v4*temp_t + a45*temp_t*temp_t/2;
else
	a = 0;
	v = 0;
	p = p5;
end

end
