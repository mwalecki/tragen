function setTrapezoidProfileDurationBottomUp(p_start, v_start, p_end, v_end, duration)
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


% Prepare data for debug plots
time = [0:.01:1.4*duration];
pos = zeros(1,length(time));
vel = zeros(1,length(time));
acc = zeros(1,length(time));


% Assure motion limits
v1 = v_start;
p1 = p_start;
if(p_end > p_max)
	v4 = 0;
	p4 = p_max;
elseif(p_end < p_min)
	v4 = 0;
	p4 = p_min;
else
	v4 = v_end;
	p4 = p_end;
end

if(v4 > v_max)
  v4 = v_max;
elseif(v4 < -v_max)
  v4 = -v_max;
end

% 1. Minimum time calculation
% 1.1. Triangular velocity profile with maximum acceleration
disp('[info] Calculating triangular velocity profile with maximum acceleration');

arr_t34 = [];
arr_a12 = [];
arr_idx = 0;

a12=a_max
delta = 2*v4^2 + 2*v1^2 + 4*a12*(p4-p1)

if(delta >= 0)
  sqrtDelta = sqrt(delta);
  solution_t34 = (-2*v4 - sqrtDelta)/(2*a12)
  if(solution_t34 >= 0)
    solution_t12 = solution_t34 + (v4-v1)/a12
    if(solution_t12 >= 0)
      arr_idx = arr_idx+1;
      arr_a12(arr_idx) = a12;
      arr_a34(arr_idx) = -a12;
      arr_t12(arr_idx) = solution_t12;
      arr_t34(arr_idx) = solution_t34;
    end
  end
  solution_t34 = (-2*v4 + sqrtDelta)/(2*a12)
  if(solution_t34 >= 0)
    solution_t12 = solution_t34 + (v4-v1)/a12
    if(solution_t12 >= 0)
      arr_idx = arr_idx+1;
      arr_a12(arr_idx) = a12;
      arr_a34(arr_idx) = -a12;
      arr_t12(arr_idx) = solution_t12;
      arr_t34(arr_idx) = solution_t34;
    end
  end
end

a12=-a_max
delta = 2*v4^2 + 2*v1^2 + 4*a12*(p4-p1)

if(delta >= 0)
  sqrtDelta = sqrt(delta);
  solution_t34 = (-2*v4 - sqrtDelta)/(2*a12)
  if(solution_t34 >= 0)
    solution_t12 = solution_t34 + (v4-v1)/a12
    if(solution_t12 >= 0)
      arr_idx = arr_idx+1;
      arr_a12(arr_idx) = a12;
      arr_a34(arr_idx) = -a12;
      arr_t12(arr_idx) = solution_t12;
      arr_t34(arr_idx) = solution_t34;
    end
  end
  solution_t34 = (-2*v4 + sqrtDelta)/(2*a12)
  if(solution_t34 >= 0)
    solution_t12 = solution_t34 + (v4-v1)/a12
    if(solution_t12 >= 0)
      arr_idx = arr_idx+1;
      arr_a12(arr_idx) = a12;
      arr_a34(arr_idx) = -a12;
      arr_t12(arr_idx) = solution_t12;
      arr_t34(arr_idx) = solution_t34;
    end
  end
end

arr_t12
arr_a12
arr_t34
arr_a34

j=0;
fig_id=0;
for t12=arr_t12
  j = j+1;
  t12 = arr_t12(j);
  t34 = arr_t34(j);
  a12 = arr_a12(j);
  a34 = arr_a34(j);
  
%  ta12 = a34;
%  a34 = a12;
%  a12 = ta12;
%  tt12 = t34;
%  t34 = t12;
%  t12 = tt12;

  t23 = 0;
  v23 = v1 + a12*t12;
  p2 = p1 + (v1 + v23)*t12/2;
  p3 = p2 + v23*t23;
  p4 = p3 + (v23 + v4)*t34/2;
  % Add a deceleration ramp at the end of the move in case following (p,v,t) fails to show up
  v5 = 0;
  
  t1 = 0;
  t2 = t1 + t12;
  t3 = t2 + t23;
  t4 = t3 + t34;
  
  if(v4 > 0)
	  a45 = -a_max;
	  t45 = -v4/a45;
  elseif(v4 < 0)
	  a45 = a_max;
	  t45 = -v4/a45;
  else
	  a45 = 0;
	  t45 = 0;
  end
  
  t5 = t4 + t45;
  p5 = p4 + v4*t45/2;
  
  i=1;
  for t=time
  [a, v, p] = posVelAcc(t);
  t;
  p;
  pos(i)=p;
  vel(i)=v;
  acc(i)=a;
  i = i+1;
  end

  fig_id=fig_id+1;
  PVTPlot(fig_id, time, pos, vel, acc, v_max, a_max, t1, t2, t3, t4, t5);
  break;
end

v23
v_max
t4

% 1.2. Trapezoidal velocity profile with maximum acceleration

if((v23 > v_max) || (v23 < -v_max))
  disp('[info] v_23 exceeds v_max');
  disp('[info] Calculating trapezoidal velocity profile with velocity saturation and maximum acceleration');

  v23 = v_max
  v4
  a34

  t12 = (v23-v1)/a12
  t34 = (v4-v23)/a34
  t23 = (p4 - (p1 + (v1+v23)*t12/2 + (v23+v4)*t34/2))/v23

  p2 = p1 + (v1 + v23)*t12/2;
  p3 = p2 + v23*t23;
  p4 = p3 + (v23 + v4)*t34/2;
  % Add a deceleration ramp at the end of the move in case following (p,v,t) fails to show up
  v5 = 0;
  
  t1 = 0;
  t2 = t1 + t12;
  t3 = t2 + t23;
  t4 = t3 + t34;
  
  if(v4 > 0)
	  a45 = -a_max;
	  t45 = -v4/a45;
  elseif(v4 < 0)
	  a45 = a_max;
	  t45 = -v4/a45;
  else
	  a45 = 0;
	  t45 = 0;
  end
  
  t5 = t4 + t45;
  p5 = p4 + v4*t45/2;
  
  
%time = time(1:400);
%pos = pos(1:400);
%vel = vel(1:400);
%acc = acc(1:400);
  i=1;
  for t=time
  [a, v, p] = posVelAcc(t);
  t;
  p;
  pos(i)=p;
  vel(i)=v;
  acc(i)=a;
  i = i+1;
  end
  
  fig_id=fig_id+1;
  PVTPlot(fig_id, time, pos, vel, acc, v_max, a_max, t1, t2, t3, t4, t5)
end

t4

if(t4 == duration)
  disp('[info] Motion will be executed with desired duration, equal to minimum possible time.');
  return;
end

if(t4 > duration)
  disp('[info] Motion will be executed at minimum possible time, greater than desired.');
  return;
end

disp('[info] Desired motion duration will be longer than minimum possible time');

% 2. Desired motion duration is greater than calculated minimum time.
% 2.1. Sort the motion phases due to their duration.

% Timestab contains duration of motion phases.
timestab = zeros(3,1);
timestab(1) = t12;
timestab(2) = t23;
timestab(3) = t34

% 2.1.1. Sort all three phases by duration.
[S, indextab] = sort(timestab);

% indextab contains indexes of motion phases, sorted by duration.
% index 1: t12
% index 2: t23
% index 3: t34
%indextab


% 2.2.2. Sort only acceleration phases by duration.
[S, accintab] = sort(timestab(1:2));
  


% 2.2. Extend the shortest phase duration until it equals the duration of the second shortest phase or desired motion duration is achieved.

method = 4

switch(method)
  case 1
    % 2.2.1 Extend evenly (for 1:1:1 phase duration proportions)
    if((3*timestab(indextab(3))) <= duration)
      timestab(1) = duration/3;
      timestab(2) = duration/3;
      timestab(3) = duration/3;
    elseif((timestab(indextab(3)) + 2*timestab(indextab(2))) <= duration)
      timestab(indextab(1)) = (duration - timestab(indextab(3)))/2;
      timestab(indextab(2)) = (duration - timestab(indextab(3)))/2;
    else
      timestab(indextab(1)) = (duration - (timestab(indextab(2)) + timestab(indextab(3))));
    end
    % Common for extend methods - set the calculated times.
    t12 = timestab(1)
    t23 = timestab(2)
    t34 = timestab(3)
    
  case 2
    % 2.2.2 Extend acceleration phases first - 
    % t12 and t34 must not be 0 for equation (2.3.1) to comply with v1 and v4 values.
    % When t12==t34, increase all phases with the same component.
    if(2*timestab(accintab(2)) + timestab(2) <= duration)
      timestab(accintab(1)) = timestab(accintab(2));
      cmp = (duration - (timestab(accintab(1)) + timestab(accintab(2)) + timestab(2))) / 3;
      timestab(1) = timestab(1) + cmp;
      timestab(2) = timestab(2) + cmp;
      timestab(3) = timestab(3) + cmp;
    else
      timestab(accintab(1)) = duration - (timestab(accintab(2)) + timestab(2));
    end
    % Common for extend methods - set the calculated times.
    t12 = timestab(1)
    t23 = timestab(2)
    t34 = timestab(3)
    
  case 3
    % 2.2.3 Extend acceleration phases first - 
    % t12 and t34 must not be 0 for equation (2.3.1) to comply with v1 and v4 values.
    % Try to achieve t12==t23==t34.
    % First achieve t12==t34.
    if(2*timestab(accintab(2)) + timestab(2) <= duration)
      timestab(accintab(1)) = timestab(accintab(2));
    else
      timestab(accintab(1)) = duration - (timestab(accintab(2)) + timestab(2));
    end
    % Then extend all phases evenly
    % indextab needs update.
    [S, indextab] = sort(timestab);
    if((3*timestab(indextab(3))) <= duration)
      timestab(1) = duration/3;
      timestab(2) = duration/3;
      timestab(3) = duration/3;
    elseif((timestab(indextab(3)) + 2*timestab(indextab(2))) <= duration)
      timestab(indextab(1)) = (duration - timestab(indextab(3)))/2;
      timestab(indextab(2)) = (duration - timestab(indextab(3)))/2;
    else
      timestab(indextab(1)) = (duration - (timestab(indextab(2)) + timestab(indextab(3))));
    end
    % Common for extend methods - set the calculated times.
    t12 = timestab(1)
    t23 = timestab(2)
    t34 = timestab(3)
    
  case 4
    % 2.2.4 Extend acceleration phases first - 
    % t12 and t34 must not be 0 for equation (2.3.1) to comply with v1 and v4 values.
    % Try to achieve t12==t23==t34 until t12==t34==2*v_max/a_max. Then increase only t23.
    % First achieve t12==t34.
    if(2*timestab(accintab(2)) + timestab(2) <= duration)
      timestab(accintab(1)) = timestab(accintab(2));
    else
      timestab(accintab(1)) = duration - (timestab(accintab(2)) + timestab(2));
    end
    % At this point either t12==t34 or sum t12+t23+t34==duration.
    % indextab needs update - sort all phases by duration.
    [S, indextab] = sort(timestab);
    % Try to extend all phases evenly until t12==t23==t34==2*v_max/a_max
    temp_ph_dur=2*v_max/a_max;
    if((3*temp_ph_dur < duration) && (t12+t23+t34 < 3*temp_ph_dur))
      if((3*timestab(indextab(3))) <= 3*temp_ph_dur)
        timestab(1) = temp_ph_dur;
        timestab(2) = temp_ph_dur;
        timestab(3) = temp_ph_dur;
      elseif((timestab(indextab(3)) + 2*timestab(indextab(2))) <= 3*temp_ph_dur)
        timestab(indextab(1)) = (3*temp_ph_dur - timestab(indextab(3)))/2;
        timestab(indextab(2)) = (3*temp_ph_dur - timestab(indextab(3)))/2;
      else
        timestab(indextab(1)) = (3*temp_ph_dur - (timestab(indextab(2)) + timestab(indextab(3))));
      end
    end
    % Common for extend methods - set the calculated times.
    t12 = timestab(1);
    t23 = timestab(2);
    t34 = timestab(3);
    % Then increase only t23.
    if(t12+t23+t34 < duration)
      t23 = duration - (t12 + t34);
    end
    t12
    t23
    t34
end



% 2.2.3 Scale all phase durations with the same factor
%fct = duration/(t12 + t23 + t34)
%t12 = fct * t12
%t23 = fct * t23
%t34 = fct * t34





t1 = 0;
t2 = t1 + t12;
t3 = t2 + t23;
t4 = t3 + t34;


% 2.3. Given the time of each phase, calculate the trapezoidal velocity profile.

if((t12 + 2*t23 + t34) == 0)
  disp('[error] Duration shouldn`t be 0');
  return;
end

v23 = (2*(p4-p1) - v1*t12 - v4*t34) / (t12 + 2*t23 + t34)

if(t12 > 0)
  a12 = (v23-v1)/t12;
else
  a12 = 0;
end

if(t34 > 0)
  a34 = (v4-v23)/t34;
else
  a34 = 0;
end

a12
a34

% Calculate positions
p2 = p1 + (v1 + v23)*t12/2;
p3 = p2 + v23*t23;
p4 = p3 + (v23 + v4)*t34/2;

% Add a deceleration ramp at the end of the move in case following (p,v,t) fails to show up
v5 = 0;

if(v4 > 0)
  a45 = -a_max;
  t45 = -v4/a45;
elseif(v4 < 0)
  a45 = a_max;
  t45 = -v4/a45;
else
  a45 = 0;
  t45 = 0;
end

t5 = t4 + t45;
p5 = p4 + v4*t45/2;

i=1;
for t=time
[a, v, p] = posVelAcc(t);
t;
p;
pos(i)=p;
vel(i)=v;
acc(i)=a;
i = i+1;
end

fig_id=fig_id+1;
PVTPlot(fig_id, time, pos, vel, acc, v_max, a_max, t1, t2, t3, t4, t5)


% Add a deceleration ramp at the end of the move in case following (p,v,t) fails to show up
%v5 = 0;
%if(v4 > 0)
%	a45 = -a_max;
%	t45 = -v4/a45;
%elseif(vel4 < 0)
%	a45 = a_max;
%	t45 = -v4/a45;
%else
%	a45 = 0;
%	t45 = 0;
%end

%t1 = 0
%t2 = t1 + t12
%t4 = t1 + t14
%t3 = t4 - t34
%t5 = t4 + t45
%t23 = t3 - t2;
%p2 = p1 + (v23+v1)*t12/2;
%p3 = p2 + v23*t23;
%p5 = p4 + v4*t45/2;

end
