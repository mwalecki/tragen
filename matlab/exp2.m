close all
clear all

step_duration = 0.05;
horizon = 2;
T_G_T = transl(0, 0, -.2);

irp = irp6_model;
ball = blueball_model;
mjt = multiJointTrajectory(6);

figure
irp.plot(0);
ball.plot(0);

disp('Adjust workspace view and press [Enter]');
pause;

for t= 0:step_duration:10
    % Ball update
    ball.plot(t);
    
    % Robot trajectory calculation
    if(mod(t, horizon) == 0)
        q_m = irp.read_q(t);
        q_t = irp.robot.ikine(ball.T_0_G(t+horizon) * T_G_T, q_m);
        for i=1:6
            q_d(i, :) = linspace(q_m(i), q_t(i), horizon/step_duration);
        end
        ind=1;
        disp('irp trajectory calculation')
    end
    
    % Robot trajecotiry execution
    irp.set_q(q_d(:,ind)', t);
    ind = ind+1;
    irp.plot(t)
    
    pause(step_duration);
end



