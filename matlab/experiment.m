close all
clear all

step_duration = 0.05;
T_G_T = transl(0, 0, -.2);

irp = irp6_model;
ball = blueball_model;
obj.mjt = multiJointTrajectory(6);

figure
irp.plot(0);
ball.plot(0);

disp('Adjust workspace view and press [Enter]');
pause;

for t= 0:step_duration:10
    ball.plot(t);
    
    if(mod(t, 1) == 0)
        irp.set_q(irp.robot.ikine(ball.T_0_G(t) * T_G_T, irp.read_q(t)), t);
        %irp.set_q(ikineIRp6(ball.T_0_G * T_G_T), t);
        disp('irp position set')
    end
    irp.plot(t)
    
    pause(step_duration);
end



