close all
clear all

step_duration = 0.05;
horizon = 2;
T_G_T = transl(0, 0, -.2);

irp = irp6_model;
ball = blueball_model;
mjt = multiJointTrajectory(6);
mjt.setVMax(irp.qd_max);
mjt.setAMax(irp.qdd_max);
mjt.setPMinMax(irp.q_min, irp.q_max);
t_off=0;


figure
irp.plot(0);
ball.plot(0);

disp('Adjust workspace view and press [Enter]');
pause;

for t= 0:step_duration:7
    % Ball update
    ball.plot(t);
    
    % Robot trajectory calculation
    if(mod(t, horizon) == 0)
        q_d = irp.robot.ikine(ball.T_0_G(t+horizon) * T_G_T, irp.read_q(t));
        J0 = irp.robot.jacob0(q_d);
        qd_d = inv(J0) * ball.V(t+horizon);
        
        [qdd_c, qd_c, q_c] = mjt.avp(t - t_off);
        if(t == 0)
            q_c = irp.read_q(t);
        end
        mjt.calculate(q_c, qd_c, q_d, qd_d, horizon);
        t_off = t;
        
        
        
        
        disp('irp trajectory calculation')
    end
    
    % Robot trajectory execution
    [qdd_d, qd_d, q_d] = mjt.avp(t - t_off);
    irp.set_q(q_d, t);
    irp.plot(t)
    
    pause(step_duration);
end



