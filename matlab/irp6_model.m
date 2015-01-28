classdef irp6_model < handle
    %irp6_model_full Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot
        qr
        q_min
        q_max
        qd_max
        qdd_max
        q_c     % current joint positions at the time of position command
        q_s     % set joint positions
        t_q_s   % joint positions set timestamp
        q_m     % measured joint positions
    end
    
    methods
        function obj = irp6_model()
            % IRp-6 Kinematics
            d1 = 0.70; % base height
            a2 = 0.45; % vertical link length
            a3 = 0.67; % horizontal link length
            d5 = 0.15; % 6. DOF displacement
            d7 = 0.20; % gripper length
            %            theta    d        a    alpha
            L(1) = Link([  0      0       0      0     0], 'modified');
            L(2) = Link([  0      0        0      -pi/2 0], 'modified');
            L(3) = Link([  0      0        a2     0     0], 'modified');
            L(4) = Link([  0      0        a3     0     0], 'modified');
            L(5) = Link([  0      d5       0      -pi/2 0], 'modified');
            L(6) = Link([  0      0        0      pi/2  0], 'modified');

            obj.robot = SerialLink(L, 'name', 'IRp6', 'comment', 'DH zmodyfikowane');
            %obj.robot.base = [1 0 0 0; 0 1 0 0; 0 0 1 d1; 0 0 0 1];
            obj.robot.tool = [1 0 0 0; 0 1 0 0; 0 0 1 d7; 0 0 0 1];
            
            obj.qr =     [0     -pi/2   pi/2    -pi/2   -pi/2   0];     % ready pose, gripper points down
            obj.q_min =  [-pi/2 -3*pi/4 1*pi/4  -pi     -pi     -pi];   % position limits
            obj.q_max =  [pi/2  -1*pi/4 3*pi/4  0       0       pi];    % position limits
            obj.qd_max = [pi    pi      pi      pi      pi      pi];    % velocity limits
            obj.qdd_max = 2*[pi pi      pi      pi      pi      pi];    % acceleration limits
            
            % Preset state
            obj.q_s = obj.qr;
            obj.t_q_s = 0;
            obj.q_c = obj.q_s;
            obj.q_m = obj.q_s;
        end
        function obj = set_q(obj, q_d, t)
            if(t < obj.t_q_s)
                error('Time seems to go back...')
            end
            % Calculate current joint positions. Keep velocity limits.
            t_elapsed = t-obj.t_q_s;
            q_c_temp = min(obj.q_s, obj.q_c + t_elapsed * obj.qd_max);
            q_c_temp = max(q_c_temp, obj.q_c - t_elapsed * obj.qd_max);
            q_c_temp = min(q_c_temp, obj.q_max);
            obj.q_c = max(q_c_temp, obj.q_min);
            % Store set joint positions and timestamp
            obj.q_s = q_d;
            obj.t_q_s = t;
        end
        function q_m = read_q(obj, t)
            if(t < obj.t_q_s)
                error('Time seems to go back...')
            end
            % Calculate current joint positions. Keep velocity limits.
            % They are valid only at the time of measure.
            t_elapsed = t-obj.t_q_s;
            q_m_temp = min(obj.q_s, obj.q_c + t_elapsed * obj.qd_max);
            q_m_temp = max(q_m_temp, obj.q_c - t_elapsed * obj.qd_max);
            q_m_temp = min(q_m_temp, obj.q_max);
            obj.q_m = max(q_m_temp, obj.q_min);
            q_m = obj.q_m;
        end
        function plot(obj,t)
            % Plot the plot
            obj.robot.plot(obj.read_q(t));
        end
    end
    
end

