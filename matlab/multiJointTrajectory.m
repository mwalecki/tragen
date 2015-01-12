classdef multiJointTrajectory < handle
    %MULTIJOINTTRAJECTORY Multi Joint Trajectory
    %   mwalecki 2015
    
    properties
        traj = trajectory.empty(); % Vector of joint trajectories
        n_traj = 0;
        % debug options
    end
    
    methods
        function obj = multiJointTrajectory(n)
            if(n > 0)
                obj.traj(n,1) = trajectory;
                obj.n_traj = n;
            else
                error('Number of joints must be > 0');
            end
        end
        function obj=setAMax(obj, a)
            if(length(a) ~= obj.n_traj)
                error('Vector length must match with the number of joints.')
            end
            for i=1:obj.n_traj
                obj.traj(i).setAMax(a(i));
            end
        end
        function obj=setVMax(obj, v)
            if(length(v) ~= obj.n_traj)
                error('Vector length must match with the number of joints.')
            end
            for i=1:obj.n_traj
                obj.traj(i).setVMax(v(i));
            end
        end
        function obj=setPMinMax(obj, mn, mx)
            if((length(mn) ~= obj.n_traj) || (length(mx) ~= obj.n_traj))
                error('Both vector lengths must match with the number of joints.')
            end
            for i=1:obj.n_traj
                obj.traj(i).setPMinMax(mn(i), mx(i));
            end
        end
        function success = calculate(obj, p_start, v_start, p_end, v_end, duration)
            success = 0;
            if((length(p_start) ~= obj.n_traj) || (length(v_start) ~= obj.n_traj) ...
                || (length(p_end) ~= obj.n_traj) || (length(v_end) ~= obj.n_traj) ...
                || ((length(duration) ~= obj.n_traj) && (length(duration) ~= 1)))
                error('Positions and velocities vector lengths must match with the number of joints.')
            end
            if(length(duration) == 1)
                for i=1:obj.n_traj
                    obj.traj(i).calculate(p_start(i), v_start(i), p_end(i), v_end(i), duration(i));
                end
            else
                for i=1:obj.n_traj
                    obj.traj(i).calculate(p_start(i), v_start(i), p_end(i), v_end(i), duration);
                end
            end
        end
    end
end

