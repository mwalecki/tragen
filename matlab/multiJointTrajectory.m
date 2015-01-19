classdef multiJointTrajectory < handle
    %MULTIJOINTTRAJECTORY Multi Joint Trajectory
    %   mwalecki 2015
    
    properties
        traj = trajectory.empty(); % Vector of joint trajectories
        n_traj = 0;
        % debug options
        verbose = 0;
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
        function [success, proper_duration] = calculate(obj, p_start, v_start, p_end, v_end, duration)
            success = 1;
            proper_duration(length(duration)) = 0;
            if((length(p_start) ~= obj.n_traj) || (length(v_start) ~= obj.n_traj) ...
                || (length(p_end) ~= obj.n_traj) || (length(v_end) ~= obj.n_traj) ...
                || ((length(duration) ~= obj.n_traj) && (length(duration) ~= 1)))
                error('Positions and velocities vector lengths must match with the number of joints.')
            end
            if(length(duration) == obj.n_traj)
                for i=1:obj.n_traj
                    [s, pd] = obj.traj(i).calculate(p_start(i), v_start(i), p_end(i), v_end(i), duration(i));
                    proper_duration(i) = pd;
                    if(s == 0);
                        success = 0;
                        warning('Trajectory generation of joint %d failed.', i)
                    end
                end
            elseif(length(duration) == 1)
                single_traj_dur(obj.n_traj) = 0;
                while(1)
                    for i=1:obj.n_traj
                        [s, pd] = obj.traj(i).calculate(p_start(i), v_start(i), p_end(i), v_end(i), duration);
                        single_traj_dur(i) = pd;
                        if(s == 0);
                            success = 0;
                            warning('Trajectory generation of joint %d failed.', i)
                        end
                    end
                    max_traj_dur = max(single_traj_dur);
                    break_cond = prod(single_traj_dur >= (max_traj_dur - eps) * ones(1, obj.n_traj)) ...
                        * prod(single_traj_dur <= (max_traj_dur + eps) * ones(1, obj.n_traj));
                    if(logical(break_cond))
                        proper_duration = max_traj_dur;
                        break;
                    end
                    duration = max_traj_dur; % The next loop iteration will try to calculate motion of extended duration.
                end
            else
                error('Duration must be of size 1 or a vector of length of joints number.')
            end
        end
        function plot(obj)
        % plot()
        % plot(trajectories)
        % plot(trajectories, t_end)
        % plot(trajectories, t_start, t_end)
        %
        
        % TODO: Work with trajecotry.plot()
            switch(nargin)
                case 1
                    trajectories = 1:obj.n_traj;
                    time = 0:0.025:1.2*max(obj.traj.t5);
                case 2
                    trajectories = vargin(2);
                    if(trajectories == [])
                        trajectories = 1:obj.n_traj;
                    end
                    time = 0:0.025:1.2*max(obj.traj(trajectories).t5);
                case 3
                    trajectories = vargin(2);
                    if(trajectories == [])
                        trajectories = 1:obj.n_traj;
                    end
                    if(varargin(3) > 0)
                        time = 0:0.025:varargin(3);
                    else
                        error('t_end must be > 0')
                    end
                case 4
                    trajectories = vargin(2);
                    if(trajectories == [])
                        trajectories = 1:obj.n_traj;
                    end
                    if(varargin(3) > 0 && varargin(4) > 0 && varargin(3) < varargin(4))
                        time = vargin(3):0.025:varargin(4);
                    else
                        error('t_start and t_end must be > 0 and t_start must be < t_end')
                    end
                otherwise
                    error('Improper argument number. Use: plot(), plot(trajectories), plot(trajectories, t_end) or plot(trajectories, t_start, t_end)')
            end
            
            close all
            for traj_plot = trajectories
                i=1;
                pos(length(time)) = 0;
                vel = pos;
                acc = pos;
                for t=time
                    [a, v, p] = obj.traj(traj_plot).posVelAcc(t);
                    pos(i)=p;
                    vel(i)=v;
                    acc(i)=a;
                    i = i+1;
                end
                figure
                PVTPlot(0, time, pos, vel, acc, obj.v_max, obj.a_max, obj.t1, obj.t2, obj.t3, obj.t4, obj.t5);
            end
        end
    end
end

