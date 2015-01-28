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
        function [acc, vel, pos] = avp(obj, t)
            acc(obj.n_traj) = 0;
            vel(obj.n_traj) = 0;
            pos(obj.n_traj) = 0;
            for traj_no = 1:obj.n_traj
                [a, v, p] = obj.traj(traj_no).posVelAcc(t);
                pos(traj_no)=p;
                vel(traj_no)=v;
                acc(traj_no)=a;
            end
        end
        function plot(obj, varargin)
        % plot()
        % plot(trajectories)
        % plot(trajectories, t_end)
        % plot(trajectories, t_start, t_end)
        %
        
        % TODO: Work with trajecotry.plot()
            for traj_no = 1:obj.n_traj
                t5s(traj_no) = obj.traj(traj_no).t5;
            end
            switch(nargin)
                case 1
                    trajectories = 1:obj.n_traj;
                    time = 0:0.025:1.2*max(t5s);
                case 2
                    trajectories = varargin{1};
                    if(trajectories == [])
                        trajectories = 1:obj.n_traj;
                    end
                    time = 0:0.025:1.2*max(t5s(trajectories));
                case 3
                    trajectories = varargin{1};
                    if(isempty(trajectories))
                        trajectories = 1:obj.n_traj;
                    end
                    if(varargin{2} > 0)
                        time = 0:0.025:varargin{2};
                    else
                        error('t_end must be > 0')
                    end
                case 4
                    trajectories = varargin{1};
                    if(trajectories == [])
                        trajectories = 1:obj.n_traj;
                    end
                    if(varargin{2} > 0 && varargin{3} > 0 && varargin{2} < varargin{3})
                        time = vargin(3):0.025:varargin{4};
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
                PVTPlot(0, time, pos, vel, acc, obj.traj(traj_plot).v_max, obj.traj(traj_plot).a_max, obj.traj(traj_plot).t1, obj.traj(traj_plot).t2, obj.traj(traj_plot).t3, obj.traj(traj_plot).t4, obj.traj(traj_plot).t5);
            end
        end
    end
end

