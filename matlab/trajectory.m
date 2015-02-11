classdef trajectory < handle
    %TRAJECTORY Single Joint Trajectory
    %   mwalecki 2015
    
    properties
        p_min = 0.0;
        p_max = 0.0;
        a_max = 0.0;
        v_max = 0.0;
        t1 = 0.0;
        t2 = 0.0;
        t3 = 0.0;
        t4 = 0.0;
        t5 = 0.0;
        a12 = 0.0;
        a34 = 0.0;
        a45 = 0.0;
        p1 = 0.0;
        p2 = 0.0;
        p3 = 0.0;
        p4 = 0.0;
        p5 = 0.0;
        v1 = 0.0;
        v23 = 0.0;
        v4 = 0.0;
        v5 = 0.0;
        % debug options
        verbose = 0;
        draw_plots = 0;
    end
    
    methods
        function obj=setVerbose(obj, k)
            obj.verbose = k;
        end
        function obj=setDrawPlots(obj, k)
            obj.draw_plots = k;
        end
        function obj=setAMax(obj, a)
            if(a > 0)
                obj.a_max = a;
            else
                error('a_max must be > 0')
            end
        end
        function obj=setVMax(obj, v)
            if(v > 0)
                obj.v_max = v;
            else
                error('v_max must be > 0')
            end
        end
        function obj=setPMinMax(obj, pn, px)
            if(pn < px)
                obj.p_min = pn;
                obj.p_max = px;
            elseif(pn == px)
                obj.p_min = pn;
                obj.p_max = px;
                warning('p_min == p_max. No position limit will be applied.')
            else
                error('p_min must be < p_max')
            end
        end
        function [a, v, p] = posVelAcc(obj, t)
        % [a, v, p] = posVelAcc(t)
        %
            if(t < obj.t1)
                % Oh, really?
                a = 0;
                v = 0;
                p = 0;
            elseif(t < obj.t2)
                temp_t = t-obj.t1;
                a = obj.a12;
                v = obj.v1 + obj.a12*temp_t;
                p = obj.p1 + obj.v1*temp_t + obj.a12*temp_t*temp_t/2;
            elseif(t < obj.t3)
                temp_t = t-obj.t2;
                a = 0;
                v = obj.v23;
                p = obj.p2 + obj.v23*temp_t;
            elseif(t < obj.t4)
                temp_t = t-obj.t3;
                a = obj.a34;
                v = obj.v23 + obj.a34*temp_t;
                p = obj.p3 + obj.v23*temp_t + obj.a34*temp_t*temp_t/2;
            elseif(t < obj.t5)
                temp_t = t-obj.t4;
                a = obj.a45;
                v = obj.v4 + obj.a45*temp_t;
                p = obj.p4 + obj.v4*temp_t + obj.a45*temp_t*temp_t/2;
            else
                a = 0;
                v = 0;
                p = obj.p5;
            end
        end
        function plot(obj, varargin)
        % plot()
        % plot(t_end)
        % plot(t_start, t_end)
        %
            switch(nargin)
                case 1
                    time = 0:0.025:1.2*obj.t5;
                case 2
                    if(varargin{1} > 0)
                        time = 0:0.025:varargin{1};
                    else
                        error('t_end must be > 0')
                    end
                case 3
                    if(varargin{1} > 0 && varargin{2} > 0 && varargin{1} < varargin{2})
                        time = varargin{1}:0.025:varargin{2};
                    else
                        error('t_start and t_end must be > 0 and t_start must be < t_end')
                    end
                otherwise
                    error('Improper argument number. Use: plot(), plot(t_end) or plot(t_start, t_end)')
            end
            
            i=1;
            pos(length(time)) = 0;
            vel = pos;
            acc = pos;
            for t=time
                [a, v, p] = obj.posVelAcc(t);
                pos(i)=p;
                vel(i)=v;
                acc(i)=a;
                i = i+1;
            end

            PVTPlot(0, time, pos, vel, acc, obj.v_max, obj.a_max, obj.t1, obj.t2, obj.t3, obj.t4, obj.t5);
        end
        function [success, proper_duration] = calculate(obj, p_start, v_start, p_end, v_end, duration)
            % success = calculate(obj, p_start, v_start, p_end, v_end, duration)
            %
            
            success = 0;
            proper_duration = 0;
            
            if(obj.draw_plots)
                % Prepare data for debug plots
                time = [0:.01:2.4*duration];
                pos = zeros(1,length(time));
                vel = zeros(1,length(time));
                acc = zeros(1,length(time));
            end

            obj.v1 = v_start;
            obj.p1 = p_start;
            
            % Assure motion limits            
            if(obj.p_max > obj.p_min)
                if(p_end > obj.p_max)
                    obj.v4 = 0;
                    obj.p4 = obj.p_max;
                elseif(p_end < obj.p_min)
                    obj.v4 = 0;
                    obj.p4 = obj.p_min;
                else
                    obj.v4 = v_end;
                    obj.p4 = p_end;
                end
            else
                obj.v4 = v_end;
                obj.p4 = p_end;
            end
            
            if(obj.v4 > obj.v_max)
              obj.v4 = obj.v_max;
            elseif(obj.v4 < -obj.v_max)
              obj.v4 = -obj.v_max;
            end
            
            % 1. Minimum time calculation
            % 1.1. Triangular velocity profile with maximum acceleration
            if(obj.verbose)
                disp('[info] Calculating triangular velocity profile with maximum acceleration');
            end

            arr_idx = 0;
            obj.a12=obj.a_max;
            delta = 2*obj.v4^2 + 2*obj.v1^2 + 4*obj.a12*(obj.p4-obj.p1);

            if(delta >= 0)
              sqrtDelta = sqrt(delta);
              solution_t34 = (-2*obj.v4 - sqrtDelta)/(2*obj.a12);
              if(solution_t34 >= 0)
                solution_t12 = solution_t34 + (obj.v4-obj.v1)/obj.a12;
                if(solution_t12 >= 0)
                  arr_idx = arr_idx+1;
                  arr_a12(arr_idx) = obj.a12;
                  arr_a34(arr_idx) = -obj.a12;
                  arr_t12(arr_idx) = solution_t12;
                  arr_t34(arr_idx) = solution_t34;
                end
              end
              solution_t34 = (-2*obj.v4 + sqrtDelta)/(2*obj.a12);
              if(solution_t34 >= 0)
                solution_t12 = solution_t34 + (obj.v4-obj.v1)/obj.a12;
                if(solution_t12 >= 0)
                  arr_idx = arr_idx+1;
                  arr_a12(arr_idx) = obj.a12;
                  arr_a34(arr_idx) = -obj.a12;
                  arr_t12(arr_idx) = solution_t12;
                  arr_t34(arr_idx) = solution_t34;
                end
              end
            end

            obj.a12=-obj.a_max;
            delta = 2*obj.v4^2 + 2*obj.v1^2 + 4*obj.a12*(obj.p4-obj.p1);

            if(delta >= 0)
              sqrtDelta = sqrt(delta);
              solution_t34 = (-2*obj.v4 - sqrtDelta)/(2*obj.a12);
              if(solution_t34 >= 0)
                solution_t12 = solution_t34 + (obj.v4-obj.v1)/obj.a12;
                if(solution_t12 >= 0)
                  arr_idx = arr_idx+1;
                  arr_a12(arr_idx) = obj.a12;
                  arr_a34(arr_idx) = -obj.a12;
                  arr_t12(arr_idx) = solution_t12;
                  arr_t34(arr_idx) = solution_t34;
                end
              end
              solution_t34 = (-2*obj.v4 + sqrtDelta)/(2*obj.a12);
              if(solution_t34 >= 0)
                solution_t12 = solution_t34 + (obj.v4-obj.v1)/obj.a12;
                if(solution_t12 >= 0)
                  arr_idx = arr_idx+1;
                  arr_a12(arr_idx) = obj.a12;
                  arr_a34(arr_idx) = -obj.a12;
                  arr_t12(arr_idx) = solution_t12;
                  arr_t34(arr_idx) = solution_t34;
                end
              end
            end
            
            if(obj.verbose)
                disp('Trangular Velocity Profile solutions (t12>=0, t34>=0 only)');
                disp('t12');
                disp(arr_t12);
                disp('obj.a12');
                disp(arr_a12);
                disp('t34');
                disp(arr_t34);
                disp('obj.a34');
                disp(arr_a34);
            end

            j=0;
            fig_id=0;
            for t12=arr_t12
              j = j+1;
              t12 = arr_t12(j);
              t34 = arr_t34(j);
              obj.a12 = arr_a12(j);
              obj.a34 = arr_a34(j);

              t23 = 0;
              obj.v23 = obj.v1 + obj.a12*t12;
              obj.p2 = obj.p1 + (obj.v1 + obj.v23)*t12/2;
              obj.p3 = obj.p2 + obj.v23*t23;
              obj.p4 = obj.p3 + (obj.v23 + obj.v4)*t34/2;
              obj.t1 = 0;
              obj.t2 = obj.t1 + t12;
              obj.t3 = obj.t2 + t23;
              obj.t4 = obj.t3 + t34;
              % Add a deceleration ramp at the end of the move in case following (p,v,t) fails to show up
              obj.v5 = 0;
              if(obj.v4 > 0)
                  obj.a45 = -obj.a_max;
                  t45 = -obj.v4/obj.a45;
              elseif(obj.v4 < 0)
                  obj.a45 = obj.a_max;
                  t45 = -obj.v4/obj.a45;
              else
                  obj.a45 = 0;
                  t45 = 0;
              end
              obj.t5 = obj.t4 + t45;
              obj.p5 = obj.p4 + obj.v4*t45/2;

              if(obj.draw_plots)
                  i=1;
                  for t=time
                      [a, v, p] = obj.posVelAcc(t);
                      pos(i)=p;
                      vel(i)=v;
                      acc(i)=a;
                      i = i+1;
                  end
                  fig_id=fig_id+1;
                  PVTPlot(fig_id, time, pos, vel, acc, obj.v_max, obj.a_max, obj.t1, obj.t2, obj.t3, obj.t4, obj.t5);
              end

            % 1.2. Trapezoidal velocity profile with maximum acceleration

              if((obj.v23 > obj.v_max) || (obj.v23 < -obj.v_max))
                if(obj.verbose)
                    disp('[info] v_23 exceeds obj.v_max');
                    disp('[info] Calculating trapezoidal velocity profile with velocity saturation and maximum acceleration');
                end
                
                if(obj.v23 > obj.v_max)
                    obj.v23 = obj.v_max;
                else
                    obj.v23 = -obj.v_max;
                end

                t12 = (obj.v23-obj.v1)/obj.a12;
                t34 = (obj.v4-obj.v23)/obj.a34;
                t23 = (obj.p4 - (obj.p1 + (obj.v1+obj.v23)*t12/2 + (obj.v23+obj.v4)*t34/2))/obj.v23;

                obj.p2 = obj.p1 + (obj.v1 + obj.v23)*t12/2;
                obj.p3 = obj.p2 + obj.v23*t23;
                obj.p4 = obj.p3 + (obj.v23 + obj.v4)*t34/2;
                % Add a deceleration ramp at the end of the move in case following (p,v,t) fails to show up
                obj.v5 = 0;

                obj.t1 = 0;
                obj.t2 = obj.t1 + t12;
                obj.t3 = obj.t2 + t23;
                obj.t4 = obj.t3 + t34;

                if(obj.v4 > 0)
                    obj.a45 = -obj.a_max;
                    t45 = -obj.v4/obj.a45;
                elseif(obj.v4 < 0)
                    obj.a45 = obj.a_max;
                    t45 = -obj.v4/obj.a45;
                else
                    obj.a45 = 0;
                    t45 = 0;
                end

                obj.t5 = obj.t4 + t45;
                obj.p5 = obj.p4 + obj.v4*t45/2;

                if(obj.draw_plots)
                    i=1;
                    for t=time
                        [a, v, p] = obj.posVelAcc(t);
                        t;
                        p;
                        pos(i)=p;
                        vel(i)=v;
                        acc(i)=a;
                        i = i+1;
                    end

                    fig_id=fig_id+1;
                    PVTPlot(fig_id, time, pos, vel, acc, obj.v_max, obj.a_max, obj.t1, obj.t2, obj.t3, obj.t4, obj.t5)
                end
              end

              arr_t12(j) = t12;
              arr_t23(j) = t23;
              arr_t34(j) = t34;
              arr_a12(j) = obj.a12;
              arr_a34(j) = obj.a34;
              arr_v23(j) = obj.v23;
              arr_t14(j) = t12+t23+t34;
            end %for t12=arr_t12

            % Sort solution arrays by the whole motion duration of each solution
            [arr_t14,I]=sort(arr_t14);
            arr_t12=arr_t12(I);
            arr_t23=arr_t23(I);
            arr_t34=arr_t34(I);
            arr_a12=arr_a12(I);
            arr_a34=arr_a34(I);
            arr_v23=arr_v23(I);

            if(obj.verbose)
                disp('Trapezoidal Velocity Profile solutions, sorted by motion duration');
                disp('t12');
                disp(arr_t12);
                disp('t23');
                disp(arr_t23);
                disp('t34');
                disp(arr_t34);
                disp('obj.a12');
                disp(arr_a12);
                disp('obj.a34');
                disp(arr_a34);
                disp('obj.v23');
                disp(arr_v23);
                disp('t14');
                disp(arr_t14);
            end

            for j=[1:length(arr_t14)];

              t12=arr_t12(j);
              t23=arr_t23(j);
              t34=arr_t34(j);
              t14=arr_t14(j);
              obj.a12=arr_a12(j);
              obj.a34=arr_a34(j);
              obj.v23=arr_v23(j);

              if(t14 == duration)
                if(obj.verbose)
                    disp('[info] Motion will be executed with desired duration, equal to minimum possible time.');
                end
                obj.t1 = 0;
                obj.t2 = obj.t1 + t12;
                obj.t3 = obj.t2 + t23;
                obj.t4 = obj.t3 + t34;
              elseif(t14 > duration)
                if(obj.verbose)
                    disp('[info] Motion will be executed at minimum possible time, greater than desired.');
                end
                obj.t1 = 0;
                obj.t2 = obj.t1 + t12;
                obj.t3 = obj.t2 + t23;
                obj.t4 = obj.t3 + t34;
              else % if(t14 < duration)
                if(obj.verbose)
                    disp('[info] Desired motion duration will be longer than minimum possible time');
                end

                % 2. Desired motion duration is greater than calculated minimum time.
                % 2.1. Sort the motion phases due to their duration.

                % Timestab contains duration of motion phases.
                timestab = zeros(3,1);
                timestab(1) = t12;
                timestab(2) = t23;
                timestab(3) = t34;

                % 2.1.1. Sort all three phases by duration.
                [S, indextab] = sort(timestab);

                % indextab contains indexes of motion phases, sorted by duration.
                % index 1: t12
                % index 2: t23
                % index 3: t34
                %indextab


                % 2.2.2. Sort only acceleration phases by duration.
                %[S, accintab] = sort(timestab(1:2)); Wrong!
                if(timestab(1) > timestab(3))  
                  accintab = [3;1];
                else
                  accintab = [1;3];
                end

                % 2.2. Extend the shortest phase duration until it equals the duration of the second shortest phase or desired motion duration is achieved.

                method = 4;

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
                    % t12 and t34 must not be 0 for equation (2.3.1) to comply with obj.v1 and obj.v4 values.
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
                    % t12 and t34 must not be 0 for equation (2.3.1) to comply with obj.v1 and obj.v4 values.
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
                    % t12 and t34 must not be 0 for equation (2.3.1) to comply with obj.v1 and obj.v4 values.
                    % Try to achieve t12==t23==t34 until t12==t34==2*obj.v_max/obj.a_max. Then increase only t23.
                    % First achieve t12==t34.
                    if(2*timestab(accintab(2)) + timestab(2) <= duration)
                      timestab(accintab(1)) = timestab(accintab(2));
                    else
                      timestab(accintab(1)) = duration - (timestab(accintab(2)) + timestab(2));
                    end
                    % At this point either t12==t34 or sum t12+t23+t34==duration.
                    % indextab needs update - sort all phases by duration.
                    [S, indextab] = sort(timestab);
                    % Try to extend all phases evenly until t12==t23==t34==2*obj.v_max/obj.a_max
                    temp_ph_dur=2*obj.v_max/obj.a_max;
                    if(3*temp_ph_dur > duration)
                      temp_ph_dur = duration/3;
                    end
                    if(t12+t23+t34 < 3*temp_ph_dur)
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
                  case 5
                    % 2.2.3 Scale all phase durations with the same factor
                    fct = duration/(t12 + t23 + t34)
                    t12 = fct * t12;
                    t23 = fct * t23;
                    t34 = fct * t34;
                end

                obj.t1 = 0;
                obj.t2 = obj.t1 + t12;
                obj.t3 = obj.t2 + t23;
                obj.t4 = obj.t3 + t34;


                % 2.3. Given the time of each phase, calculate the trapezoidal velocity profile.

                if((t12 + 2*t23 + t34) == 0)
                  error('Duration shouldn`t be 0');
                  %return;
                end

                obj.v23 = (2*(obj.p4-obj.p1) - obj.v1*t12 - obj.v4*t34) / (t12 + 2*t23 + t34);

                if(t12 > 0)
                  obj.a12 = (obj.v23-obj.v1)/t12;
                else
                  obj.a12 = 0;
                end

                if(t34 > 0)
                  obj.a34 = (obj.v4-obj.v23)/t34;
                else
                  obj.a34 = 0;
                end
              end % if(t14 < duration)

              if(obj.verbose)
                disptable([obj.t4 t12 t23 t34 obj.a12 obj.a34 obj.v23], 'obj.t4|t12|t23|t34|obj.a12|obj.a34|obj.v23', 'val');
              end

              % If the conditions are met, this is the proper solution.
              % Otherwise, continue with subsequent solutions.
              if((obj.a12 >= -obj.a_max) && (obj.a12 <= obj.a_max) && (obj.v23 >= -obj.v_max) && (obj.v23 <= obj.v_max))
                if(obj.verbose)
                    disp('[info] Proper solution found.');
                end
                success = 1;
                break;
              else
                if(obj.verbose)
                    disp('[info] Motion limits exceeded. Calculations proceed.');
                end

                % Calculate positions
                obj.p2 = obj.p1 + (obj.v1 + obj.v23)*t12/2;
                obj.p3 = obj.p2 + obj.v23*t23;
                obj.p4 = obj.p3 + (obj.v23 + obj.v4)*t34/2;
                % Add a deceleration ramp at the end of the move in case following (p,v,t) fails to show up
                obj.v5 = 0;
                if(obj.v4 > 0)
                  obj.a45 = -obj.a_max;
                  t45 = -obj.v4/obj.a45;
                elseif(obj.v4 < 0)
                  obj.a45 = obj.a_max;
                  t45 = -obj.v4/obj.a45;
                else
                  obj.a45 = 0;
                  t45 = 0;
                end
                obj.t5 = obj.t4 + t45;
                obj.p5 = obj.p4 + obj.v4*t45/2;
                
                if(obj.draw_plots)
                    i=1;
                    for t=time
                    [a, v, p] = obj.posVelAcc(t);
                    t;
                    p;
                    pos(i)=p;
                    vel(i)=v;
                    acc(i)=a;
                    i = i+1;
                    end
                    fig_id=fig_id+1;
                    PVTPlot(fig_id, time, pos, vel, acc, obj.v_max, obj.a_max, obj.t1, obj.t2, obj.t3, obj.t4, obj.t5);
                end
              end
            end%for j=[1:length(arr_t14)];

            % Calculate positions
            obj.p2 = obj.p1 + (obj.v1 + obj.v23)*t12/2;
            obj.p3 = obj.p2 + obj.v23*t23;
            obj.p4 = obj.p3 + (obj.v23 + obj.v4)*t34/2;

            % Add a deceleration ramp at the end of the move in case following (p,v,t) fails to show up
            obj.v5 = 0;

            if(obj.v4 > 0)
              obj.a45 = -obj.a_max;
              t45 = -obj.v4/obj.a45;
            elseif(obj.v4 < 0)
              obj.a45 = obj.a_max;
              t45 = -obj.v4/obj.a45;
            else
              obj.a45 = 0;
              t45 = 0;
            end

            obj.t5 = obj.t4 + t45;
            obj.p5 = obj.p4 + obj.v4*t45/2;

            if(obj.draw_plots)
                i=1;
                for t=time
                [a, v, p] = obj.posVelAcc(t);
                t;
                p;
                pos(i)=p;
                vel(i)=v;
                acc(i)=a;
                i = i+1;
                end

                fig_id=fig_id+1;
                PVTPlot(fig_id, time, pos, vel, acc, obj.v_max, obj.a_max, obj.t1, obj.t2, obj.t3, obj.t4, obj.t5);
            end
            if(success == 0)
                warning('No proper solution found. Calculated values may exceed motion limits.')
            end
            proper_duration = obj.t4 - obj.t1;
        end
    end
end

