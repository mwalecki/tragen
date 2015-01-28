classdef blueball_model < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        radius
        P_start
        handle
        xx
        yy
        zz
        yd
    end
    
    methods
        function obj = blueball_model()
            obj.radius = 0.05;
            [obj.xx, obj.yy, obj.zz] = sphere(5);
            obj.xx = obj.xx*obj.radius;
            obj.yy = obj.yy*obj.radius;
            obj.zz = obj.zz*obj.radius;
            obj.P_start = [.8 -.5 0];
            obj.yd = .1;
        end
        function obj=plot(obj, t)
            if(ishandle(obj.handle))
                delete(obj.handle)
            end
            P = obj.P_start + [0 min(t/10, 1) 0];
            hold on
            obj.handle = surf(obj.xx+P(1), obj.yy+P(2), obj.zz+P(3));
        end
        function A = T_0_G(obj, t)
            P = obj.P_start + [0 min(t*obj.yd, 1) 0];
            A =[0 1 0 P(1);...
                1 0 0 P(2);...
                0 0 -1 P(3);...
                0 0 0 1];
        end
        function V = V(obj, t)
            if(t<10)
                V = [0 obj.yd 0 0 0 0]';
            else
                V = [0 0 0 0 0 0]';
            end
        end
    end
    
end

