classdef Path_Planner
    methods
        function obj = linear_traj(~,p0,pf)
            x = linspace(p0(1),pf(1),12);
            y = linspace(p0(2),pf(2),12);
            z = linspace(p0(3),pf(3),12);
            obj = [x; y; z];
        end
        function obj = quintic_traj(~,t0,tf,v0,vf,p0,pf,a0,af)
            
                M= [ 1 t0 t0^2 t0^3 t0^4 t0^5;
                    0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
                    0 0 2 6*t0 12*t0^2 20*t0^3;
                    1 tf tf^2 tf^3 tf^4 tf^5;
                    0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
                    0 0 2 6*tf 12*tf^2 20*tf^3];
                
                obj = M\[p0;v0;a0;pf;vf;af];
        end
        
        function obj = quintic_poly(~,coef,t)
                
            obj = coef(1) + coef(2)*t + coef(3)*t^2 + coef(4)*t^3 + coef(5)*t^4 + coef(6)*t^5;
        end
            
    end
end

