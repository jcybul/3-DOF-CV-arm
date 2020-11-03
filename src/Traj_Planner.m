classdef Traj_Planner
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        viaPoints
    end
    
    methods  
        function obj = Traj_Planner(viaPoints)
            obj.viaPoints = viaPoints;
        end
        
        function coefficients = cubic_traj(~,t0,tf,v0,vf,q0,qf)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            M = [
                1 t0 t0^2 t0^3;
                0 1 2*t0 3*t0^2;
                1 tf tf^2 tf^3;
                0 1 2*tf 3*tf^2];
            
            coefficients = M\[q0;v0;qf;vf];
        end
        
        function q = cubic_poly(~,coef,t)
           
            q = coef(1) + coef(2)*t + coef(3)*t^2 + coef(4)*t^3;
            
        end
        
    end
end

