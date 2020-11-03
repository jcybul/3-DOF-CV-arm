classdef Interpolator
    %INTERPOLATOR Interpolates movement between two points according to
    %specified curve
    %   type: motion type (cubic, quintic)
    %   duration: duration of motion in seconds
    
    properties
        type
        duration
    end
    
    methods
        function I = Interpolator(type,duration)
            %INTERPOLATOR Construct an instance of this class
            I.type = type;
            I.duration = duration;
        end
        
        function sc = get_sc(self, time)
            %GET_SC returns a value from 0 to 1 of how far through the
            %motion the joint should be based on current time
            %   time: time through movement (between 0 and self.duration)
            
            if self.type == "cubic"
                %cubic trajectory
                M = [1 0 0 0;
                     0 1 0 0;
                     1 self.duration self.duration^2 self.duration^3;
                     0 1 2*self.duration 3*self.duration^2];

                %4x1 vector of trajectory properties (position and velocity)
                q = [0; 0; 1; 0];

                %4x1 vector of coefficients
                a = M\q;

                sc = a(1) + a(2)*time + a(3)*time^2 + a(4)*time^3;
            elseif self.type == "quintic"
                %quintic trajectory
                M = [1 0 0 0 0 0;
                     0 1 0 0 0 0;
                     0 0 2 0 0 0;
                     1 self.duration self.duration^2 self.duration^3 self.duration^4 self.duration^5;
                     0 1 2*self.duration 3*self.duration^2 4*self.duration^3 5*self.duration^4;
                     0 0 2 6*self.duration 12*self.duration^2 20*self.duration^3];

                %6x1 vector of trajectory properties (position, velocity, acceleration)
                q = [0; 0; 0; 1; 0; 0];

                %6x1 vector of coefficients
                a = M\q;

                sc = a(1) + a(2)*time + a(3)*time^2 + a(4)*time^3 + a(5)*time^4 + a(6)*time^5;
            else
                %base case check to handle if incorrect movement type is set
                sc = 0;
            end
        end
    end
end

