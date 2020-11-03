
classdef Robot
    properties
        %hidDevice;
        %hidService;
        myHIDSimplePacketComs
        pol
    end
    methods
        %The is a shutdown function to clear the HID hardware connection
        function  shutdown(packet)
            %Close the device
            packet.myHIDSimplePacketComs.disconnect();
        end
        % Create a packet processor for an HID device with USB PID 0x007
        function packet = Robot(dev)
            packet.myHIDSimplePacketComs=dev;
            packet.pol = java.lang.Boolean(false);
        end
        %Perform a command cycle. This function will take in a command ID
        %and a list of 32 bit floating point numbers and pass them over the
        %HID interface to the device, it will take the response and parse
        %them back into a list of 32 bit floating point numbers as well
        function com = command(packet, idOfCommand, values)
            com= zeros(15, 1, 'single');
            try
                ds = javaArray('java.lang.Double',length(values));
                for i=1:length(values)
                    ds(i)= java.lang.Double(values(i));
                end
                % Default packet size for HID
                intid = java.lang.Integer(idOfCommand);
                %class(intid);
                %class(idOfCommand);
                %class(ds);
                packet.myHIDSimplePacketComs.writeFloats(intid,  ds);
                ret = 	packet.myHIDSimplePacketComs.readFloats(intid) ;
                for i=1:length(com)
                    com(i)= ret(i).floatValue();
                end
                %class(com)
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        function com = read(packet, idOfCommand)
            com= zeros(15, 1, 'single');
            try
                
                % Default packet size for HID
                intid = java.lang.Integer(idOfCommand);
                %class(intid);
                %class(idOfCommand);
                %class(ds);
                ret = 	packet.myHIDSimplePacketComs.readFloats(intid) ;
                for i=1:length(com)
                    com(i)= ret(i).floatValue();
                end
                %class(com)
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
          end

         function check_works_space(self,p)
                    dist1 = sqrt(p(1)^2 + p(2)^2);

            % checking for underground point
            if p(3) < 0 || p(3) > 295
                error('Error: value out of range');
            end

            % checking for out of range point
            if sqrt(dist1^2 + (p(3) - 95)^2) > 200
                error('Error: value out of range');
            end
    
                % J1 calcs
            C1 = p(2) / dist1;
            D1 = p(1) / dist1;
            theta1 = atan2d(C1, D1);
            if theta1 < -90 || theta1 > 90
                error('Error: value out of range');
            end

            
         end
 
        function  write(packet, idOfCommand, values)
            try
                ds = javaArray('java.lang.Double',length(values));
                for i=1:length(values)
                    ds(i)= java.lang.Double(values(i));
                end
                % Default packet size for HID
                intid = java.lang.Integer(idOfCommand);
                %class(intid);
                %class(idOfCommand);
                %class(ds);
                packet.myHIDSimplePacketComs.writeFloats(intid,  ds,packet.pol);
                
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        %Function that consumes a 1x3 matrix and sets the Setpoints
        %for the 3 joints
        % the val input is a 1x3 matrix for the position
        function setSetPoints(packet, val)
            
            try
                
                data = zeros(15, 1, 'single');
                data(1)= 1000;
                % add val to data indexing 
                data(3) = val(1);
                data(4) = val(2);
                data(5) = val(3);
                %use write to send the desried data 
                write(packet,1848,data);
                
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        %Function that consumes a Command ID of 1822 and reads information from the
        %microcontroller
        %returns a 1x3 matrix of servo velocities
        function vels = getVelocities(packet)
            %initialize return matrix to zeros
            vels = zeros(1,3);
            try
                %retrieve packet from microcontroller
                returnPacket = read(packet, 1822);
                %load velocity values into return matrix
                vels = returnPacket(3:3:9).';
                
            catch exception
                getReport(exception)
                disp('Exited on error, clean shutdown');
            end
        end
        
        %Function that consumes a Command ID of 1910 and reads information from the
        %microcontroller
        %returns a 1x3 matrix of servo positions
        function pos = getPositions(packet)
            %initialize return matrix to zeros
            pos = zeros(1,3);
            %retrieve packet from microcontroller
            full_packet = packet.read(1910);
            %load position values into return matrix
            pos(1) = full_packet(3);
            pos(2) = full_packet(5);
            pos(3) = full_packet(7);
        end
        
        %Function takes in a  servo betwee n0-1801 byte
        % 0 is close
        % 180 is open
        function setGripper(packet,values)
            try
             ds = javaArray('java.lang.Byte',1);
                for i=1
                    ds(i)= java.lang.Byte(values(i));
                end
                % Default packet size for HID
                intid = java.lang.Integer(1962);
                %class(intid);
                %class(idOfCommand);
                %class(ds);
                packet.myHIDSimplePacketComs.writeBytes(intid,  ds,packet.pol);
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        
        
        function J = jacob3001(~, q)
            J1 = [-100*sind(q(1))*sind(q(2)) + 100*sind(q(1))*(sind(q(2))*sind(q(3)) - cosd(q(2))*cosd(q(3)));
                  100*cosd(q(1))*sind(q(2)) - 100*cosd(q(1))*(sind(q(2))*sind(q(3)) - cosd(q(2))*cosd(q(3)));
                  0;
                  0;
                  0;
                  1];
            J2 = [100*cosd(q(1))*cosd(q(2)) - 100*cosd(q(1))*(cosd(q(2))*sind(q(3)) + sind(q(2))*cosd(q(3)));
                  100*sind(q(1))*cosd(q(2)) - 100*sind(q(1))*(cosd(q(2))*sind(q(3)) + sind(q(2))*cosd(q(3)));
                  -100*sind(q(2)) + 100*(sind(q(2))*sind(q(3)) - cosd(q(2))*cosd(q(3)));
                  0;
                  0;
                  1];
            J3 = [-100*cosd(q(1))*(sind(q(2))*cosd(q(3)) + cosd(q(2))*sind(q(3)));
                  -100*sind(q(1))*(sind(q(2))*cosd(q(3)) + cosd(q(2))*sind(q(3)));
                   100*(sind(q(2))*sind(q(3)) - cosd(q(2))*cosd(q(3)));
                  0;
                  0;
                  1];
            J = [J1 J2 J3];
        end
        
        function dpdt = fdk3001(self, q, dqdt)
            J = self.jacob3001(q)
            dpdt = J(1:3, :) * dqdt
        end
        
        function ret = singularity(self, q)
            %detects singular condition if determinant is close to 0
            J = self.jacob3001(q);
            D = det(J(1:3,:));
            %cut-off value may need fine tuning
            if D < 30000 && D > -30000
                ret = 1;
            else
                ret = 0;
            end
        end
        
        function ik_3001_numerical(self,in)
            %choosing target position
            disp("Input coordinates:");
            p = [in(1); in(2);in(3)]
            
            %threshold for convergence
            err = 0.1;
            
            %initilize q0 to an arbitrary initial position
            qi = [0 0 0];
            pi = fk3001(qi(1),qi(2),qi(3));
            xzStickModel(qi);
            
            %loops until end effector is within threshold of target
            while norm(pi(:,3) - p) > err
                
                %finding and inverting jacobian
                jac = jacob3001(self,qi);
                inv_j = pinv(jac(1:3,:));
                
                %calculating necessary change in joint angles
                deltap = (p-pi(:,3));
                deltaq = inv_j*(deltap);
                
                %updating variables and stick model
                qi = qi + deltaq;
                xzStickModel(qi);
                pi = fk3001(qi(1),qi(2),qi(3));
                if singularity(self, qi) == 1
                        error("Error: approaching singularity");
                end
                pause(0.01);
            end
            disp("Forward kinematics result:");
            disp(fk3001(qi(1),qi(2),qi(3)));
        end
        
        
        function final_traj(self,viaPts,times)
            
            s = size(viaPts);
            qi = zeros(s);
            
            
            for i = 1:s(2)
               qi(:,i) = ik3001(viaPts(:,i));
                
            end
            
            for i = 2:s(2)
                I = Interpolator("cubic",times(i));
               
                tic;
                curr_time = toc;
                while curr_time < times(i)
                    sc = I.get_sc(curr_time);

                    q_1 = (qi(1,i) - qi(1,i-1)) * sc + qi(1,i-1);
                    q_2 = (qi(2,i) - qi(2,i-1)) * sc + qi(2,i-1);
                    q_3 = (qi(3,i) - qi(3,i-1)) * sc + qi(3,i-1);


                    if self.singularity([q_1, q_2, q_3]) == 1
                        error("Error: approaching singular configuration");
                    else
                        self.setSetPoints([q_1, q_2, q_3]);
                    end

                    pause(0.1);
                    curr_time = toc;
                end 
            end
        end
        
        function ik_3001_numerical_ec(self,in)
            %threshold for convergence
            err = 0.1;
            
            %initilize q to an arbitrary initial position
            qi = self.getPositions();
            pi = fk3001(qi(1),qi(2),qi(3));
            disp(pi(:,3));
            xzStickModel(qi);
            hold on;
            for k = 1:3
                %selecting target position                
                pd = [in(1); in(2);in(3)]
                
                hold off;
                while norm(pi(:,3) - pd) > err
                    %finding and inverting jacobian
                    jac = jacob3001(self,qi);
                    inv_j = pinv(jac(1:3,:));

                    %calculating necessary change in joint angles
                    deltap = (pd-pi(:,3));
                    deltaq = inv_j*(deltap);

                    %updating variables and stick model
                    qi = qi + deltaq;
                    xzStickModel(qi);
                    pi = fk3001(qi(1),qi(2),qi(3));
                    
                    %checking for singular condition
                    if singularity(self, qi) == 1
                        error("Error: approaching singularity");
                    end
                    disp("send robot to");
                    disp(qi);
                    self.setSetPoints(qi(:,1));
                    pause(0.005);
                end
                hold on;
                disp("Target Reached!");
            end
            hold off;
        end
    end
    
end

