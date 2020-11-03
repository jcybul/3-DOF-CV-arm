

%RBE3001 - Laboratory 3
clear
clear java
clear classes;

vid = hex2dec('16c0');% Welcome again! This MATLAB script is your starting point for Lab

pid = hex2dec('0486');

disp (vid);
disp (pid);


javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();


% Create a PacketProcessor object to send data to the nucleo firmware
pp = Robot(myHIDSimplePacketComs); 


try
  SERV_ID = 1848;            % we will be talking to server ID 1848 on
                           % the Nucleo
  SERVER_ID_READ =1910;% ID of the read packet
  DEBUG   = true;          % enables/disables debug prints

  % Instantiate a packet - the following instruction allocates 64
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.

  % The following code generates a sinusoidal trajectory to be
  % executed on joint 1 of the arm and iteratively sends the list of
  % setpoints to the Nucleo firmware. 
  
   viaPts = [38.2 39 111;
            141 72.4 175;
            119 52.2 178
            38.2  39 111];
        
  temp_size = size(viaPts);
  joint_angles = zeros(temp_size);
  
  for i =1:temp_size
      %transform all x,y,z to angles
      joint_angles(i,:) = ik3001(viaPts(i,:));
  end
       
  joint1 = Traj_Planner(joint_angles(:,1));
  joint2 = Traj_Planner(joint_angles(:,2));
  joint3 = Traj_Planner(joint_angles(:,3));
  
  
  t0 = 0;
  tedge = [2 1 2];
  for k = 1:3
     tk = t0;
     tf = t0 + tedge(k);
     j1_coef = joint1.cubic_traj(t0,tf,0,0,joint1.viaPoints(k),joint1.viaPoints(k+1));
     
     j2_coef = joint2.cubic_traj(t0,tf,0,0,joint2.viaPoints(k),joint2.viaPoints(k+1));
     
     j3_coef = joint3.cubic_traj(t0,tf,0,0,joint3.viaPoints(k),joint3.viaPoints(k+1));
     
     tdelta = (tf-t0)/10;
     for l = 1:10
     tk = tk + tdelta;
     j1_angle = joint1.cubic_poly(j1_coef,tk);
     j2_angle = joint2.cubic_poly(j2_coef,tk);
     j3_angle = joint3.cubic_poly(j3_coef,tk);
     
     stickModel([j1_angle j2_angle j3_angle]);
     %disp([j1_angle j2_angle j3_angle]);
     pp.setSetPoints([j1_angle j2_angle j3_angle]);
     pause(tdelta);
     end
     t0 = tk;
  end
  
  
%    
%   viaPts1 = [transpose(linspace(38.2,141,12));
%              transpose(linspace(141,119,12));
%              transpose(linspace(119,38.2,12))];
%       
%   viaPts2 = [transpose(linspace(39,72.4,12));
%              transpose(linspace(72.4,52.2,12));
%              transpose(linspace(52.2,39,12))];
%          
%    viaPts3 = [transpose(linspace(111, 175,12));
%               transpose(linspace(175,178,12));  
%               transpose(linspace(178,175,12))];

%   tic
%   for k = 1:3
%       
%       % Send packet to the server and get the response      
%       %pp.write sends a 15 float packet to the micro controller
%        joint_val = (ik3001(viaPts(k,:)));
%        
%        pp.setSetPoints(joint_val);
%        %pp.read reads a returned 15 float backet from the micro controller.
%        returnPacket = pp.getPositions();
%        
%        time(k) = toc;
%       toc
%       pause(1) 
%       
%   en
%   disp(time);
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

