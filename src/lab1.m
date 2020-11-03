
% 
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
% 
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.

% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
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
  packet = zeros(15, 1, 'single');

  % The following code generates a sinusoidal trajectory to be
  % executed on joint 1 of the arm and iteratively sends the list of
  % setpoints to the Nucleo firmware. 
  
%   viaPts = [transpose(linspace(38.2,141,12)) transpose(linespace(39,72.4,12)) trasnpose(linspace(111, 175,12));
%            transpose(linspace(141,119,12)) transpose(linespace(72.4,52.2,12)) trasnpose(linspace(175,178,12));
%            transpose(linspace(119,38.2,12)) transpose(linespace(52.2,39,12)) trasnpose(linspace(178,175,12))];
%            
  viaPts = [38.2 39 111;
            141 72.4 175;
            119 52.2 178];
  s = size(viaPts(:,1));
  traj = Traj_Planner(viaPts);
  time = zeros(s);
  %%
  %%tic
  pp.setSetPoints([0 0 0]);
  pause(3) 
  disp("close");
  pp.setGripper(0)
  pause(3)
  disp("open")
  pp.setGripper(180);
  %%
  
  %%
  
  
  
%   for k = 1:s
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
%       if DEBUG
%           disp('Sent Packet:');
%           disp(packet);
%           disp('Received Packet:');
%           disp(returnPacket);
%       end
%       
%       toc
%       pause(1) 
%       
%   end
  disp(time);
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
