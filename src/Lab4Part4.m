
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

home = [0 0 0];
try
    
    pp.ik_3001_numerical();
    
    %extra credit
    while norm(pp.getPositions() - home) > 2
        pp.setSetPoints([0,0,0]);
        pause(0.2);
    end

    disp("Ready!");
    % function to make the plot and choose point to go to
    pp.ik_3001_numerical_ec();
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end
    
% Clear up memory upon termination
pp.shutdown()


