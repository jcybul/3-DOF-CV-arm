clear
clear java
clear classes;

vid = hex2dec('16c0');

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
    % 5x3 matrix for arbitrary setpoints
    % setPoints = [-45, 86, 33; -45, 0, 33; 0, 0, 33; 0, 0, 0; 0, 86, 33];
    
    % 3x3 matrix for triangle setpoints
    setPoints = [0, 54.7, 52.91; 0, 72.94, -34.45; 0, 11.74, 6.59];
    
    % 10 loops per position for better read accuracy
    runs = size(setPoints, 1) * 10;
    
    % 7xruns matrix for logging joint values, position values, and time
    log = zeros(7, runs);
    
    %start timer
    tic
    
    for k = 1:runs
        idx = ceil(k / 10);
        
        % send packet information
        pp.setSetPoints(setPoints(idx,:));
        pause(0.25);
        
        % read joint values from encoder
        ret = pp.getPositions();
        
        % calculate end effector position using joint values
        ee = stickModel(ret);
        
        % log information
        log(7,k) = toc;
        log(1:3,k) = ret.';
        log(4:6,k) = ee;
    end
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

pp.shutdown()