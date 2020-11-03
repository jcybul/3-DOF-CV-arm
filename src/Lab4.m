
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
    %sign-off 1
    q = [0, 0, -90];
    J = pp.jacob3001(q)
    
     %sign-off 2
     viaPts = [38.2 39 111;
               141 72.4 175;
               119 52.2 178
               38.2  39 111];
     
     %converting positions to Joint Space values
     viaPts_JS = [ik3001(viaPts(1,:)).';
                  ik3001(viaPts(2,:)).';
                  ik3001(viaPts(3,:)).';
                  ik3001(viaPts(4,:)).'];
     
     %matrix of movement durations
     movement_durations = [3,3,3];
     
     log = zeros(35,6);
              
     it = 1;
     for k = 1:3
         
         I_1 = Interpolator("quintic", movement_durations(k));
         I_2 = Interpolator("quintic", movement_durations(k));
         I_3 = Interpolator("quintic", movement_durations(k));
         
         tic;
         
         curr_time = toc;
         while curr_time < movement_durations(k)
             
             sc_1 = I_1.get_sc(curr_time);
             sc_2 = I_2.get_sc(curr_time);
             sc_3 = I_3.get_sc(curr_time);
             
             q_1 = (viaPts_JS(k+1,1) - viaPts_JS(k,1)) * sc_1 + viaPts_JS(k,1);
             q_2 = (viaPts_JS(k+1,2) - viaPts_JS(k,2)) * sc_2 + viaPts_JS(k,2);
             q_3 = (viaPts_JS(k+1,3) - viaPts_JS(k,3)) * sc_3 + viaPts_JS(k,3);
             
             pp.setSetPoints([q_1, q_2, q_3]);
             
             q = pp.getPositions();
             dqdt = pp.getVelocities();
             
             dpdt = pp.fdk3001(q, dqdt.');
             
             stickModel(q, dpdt);
             
             log(it,1:3) = q;
             log(it,4:6) = dqdt;
             
             pause(0.1);
             curr_time = toc;
             it = it + 1;
         end
 
     end
    
    %sign-off 3
    
    sngPts = [0,0,-90];
    
    initPos = pp.getPositions();
    
    I = Interpolator("cubic",5);
    
    tic;
    curr_time = toc;
    while curr_time < 5
        sc = I.get_sc(curr_time);
            
        q_1 = (sngPts(1) - initPos(1)) * sc + initPos(1);
        q_2 = (sngPts(2) - initPos(2)) * sc + initPos(2);
        q_3 = (sngPts(3) - initPos(3)) * sc + initPos(3);

        
        if pp.singularity([q_1, q_2, q_3]) == 1
            error("Error: approaching singular configuration");
        else
            pp.setSetPoints([q_1, q_2, q_3]);
        end
        
        pause(0.1);
        curr_time = toc;
    end
        
    
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()
