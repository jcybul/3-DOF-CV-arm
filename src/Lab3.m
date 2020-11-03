

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
  
  viaPts = [38.2 39 111;
            141 72.4 175;
            119 52.2 178
            38.2  39 111];
        
  planer = Path_Planner();
  conunter = 0;
  tic
  for k = 1:3
        trajp = planer.linear_traj(viaPts(k,:),viaPts(k+1,:));
        for  i = 1:12
            p = [trajp(1,i) trajp(2,i) trajp(3,i)];
            stickModel(ik3001(p));
            pp.setSetPoints(ik3001(p));
            pause(0.1);
            data = [trajp(1,i) trajp(2,i) trajp(3,i) toc];
            disp(data);
            writematrix(data,'3D_plot.txt','WriteMode','append');
            toc
        end
  end
  
  


catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

