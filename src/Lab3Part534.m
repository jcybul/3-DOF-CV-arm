

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
  planerx = Path_Planner();
  planery = Path_Planner();
  planerz = Path_Planner();
  tedge = [2 1 2];
  t0=0;
  tic
  for k = 1:3
     tk = t0;
     tf = t0 + tedge(k);
     x_coef = planerx.quintic_traj(t0,tf,0,0,viaPts(k,1),viaPts(k+1,1),0,0);
     y_coef = planery.quintic_traj(t0,tf,0,0,viaPts(k,2),viaPts(k+1,2),0,0);
     z_coef = planerz.quintic_traj(t0,tf,0,0,viaPts(k,3),viaPts(k+1,3),0,0);
     
     tdelta = (tf-t0)/10;
     for l = 1:10
     tk = tk + tdelta;
     j_x = planerx.quintic_poly(x_coef,tk);
     j_y = planery.quintic_poly(y_coef,tk);
     j_z = planerz.quintic_poly(z_coef,tk);
     
     angles =ik3001([j_x j_y j_z]);
     
     stickModel(angles);
     pp.setSetPoints(angles);
     toc
     data = [j_x j_y j_z toc];
     writematrix(data,'3D_plot_54.txt','WriteMode','append');
     pause(tdelta);
     end
     t0 = tk;
  end
  
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

