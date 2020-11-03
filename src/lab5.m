%%
% RBE 3001 Lab 5 example code!
% Developed by Alex Tacescu (https://alextac.com)
%%
clc;
clear;
clear java;
format short

%% Flags
DEBUG = false;
STICKMODEL = false;
DEBUG_CAM = false;

%% Setup
vid = hex2dec('16c0');
pid = hex2dec('0486');

if DEBUG
    disp(vid);
    disp(pid);
end

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java;
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

robot = Robot(myHIDSimplePacketComs);

cam = Camera();
cam.DEBUG = DEBUG_CAM;

%% Place Poses per color
purple_place = [165, 60, 11];
green_place = [60, 125, 11];
pink_place = [165, -40, 11];
yellow_place = [100, 135, 11];
cap_place = [165,-75,11];

%% Main Loop
try
    % Set up camera
    if cam.params == 0
        error("No camera parameters found!");
    end
    %cam.cam_pose = cam.getCameraPose();
    %img = cam.cam.snapshot()
    %imshow(img)
    matrix = cam.getCameraPose()
    m = cam.cam_pose
    %xy = [361 338]; % 100 25
    xy =[230 280] % 50 -50
     %xy =[300 390] % 125 0
      %xy =[250 350] % 100 -25
    p = pointsToWorld(cam.params.Intrinsics,matrix(1:3,1:3),matrix(1:3,4),xy);
    
    v = [p(1);p(2); 0;1];
    
    t0b = [ 0 1 0 50;
            1 0 0 -100;
            0 0 -1 0;
            0 0 0 1;
       
    ];
%%
t0b*v

%%
        disp("Place the Objects")
        pause(1);
       for i = 1:50
        img = cam.cam.snapshot();
        [r cent] = cam.frame_reader(img,matrix);
        imshow(r)
        cent
        pause(0.1)
       end
%% 
robot.setSetPoints([0 0 0]);
%% Final Script
matrix = cam.getCameraPose();
%%
% 1 Determine the posiotion of the object 
%get the centroid positions of the objects on screen
disp("put ball in frame");
pause(8);

ballnum = 1;
while ballnum >= 1
    robot.setGripper(180);
    img = cam.cam.snapshot();
    [r cent color] = cam.frame_reader(img,matrix);
    color
    imshow(r)
    pause(1);
    cent = cent(any(cent,2),:); 
    if(size(cent) ~= 0)
    % calculate the xyz coordinates from the pixel values 
    xyz = cam.get_object_position(cent(1,1),cent(1,2),matrix)
    % send the robot to tha position
    figure(2);
    robot.check_works_space(xyz);
    viaPts = [100 xyz(1)+5 xyz(1)+5;
              0 xyz(2)-11 xyz(2)-11;
              195 22 4;  
    ]
    times = [4 4 1];
    %robot.ik_3001_numerical_ec(xyz);
    robot.final_traj(viaPts,times);
    pause(1);
    robot.setGripper(0);
    robot.setSetPoints([0 0 0]);
    pause(1);
    ball_return_times = [5,5,5];
    if(color(1) == 1)
        p =[100 yellow_place(1) yellow_place(1)
            0   yellow_place(2) yellow_place(2)
            95 40 yellow_place(3)
            ];
     robot.final_traj(p,ball_return_times);
    elseif(color(2) == 2)
       p =[100 green_place(1) green_place(1) 
            0   green_place(2) green_place(2) 
            95 40 green_place(3) 
            ];
     robot.final_traj(p,ball_return_times);
    elseif(color(3) == 3)
       p =[100 pink_place(1) pink_place(1);
            0   pink_place(2) pink_place(2);
            95 40 pink_place(3)
            ];
     robot.final_traj(p,ball_return_times);
    elseif(color(4) == 4)
       p =[100 purple_place(1) purple_place(1)
            0   purple_place(2) purple_place(2)
            95 40 purple_place(3) 
            ];
     robot.final_traj(p,ball_return_times);
     elseif(color(5) == 5)
       p =[100 cap_place(1) cap_place(1);
            0   cap_place(2) cap_place(2);
            95 40 cap_place(3)
            ];
     robot.final_traj(p,ball_return_times);
    end
    robot.setGripper(180);
    %release and then go back 
    pause(1);
    robot.setSetPoints([0 0 0]);
    pause(2);
    %set the robot to home positon after grabbing the ball 
    else
    ballnum = 0;
    disp("no more objects to sort");
    end
end
robot.shutdown()
cam.shutdown()
catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end

%% Shutdown Procedure
robot.shutdown()
cam.shutdown()
