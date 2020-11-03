classdef Camera
    % CAMERA Example Camera class for RBE 3001 Lab 5
    %   You can add your image processing in this camera class,
    %   as well as any other functions related to the camera.
    
    properties
        % Flags
        DEBUG = false;
        POSE_PLOT = false;  
        DEBUG_BALLDETECTION = false;
        
        % Image Processing Variables
        
        % Colors
        
        % Properties
        params;
        cam;
        cam_pose;
    end
    
    methods
        function self = Camera()
            % CAMERA Construct an instance of this class
            self.cam = webcam(); % Get camera object
            self.params = self.calibrate(); % Run Calibration Function
            
        end

        function shutdown(self)
            % SHUTDOWN shutdown script which clears camera variable
            clear self.cam;
        end
        
        function [pose, color] = detectBestBall(self)
            % DETECTBESTBALL Detect Balls + Report back positions + colors
            % Note: you do NOT need to use this function, this is just a recommendation
            
        end
        
        function xyz = get_object_position(self,px,py,matrix)         
            xy =[px py]; % 50 -50
            p = pointsToWorld(self.params.Intrinsics,matrix(1:3,1:3),matrix(1:3,4),xy);
            v = [p(1);p(2); 0;1];
            t0b = [ 0 1 0 50;
                    1 0 0 -100;
                    0 0 -1 11;
                    0 0 0 1;
                    ];

            m = t0b*v;
            xyz = [m(1),m(2),m(3)];
            
        end
        
        %return 1 when out of bounds 
        function bool = boundCheck(self,pixel,matrix)
            xyz = self.get_object_position(pixel(1),pixel(2),matrix);
            if(xyz(1) > 125 || xyz(1) < 0 || xyz(2) > 125 || xyz(2) < -125)
                bool = 1;
            else
                bool = 0;
            end    
        end 
              
%         
%         function pointer = RGBtoColor(self,r,g,b)
%             
%             r = double(r);
%             g = double(g);
%             b = double(b);
%             mag = norm([r,g,b]);
%             green = [97,142,28];
%             yellow = [236,195,83];
%             purple = [59,64,164];
%             pink = [185,84,134];
%             black = [50,50,50];
%             list =[norm(green),norm(yellow),norm(purple),norm(pink),norm(black)];
%             cur = 1000;
%             pointer = 0;
%             for i = 1:5
%                 if(abs(mag - list(i)) < cur)
%                     disp("subs =");
%                     disp(i);
%                     disp(mag - list(i))
%                     cur = abs(mag-list(i));
%                     pointer = i;
%                 end
%             end
%         end
%         
        
        function [r cent,color] = frame_reader(self,img,matrix)
           colors = ["Yellow","Green","Pink","Purple","Cap"];
           r = img;
           cent = []
           color =[0,0,0,0];
           for i = 1:5
               box = self.find_ball(img,colors(i));
               if(box ~= 0)
               boxes = box(1:4);
               centroid = box(5:6);
               bc= self.boundCheck(centroid,matrix)
                   if(bc~= 1)
                    r  = insertObjectAnnotation(r, 'rectangle',boxes(1,:), colors(i));
                    r = insertMarker(r,centroid(1,:),'circle');
                    cent(i,:) = centroid;
                    color(i) = i;
                   end
               end
           end
        end 
        
        function rimg = find_ball(self,img,color_name)
            
        if(color_name == "Yellow")
            [bw color] = YellowMask(img);
        elseif(color_name == "Green")
            [bw color] = GreenMask(img);
        elseif(color_name == "Pink")
            [bw color] = PinkMask(img);
        elseif(color_name == "Purple")
            [bw color] = PurpleMask(img);
        elseif(color_name == "Cap")
            [bw color] = CapMask(img);
        end
        
        [im, newOrigin] = undistortImage(img, self.params.Intrinsics, 'OutputView', 'full');
        title('Undistorted Image');
        %figure; imshow(im);
        
        imHSV = rgb2hsv(color);

        % Get the saturation channel.
        saturation = imHSV(:, :, 2);

        % Threshold the image
        t = graythresh(saturation);
        imCoin = (saturation > t);
        
        %figure; imshow(imCoin);
        title('Segmented Coins');
        
        % Find connected components.
        blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true,...
            'CentroidOutputPort', true,...
            'BoundingBoxOutputPort', true,...
            'MinimumBlobArea', 300,'MaximumBlobArea',3000,'ExcludeBorderBlobs', true);
        [areas,centroid, boxes] = step(blobAnalysis, imCoin);
        try
        % Sort connected components in descending order by area
        [~, idx] = sort(areas, 'Descend');

        % Get the two largest components.
        boxes = double(boxes(idx(1:1),:));
        centroid =double(centroid(idx(1:1),:));
        rimg = [boxes,centroid];
        
   
        catch
            disp("No ball of color")
            disp(color_name);
            rimg = 0;
        end 
    end
        
        function params = calibrate(self)
            % CALOBRATE Calibration function
            % This function will run the camera calibration, save the camera parameters,
            % and check to make sure calibration worked as expected
            % The calibrate function will ask if you are ready. To calibrate, you must press
            % any key, then the system will confirm if the calibration is successful

            % NOTE: This uses the camcalib.m file for camera calibration. If you have placed
            % your camera calibration script elsewhere, you will need to change the command below

            DEBUG = self.DEBUG;
            params = 0;
            try
                disp("Clear surface of any items, then press any key to continue");
                pause;
                camcalib; % Change this if you are using a different calibration script
                params = cameraParams;
                disp("Camera calibration complete!");
            catch exception
                getReport(exception);
                disp("No camerea calibration file found. Plese run camera calibration");
            end          
        end
        
        function pose = getCameraPose(self)
            % GETCAMERAPOSE Get transformation from camera to checkerboard frame
            % This function will get the camera position based on checkerboard.
            % You should run this function every time the camera position is changed.
            % It will calculate the extrinsics, and output to a transformation matrix.
            % Keep in mind: this transformation matrix is a transformation from pixels
            % to x-y coordinates in the checkerboard frame!

            % There are a few debugging options included as well! Simply set POSE_PLOT
            % to true to show the checkerboard frame of reference on the picture!

            % 1. Capture image from camera
            raw_img =  snapshot(self.cam);
            % 2. Undistort Image based on params
            [img, new_origin] = undistortImage(raw_img, self.params, 'OutputView', 'full');
            % 3. Detect checkerboard in the image
            [imagePoints, boardSize] = detectCheckerboardPoints(img);
            % 4. Adjust imagePoints so they are in the same frame of
            % reference as original image
            imagePoints = imagePoints + new_origin;
            % 5. Compute transformation
            [R, t] = extrinsics(imagePoints, self.params.WorldPoints, self.params);
            
            pose = [   R,    t';
                    0, 0, 0, 1];
                
            if self.POSE_PLOT
                axesPoints = worldToImage(self.params, R, t, [0 0 0; 0 50 0; 50 0 0]);
                
                x1 = [axesPoints(1, 1), axesPoints(2, 1)]';
                y1 = [axesPoints(1, 2), axesPoints(2, 2)]';
                
                img = insertText(img, [x1(2), y1(2)], 'Y Axis', 'TextColor', 'green', 'FontSize', 18);
                x2 = [axesPoints(1, 1), axesPoints(3, 1)]';
                y2 = [axesPoints(1, 2), axesPoints(3, 2)]';
                
                img = insertText(img, axesPoints(3, 1:2), 'X Axis', 'TextColor', 'red', 'FontSize', 18);
                
                imshow(img)
                title('Undistorted Image with checkerboard axes');
                
                line(x1, y1, 'linewidth', 5, 'Color', 'green');
                line(x2, y2, 'linewidth', 5, 'Color', 'red');
                
            end     
        end        
    end
end

