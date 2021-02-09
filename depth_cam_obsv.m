clear
clc
imaqreset

% **THE WHILE LOOP**
% create the video writer with 1 fps
% writerObj = VideoWriter('stiffDemo1.avi');
% writerObj.FrameRate = 15;
% open the video writer
% open(writerObj);

% Make Pipeline object to manage streaming
pipe = realsense.pipeline();
colorizer = realsense.colorizer(0);
% 0-Jet//1-Classic//2-w2b//3-b2w//4-Bio
% 5-Cold//6-Warm//7-Quantized//8-Pattern

% Start streaming on an arbitrary camera with default settings
profile = pipe.start();
align_to = realsense.stream.color;
alignedFs = realsense.align(align_to);

% Get streaming device's name
dev = profile.get_device();
name = dev.get_info(realsense.camera_info.name);

% Track points in video using Kanade-Lucas-Tomasi (KLT) algorithm
tracker = vision.PointTracker('NumPyramidLevels',4,'MaxBidirectionalError',2,'BlockSize',[11 11]);
first = true;

% Define images to process
imageFileNames = {'files for calibration'};
% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);

% Generate world coordinates of the corners of the squares
squareSize = 12;  % in units of 'millimeters'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 3, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

orientation = eye(3)*rotx(.1);
location = [0 0 -220];
[rotationMatrix,translationVector] = cameraPoseToExtrinsics(orientation,location);
store_d3 = [];
storeDistalVelocity = [];
storeDistalVelocity3d = [];
timestep = [];
tipOrientation = [];
i = 1;

while i
    drawnow
    tic;
    fs = pipe.wait_for_frames();
    
    % Select depth frame
    aligned_frames = alignedFs.process(fs);
    depth = aligned_frames.get_depth_frame();
    
    % get depth image parameters
    depthSensor = dev.first('depth_sensor');
    depthScale = depthSensor.get_depth_scale();
    depthWidth = depth.get_width();
    depthHeight = depth.get_height();
    
    % retrieve UINT16 depth vector
    depthVector = depth.get_data();
    
    % reshape vector, and scale to depth in (mm)
    depthMap = double(transpose(reshape(depthVector, [depthWidth,depthHeight]))) .* depthScale * 1000;
    depthMap = medfilt2(depthMap,[9 9]);%%
    
    %Select rgb frame
    color = fs.get_color_frame();
    colordata = color.get_data();
    
    rgb = permute(reshape(colordata',[3,color.get_width(),color.get_height()]),[3 2 1]);
    
    % rgb-image processing
    r = rgb(:,:,1);
    g = rgb(:,:,2);
    b = rgb(:,:,3);
    
    b = imsubtract(b,rgb2gray(rgb));
    r = imsubtract(r,rgb2gray(rgb));
    
    bw = b > 20;
    bw = medfilt2(bw,[9 9]);
    rw = r > 20;
    %rw = medfilt2(rw,[9 9]);
    
    % Removes all objects containing fewer than 20 pixels
    bw = bwareaopen(bw, 20);
    rw = bwareaopen(rw, 20);
    
    % Pixels are connected only to their immediate 8 neighbors in each frame
    bw = bwconncomp(bw, 8);
    rw = bwconncomp(rw, 8);
    
    stats = regionprops(bw(:,:,1),'Centroid');
    stats_red = regionprops(rw(:,:,1),'Centroid');
    
    if length(stats) >= 3 && length(stats_red) >= 1
        
        % Original detection of the centroid corrdinates
        cent1 = stats(1).Centroid;
        cent2 = stats(2).Centroid;
        cent_marker = stats(3).Centroid;
        points= [cent1;cent2;cent_marker];
        cent_red = stats_red(1).Centroid;
        
        % Sorting Centroids w.r.t. Z-axis
        points = sortrows(points,2);
        sort_cent1 = uint16([points(1,1),points(1,2)]);
        sort_cent2 = uint16([points(2,1),points(2,2)]);
        sort_marker = uint16([points(3,1),points(3,2)]);
        
        undistortedPointsCentBase = undistortPoints(cent_red,cameraParams);
        undistortedPointsCentMid = undistortPoints(sort_cent1,cameraParams);
        undistortedPointsCentDist = undistortPoints(sort_cent2,cameraParams);
        undistortedPointsCentMarker = undistortPoints(sort_marker,cameraParams);
   
        worldMarker = pointsToWorld(cameraParams,rotationMatrix,translationVector,...
            [undistortedPointsCentMarker(1,1), undistortedPointsCentMarker(1,2)]);
        worldDist = pointsToWorld(cameraParams,rotationMatrix,translationVector,...
            [undistortedPointsCentDist(1,1), undistortedPointsCentDist(1,2)]);
        worldMid = pointsToWorld(cameraParams,rotationMatrix,translationVector,...
            [undistortedPointsCentMid(1,1), undistortedPointsCentMid(1,2)]);
        worldBase = pointsToWorld(cameraParams,rotationMatrix,translationVector,...
            [undistortedPointsCentBase(1,1), undistortedPointsCentBase(1,2)]);

        if first == true
            initialize(tracker,points,rgb);
            first = false;
        end
        
        [points,validity,scores] = tracker(rgb);

        colorizedDepth = colorizer.colorize(depth);
        data = colorizedDepth.get_data();
        colorizedDepthImage = permute(reshape(data',[3,colorizedDepth.get_width(),colorizedDepth.get_height()]),[3 2 1]);
%         imshow(colorizedDepthImage);
%         imshow(depthMap);
%         title('Depth Map (mm)');
        
        markerPointDepth = depthMap(sort_marker(1,2), sort_marker(1,1));
        distalPointDepth = depthMap(sort_cent2(1,2), sort_cent2(1,1));
        midPointDepth = depthMap(sort_cent1(1,2), sort_cent1(1,1));
        basePointDepth = depthMap(uint16(cent_red(1,2)), uint16(cent_red(1,1)));
        
        % test distal coord.% test distal coord.% test distal coord.% test distal coord.% test distal coord.
        marker3d = round([worldMarker(1,1) + 81.3440, 220-markerPointDepth, worldMarker(1,2) + 12.8 + 150],3);
        distal3d = round([worldDist(1,1) + 81.3440, 220-distalPointDepth, worldDist(1,2) + 12.8 + 150],3);
        mid3d = round([worldMid(1,1) + 82.3440, 220-midPointDepth, worldMid(1,2) + 12.8 + 150 - 15],3);
        base3d = round([worldBase(1,1) + 82.3440, 220-basePointDepth, worldBase(1,2) + 80.73 + 50],3);
        % test distal coord.% test distal coord.% test distal coord.% test distal coord.% test distal coord.
        
        if isempty(tipOrientation)
            tipOrientationPrev = tipOrientation;
        end
        
        tipOrientation = round(rad2deg(atan2(norm(distal3d(1,1:2)-marker3d(1,1:2)),...
            norm(distal3d(1,3)-marker3d(1,3)))),2);
        
        if ((tipOrientationPrev -  tipOrientation) >= 5)
            tipOrientation = tipOrientationPrev;
        end
        
        if tipOrientation > 50
            tipOrientation = 90 -  tipOrientation;
        end

        elaspsedTime = toc;
%         
%         store_d3 = [store_d3;distal3d];
%         
%         return the distal tip velocity
%         if size(store_d3,1) > 1
%             distalVelocity = (store_d3(end,:)-store_d3(end-1,:))/elaspsedTime;
%             distalVelocity3d = (norm(store_d3(end,:)-store_d3(end-1,:)))/elaspsedTime;
%             %disp(distalVelocity);
%             storeDistalVelocity = [storeDistalVelocity; distalVelocity, elaspsedTime];
%             storeDistalVelocity3d = [storeDistalVelocity3d; distalVelocity3d, elaspsedTime];
%             
%             figure: tip velocity
%             timestep = [];
%             new = smoothdata(storeDistalVelocity(:,1:3),'movmedian');
%             for i = 1:length(storeDistalVelocity)
%                 timestep = [timestep;sum(storeDistalVelocity(1:i,4))];
%             end
%             figure(1);
%             hold on;
%             plot(timestep,storeDistalVelocity(:,1),'r-');
%             plot(timestep,storeDistalVelocity(:,2),'g-');
%             plot(timestep,storeDistalVelocity(:,3),'b-');
%             
%             % plot the 3d-velocity in real time
%             if i > 2
%                 figure(3);
%                 timestep = [timestep;sum(storeDistalVelocity3d(i,2))];
%                 hold on;
%                 line([sum(timestep(1:end-1)),sum(timestep)],[storeDistalVelocity3d(i-1,1),storeDistalVelocity3d(i,1)],...
%                     'linewidth',1,'color','r');
%                 F(i) = getframe(gcf);
%                 hold on
%                 drawnow
%                 
%             end
%             i = i + 1;
%        end

        figure(1)
        set(gcf,'color','w');
        subplot(1,2,1)
        imshow(rgb);
        hold on;
        plot(cent_red(1,1),cent_red(1,2),'g.');
        plot(sort_cent1(1,1),sort_cent1(1,2),'y.');
        plot(sort_cent2(1,1),sort_cent2(1,2),'y.'); 
        plot(sort_marker(1,1),sort_marker(1,2),'y.');
        hold off;
        set(gca, 'YDir','reverse');
        
        title({['tipPosition: x = ',num2str(distal3d(1,1)),' y = ',num2str(distal3d(1,2)),...
            ' z = ',num2str(distal3d(1,3))];...
            [' pointingOffset = ',num2str(tipOrientation),' deg']});

        subplot(1,2,2)
        set(gcf,'color','w');
        hold on;
        plot3(marker3d(1,1),marker3d(1,2),marker3d(1,3),'bs','MarkerFaceColor','b',...
            'MarkerEdgeColor','b','MarkerSize',2);
        plot3(distal3d(1,1),distal3d(1,2),distal3d(1,3),'bo','MarkerFaceColor','b',...
            'MarkerEdgeColor','b','MarkerSize',2);
        plot3(mid3d(1,1),mid3d(1,2),mid3d(1,3),'ms','MarkerFaceColor','b',...
            'MarkerEdgeColor','b','MarkerSize',2);
        plot3(base3d(1,1),base3d(1,2),base3d(1,3),'g^','MarkerFaceColor','g',...
            'MarkerEdgeColor','b','MarkerSize',2);
        hold off
        axis equal
        box on
        grid on
        view(200,30);
        axis([-100 100 -100 100 0 200]);
        camproj('perspective');

    
        i = i + 1;
        toc
    
%         FF(i) = getframe(gcf);
%         writeVideo(writerObj, FF(end).cdata);
    end
end

% figure(2)
% plot3(store_d3(:,1),store_d3(:,2),store_d3(:,3),'b.');
% box on;
% grid on;
% axis square
% axis([-100 100 -100 100 0 200]);
% camproj('perspective');

% Stop streaming
pipe.stop();
