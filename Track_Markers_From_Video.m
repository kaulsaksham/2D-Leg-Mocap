% This program tracks four checkerboard markers in a video of a leg moving
% underwater using vision.PointTracker and plots the results. The video has already been lens-corrected 
% using another MATLAB program.

%% Track Points from Video 
clc;
clear;
close all;

% load GoPro Video + Video Player
v = VideoReader('Apr17Clip.mp4');
videoPlayer = vision.VideoPlayer('Position', [0,0,500,500]);

numOR = 4; % four tracking regions -- should stay 4 for this program

% read first frame, initialize object regions
objectFrame = readFrame(v);
objectRegions = zeros(numOR, 4);

% use mouse to select tracking regions by hand
% Region 1: Upper Thigh Marker
% Region 2: Lower Thigh Marker
% Region 3: Upper Shank Marker
% Region 4: Lower Shank Marker

hold on;
for i = 1:numOR
    imshow(objectFrame);
    title("Select Region " + num2str(i) + " to Track:");
    objectRegions(i,:) = round(getPosition(imrect));
end
hold off;
close;

% display selected regions
imshow(objectFrame);

% center of region is defined as point to track throughout video
regionCenters = zeros(numOR, 2);
regionCenters(:,1) = objectRegions(:,1) + objectRegions(:,3)./2;
regionCenters(:,2) = objectRegions(:,2) + objectRegions(:,4)./2;

objectImage = objectFrame;
for i = 1:numOR
    objectImage = insertShape(objectImage, 'Rectangle', objectRegions(i,:), ...
        'Color', 'red', LineWidth=4);
    objectImage = insertShape(objectImage, 'Circle', [regionCenters(i,:), ...
        5], 'Color', 'green', LineWidth=4);
end

figure;
imshow(objectImage);
title('Selected Tracking Regions');

% initialize interest points and weights of interest points for each
% tracking region
interestPointsArr = cell(1, numOR);
weightsArr = cell(1, numOR);
weightedCenters = cell(1, numOR);
pointImage = objectFrame;

% detect "good" features to track in video
for i = 1:numOR
    interestPointsArr{i} = detectMinEigenFeatures(im2gray(objectFrame), ...
        'ROI', objectRegions(i,:));
    weightsArr{i} = findWeights(interestPointsArr{i}.Location, ...
        regionCenters(i,:));
    weightedCenters{i} = findWeightedCenter(interestPointsArr{i}.Location, ...
        weightsArr{i});
    pointImage = insertMarker(pointImage, interestPointsArr{i}.Location, ...
        '+', 'Color', 'red');
end

figure;
imshow(pointImage);
title('Detected Interest Points in Each Region');

% initialize trackers for each tracking region
trackerArr = cell(1, numOR);

for i = 1:numOR
    tracker = vision.PointTracker('MaxBidirectionalError', 1);
    initialize(tracker, interestPointsArr{i}.Location, objectFrame);
    trackerArr{i} = tracker;
end

hold on;

% initialize matrix, each column contains center point tracked in each
% tracking region
pointsOverTime = cell(floor(v.Duration * v.FrameRate), numOR);
numFrames = 0;

% loop through video frames
while hasFrame(v)
    frame = readFrame(v); % read current frame
    out = frame;
    for i = 1:numOR
        tracker = trackerArr{i};
        weights = weightsArr{i};

        [points, validity] = tracker.step(frame); % track points in frame

        validPoints = points(validity,:); % filter out invalid points
        weightedCenter = findWeightedCenter(validPoints, ...
            weights(validity,:));
        pointsOverTime{numFrames+1,i} = weightedCenter;

        out = insertMarker(out, validPoints, '+', 'Color', 'red'); % draw visuals
        out = insertMarker(out, weightedCenter, 'o', 'Color', 'green');
    end
    videoPlayer(out);
    numFrames = numFrames + 1;
end

% close Video Player
release(videoPlayer);

%%

% convert pixels to mm -- requires measuring number of pixels in a known
% distance in tracking plane (use a photo of the checkerboard!)
pixelsPerMM = 57.1/25.4;

% convert matrix into four sets of coordinates, one for each region
coords1 = cell2mat(pointsOverTime(:,1));
coords1(:,2) = size(objectFrame, 1) - coords1(:,2);

coords2 = cell2mat(pointsOverTime(:,2));
coords2(:,2) = size(objectFrame, 1) - coords2(:,2);

coords3 = cell2mat(pointsOverTime(:,3));
coords3(:,2) = size(objectFrame, 1) - coords3(:,2);

coords4 = cell2mat(pointsOverTime(:,4));
coords4(:,2) = size(objectFrame,1) - coords4(:,2);
coords4_mm = coords4 ./ pixelsPerMM;
coords4avg_mm = mean(coords4_mm,1);

figure;
plot(coords4_mm(:,1), coords4_mm(:,2));

xlabel('X Pos. (mm)');
ylabel('Y Pos. (mm)');
title('Shin Trajectory Measured by GoPro');
axis equal;

%%
vecThigh = coords2 - coords1;
vecShank = coords4 - coords3;
shankAngle = rad2deg(atan2(vecShank(:,2), vecShank(:,1))) + 90;
thighAngle = rad2deg(atan2(vecThigh(:,2), vecThigh(:,1))) + 90;

figure;
plot(linspace(0,floor(numFrames/60),numFrames), shankAngle(1:numFrames));

title('Shank Angle Determined by GoPro')
xlabel('Time (sec)');
ylabel('Angle (deg)');

figure;
plot(linspace(0,floor(numFrames/60),numFrames), thighAngle(1:numFrames));

title('Thigh Angle Determined by GoPro')
xlabel('Time (sec)');
ylabel('Angle (deg)');

%%

% inverse distance weighting function
function w = findWeights(points, centerPoint)
    p = 2.5; % smoothing factor of weights, lower is smoother
    pointsX = points(:,1);
    pointsY = points(:,2);
    cenX = centerPoint(1);
    cenY = centerPoint(2);

    distances = sqrt((pointsX - cenX).^2 + (pointsY - cenY).^2);
    distances(distances == 0) = eps;

    rawWeights = 1 ./ (distances.^p);

    if sum(rawWeights) == 0
        w = zeros(size(rawWeights)); % fallback if no markers visible
    else
        w = rawWeights / sum(rawWeights);
    end
end

function wCen = findWeightedCenter(points, weights)
    wCen = mean(points, "Weights", weights);
end