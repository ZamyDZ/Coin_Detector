%% Loading the horizontal Images
hor_1 = imread('/home/zamy/cV/kamerakalibrierung/hor_1.png');

load('/home/zamy/cV/kamerakalibrierung/cameraParams.mat');
load('/home/zamy/cV/kamerakalibrierung/estimationErrors.mat');
%% Estimate Camera Parameters
numImages = 5;
files = cell(1,numImages);
for i = 1:numImages
    % Data path to the image files
    files{i} = fullfile('/','home','zamy','cV','kamerakalibrierung',sprintf('hor_%d.png',i));
end
% '/home/zamy/cV/kamerakalibrierung/hor_1.png'
[imagePoints, boardSize] = detectCheckerboardPoints(files);
squareSize = 15;
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

%% Calibrate the camera
%I = imread(files{1});
%imagesize of hor_1
imageSize = [size(hor_1,1), size(hor_1,2)];
cameraParams = estimateCameraParameters(imagePoints, worldPoints,'ImageSize', imageSize);
%% Undisort the Image
% Magnification = 15 mm
magnification = 15;
%hor_1
[d_hor_1, newOrigin] = undistortImage(hor_1, cameraParams,'OutputView','full');
%figure; imshow(d_hor_1,'InitialMagnification',magnification); title('Undisorted Image');

%% Segmentaiton of the Coins
% Convert the image to the HSV color space
im = rgb2hsv(d_hor_1);

% Get the saturation channel
saturation_1 = im(:, :, 2);

% Threshold the image
t_1 = graythresh(saturation_1);
imCoin = (saturation_1 > t_1);

%% Detect Coins
% Find connected components
blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true,...
    'CentroidOutputPort', false,...
    'BoundingBoxOutputPort', true,...
    'MinimumBlobArea', 200, 'ExcludeBorderBlobs', true);

[areas, boxes] = step(blobAnalysis, imCoin);

% Sort connected components in descending order by area
[~, idx_1] = sort(areas, 'Descend');

% Get the THREE largest components  1:3 bei idx_1
boxes = double(boxes(idx_1(1:3),:));

% Reduce the size of the image for display
scale = magnification / 100;
imDetectedCoins_1 = imresize(d_hor_1, scale);

% Insert labels for the coins
imDetectedCoins_1 = insertObjectAnnotation(imDetectedCoins_1, 'rectangle', ...
    scale * boxes, 'Münze');

%% Compute Extrinsics
% Detect the checkboard
%[imagePoints, boardSize] = detectCheckerboardPoints(im);
%imagePoints = imagePoints + newOrigin;
% Compute rotation and translation of the camera.
%[R, t] = extrinsics(imagePoints, worldPoints1, cameraParams);
disp(imagePoints(:,1:2));
disp(worldPoints);
disp(size(imagePoints(:,1:2)));
disp(size(worldPoints));
[R, t] = extrinsics(imagePoints(:,1:2), worldPoints, cameraParams);

%% Measure the first (biggest) Coin
boxes = boxes + [newOrigin, 0, 0];

% Get the top-left and the top-right corners.
box1 = double(boxes(1, :));
imagePoints1 = [box1(1:2); ...
                box1(1) + box1(3), box1(2)];
            
% Get the world coordinates of the corners            
worldPoints1 = pointsToWorld(cameraParams, R, t, imagePoints1);

% Compute the diameter of the coin in millimeters.
d = worldPoints1(2, :) - worldPoints1(1, :);
diameterInMillimeters = hypot(d(1), d(2));
fprintf('Measured diameter of one penny = %0.2f mm\n', diameterInMillimeters);

%% plot the undisorted images
subplot(2,2,1),imshow(d_hor_1,'InitialMagnification',magnification), title('Undisorted Rechts');

subplot(2,2,2),imshow(imCoin, 'InitialMagnification',magnification), title('Segmented Coins');  

subplot(2,2,3),imshow(imDetectedCoins_1), title('Detected Coins');

subplot(2,2,4),showReprojectionErrors(cameraParams),title('Error');

