clc; close all
clear all
format compact
%% Starting the Camera Pipeline
% Create a context and pipeline
tic
ctx = realsense.context();
pipe = realsense.pipeline(ctx);

% Configure the pipeline
cfg = realsense.config();
cfg.enable_stream(realsense.stream.depth, 1280, 720, realsense.format.z16, 30);
cfg.enable_stream(realsense.stream.color, 1280, 720, realsense.format.rgb8, 30);

% Start the pipeline
profile = pipe.start(cfg);

% Create an align object
align_to_color = realsense.align(realsense.stream.color);

% Warm-up the camera and get frames.
for i = 1:15
    frames = pipe.wait_for_frames();
end

% Align the depth frame to the color frame
aligned_frames = align_to_color.process(frames);

% Get the aligned depth and color frames
depth_frame = aligned_frames.get_depth_frame();
color_frame = aligned_frames.get_color_frame();

% Stop the pipeline
pipe.stop();

%% Prepare the Aligned Depth and Color Frames for Processing
% Convert the depth and color frames to MATLAB images
depth_image = uint16(reshape(depth_frame.get_data(), [depth_frame.get_width(), depth_frame.get_height()]).');
color_image = uint8(permute(reshape(color_frame.get_data(), [3, color_frame.get_width(), color_frame.get_height()]), [3, 2, 1]));

% DEBUGGING & TEST 08/28/2023: For saving a new color-only image for cropping (See
% apply ROI section). Be sure to crop using the crop_roi function
color_image = imread("newMain.png");
% END OF DEBUGGING & TEST

%% Check for Existing ROI File. If Absent, Define an ROI
roi_file = 'saved_roi.mat';

if exist(roi_file, 'file')
    load(roi_file, 'roi_positions', 'mask_roi');
    use_saved_roi = true;
else
    use_saved_roi = false;
end

if ~use_saved_roi
    % Display the color image and manually select ROI
    figure, imshow(color_image);
    title('Select ROI');
    h = drawassisted;

    % Get ROI coordinates (x, y coords in 1, 2 columns)
    roi_positions = h.Position;

    % Create and save a mask that defines the region of interest (ROI)
    mask_roi = roipoly(color_image, roi_positions(:, 1), roi_positions(:, 2));

    % Save the drawn ROI to a file
    save(roi_file, 'roi_positions', 'mask_roi');
end

%% Apply ROI, then Crop the image

% DEBUGGING & TEST 08/28/2023: For saving a new color-only image for cropping (See
% Prepare the Aligned Depth and Color Frames for Processing section)
[depth_roi, color_roi] = apply_roi(roi_positions, color_image);
color_roi_cropped = crop_roi_image(color_roi, roi_positions, true);
imwrite(color_roi_cropped, "newMain.png"); % Save to new image variable
% END OF DEBUGGING & TEST

[depth_roi, color_roi] = apply_roi(roi_positions, color_image, depth_image);
depth_roi_cropped = crop_roi_image(depth_roi, roi_positions, false);
color_roi_cropped = crop_roi_image(color_roi, roi_positions, true);

imwrite(color_roi_cropped, "main2.png");

%% Segment the Image: Same Method used in "Auto_segm_Test.mlx"

% DEBUGGING: Load and auto-segment a test image
% color_image = imread("TestImages_0001_BChess.png");

% % Load the ROI and apply the mask
% load("saved_roi.mat");

% Segment the roi_cropped image using the segmentImage() function
[BW, maskedImage, images] = segmentImage(color_roi_cropped);

figure, imshow(BW)
figure, montage(images, 'Size', [2,3]);
pause(1.5)

% TESTING: Try out a different image segmentation function
[BW, maskedImage, images] = segmentImage_v2(color_roi_cropped);
figure, imshow(BW)
figure, montage(images, 'Size', [2,3]);
pause(1.5)

close all

%% Interactively Define the Origin on the Image if an Origin isn't Defined
origin_file = 'saved_origin.mat';

% Load camera intrinsics for pixel to real-world mapping
load('intrinsics.mat', 'color_intrinsics'); pause(1);

% Check for existing origin point
if exist(origin_file, 'file')
    load(origin_file, 'world_origin_coordinates', 'pixel_origin');
    use_saved_origin = true;
    disp(world_origin_coordinates)
else
    use_saved_origin = false;
end

if ~use_saved_origin
    % Display image with limits to help select the origin (change if needed)
    figure; imshow(color_roi_cropped); impixelinfo;
    xlim([1000, size(color_roi_cropped, 2)]); ylim([1, 200]); % Define zoomed area
    title('Select Origin');
    [x_origin, y_origin] = ginput(1);
    x_origin = round(x_origin);
    y_origin = round(y_origin);
    close

    % Obtain the depth value (d_origin) at the origin point from the cropped depth image
    d_origin = depth_roi_cropped(y_origin, x_origin); % Remember to swap rows/columns

    % Combine x_origin, y_origin, and d_origin into the pixel_origin variable
    pixel_origin = [x_origin, y_origin, d_origin]

    % Load camera intrinsics for pixel to real-world mapping
    % load('intrinsics.mat', 'color_intrinsics'); pause(1);

    % Convert the origin's pixel coordinates (x_origin, y_origin) and depth value (d_origin) to real-world coordinates (X_origin, Y_origin, Z_origin)
    world_origin_coordinates = pixel_to_world(pixel_origin, depth_roi_cropped, color_intrinsics)

    save(origin_file, 'world_origin_coordinates', 'pixel_origin');
end

%% Find and Draw the Bounding Boxes and Pixel Centroids of Objects
cc = BW;

% Compute and draw the bounding boxes and centroids for each object
props = regionprops(cc, 'BoundingBox', 'Centroid');
bboxes = vertcat(props.BoundingBox);
pixel_centroids = vertcat(props.Centroid);
x = draw_bboxes_centroids(color_roi_cropped, props);
% figure, imshow(x)

%% Calculate Real-World Coordinates of Object Centroids
num_objects = size(bboxes,1);
world_centroids = zeros(num_objects, 3);
spatial_centroids = zeros(num_objects, 3);

% Rotation matrix: 
Rcs = [-1 0 0; 0 1 0; 0 0 -1]; % such robotics

for i = 1:size(pixel_centroids, 1)
    % Get the depth value at the object centroid
    d_centroid = depth_roi_cropped(round(pixel_centroids(i, 2),0), round(pixel_centroids(i, 1),0));

    % Combine x, y pixel coordinates and depth value into the pixel_centroid variable
    pixel_centroid = [pixel_centroids(i, 1), pixel_centroids(i, 2), d_centroid];

    % Convert each centroid's pixel coordinates and depth value to real-world coordinates
    world_centroids(i, :) = pixel_to_world(pixel_centroid, depth_roi_cropped, color_intrinsics);
    spatial_centroids(i,:)= (world_centroids(i,:) - world_origin_coordinates) * Rcs;
end

% Display real-world coordinates of the object centroids
disp('Real-world coordinates of object centroids (X, Y, Z):')
disp(world_centroids)

% Display spatial coordinates of the object centroids
disp('Spatial coordinates of object centroids (X, Y, Z):')
disp(spatial_centroids)

%% DEBUGGING: Interactively Convert Pixel to Spatial Coordinates 
close all

% s = serialport("COM3", 115200); % Define arduino serialport object
% pause(2);

figure;
imshow(x);
title('Click on a point to get its real-world coordinates. Press "c" to continue, any other key to exit.');

while true
    [x_click, y_click, button] = ginput(1);

    if button == 'c'
        break;
    end

    x_click = round(x_click);
    y_click = round(y_click);

    % Obtain the depth value at the clicked point from the cropped depth image
    d_click = depth_roi_cropped(y_click, x_click);

    % Combine x_click, y_click, and d_click into the pixel_click variable
    pixel_click = [x_click, y_click, d_click];

    % Convert the clicked pixel coordinates and depth value to real-world coordinates
    world_click_coordinates = pixel_to_world(pixel_click, depth_roi_cropped, color_intrinsics);

    % Transform real-world coordinates of the clicked point with respect to the user-defined origin
    % Swap signs on X and Z to flip coordinates into robot's frame     
    Spatial_Coordinates = (world_click_coordinates - world_origin_coordinates) * Rcs; % much wow

    % Display transformed real-world coordinates of the clicked point
    disp('Spatial (Robot) Coordinates of the clicked point (X, Y, Z):')
    disp(Spatial_Coordinates);

% %%% ******* Interactively Send coordinate strings to Arduino ********** %%%
% % ! Comment-out to evaluate accuracy of points on the work area ! %
%     % Convert to string array for passing strings one-by-one
%     Spatial_Coordinates_str_array = string(Spatial_Coordinates);
% 
%     phrase = affirmative();
%     disp(['Hans says: ', char(phrase)]);
% 
%     for i = 1:length(Spatial_Coordinates_str_array)
%         write(s, Spatial_Coordinates_str_array(i), "string");
%         write(s, '\n', "char"); % Send a newline character to indicate the end of the string
%         pause(.5);
    end
% 
% % Read the response from the Arduino
% pause(.5);
% 
% targetString = "done";
% receivedString = "";
% 
% % Keep reading data until the target string is received
% while ~strcmp(receivedString, targetString)
%     if s.NumBytesAvailable > 0
%         receivedString = readline(s);
%         disp(receivedString);
%     end
%     pause(0.1);
% end
% 
% % Continue with the rest of the script
% disp('Sequence Complete!');

% end

% Close the serial connection and the figure
% delete(s);
% clear s;
% close

%% Automatically Send coordinate strings to Arduino
toc
clc
close all
clear scentroids  % Clears coordinates between jobs

figure, imshow(x)
title('Click to activate Mr. Hans.');
waitforbuttonpress;
% close;

s = serialport("COM3", 115200); % Define arduino serialport object
pause(2);

% Convert to string array for passing coordinate strings one-by-one
scentroids(:,:) = string(spatial_centroids(:,:));

for i = 1:num_objects

    disp(['Hans says: ', char(affirmative())]);
    disp(scentroids(i,:));

    % Write data to Hans
    for j = 1:3
        % scentroids(i,j)
        write(s, scentroids(i,j), "string");
        write(s, '\n', "char"); % Send a newline character to indicate the end of the string
        pause(.5);
    end

    % Read the response from Hans
    pause(.5);

    targetString = "done";
    receivedString = "";

    % Keep reading data until the target string is received
    while ~strcmp(receivedString, targetString)
        if s.NumBytesAvailable > 0
            receivedString = readline(s);
            % disp(receivedString);
        end
        pause(0.1);
    end
clc
disp('***Sequence complete***');
pause(.2)

end

disp("Job done")
% Close the serial connection and the figure
delete(s);
clear s;
close

%% Save parameters in a .mat file
save('Preprocessing.mat', 'color_roi_cropped', 'depth_roi_cropped', ...
     'mask_roi', 'roi_positions', 'pixel_origin', 'world_origin_coordinates')

