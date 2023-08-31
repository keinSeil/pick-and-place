clear all; clc
format compact

load('Preprocessing.mat');
load('saved_roi.mat');

% Create a context and pipeline
ctx = realsense.context();
pipe = realsense.pipeline(ctx);

% Configure the pipeline
cfg = realsense.config();
cfg.enable_stream(realsense.stream.color,...
                  1280,720, realsense.format.rgb8, 30); % Change if resolution changes

% Start the pipeline
profile = pipe.start(cfg);

% Warm-up the camera and get frames
for i = 1:30
    frames = pipe.wait_for_frames();
end

% Image counter
image_counter = 80;

% Folder where the image will be saved
% output_folder = 'CNN_Dataset\Robot';
output_folder = 'TestImages';

% Create a figure for the live view
figure;
title('Live View');

% Set initial size to match acquired image
h_rgb_image = imshow(zeros(720,1280, 3, 'uint8')); % Change if you change the resolution
title('Acquired Color Image');

disp('Press Enter to capture an image or type "c" to exit.');

while true
    % Get user input
    user_input = input('', 's');
    
    if strcmp(user_input, 'c')
        break;
    end
    
    % Capture frames
    frames = pipe.wait_for_frames();
    
    % Get the RGB frame
    rgb_frame = frames.get_color_frame();
    
    % Convert the RGB frame to a MATLAB image
    rgb_frame = uint8(reshape(rgb_frame.get_data(), [3, rgb_frame.get_width(), rgb_frame.get_height()]));
    rgb_frame = permute(rgb_frame, [3, 2, 1]);

    % Update the live view with the latest RGB image
    set(h_rgb_image, 'CData', rgb_frame); % Update 'CData' property of h_rgb_image directly
    drawnow;

    % Increment the image counter
    image_counter = image_counter + 1;

    % Save the image with a unique name
    imwrite(rgb_frame, fullfile(output_folder, sprintf(['%s_%04d_Rook' ...
        '.png'], output_folder, image_counter)));

    fprintf('%s_%d saved.\n', output_folder, image_counter);
    disp('Press Enter to capture another image or type "c" to exit.');
end

% Stop the pipeline
pipe.stop();
