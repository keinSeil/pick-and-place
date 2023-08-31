clear all; clc
format compact
% Only for IR Cameras!

% Create a context and pipeline
ctx = realsense.context();
pipe = realsense.pipeline(ctx);

% Configure the pipeline
cfg = realsense.config();
cfg.enable_stream(realsense.stream.infrared, 1, 1280, 720, realsense.format.y8, 30); % Left IR camera
cfg.enable_stream(realsense.stream.infrared, 2, 1280, 720, realsense.format.y8, 30); % Right IR camera

% Start the pipeline
profile = pipe.start(cfg);

% Warm-up the camera and get frames
for i = 1:50
    frames = pipe.wait_for_frames();
end

% Image counter
image_counter = 0;

% Folder where the image pairs will be saved
output_folder_right = 'ir_trial_right';
output_folder_left = 'ir_trial_left';

% Create a figure for the live view
figure;
title('Live View');
h_left_image = subplot(1, 2, 1);
title('Left IR Image');
imshow(zeros(720, 1280, 'uint8'));

h_right_image = subplot(1, 2, 2);
title('Right IR Image');
imshow(zeros(720, 1280, 'uint8'));

disp('Press Enter to capture an image pair or type "quit" to exit.');

while true
    % Get user input
    user_input = input('', 's');
    
    if strcmp(user_input, 'quit')
        break;
    end
    
    % Capture frames
    frames = pipe.wait_for_frames();
    
    % Get the left and right IR frames
    left_ir_frame = frames.get_infrared_frame(1);
    right_ir_frame = frames.get_infrared_frame(2);
    
    % Convert the left and right IR frames to MATLAB images
    left_ir_image = uint8(reshape(left_ir_frame.get_data(), [left_ir_frame.get_width(), left_ir_frame.get_height()]).');
    right_ir_image = uint8(reshape(right_ir_frame.get_data(), [right_ir_frame.get_width(), right_ir_frame.get_height()]).');
    
    % Update the live view with the latest IR images
    set(get(h_left_image, 'children'), 'CData', left_ir_image);
    set(get(h_right_image, 'children'), 'CData', right_ir_image);
    drawnow;
    
    % Increment the image counter
    image_counter = image_counter + 1;

    % Save the images with unique names
    imwrite(left_ir_image, fullfile(output_folder_left, sprintf('left_ir_image_%04d.png', image_counter)));
    imwrite(right_ir_image, fullfile(output_folder_right, sprintf('right_ir_image_%04d.png', image_counter)));
    
    fprintf('Image pair %d saved.\n', image_counter);
    disp('Press Enter to capture another image pair or type "quit" to exit.');
end

% Stop the pipeline
pipe.stop();
