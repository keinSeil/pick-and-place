% Find intrinsics of the depth camera and rgb camera. Do this anytime
% camera parameters are changed.
clc
% Create a context and pipeline
ctx = realsense.context();
pipe = realsense.pipeline(ctx);

% Configure the pipeline
cfg = realsense.config();
cfg.enable_stream(realsense.stream.depth, 1280, 720, realsense.format.z16, 30);
cfg.enable_stream(realsense.stream.color, 1280, 720, realsense.format.rgb8, 30);

% Start the pipeline
profile = pipe.start(cfg);

% Get the intrinsic properties of the depth and color cameras
depth_profile = profile.get_stream(realsense.stream.depth).as('video_stream_profile');
color_profile = profile.get_stream(realsense.stream.color).as('video_stream_profile');

depth_intrinsics = depth_profile.get_intrinsics();
color_intrinsics = color_profile.get_intrinsics();

fprintf('Depth camera intrinsics:\n');
fprintf('Focal length: fx=%f, fy=%f\n', depth_intrinsics.fx, depth_intrinsics.fy);
fprintf('Optical center: cx=%f, cy=%f\n', depth_intrinsics.ppx, depth_intrinsics.ppy);
fprintf('Distortion coefficients: [%f, %f, %f, %f, %f]\n', depth_intrinsics.coeffs);

fprintf('Color camera intrinsics:\n');
fprintf('Focal length: fx=%f, fy=%f\n', color_intrinsics.fx, color_intrinsics.fy);
fprintf('Optical center: cx=%f, cy=%f\n', color_intrinsics.ppx, color_intrinsics.ppy);
fprintf('Distortion coefficients: [%f, %f, %f, %f, %f]\n', color_intrinsics.coeffs);

% Stop streaming
pipe.stop();

% Save depth and color intrinsics to a .mat file called intrinsics
% To access in another program, add the saved file's location to the path,
% Then call that value via: depth_intrinsics.fx, for example.
save intrinsics depth_intrinsics color_intrinsics


