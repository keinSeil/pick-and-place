function [world_coords] = pixel_to_world(pixel_coords, depth_roi_cropped, color_intrinsics)
    % Takes a pixel-coordinate and transforms it to real-world coordinates
    % i.e., millimeters with respect to the camera's default origin.
    
    % Unpack the pixel_origin values
    x_coord = double(pixel_coords(1));
    y_coord = double(pixel_coords(2));
    d_coord = double(pixel_coords(3));
    
    % If the location has a '0' depth, make the depth at that point the
    % average depth of the robot's work area
    ddata = depth_roi_cropped;
    nonZeroIdx = ddata ~= 0; % Gets the non-zero indices of the depth data
    nonZeroVals = ddata(nonZeroIdx);
    avg = round(mean(nonZeroVals));

    if d_coord == 0     
        d_coord = avg;
    end

    % Extract the camera intrinsics
    fx = color_intrinsics.fx;
    fy = color_intrinsics.fy;
    cx = color_intrinsics.ppx;
    cy = color_intrinsics.ppy;

%     % Print out intermediate variables for debugging
%     fprintf('fx: %f, fy: %f, cx: %f, cy: %f\n', fx, fy, cx, cy);

    % Convert the origin's pixel coordinates (x_origin, y_origin) and depth value (d_origin) to real-world coordinates (X_origin, Y_origin, Z_origin)
    Z = double(d_coord);
    X = (x_coord - cx) * Z / fx;
    Y = (y_coord - cy) * Z / fy;

%     % Print out calculated world coordinates for debugging
%     fprintf('X: %f, Y: %f, Z: %f\n', X, Y, Z);

    world_coords = [X, Y, Z];
end
