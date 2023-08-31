function [roi_cropped] = crop_roi_image(input_image, roi_positions, is_color_image)
    %

    % Compute the bounding box from the ROI positions
    min_x = round(min(roi_positions(:, 1)));
    max_x = round(max(roi_positions(:, 1)));
    min_y = round(min(roi_positions(:, 2)));
    max_y = round(max(roi_positions(:, 2)));

    % Check if the input image is a color image or a depth image
    if is_color_image
        % Crop the color image using the bounding box
        roi_cropped = input_image(min_y:max_y, min_x:max_x, :);
    else
        % Crop the depth image using the bounding box
        roi_cropped = input_image(min_y:max_y, min_x:max_x);
    end
end
