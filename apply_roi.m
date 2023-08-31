function [depth_roi, color_roi] = apply_roi(roi_positions, varargin)
%APPLY_ROI Applies a region of interest (ROI) to input color and/or depth images.
%
%   [depth_roi, color_roi] = apply_roi(roi_positions, color_image) applies the
%   region of interest (ROI) specified by roi_positions to the color_image.
%   Returns color_roi and an empty depth_roi.

%   [depth_roi, color_roi] = apply_roi(roi_positions, depth_image) applies the
%   region of interest (ROI) specified by roi_positions to the depth_image.
%   Returns depth_roi and an empty color_roi.

%   [depth_roi, color_roi] = apply_roi(roi_positions, color_image, depth_image)
%   applies the region of interest (ROI) specified by roi_positions to both
%   color_image and depth_image. Returns both depth_roi and color_roi.

    % Check input arguments and assign color_image and depth_image
    if nargin == 2
        if size(varargin{1}, 3) == 3
            color_image = varargin{1};
            depth_image = [];
        else
            depth_image = varargin{1};
            color_image = [];
        end
    elseif nargin == 3
        color_image = varargin{1};
        depth_image = varargin{2};
    else
        error('Invalid number of input arguments.')
    end

    % Create mask that defines the region of interest (ROI)
    if ~isempty(color_image)
        mask_roi = roipoly(color_image, roi_positions(:, 1), roi_positions(:, 2));
    elseif ~isempty(depth_image)
        mask_roi = roipoly(depth_image, roi_positions(:, 1), roi_positions(:, 2));
    end

    % Extract the region of interest (ROI) from the depth and color images
    if ~isempty(depth_image)
        depth_roi = depth_image .* uint16(mask_roi);
    else
        depth_roi = [];
    end
    
    if ~isempty(color_image)
        color_roi = bsxfun(@times, color_image, uint8(mask_roi));
    else
        color_roi = [];
    end
end
