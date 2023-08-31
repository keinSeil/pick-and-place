function rgb_image = draw_bboxes_centroids(rgb_image, props)
%DRAW_BBOXES_CENTROIDS Draws bounding boxes and centroids of objects from a binary image onto an RGB image.
%
%   rgb_image = DRAW_BBOXES_CENTROIDS(rgb_image, props) takes an RGB image (rgb_image) and
%   a struct array (props) containing the properties 'BoundingBox' and 'Centroid' for each
%   object, and returns an RGB image with the bounding boxes and centroids drawn.
%
%   Example:
%       bw_image = imread('path_to_binary_image');
%       cc = bwconncomp(bw_image);
%       props = regionprops(cc, 'BoundingBox', 'Centroid');
%       rgb_image = repmat(uint8(255 * bw_image), [1, 1, 3]);
%       rgb_image = draw_bboxes_centroids(rgb_image, props);
%       imshow(rgb_image);

% Iterate through the object properties and draw bounding boxes and centroids
for i = 1:length(props)
    % Draw the bounding box
    rgb_image = insertShape(rgb_image, 'Rectangle', props(i).BoundingBox, 'LineWidth', 3, 'Color', 'cyan');
    
    % Draw the centroid
    rgb_image = insertMarker(rgb_image, props(i).Centroid, '+', 'Color', 'red', 'Size', 20);
end
end
