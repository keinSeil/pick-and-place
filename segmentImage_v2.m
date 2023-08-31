function [BW,maskedImage, images] = segmentImage_v2(RGB)
%segmentImage Segment image using auto-generated code from Image Segmenter app
%  [BW,MASKEDIMAGE] = segmentImage(RGB) segments image RGB using
%  auto-generated code from the Image Segmenter app. The final segmentation
%  is returned in BW, and a masked image is returned in MASKEDIMAGE.

% Auto-generated by imageSegmenter app on 29-Apr-2023
%----------------------------------------------------

%%
%%% For Testing Only!!! %%%
close all; clear all
RGB = imread("newMain.png");
figure, imshow(RGB)
%%% For Testing Only!!! %%%

tic
% Threshold image with global threshold
BW = imbinarize(im2gray(RGB));
m1 = BW;
figure, imshow(BW)

% Invert mask
BW = imcomplement(BW);
m2 = BW;
figure, imshow(BW)

% Canny Filter
BW = edge(BW,"canny",.5);
m3 = BW;
figure, imshow(BW)

% % DEBUGGING: Convert the perimeter values to an array
% perimeters = regionprops(BW, 'Perimeter');
% perimeterValues = [perimeters.Perimeter] % View to check it out

% Filter out high and low perimeters (measured in pixels)
BW = bwpropfilt(BW,'Perimeter',[180, 1000]);
m4 = BW;
figure, imshow(BW)


% Fill holes
BW = imfill(BW,"holes");
m5 = BW;
figure, imshow(BW)

% Remove small objects
BW = bwareaopen(BW, 500);
m6 = BW;
figure, imshow(BW)
toc

images = {m1, m2, m3, m4, m5, m6}; % m1, m2... etc. All for debugging
% figure, montage(images, 'Size', [2,3]);



% Create masked image.
maskedImage = RGB;
maskedImage(repmat(~BW,[1 1 3])) = 0;
end

