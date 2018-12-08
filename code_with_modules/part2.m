%%

clear;
close all;

%%
load('cameraparametersAsus.mat');

im1=imread('duascamaras/Cam1/rgb_image1_4.png');
im2=imread('duascamaras/Cam2/rgb_image2_8.png');
load('duascamaras/Cam1/depth1_4.mat');
imdepth1 = double(depth_array)/1000;
load('duascamaras/Cam2/depth2_4.mat');
imdepth2 = double(depth_array)/1000;

[f1,d1]=vl_sift(single(rgb2gray(im1)));
[f2,d2]=vl_sift(single(rgb2gray(im2)));

[matches score] = vl_ubcmatch(d1,d2,1.5); 

% figure(1);
% subplot(1,2,1);
% imshow(uint8(im1));
% hold on;
% plot(f1(1,matches(1,:)),f1(2,matches(1,:)),'b*');
% 
% subplot(1,2,2);
% imshow(uint8(im2));
% hold on;
% plot(f2(1,matches(2,:)),f2(2,matches(2,:)),'r*');
% 
% [row1,ia1, ~] = unique(matches(1,:));
% row2 = matches(2,:);
% row2 = row2(ia1);
% score = score(ia1);
% [unique_row2, ia2, ~] = unique(row2);
% unique_row1 = row1(ia2);
% score = score(ia2);
% unique_matches = [unique_row1; unique_row2];
%uniqueMatches= transpose(unique(transpose(matches), 'rows'))

picture1data = [transpose(f1(1,matches(1,:))), transpose(f1(2,matches(1,:)))];
picture2data = [transpose(f2(1,matches(2,:))), transpose(f2(2,matches(2,:)))];

picture1Z = zeros(length(picture1data(:, 1)), 1);
picture2Z = zeros(length(picture1data(:, 1)), 1);
for i = 1:length(picture1data(:, 1))
    picture1Z(i) = imdepth1(floor(picture1data(i, 2)), floor(picture1data(i, 1)));
    picture2Z(i) = imdepth2(floor(picture2data(i, 2)), floor(picture2data(i, 1)));
end

picture1data = [floor(picture1data), picture1Z];
picture2data = [floor(picture2data), picture2Z];

xyz_cam1 = cam_params.Kdepth\[picture1data(:, 3)'.*picture1data(:, 1)'; picture1data(:, 3)'.*picture1data(:, 2)'; picture1data(:, 3)'];
xyz_cam1 = xyz_cam1';

xyz_cam2 = cam_params.Kdepth\[picture2data(:, 3)'.*picture2data(:, 1)'; picture2data(:, 3)'.*picture2data(:, 2)'; picture2data(:, 3)'];
xyz_cam2 = xyz_cam2';

figure(2);
subplot(1,2,1);
imshow(uint8(im1));
hold on;
plot(picture1data(:, 1), picture1data(:, 2),'b*');

subplot(1,2,2);
imshow(uint8(im2));
hold on;
plot(picture2data(:, 1), picture2data(:, 2),'r*');


[ cam2toW.R, cam2toW.T, inliers ] = ransac_procrustes( xyz_cam1, xyz_cam2, 0.005, 500 );
cam1toW.R = eye(3);
cam1toW.T = zeros(3,1);
