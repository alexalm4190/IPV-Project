%%

clear;
close all;

%%

%% load images to variables

load('cameraparametersAsus.mat');
d=dir('filinha\*.jpg');
dd=dir('filinha\*.mat');
imgs=zeros(480,640, 3, length(d));
imgsd=zeros(480,640,length(d));
P = zeros(3, 480*640, length(d));

for i=1:length(d),
    imgs(:,:, i)=rgb2gray(imread(strcat('filinha\', d(i).name)));
    load(strcat('filinha\', dd(i).name));
    imgsd(:,:,i)=double(depth_array)/1000;
    
    figure(1);
    imshow(uint8(imgs(:, :, i)));
    figure(2);
    imagesc(imgsd(:, :, i));
    
    P(:, :, i) = xyz_points(depth_array, cam_params.Kdepth);
       
end

bgdepth=median(imgsd,3);

%figure(3);
%imagesc(bgdepth);

imgdiffiltered = zeros(480, 640, length(d));
imdiff = zeros(480, 640, length(d));
abs_gradient = zeros(480, 640, length(d));
connected_objs = zeros(480, 640, length(d));

threshold = 1000; %minimum number of pixels to be considered an object

for i=1:length(d),
    imdiff(:, :, i) = abs(imgsd(:,:,i)-bgdepth)>.20;
  % imgdiffiltered(:, :, i)=imopen(imdiff(:, :, i),strel('disk',5));
    imgdiffiltered(:, :, i)=imdiff(:, :, i);
%     figure(3); imshow(imgdiffiltered(:, :, i));
    
    %imgdiffiltered(:, :, i) = bwareaopen(imgdiffiltered(:, :, i),threshold);

    %[labeled_img, num] = bwlabel(imgdiffiltered(:, :, i));
    
    %figure(4); imagesc(labeled_img)
    
    A = imgsd(:, :, i);
    B = double(imgdiffiltered(:, :, i));
    foreground_depth = A.*B;
    
    [fx, fy] = gradient(foreground_depth);
    abs_grad = sqrt(fx.^2 + fy.^2);
    abs_grad(abs_grad < 0.7) = 0;
    abs_grad(abs_grad > 0) = 1;
    abs_gradient(:, :, i) = abs_grad;
    %figure(5); imshow(abs_gradient(:, :, i));
    
    A = imgdiffiltered(:, :, i);
    A(abs_gradient(:, :, i) == 1) = 0;
   % A = imopen(A, strel('disk', 5));
    A = bwareaopen(A,threshold);
    connected_objs(:, :, i) = bwlabel(A);
    %figure(6); imagesc(connected_objs(:, :, i));
    
%     pause(0.5);
end

A = imgsd(:, :, 40);
B = double(imgdiffiltered(:, :, 40));
C = connected_objs(:, :, 40);
foreground_depth = A.*B;

figure(6); imshow(imgdiffiltered(:, :, 40));

figure(7); imagesc(connected_objs(:, :, 40));

objects = boxes_3D(C, A, cam_params.Kdepth);

point_cloud = xyz_points(foreground_depth, cam_params.Kdepth);

pc=pointCloud(point_cloud');
figure(8);showPointCloud(pc); hold on;

for i=1:max(C(:))

    plot3([objects(1, 1, i); objects(1, 2, i)], [objects(2, 1, i); objects(2, 2, i)], [objects(3, 1, i); objects(3, 2, i)]);
    plot3([objects(1, 2, i); objects(1, 3, i)], [objects(2, 2, i); objects(2, 3, i)], [objects(3, 2, i); objects(3, 3, i)]);
    plot3([objects(1, 3, i); objects(1, 4, i)], [objects(2, 3, i); objects(2, 4, i)], [objects(3, 3, i); objects(3, 4, i)]);
    plot3([objects(1, 1, i); objects(1, 4, i)], [objects(2, 1, i); objects(2, 4, i)], [objects(3, 1, i); objects(3, 4, i)]);
    
    plot3([objects(1, 5, i); objects(1, 6, i)], [objects(2, 5, i); objects(2, 6, i)], [objects(3, 5, i); objects(3, 6, i)]);
    plot3([objects(1, 6, i); objects(1, 7, i)], [objects(2, 6, i); objects(2, 7, i)], [objects(3, 6, i); objects(3, 7, i)]);
    plot3([objects(1, 7, i); objects(1, 8, i)], [objects(2, 7, i); objects(2, 8, i)], [objects(3, 7, i); objects(3, 8, i)]);
    plot3([objects(1, 5, i); objects(1, 8, i)], [objects(2, 5, i); objects(2, 8, i)], [objects(3, 5, i); objects(3, 8, i)]);
    
    plot3([objects(1, 1, i); objects(1, 5, i)], [objects(2, 1, i); objects(2, 5, i)], [objects(3, 1, i); objects(3, 5, i)]);
    plot3([objects(1, 2, i); objects(1, 6, i)], [objects(2, 2, i); objects(2, 6, i)], [objects(3, 2, i); objects(3, 6, i)]);
    plot3([objects(1, 3, i); objects(1, 7, i)], [objects(2, 3, i); objects(2, 7, i)], [objects(3, 3, i); objects(3, 7, i)]);
    plot3([objects(1, 4, i); objects(1, 8, i)], [objects(2, 4, i); objects(2, 8, i)], [objects(3, 4, i); objects(3, 8, i)]);
end
