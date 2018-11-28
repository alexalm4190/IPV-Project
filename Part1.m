%%

clear;
close all;

%% load images to variables

load('cameraparametersAsus.mat');
d=dir('imageData\*.jpg');
dd=dir('imageData\*.mat');
imgs=zeros(480,640, 3, length(d));
imgsd=zeros(480,640,length(d));
P = zeros(3, 480*640, length(d));

for i=1:length(d),
    imgs(:,:,:, i)=imread(strcat('imageData\', d(i).name));
    load(strcat('imageData\', dd(i).name));
    imgsd(:,:,i)=double(depth_array)/1000;
    
    P(:, :, i) = xyz_points(depth_array, cam_params.Kdepth);
   
end

pc=pointCloud(P(:, :, 20)');
figure(1);showPointCloud(pc);   
figure(2);
imagesc(imgsd(:,:,20));
    
bgdepth=median(imgsd,3);

%%
% Bg subtraction for depth

imgdiffiltered = zeros(480, 640, length(d));
imdiff = zeros(480, 640, length(d));
threshold = 3000;

for i=1:length(d),
    imdiff(:, :, i) =abs(imgsd(:,:,i)-bgdepth)>.20;
    imgdiffiltered(:, :, i)=imopen(imdiff(:, :, i),strel('disk',5));
    
    figure(3); imshow(imgdiffiltered(:, :, i));
    
    [labeled_img, num] = bwlabel(imgdiffiltered(:, :, i));
    
    figure(4); imagesc(labeled_img)
    
    for j = 1:num
        X = find(labeled_img==j);
        if(length(X) < threshold)
            aux = imgdiffiltered(:, :, i);
            aux(labeled_img==j) = 0;
            imgdiffiltered(:, :, i) = aux;
        end    
    end
    
    figure(5); imshow(imgdiffiltered(:, :, i));
    
end

A = imgsd(:, :, 20);
B = double(imgdiffiltered(:, :, 20));
filtered_PC = A.*B;

point_cloud_labeled = xyz_points(filtered_PC, cam_params.Kdepth);

pc=pointCloud(point_cloud_labeled');
figure(6);showPointCloud(pc);   
