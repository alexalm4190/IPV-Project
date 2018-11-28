%%

clear;
close all;

%% load images to variables

load('cameraparametersAsus.mat');
d=dir('imageData\*.jpg');
dd=dir('imageData\*.mat');
imgs=zeros(480,640,length(d));
imgsd=zeros(480,640,length(d));
for i=1:length(d),
    imgs(:,:,i)=rgb2gray(imread(strcat('imageData\', d(i).name)));
    load(strcat('imageData\', dd(i).name));
    imgsd(:,:,i)=double(depth_array)/1000;
    figure(1)
%     imshow(uint8(imgs(:,:,i)));
    figure(2);
%     imagesc(imgsd(:,:,i));
    %colormap(gray);
end

%% get background image in depth and rgb

bgdepth=median(imgsd,3);
bggray=median(imgs,3);
figure(1);
subplot(211);imagesc(bgdepth);
subplot(212);imagesc(bggray);

%%
% Bg subtraction for depth (try with gray too)

figure(1);clf;
figure(2);clf;
for i=1:length(d),
    imdiff=abs(imgsd(:,:,i)-bgdepth)>.20;
    imgdiffiltered=imopen(imdiff,strel('disk',5));
    figure(1);
    imagesc([imdiff imgdiffiltered]);
    title('Difference image and morph filtered');
    colormap(gray);
    figure(2);
    imagesc([imgsd(:,:,i) bgdepth]);
    title('Depth image i and background image');
    figure(3);
   
    BW = imgdiffiltered;
    stats = regionprops(BW, 'basic');
    imagesc(imgsd(:,:,i));
    
    figure(4);
    imagesc(imgs(:,:,i));
    
    labeled_img = bwlabel(imgdiffiltered);
    figure(5);
    imagesc(labeled_img);
    
    sz = size(stats); 
    max = 0;
    index = 1;
    for j=1:sz(1)
        if stats(j).Area > max
            index = j;
            max = stats(j).Area;
        end    
    end
    rectangle('Position', stats(index).BoundingBox);
    title('Connected components');
    pause(0.5);

    if(i == 20)
        break;
    end    
    
end

