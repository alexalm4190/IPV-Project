%%

clear;
close all;

%%

d=dir('imageData\*.jpg');
dd=dir('imageData\*.mat');

im=imread(strcat('imageData\', d(20).name));
load(strcat('imageData\', dd(20).name));
load cameraparametersAsus.mat
Kd=cam_params.Kdepth;
Z=double(depth_array(:)')/1000;
% Compute correspondence between two imagens in 5 lines of code
[v u]=ind2sub([480 640],(1:480*640));
P=inv(Kd)*[Z.*u ;Z.*v;Z];
niu=Kd*[cam_params.R cam_params.T]*[P;ones(1,640*480)];
u2=round(niu(1,:)./niu(3,:));
v2=round(niu(2,:)./niu(3,:));

tic
% If you are  MATLAB "proficient"
im2=zeros(640*480,3);
indsclean=find((u2>=1)&(u2<=641)&(v2>=1)&(v2<=480));
indscolor=sub2ind([480 640],v2(indsclean),u2(indsclean));
im1aux=reshape(im,[640*480 3]);
im2(indsclean,:)=im1aux(indscolor,:);
% If you are really a MATLAB Pro(fessional) you can figure out a faster
% way!
fprintf('Fast mode %g seconds \n',toc);
pc=pointCloud(P', 'color', uint8(im2));
figure(1);showPointCloud(pc);
figure(2);imshow(uint8(reshape(im2,[480,640,3])));

%%

load('labeled_img.mat');
load('stats.mat');
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