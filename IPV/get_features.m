%%

clear;
close all;

%%
im1=imread('duascamaras/rgb_image1_4.png');
im2=imread('duascamaras/rgb_image2_1.png');
load('duascamaras/depth1_4.mat');
imdepth1 = double(depth_array)/1000;
load('duascamaras/depth2_1.mat');
imdepth2 = double(depth_array)/1000;

[f1,d1]=vl_sift(single(rgb2gray(im1)));
[f2,d2]=vl_sift(single(rgb2gray(im2)));

[matches score] = vl_ubcmatch(d1,d2,1.5); 

figure(1);
subplot(1,2,1);
imshow(uint8(im1));
hold on;
plot(f1(1,matches(1,:)),f1(2,matches(1,:)),'b*');

subplot(1,2,2);
imshow(uint8(im2));
hold on;
plot(f2(1,matches(2,:)),f2(2,matches(2,:)),'r*');

[row1,ia1, ~] = unique(matches(1,:));
row2 = matches(2,:);
row2 = row2(ia1);
score = score(ia1);
[unique_row2, ia2, ~] = unique(row2);
unique_row1 = row1(ia2);
score = score(ia2);
unique_matches = [unique_row1; unique_row2];
%uniqueMatches= transpose(unique(transpose(matches), 'rows'))


picture1data = [transpose(f1(1,unique_matches(1,:))), transpose(f1(2,unique_matches(1,:)))];
picture2data = [transpose(f2(1,unique_matches(2,:))), transpose(f2(2,unique_matches(2,:)))];

picture1Z = zeros(length(picture1data(:, 1)), 1);
picture2Z = zeros(length(picture1data(:, 1)), 1);
for i = 1:length(picture1data(:, 1))
    picture1Z(i) = imdepth1(floor(picture1data(i, 2)), floor(picture1data(i, 1)));
    picture2Z(i) = imdepth2(floor(picture2data(i, 2)), floor(picture2data(i, 1)));
end

picture1data = [picture1data, picture1Z];
picture2data = [picture2data, picture2Z];

% figure(2);
% subplot(1,2,1);
% imshow(uint8(im1));
% hold on;
% plot(picture1data(:, 1), picture1data(:, 2),'b*');
% 
% subplot(1,2,2);
% imshow(uint8(im2));
% hold on;
% plot(picture2data(:, 1), picture2data(:, 2),'r*');

% data = [picture1data, picture2data];
% 
% 
% [model,inlierIdx] = ransac(data,fitFcn,distFcn,sampleSize,maxDistance);

%hand implemented
n = 500;
inliers = [];
for i=1:n

    indices = randperm(length(picture1data),4);
    group1 = picture1data(indices,:);
    group2 = picture2data(indices,:);

    [~,~,tr]=procrustes(group1,group2,'scaling',false,'reflection',false);
    points_ransac = picture2data*tr.T+ones(length(picture2data),1)*tr.c(1,:);

    distances = (points_ransac-picture1data).^2;
    distances = sqrt(distances(:,1)+distances(:,2)+distances(:,3));

    if sum(distances<0.05)>length(inliers)
        inliers = find(distances<0.05);
    end

end

% Calculation of transform with inliers

[~,~,tr]=procrustes(picture1data(inliers,:),picture2data(inliers,:),'scaling',false,'reflection',false);
cam2toW.R = tr.T';
cam2toW.T = tr.c(1,:)';
cam1toW.R = eye(3);
cam1toW.T = zeros(3,1);
