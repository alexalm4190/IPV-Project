%%

clear;
close all;

%%

load('cameraparametersAsus.mat');

[imgs_rgb_cam1, imgs_depth_cam1, num_imgs_cam1] = load_images('2cams_people\Cam1', 'png');
[imgs_rgb_cam2, imgs_depth_cam2, num_imgs_cam2] = load_images('2cams_people\Cam2', 'png');

bg_depth_cam1 = median(imgs_depth_cam1, 3);
bg_depth_cam2 = median(imgs_depth_cam2, 3);

connected_objs_cam1 = zeros(480, 640, num_imgs_cam1);
num_objects_cam1 = zeros(1, num_imgs_cam1);
connected_objs_cam2 = zeros(480, 640, num_imgs_cam2);
num_objects_cam2 = zeros(1, num_imgs_cam2);
rgbd_cam1 = zeros(480, 640, 3, num_imgs_cam1); 
rgbd_cam2 = zeros(480, 640, 3, num_imgs_cam2);

for i=1:num_imgs_cam1
   
    [connected_objs_cam1(:, :, i), num_objects_cam1(i)] = find_connected_objs(bg_depth_cam1, imgs_depth_cam1(:, :, i), 300, 0.20, 2, 0.3);
    [connected_objs_cam2(:, :, i), num_objects_cam2(i)] = find_connected_objs(bg_depth_cam2, imgs_depth_cam2(:, :, i), 300, 0.20, 2, 0.3);

    %DEBUG    
%     figure(1); imagesc(connected_objs_cam1(:, :, i));
%     figure(2); imagesc(connected_objs_cam2(:, :, i));
%     pause(0.5);

    dep1 = imgs_depth_cam1(:, :, i);
    dep2 = imgs_depth_cam2(:, :, i);

    xyz1=get_xyzasus(dep1(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
    xyz2=get_xyzasus(dep2(:),[480 640],(1:640*480)', cam_params.Kdepth,1,0);
    %REGISTER RGB TO DEPTH
    rgbd_cam1(:, :, :, i) = get_rgbd(xyz1, imgs_rgb_cam1(:, :, :, i), cam_params.R, cam_params.T, cam_params.Krgb);
    rgbd_cam2(:, :, :, i) = get_rgbd(xyz2, imgs_rgb_cam2(:, :, :, i), cam_params.R, cam_params.T, cam_params.Krgb);

end

[f1,d1] = vl_sift(single(rgb2gray(uint8(rgbd_cam1(:, :, :, 1)))));
[f2,d2] = vl_sift(single(rgb2gray(uint8(rgbd_cam2(:, :, :, 1)))));

[matches score] = vl_ubcmatch(d1,d2,1.4); 

I1 = rgb2gray(uint8(rgbd_cam1(:, :, :, 1)));
I2 = rgb2gray(uint8(rgbd_cam2(:, :, :, 1)));

matched_points_1 = [f1(1,matches(1,:))' f1(2,matches(1,:))'];
matched_points_2 = [f2(1,matches(2,:))' f2(2,matches(2,:))'];

figure(3); ax = axes;
showMatchedFeatures(I1, I2, matched_points_1, matched_points_2, 'montage', 'parent', ax);
% figure(3);
% subplot(1,2,1);
% imshow(uint8(rgbd_cam1(:, :, :, 1)));
% hold on;
% plot(f1(1,matches(1,:)),f1(2,matches(1,:)),'b*');
% 
% subplot(1,2,2);
% imshow(uint8(rgbd_cam2(:, :, :, 1)));
% hold on;
% plot(f2(1,matches(2,:)),f2(2,matches(2,:)),'r*');

% CONVERT POINTS TO XYZ COORDINATES
picture1data = [transpose(f1(1,matches(1,:))), transpose(f1(2,matches(1,:)))];
picture2data = [transpose(f2(1,matches(2,:))), transpose(f2(2,matches(2,:)))];

picture1Z = zeros(length(picture1data(:, 1)), 1);
picture2Z = zeros(length(picture1data(:, 1)), 1);
for i = 1:length(picture1data(:, 1))
    picture1Z(i) = imgs_depth_cam1(floor(picture1data(i, 2)), floor(picture1data(i, 1)));
    picture2Z(i) = imgs_depth_cam2(floor(picture2data(i, 2)), floor(picture2data(i, 1)));
end

picture1data = [floor(picture1data), picture1Z];
picture2data = [floor(picture2data), picture2Z];

xyz_cam1 = cam_params.Kdepth\[picture1data(:, 3)'.*picture1data(:, 1)'; picture1data(:, 3)'.*picture1data(:, 2)'; picture1data(:, 3)'];
xyz_cam1 = xyz_cam1';

xyz_cam2 = cam_params.Kdepth\[picture2data(:, 3)'.*picture2data(:, 1)'; picture2data(:, 3)'.*picture2data(:, 2)'; picture2data(:, 3)'];
xyz_cam2 = xyz_cam2';

[ R, T, inliers ] = ransac_procrustes( xyz_cam1, xyz_cam2, 0.05, 500 );
cam2toW.R = R;
cam2toW.T = T;
cam1toW.R = eye(3);
cam1toW.T = zeros(3,1);

frame = 8;

figure(21561);
imshow(uint8(imgs_rgb_cam1(:, :, :, frame)));
figure(22561);
imshow(uint8(imgs_rgb_cam2(:, :, :, frame)));

labeled_cam1 = connected_objs_cam1(:, :, frame);
labeled_cam2 = connected_objs_cam2(:, :, frame);

P_cam1 = xyz_points( imgs_depth_cam1(:, :, frame), cam_params.Kdepth);
P_cam2 = xyz_points( imgs_depth_cam2(:, :, frame), cam_params.Kdepth);
P_cam2 = cam2toW.R*P_cam2 + cam2toW.T*ones(1, length(P_cam2(1, :))); %transform camera 2 coordinates

pc1=pointCloud(P_cam1');
pc2=pointCloud(P_cam2');
pc_merged = pcmerge(pc1, pc2, 0.01);
figure(8);showPointCloud(pc_merged); hold on;

match_table = zeros(num_objects_cam1(frame), num_objects_cam2(frame));
for obj_cam1=1:num_objects_cam1(frame)
    indexes = find(labeled_cam1==obj_cam1);
    xyz_obj_cam1 = P_cam1(:, indexes);
    centroid_obj_cam1 = mean(xyz_obj_cam1, 2);
    
    for obj_cam2=1:num_objects_cam2(frame)
    
        indexes = find(labeled_cam2==obj_cam2);
        xyz_obj_cam2 = P_cam2(:, indexes);
        centroid_obj_cam2 = mean(xyz_obj_cam2, 2);
        
        match_table(obj_cam1, obj_cam2) = norm(centroid_obj_cam2 - centroid_obj_cam1);
        
    end    
end    

matches = [];
match_threshold = 1.5; %10 centimeters
if(num_objects_cam1(frame) <= num_objects_cam2(frame))
    missing = linspace(1, num_objects_cam2(frame), num_objects_cam2(frame));
    for i = 1:num_objects_cam1(frame)
        [min_val, min_index] = min(match_table(i, :));
        if( min_val <= match_threshold )
            matches = cat(2, matches, [i; min_index]);
            missing(missing == min_index) = [];
        else
            matches = cat(2, matches, [i; 0]);
        end
    end
    
    missing = [zeros(1,length(missing)); missing];
    matches = cat(2, matches, missing);
else
    missing = linspace(1, num_objects_cam1(frame), num_objects_cam1(frame));
    for i = 1:num_objects_cam2(frame)
        [min_val, min_index] = min(match_table(:, i));
        if( min_val <= match_threshold )
            matches = cat(2, matches, [min_index; i]);
            missing(missing == min_index) = [];
        else
            matches = cat(2, matches, [0; i]);
        end
    end
    
    missing = [missing; zeros(1,length(missing))];
    matches = cat(2, matches, missing);
end    


