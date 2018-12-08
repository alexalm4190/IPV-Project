%%

clear;
close all;

%%

load('cameraparametersAsus.mat');

[imgs_rgb, imgs_depth, num_imgs] = load_images('2cams_people\Cam2', 'png');

bg_depth = median(imgs_depth, 3);

connected_objs = zeros(480, 640, num_imgs);
num_objects = zeros(1, num_imgs);

for i=1:num_imgs
   
    [connected_objs(:, :, i), num_objects(i)] = find_connected_objs(bg_depth, imgs_depth(:, :, i), 300, 0.20, 2, 0.3);

    %DEBUG    
    figure(1); imagesc(connected_objs(:, :, i));
    pause(0.5);

end

%DEBUG
show = 8;
C = connected_objs(:, :, show);
figure(111); imagesc(C);

objects = boxes_3D(C, imgs_depth(:, :, show), cam_params.Kdepth);

point_cloud = xyz_points(imgs_depth(:, :, show), cam_params.Kdepth);

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

