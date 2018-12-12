%%

clear;
close all;

%%

load('cameraparametersAsus.mat');

directory_cam1 = '2cams_people\Cam1';
directory_cam2 = '2cams_people\Cam2';
img_format = 'png';

imgseq1 = struct();
imgseq2 = struct();

d_cam1 = dir( strcat(directory_cam1, '\*.', img_format) );
dd_cam1 = dir( strcat(directory_cam1, '\*.mat') );
d_cam2 = dir( strcat(directory_cam2, '\*.', img_format) );
dd_cam2 = dir( strcat(directory_cam2, '\*.mat') );

for i = 1:length(d_cam1)
    imgseq1(i).rgb = strcat(directory_cam1, '\', d_cam1(i).name);
    imgseq1(i).depth = strcat(directory_cam1, '\', dd_cam1(i).name);
    imgseq2(i).rgb = strcat(directory_cam2, '\', d_cam2(i).name);
    imgseq2(i).depth = strcat(directory_cam2, '\', dd_cam2(i).name);
end

