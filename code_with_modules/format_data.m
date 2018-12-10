%%

clear;
close all;

%%

load('cameraparametersAsus.mat');

directory = '2cams_people\Cam2';
img_format = 'png';

imgseq1 = struct();

d = dir( strcat(directory, '\*.', img_format) );
dd = dir( strcat(directory, '\*.mat') );

for i = 1:length(d)
    imgseq1(i).rgb = strcat(directory, '\', d(i).name);
    imgseq1(i).depth = strcat(directory, '\', dd(i).name);
end

