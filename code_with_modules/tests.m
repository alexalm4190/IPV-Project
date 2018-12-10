%%

clear;
close all;

%% load files
% [imgs_rgb, imgs_depth, num_imgs] = load_images('filinha', 'jpg');
% 
% for i = 1:num_imgs
%     
%     figure(1);
%     imshow(uint8(imgs_rgb(:, :, :, i)));
%     
%     figure(2);
%     imagesc(imgs_depth(:, :, i));
%     
% end  

%% test ransac

while(1)
    [U,S,V] = svd(rand(3,3));
    R_rand = U*V';
    if( abs(det(R_rand)-1) < 0.005 )
        break;
    end    
end

T_rand = rand(3,1);

p1 = rand(50, 3);
p2 = p1*R_rand + ones(length(p1),1)*T_rand';

p1_outliers = p1;
outliers = linspace(1, 40, 40);
num_outliers = length(outliers);
p1_outliers(outliers, 1)=50;

[R, T, inliers] = ransac_procrustes(p2, p1_outliers, 0.05, 500);

errorR = R - R_rand
errorT = T - T_rand

