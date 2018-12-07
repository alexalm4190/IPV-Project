function [ imgs_rgb, imgs_depth, num_imgs ] = load_images( directory, img_format )
%Receives a directory and an image format
%Returns the rgb and depth images, inside the input directory along with
%the number of images

    d = dir( strcat(directory, '\*.', img_format) );
    dd = dir( strcat(directory, '\*.mat') );
    imgs_rgb=zeros(480,640, 3, length(d));
    imgs_depth=zeros(480,640,length(d));

    num_imgs = length(d);
    for i=1:num_imgs
        imgs_rgb(:, :, :, i) = imread( strcat(directory, '\', d(i).name) );
        load(strcat(directory, '\', dd(i).name));
        imgs_depth(:, :, i) = double(depth_array)/1000;
    end
    
end

