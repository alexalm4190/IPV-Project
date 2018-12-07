function [ connected_objs, num_objs ] = find_connected_objs( background, img, pixel_thres, diff_thres, filter_thres, gradient_thres )

    imdiff = abs(img - background)>diff_thres;
    
    foreground = img.*double(imdiff);
    
    [fx, fy] = gradient(foreground);
    abs_gradient = sqrt(fx.^2 + fy.^2);
    abs_gradient(abs_gradient < gradient_thres) = 0;
    abs_gradient(abs_gradient > 0) = 1;
    %figure(5); imshow(abs_gradient(:, :, i));
    
    imdiff(abs_gradient == 1) = 0;
    imdiff = imopen(imdiff, strel('disk', filter_thres));
    imdiff = bwareaopen(imdiff, pixel_thres);
    connected_objs = bwlabel(imdiff);
    
    connected_objs(img == 0) = 0; %removes 0 depth points because kinect is faulty 
    
    num_objs = max(max(connected_objs));
    
end

