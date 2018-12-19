function [ objects ] = boxes_3D( labeled_img, depth_img, Kdepth )

    objects = zeros(3, 8, max(labeled_img(:)));

    for i = 1:max(labeled_img(:))
        BW = labeled_img;
        BW(BW ~= i) = 0;

        object_i_depth = (depth_img.*double(BW))/i;

        object_i_pc = xyz_points(object_i_depth, Kdepth);
        A = object_i_pc(1, :);
        B = object_i_pc(2, :);
        C = object_i_pc(3, :);

        object_i_point_cloud = [A(A~=0); B(B~=0); C(C~=0)];

        min_x = min(object_i_point_cloud(1, :));
        max_x = max(object_i_point_cloud(1, :));

        min_y = min(object_i_point_cloud(2, :));
        max_y = max(object_i_point_cloud(2, :));

        min_z = min(object_i_point_cloud(3, :));
        max_z = max(object_i_point_cloud(3, :));

        objects(:, 1 , i) = [max_x; min_y; min_z]; %point 1
        objects(:, 2 , i) = [max_x; max_y; min_z]; %point 2
        objects(:, 3 , i) = [min_x; max_y; min_z]; %point 3
        objects(:, 4 , i) = [min_x; min_y; min_z]; %point 4
        objects(:, 5 , i) = [max_x; min_y; max_z]; %point 5
        objects(:, 6 , i) = [max_x; max_y; max_z]; %point 6
        objects(:, 7 , i) = [min_x; max_y; max_z]; %point 7
        objects(:, 8 , i) = [min_x; min_y; max_z]; %point 8
    end
    
end

