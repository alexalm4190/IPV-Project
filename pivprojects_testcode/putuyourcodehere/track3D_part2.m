function [ objects, cam2toW ] = track3D_part2( imgseq1, imgseq2, cam_params )

    num_imgs = length(imgseq1); %both cameras must have the same number of frames
    imgs_rgb_cam1 = zeros(480,640, 3, num_imgs);
    imgs_depth_cam1 = zeros(480, 640, num_imgs);
    imgs_rgb_cam2 = zeros(480, 640, 3, num_imgs);
    imgs_depth_cam2 = zeros(480, 640, num_imgs);

    min_frames = 1; %number of minimum frames to consider an object 
    
    for i=1:num_imgs
        %load camera 1 rgb and depth images
        imgs_rgb_cam1(:, :, :, i) = imread( imgseq1(i).rgb );
        load( imgseq1(i).depth );
        imgs_depth_cam1(:, :, i) = double(depth_array)/1000; %convert to meters
        
        %load camera 2 rgb and depth images
        imgs_rgb_cam2(:, :, :, i) = imread( imgseq2(i).rgb );
        load( imgseq2(i).depth );
        imgs_depth_cam2(:, :, i) = double(depth_array)/1000; %convert to meters
    end

    bg_depth_cam1 = median(imgs_depth_cam1, 3); %background depth image for camera 1
    bg_depth_cam2 = median(imgs_depth_cam2, 3); %background depth image for camera 2

    connected_objs_cam1 = zeros(480, 640, num_imgs);
    num_objects_cam1 = zeros(1, num_imgs);
    connected_objs_cam2 = zeros(480, 640, num_imgs);
    num_objects_cam2 = zeros(1, num_imgs);
    rgbd_cam1 = zeros(480, 640, 3, num_imgs); 
    rgbd_cam2 = zeros(480, 640, 3, num_imgs);

    for i=1:num_imgs

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

    
    %DEBUG
%     I1 = rgb2gray(uint8(rgbd_cam1(:, :, :, 1)));
%     I2 = rgb2gray(uint8(rgbd_cam2(:, :, :, 1)));
% 
%     matched_points_1 = [f1(1,matches(1,:))' f1(2,matches(1,:))'];
%     matched_points_2 = [f2(1,matches(2,:))' f2(2,matches(2,:))'];

%     figure(3); ax = axes;
%     showMatchedFeatures(I1, I2, matched_points_1, matched_points_2, 'montage', 'parent', ax);
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
        %index the points from the rgbd to the depth, to get the Z coordinates
        picture1Z(i) = imgs_depth_cam1(floor(picture1data(i, 2)), floor(picture1data(i, 1))); 
        picture2Z(i) = imgs_depth_cam2(floor(picture2data(i, 2)), floor(picture2data(i, 1)));
    end

    picture1data = [floor(picture1data), picture1Z];
    picture2data = [floor(picture2data), picture2Z];

    xyz_cam1 = cam_params.Kdepth\[picture1data(:, 3)'.*picture1data(:, 1)'; picture1data(:, 3)'.*picture1data(:, 2)'; picture1data(:, 3)'];
    xyz_cam1 = xyz_cam1';

    xyz_cam2 = cam_params.Kdepth\[picture2data(:, 3)'.*picture2data(:, 1)'; picture2data(:, 3)'.*picture2data(:, 2)'; picture2data(:, 3)'];
    xyz_cam2 = xyz_cam2';

    [ R, T, ~ ] = ransac_procrustes( xyz_cam1, xyz_cam2, 0.05, 500 );
    cam2toW.R = R;
    cam2toW.T = T;
    %since camera 1 coordinate frame is the world coordinate frame,
    %the rotation is the identity and the translation is zero
    cam1toW.R = eye(3);
    cam1toW.T = zeros(3,1);
    
    frames = struct();
    
    num_objects = zeros(1, num_imgs);
    for frame=1:num_imgs
        if(num_objects_cam1(frame) == 0 && num_objects_cam2(frame) == 0 )
            continue;
        end    
        labeled_cam1 = connected_objs_cam1(:, :, frame);
        labeled_cam2 = connected_objs_cam2(:, :, frame);
        %DEBUG
%         figure(23561);
%         imagesc(connected_objs_cam1(:, :, frame));
%         figure(246);
%         imagesc(connected_objs_cam2(:, :, frame));

        P_cam1 = xyz_points( imgs_depth_cam1(:, :, frame), cam_params.Kdepth);
        P_cam2 = xyz_points( imgs_depth_cam2(:, :, frame), cam_params.Kdepth);
        %transform camera 2 coordinates to world coordinates
        P_cam2 = cam2toW.R*P_cam2 + cam2toW.T*ones(1, length(P_cam2(1, :))); 

        %DEBUG
%         pc1=pointCloud(P_cam1');
%         pc2=pointCloud(P_cam2');
%         pc_merged = pcmerge(pc1, pc2, 0.01);
%         figure(8);showPointCloud(pc_merged); hold on;

        if(num_objects_cam2(frame) == 0)
            obj_matches = [linspace(1, num_objects_cam1(frame), num_objects_cam1(frame)); zeros(1, num_objects_cam1(frame))];
        elseif(num_objects_cam1(frame) == 0)
            obj_matches = [zeros(1, num_objects_cam2(frame)); linspace(1, num_objects_cam2(frame), num_objects_cam2(frame))];
        else    
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

            obj_matches = [];
            match_threshold = 0.5; %half a meter
            min_matrix = 0;
            while( min_matrix <= match_threshold )

                min_matrix = min(match_table(:));
                if(min_matrix <= match_threshold)
                    [min_row, min_col] = find(match_table==min_matrix);
                    match_table(min_row, :) = ones(1, length(match_table(min_row, :)));
                    match_table(:, min_col) = ones(length(match_table(:, min_col)), 1);

                    obj_matches = [obj_matches, [min_row; min_col]];
                end
            end

            for i=1:num_objects_cam1(frame)
                if(isempty(obj_matches))
                    obj_matches = [obj_matches, [i; 0]];
                elseif( isempty( obj_matches(obj_matches(1, :) == i ) ) )
                    obj_matches = [obj_matches, [i; 0]];
                end
            end    
            for i=1:num_objects_cam2(frame)
                if(isempty(obj_matches))
                    obj_matches = [obj_matches, [0; i]];
                elseif( isempty( obj_matches(obj_matches(2, :) == i ) ) )
                    obj_matches = [obj_matches, [0; i]];
                end
            end
        end
        
        num_objects(frame) = length(obj_matches(1,:));
        for match_i = 1:length(obj_matches(1,:))
            
            if(obj_matches(1, match_i) == 0)
                xyz_obj_cam1 = [];
            else    
                indexes = find(labeled_cam1==obj_matches(1, match_i));
                xyz_obj_cam1 = P_cam1(:, indexes);
            end    
            
            if(obj_matches(2, match_i) == 0)
                xyz_obj_cam2 = [];
            else    
                indexes = find(labeled_cam2==obj_matches(2, match_i));
                xyz_obj_cam2 = P_cam2(:, indexes);
            end  

            %merge the xyz points of the objects, from both cameras
            object_i_point_cloud = [xyz_obj_cam1, xyz_obj_cam2];
            
            A = object_i_point_cloud(1, :);
            B = object_i_point_cloud(2, :);
            C = object_i_point_cloud(3, :);
            
            object_i_point_cloud = [A(A~=0); B(B~=0); C(C~=0)];
            
            min_x = min(object_i_point_cloud(1, :));
            max_x = max(object_i_point_cloud(1, :));

            min_y = min(object_i_point_cloud(2, :));
            max_y = max(object_i_point_cloud(2, :));

            min_z = min(object_i_point_cloud(3, :));
            max_z = max(object_i_point_cloud(3, :));
            
%             obj_volume = (max_z-min_z)*(max_y-min_y)*(max_x-min_x);
          
            frames(frame, match_i).X = [max_x, max_x, min_x, min_x, max_x, max_x, min_x, min_x];
            frames(frame, match_i).Y = [min_y, max_y, max_y, min_y, min_y, max_y, max_y, min_y];
            frames(frame, match_i).Z = [min_z, min_z, min_z, min_z, max_z, max_z, max_z, max_z];
            frames(frame, match_i).frames_tracked = frame;
        end    

    end
    
    %DEBUG
%     frame = 8;
% 
%     P_cam1 = xyz_points( imgs_depth_cam1(:, :, frame), cam_params.Kdepth);
%     P_cam2 = xyz_points( imgs_depth_cam2(:, :, frame), cam_params.Kdepth);
%     P_cam2 = cam2toW.R*P_cam2 + cam2toW.T*ones(1, length(P_cam2(1, :))); %transform camera 2 coordinates
% 
%     pc1=pointCloud(P_cam1');
%     pc2=pointCloud(P_cam2');
%     pc_merged = pcmerge(pc1, pc2, 0.01);
%     figure(32523);showPointCloud(pc_merged); hold on;
%     
%     for i=1:num_objects_cam1(frame)
%         if(isempty(frames(frame, i).X))
%             continue;
%         end    
%         objects = [frames(frame, i).X; frames(frame, i).Y; frames(frame, i).Z];
%         plot3([objects(1, 1); objects(1, 2)], [objects(2, 1); objects(2, 2)], [objects(3, 1); objects(3, 2)]);
%         plot3([objects(1, 2); objects(1, 3)], [objects(2, 2); objects(2, 3)], [objects(3, 2); objects(3, 3)]);
%         plot3([objects(1, 3); objects(1, 4)], [objects(2, 3); objects(2, 4)], [objects(3, 3); objects(3, 4)]);
%         plot3([objects(1, 1); objects(1, 4)], [objects(2, 1); objects(2, 4)], [objects(3, 1); objects(3, 4)]);
% 
%         plot3([objects(1, 5); objects(1, 6)], [objects(2, 5); objects(2, 6)], [objects(3, 5); objects(3, 6)]);
%         plot3([objects(1, 6); objects(1, 7)], [objects(2, 6); objects(2, 7)], [objects(3, 6); objects(3, 7)]);
%         plot3([objects(1, 7); objects(1, 8)], [objects(2, 7); objects(2, 8)], [objects(3, 7); objects(3, 8)]);
%         plot3([objects(1, 5); objects(1, 8)], [objects(2, 5); objects(2, 8)], [objects(3, 5); objects(3, 8)]);
% 
%         plot3([objects(1, 1); objects(1, 5)], [objects(2, 1); objects(2, 5)], [objects(3, 1); objects(3, 5)]);
%         plot3([objects(1, 2); objects(1, 6)], [objects(2, 2); objects(2, 6)], [objects(3, 2); objects(3, 6)]);
%         plot3([objects(1, 3); objects(1, 7)], [objects(2, 3); objects(2, 7)], [objects(3, 3); objects(3, 7)]);
%         plot3([objects(1, 4); objects(1, 8)], [objects(2, 4); objects(2, 8)], [objects(3, 4); objects(3, 8)]);
%     end
    
    distance_weight= 1;
    %color_weight= 0.5;
    objects = struct('X',{},'Y',{},'Z',{},'frames_tracked',{});
    temp_objects = struct();
    %last_obj = 0;
 
    temp_objects.X = [];
    temp_objects.Y = [];
    temp_objects.Z = [];
    temp_objects.frames_tracked = [];
    
     
    for i=1:num_imgs
        if (isempty(frames(i, 1).X))
            continue;
        end
        objects_in_frame = frames(i, :);
      
        if isempty(temp_objects(1).X)   
            for obj = 1:num_objects(i)
                temp_objects(obj).X = objects_in_frame(obj).X;
                temp_objects(obj).Y = objects_in_frame(obj).Y;
                temp_objects(obj).Z = objects_in_frame(obj).Z;
                temp_objects(obj).frames_tracked = objects_in_frame(obj).frames_tracked;
            end
        else
                
            match_table = zeros(length(temp_objects),length(objects_in_frame));
            for j=1:length(temp_objects) %for all objects in a frame
                obj_1 = temp_objects(j);
                Xs= [obj_1.X];
                Ys= [obj_1.Y];
                Zs= [obj_1.Z];
                centroid_obj_in_temp = mean([Xs; Ys; Zs]);
                for k=1:length(objects_in_frame)
                    obj_2 = objects_in_frame(k);
                    Xs= [obj_2.X];
                    Ys= [obj_2.Y];
                    Zs= [obj_2.Z];
                    centroid_obj_in_frame= mean([Xs; Ys; Zs]);
                    %color_diff = obj_1.avg_color - obj_2.avg.color;
                    match_table(j, k)= norm(centroid_obj_in_temp - centroid_obj_in_frame) * distance_weight; %+ color_diff * color_weight;
                    
                end
                
            end
            
            

            threshold = 0.2;
            min_matrix = 0;
            match_matrix = [];
            while(min_matrix <= threshold && ~isnan(min_matrix))
                min_matrix = min(match_table(:));
                [min_row, min_col] = find(match_table==min_matrix);
                temp_objects(min_row).X = [temp_objects(min_row).X ; objects_in_frame(min_col).X];
                temp_objects(min_row).Y = [temp_objects(min_row).Y ; objects_in_frame(min_col).Y];
                temp_objects(min_row).Z = [temp_objects(min_row).Z ; objects_in_frame(min_col).Z];
                temp_objects(min_row).frames_tracked = [temp_objects(min_row).frames_tracked,i];
                if size(match_table,1)== 1
                    break;
                end
                match_table(min_row, :) = NaN;
                match_table(:, min_col) = NaN;
                match_matrix = [match_matrix, [min_row; min_col]];
                min_matrix = min(match_table(:));

            end
            [row, col] = find(~isnan(match_table));
            row = unique(row);
            col = unique(col);
            
            for w=1:length(col)
                
                temp_objects(length(temp_objects)+1) = objects_in_frame(col(w));     
            end
            row=sort(row, 'descend');
            for w=1:length(row)   
                objects(length(objects)+1) = temp_objects(row(w)); 
                temp_objects(row(w))= [];
            end
        end
    end
    
    for w=1:length(temp_objects)
        objects(length(objects)+1) = temp_objects(w);
    end
   
    not_objects= [];
    for i=1:length(objects)
       if length(objects(i).frames_tracked) < min_frames
           not_objects= [not_objects i];
       end
    end
    objects(not_objects)= [];
    
end

