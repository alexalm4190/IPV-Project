function [ objects ] = track3D_part1( imgseq1, cam_params, min_frames )

    imgs_rgb = zeros(480,640, 3, length(imgseq1));
    imgs_depth = zeros(480,640,length(imgseq1));

    num_imgs = length(imgseq1);
    for i=1:num_imgs
        imgs_rgb(:, :, :, i) = imread( imgseq1(i).rgb );
        load( imgseq1(i).depth );
        imgs_depth(:, :, i) = double(depth_array)/1000; %convert to meters
    end

    bg_depth = median(imgs_depth, 3);

    connected_objs = zeros(480, 640, num_imgs);
    num_objects = zeros(1, num_imgs);
    frames = struct();
    for i=1:num_imgs

        [connected_objs(:, :, i), num_objects(i)] = find_connected_objs(bg_depth, imgs_depth(:, :, i), 300, 0.20, 2, 0.3);

        objects_frame = boxes_3D(connected_objs(:, :, i), imgs_depth(:, :, i), cam_params.Kdepth);
        for obj = 1:num_objects(i)
           frames(i, obj).X = objects_frame(1, :, obj);
           frames(i, obj).Y = objects_frame(2, :, obj);
           frames(i, obj).Z = objects_frame(3, :, obj);
           frames(i, obj).frames_tracked = i;
        end    
        
        %DEBUG    
%         figure(1); imagesc(connected_objs(:, :, i));
%         pause(0.5);

    end
    
    %DEBUG
%     show = 8;
%     C = connected_objs(:, :, show);
%     figure(111); imagesc(C);
%     point_cloud = xyz_points(imgs_depth(:, :, show), cam_params.Kdepth);
% 
%     pc=pointCloud(point_cloud');
%     figure(8);showPointCloud(pc); hold on;
% 
%     for i=1:max(C(:))
%         objects = [frames(show, i).X; frames(show, i).Y; frames(show, i).Z];
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
    last_obj = 0;
 
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
        

