function [ objects ] = track3D_part1( imgseq1, cam_params )

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
           frames(i, obj).X = objects_frame(1, :, obj);
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
    objects = struct();
    last_obj = 0;
    start = 1;
    while(isempty(frames(start, 1).X))
        start = start + 1;
    end      
    
    for i=start:num_imgs-1 
        objects_in_frame_A = frames(i, :);
        objects_in_frame_B = frames(i + 1, :);
                    
        if ( ~isempty(objects_in_frame_A(1).X) && ~isempty(objects_in_frame_B(1).X) )
            if(i==1)
                fobjs = 1;
                while( ~isempty(objects_in_frame_A(fobjs).X) )
                    last_obj = last_obj + 1;
                    objects(last_obj)
                    fobjs = fobjs+1;
                end
            elseif( isempty(frames(i - 1, 1).X) )    
                fobjs = 1;
                while( ~isempty(objects_in_frame_A(fobjs).X) )
                    last_obj = last_obj + 1;
                    objects(last_obj)
                    fobjs = fobjs+1;
                end
            end    
                
            match_table = zeros(length(objects_in_frame_A),length(objects_in_frame_B));
            for j=length(objects_in_frame_A) %for all objects in a frame
                obj_1 = objects_in_frame_A(j);
                Xs= [obj_1.X];
                Ys= [obj_1.Y];
                Zs= [obj_1.Z];
                centroid_obj_in_frameA = mean([Xs; Ys; Zs]);
                for k=1:length(objects_in_frame_B)
                    obj_2 = objects_in_frame_B(k);
                    Xs= [obj_2.X];
                    Ys= [obj_2.Y];
                    Zs= [obj_2.Z];
                    centroid_obj_in_frameB= mean([Xs; Ys; Zs]);
                    %color_diff = obj_1.avg_color - obj_2.avg.color;
                    match_table(j, k)= norm(centroid_obj_in_frameA - centroid_obj_in_frameB) * distance_weight; %+ color_diff * color_weight;
                end
            end

            threshold = 0.5;
            min_matrix = 0;
            match_matrix = [];
            while( ~isempty(match_table) && min_matrix <= threshold )
                min_matrix = min(match_table(:));
                [min_row, min_col] = find(match_table==min_matrix);
                match_table(min_row, :) = [];
                match_table(:, min_col) = [];

                match_matrix = [match_matrix, [min_row; min_col]];
            end
        end
    end
        
end

