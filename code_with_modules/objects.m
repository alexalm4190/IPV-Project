distance_weight= 0.5;
color_weight= 0.5;
for i=1:length(frames(1,:))-1 
    objects_in_frame_A= frames(i,:);
    objects_in_frame_B= frames(i + 1,:);
    table = zeros(length(objects_in_frame_A),length(objects_in_frame_B));
    for j=length(objects_in_frame_A) %for all objects in a frame
        obj_1= objects_in_frame(j);
            Xs= [obj_1.X];
            Ys= [obj_1.Y];
            Zs= [obj_1.Z];
        centroid_obj_in_frameA= mean([Xs, Ys, Zs]);
        for k=1:length(objects_in_frame_B)
            Xs= [obj_1.X];
            Ys= [obj_1.Y];
            Zs= [obj_1.Z];
            centroid_obj_in_frameB= mean([Xs, Ys, Zs]);
            color_diff= obj_1.avg_color - obj_2.avg.color;
            table(j, k)= norm(centroid_obj_in_frameA - centroid_obj_in_frameB) * distance_weight + color_diff * color_weight;
        
        end
    end
end

for i=1:length(table())

