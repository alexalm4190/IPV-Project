function [ R, T, inliers ] = ransac_procrustes( xyz1, xyz2, distance_threshold, num_it )
%Receives input data points xyz1 and xyz2 (Nx3 matrixes), a threshold (distance_threshold) and a
%maximum number of iterations (num_it)
%Returns the rotation matrix R and translation vector T that maps points
%from xyz2 coordinate frame to xyz1 coordinate frame


    %hand implemented
    inliers = [];
    for i=1:num_it

        indices = randperm(length(xyz1),4);
        group1 = xyz1(indices,:);
        group2 = xyz2(indices,:);

        [~,~,tr]=procrustes(group1,group2,'scaling',false,'reflection',false);
        points_ransac = xyz2*tr.T+ones(length(xyz2),1)*tr.c(1,:);

        distances = (points_ransac-xyz1).^2;
        distances = sqrt(distances(:,1)+distances(:,2)+distances(:,3));

        if sum(distances<distance_threshold)>length(inliers)
            inliers = find(distances<distance_threshold);
        end

    end

    % Calculation of transform with inliers
    [~,~,tr]=procrustes(xyz1(inliers,:),xyz2(inliers,:),'scaling',false,'reflection',false);
    R = tr.T;
    T = tr.c(1,:)';

end
