function [ P ] = xyz_points( depth_array, Kd )

Z=double(depth_array(:)');
[v u]=ind2sub([480 640],(1:480*640));
P = Kd\[Z.*u ;Z.*v;Z];

end
