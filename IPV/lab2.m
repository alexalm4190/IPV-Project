im=imread('rgb_image_10.png');
load depth_10.mat
load CalibrationData.mat
Kd=Depth_cam.K;
Z=double(depth_array(:)')/1000;
% Compute correspondence between two imagens in 5 lines of code
[v u]=ind2sub([480 640],(1:480*640));
P=inv(Kd)*[Z.*u ;Z.*v;Z];
niu=RGB_cam.K*[R_d_to_rgb T_d_to_rgb]*[P;ones(1,640*480)];
u2=round(niu(1,:)./niu(3,:));
v2=round(niu(2,:)./niu(3,:));
% Compute new image - the easy understandable way
tic
im3=zeros(size(im));
for i=1:length(v2),
    if ((v2(i)>0 &v2(i)<482)&&(u2(i)>0 & u2(i)<642)),
        vv=max([1 min([v2(i) 480])]);
        uu=max([1 min([u2(i) 640])]);
        im3(v(i),u(i),:)=im(vv,uu,:);
    end
end
fprintf('Normal mode %g seconds \n',toc)
tic
% If you are  MATLAB "proficient"
im2=zeros(640*480,3);
indsclean=find((u2>=1)&(u2<=641)&(v2>=1)&(v2<=480));
indscolor=sub2ind([480 640],v2(indsclean),u2(indsclean));
im1aux=reshape(im,[640*480 3]);
im2(indsclean,:)=im1aux(indscolor,:);
% If you are really a MATLAB Pro(fessional) you can figure out a faster
% way!
fprintf('Fast mode %g seconds \n',toc);
pc=pointCloud(P', 'color',uint8(im2));
figure(1);showPointCloud(pc);
figure(2);imshow(uint8(reshape(im2,[480,640,3])));