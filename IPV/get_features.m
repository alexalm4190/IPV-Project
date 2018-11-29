
im1=imread('duascamaras/rgb_image1_4.png');
im2=imread('duascamaras/rgb_image2_1.png');
[f1,d1]=vl_sift(single(rgb2gray(im1)));
[f2,d2]=vl_sift(single(rgb2gray(im2)));

[matches score] = vl_ubcmatch(d1,d2,1.5); 

figure(1);
subplot(1,2,1);
imshow(uint8(im1));
hold on;
plot(f1(1,matches(1,:)),f1(2,matches(1,:)),'b*');

subplot(1,2,2);
imshow(uint8(im2));
hold on;
plot(f2(1,matches(2,:)),f2(2,matches(2,:)),'r*');

A = matches(2, :);
B = unique(A)

% figure(1);
% imagesc(im1);hold on;plot(f1(1,:),f1(2,:),'*');hold off;
% figure(2);
% imagesc(im2);hold on;plot(f2(1,:),f2(2,:),'*');hold off;