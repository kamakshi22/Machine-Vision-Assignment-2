function practical1B

%the aim of the second part of practical 1 is to use the homography routine
%that you established in the first part of the practical.  We are going to
%make a panorama of several images that are related by a homography.  I
%provide 3 images (one of which is has a large surrounding region) and a
%matching set of points between these images.

%close all open figures
close all;

%load in the required data
load('PracticalData','im1','im2','im3','pts1','pts2','pts3','pts1b');
%im1 is center image with grey background
%im2 is left image 
%pts1 and pts2 are matching points between image1 and image2
%im3 is right image
%pts1b and pts3 are matching points between image 1 and image 3

%show images and points
figure; set(gcf,'Color',[1 1 1]);image(uint8(im1));axis off;hold on;axis image;
plot(pts1(1,:),pts1(2,:),'r.'); 
plot(pts1b(1,:),pts1b(2,:),'m.');
print('Ex_7part2_1','-depsc');
figure; set(gcf,'Color',[1 1 1]);image(uint8(im2));axis off;hold on;axis image;
plot(pts2(1,:),pts2(2,:),'r.'); 
print('Ex_7part2_2','-depsc');
figure; set(gcf,'Color',[1 1 1]);image(uint8(im3));axis off;hold on;axis image;
plot(pts3(1,:),pts3(2,:),'m.'); 
print('Ex_7part2_3','-depsc');

%****TO DO**** 
%calculate homography from pts1 to pts2
H= calcBestHomography(pts1, pts2)

%****TO DO**** 
%for every pixel in image 1
    %transform this pixel position with your homography to find where it 
    %is in the coordinates of image 2
    %if it the transformed position is within the boundary of image 2 then 
        %copy pixel colour from image 2 pixel to current position in image 1 
        %draw new image1 (use drawnow to force it to draw)
    %end
%end;
[im1r im1c im1h]= size(im1);
[im2r im2c im2h]= size(im2);
 for i= 1:im1c
     for j= 1:im1r
         P1= [i,j,1];
         P2= H*P1';
         x2= P2(1,1)./P2(3,1);
         y2= P2(2,1)./P2(3,1);
         
         if(round(x2) > 0 && round(x2) < im2c && round(y2) > 0 && round(y2) < im2r )
            
             im1(j,i,:)= im2(round(y2),round(x2),:);
             
         end
         
     end
 end
 
figure; set(gcf,'Color',[1 1 1]);image(uint8(im1));axis off;hold on;axis image;
%print('Ex_7part2_4','-depsc');

H= calcBestHomography(pts1b, pts3)

[im1r im1c im1h]= size(im1);
[im3r im3c im3h]= size(im3);
 for i= 1:im1c
     for j= 1:im1r
         P1= [i,j,1];
         P2= H*P1';
         x2= P2(1,1)./P2(3,1);
         y2= P2(2,1)./P2(3,1);
         
         if(round(x2) > 0 && round(x2) < im2c && round(y2) > 0 && round(y2) < im2r )
            
             im1(j,i,:)= im3(round(y2),round(x2),:);
             
         end
         
     end
 end
 
figure; set(gcf,'Color',[1 1 1]);image(uint8(im1));axis off;hold on;axis image;
%print('Ex_7part2_5','-depsc');




 
 
function H = calcBestHomography(pts1Cart, pts2Cart)

%should apply direct linear transform (DLT) algorithm to calculate best
%homography that maps the points in pts1Cart to their corresonding matchin in 
%pts2Cart

%****TO DO ****: replace this

A= zeros(10,9);
for i= 1: size(pts1Cart,2)
A(2*i-1,:)= [ 0,0, 0, -pts1Cart(1,i),-pts1Cart(2,i), -1,  pts2Cart(2,i) *pts1Cart(1,i), pts2Cart(2,i) *pts1Cart(2,i), pts2Cart(2,i)] 
A(2*i,:)= [pts1Cart(1,i),pts1Cart(2,i),1,0, 0,0, -pts2Cart(1,i) *pts1Cart(1,i), -pts2Cart(1,i) *pts1Cart(2,i), -pts2Cart(1,i)]

end


%**** TO DO ****;
%first turn points to homogeneous
%then construct A matrix which should be (10 x 9) in size
%solve Ah = 0 by calling
h = solveAXEqualsZero(A); %(you have to write this routine too - see below)

%reshape h into the matrix H
H= reshape(h,3,3)';

%Beware - when you reshape the (9x1) vector x to the (3x3) shape of a homography, you must make
%sure that it is reshaped with the values going first into the rows.  This
%is not the way that the matlab command reshape works - it goes columns
%first.  In order to resolve this, you can reshape and then take the
%transpose


%==========================================================================
function x = solveAXEqualsZero(A);
[U,S,V]= svd(A);
x= V(:,end);






%****TO DO****
%repeat the above process mapping image 3 to image 1.

