

function practical2b

%The goal of this part of the practical is to take a real image containing
%a planar black square and figure out the transformation between the square
%and the camera.  We will then draw a wire-frame cube with it's base
%corners at the corner of the square.  You should use this
%template for your code and fill in the missing sections marked "TO DO"

%load in image 
im = imread('test104.jpg');

%define points on image
xImCart = [  140.3464  212.1129  346.3065  298.1344   247.9962;...
             308.9825  236.7646  255.4416  340.7335   281.5895];
         
%define 3D points of plane
XCart = [-50 -50  50  50 0 ;...
          50 -50 -50  50 0;...
           0   0   0   0 0];

%We assume that the intrinsic camera matrix K is known and has values
K = [640  0    320;...
     0    640  240;
     0    0    1];

%draw image and 2d points
figure; set(gcf,'Color',[1 1 1]);
imshow(im); axis off; axis image; hold on;
plot(xImCart(1,:),xImCart(2,:),'r.','MarkerSize',10);
%print('Ex_7part4_1','-depsc');
       
%TO DO Use your routine to calculate TEst, the extrinsic matrix relating the
%plane position to the camera position.



 TEst = estimatePlanePose(xImCart,XCart,K);


%define 3D points of plane
XWireFrameCart = [-50 -50  50  50 -50 -50  50  50;...
                   50 -50 -50  50  50 -50 -50  50;...
                    0   0   0   0 -100 -100 -100 -100];
                
XHom = [XWireFrameCart; ones(1,size(XWireFrameCart,2))];
 mul= TEst* XHom;
 K = [K,[0;0;0]];
 
  xImHom= K* mul;
  xImCart = xImHom(1:2,:)./repmat(xImHom(3,:),2,1);                

%TO DO Draw a wire frame cube, by projecting the vertices of a 3D cube
%through the projective camera and drawing lines betweeen the resulting 2d image
%points


    %plot a green line pairs of actual and estimated points
    hold on;
    plot([xImCart(1,1) xImCart(1,2)],[xImCart(2,1) xImCart(2,2)],'g-');
    plot([xImCart(1,2) xImCart(1,3)],[xImCart(2,2) xImCart(2,3)],'g-');
    plot([xImCart(1,3) xImCart(1,4)],[xImCart(2,3) xImCart(2,4)],'g-');
    plot([xImCart(1,4) xImCart(1,1)],[xImCart(2,4) xImCart(2,1)],'g-');
    
    plot([xImCart(1,5) xImCart(1,6)],[xImCart(2,5) xImCart(2,6)],'g-');
    plot([xImCart(1,6) xImCart(1,7)],[xImCart(2,6) xImCart(2,7)],'g-');
    plot([xImCart(1,7) xImCart(1,8)],[xImCart(2,7) xImCart(2,8)],'g-');
    plot([xImCart(1,8) xImCart(1,5)],[xImCart(2,8) xImCart(2,5)],'g-');
    
    plot([xImCart(1,1) xImCart(1,5)],[xImCart(2,1) xImCart(2,5)],'g-');
    plot([xImCart(1,2) xImCart(1,6)],[xImCart(2,2) xImCart(2,6)],'g-');
    plot([xImCart(1,3) xImCart(1,7)],[xImCart(2,3) xImCart(2,7)],'g-');
    plot([xImCart(1,4) xImCart(1,8)],[xImCart(2,4) xImCart(2,8)],'g-');
    %make sure we don't replace with next point
    %print('Ex_7part4_2','-depsc');
    








%==========================================================================
%==========================================================================

%goal of function is to estimate pose of plane relative to camera
%(extrinsic matrix) given points in image xImCart, points in world XCart
%and intrinsic matrix K.

function T = estimatePlanePose(xImCart,XCart,K)

%replace this
T = [];

%TO DO Convert Cartesian image points xImCart to homogeneous representation
%xImHom
xImHom = [xImCart; ones(1,size(xImCart,2))];

%TO DO Convert image co-ordinates xImHom to normalized camera coordinates
%xCamHom

xCamHom= K\xImHom;

%TO DO Estimate homography H mapping homogeneous (x,y)
%coordinates of positions in real world to xCamHom.  Use the routine you wrote for
%Practical 1B.
H = calcBestHomography(XCart,xCamHom);


%TO DO Estimate first two columns of rotation matrix R from the first two
%columns of H using the SVD

[U,S,V]= svd(H(:,1:2));
rot2= U *[1,0;0,1;0,0] * V';

%TO DO Estimate the third column of the rotation matrix by taking the cross
%product of the first two columns
rot3= cross(rot2(:,1), rot2(:,2));
R= [rot2, rot3];

%TO DO Check that the determinant of the rotation matrix is positive - if
%not then multiply last column by -1.
if (det(R) <0)
    R(:,3)= R(:,3) * (-1);
end
    
%TO DO Estimate the translation t by finding the appropriate scaling factor k
%and applying it to the third colulmn of H

scale= mean(mean(H(:,1:2)./rot2));
t= (H(:,3))./scale;

%TO DO Check whether t_z is negative - if it is then multiply t by -1 and
%the first two columns of R by -1.
if( t(3,1) <0)
    t= t* (-1);
    R(:,1:2)= R(:,1:2) * (-1);
end

%assemble transformation into matrix form
T  = [R t;0 0 0 1];




function H = calcBestHomography(pts1Cart, pts2Cart)

%should apply direct linear transform (DLT) algorithm to calculate best
%homography that maps the points in pts1Cart to their corresonding matchin in 
%pts2Cart

%****TO DO ****: replace this

A= zeros(10,9);
for i= 1: size(pts1Cart,2)
A(2*i-1,:)= [ 0,0, 0, -pts1Cart(1,i),-pts1Cart(2,i), -1,  pts2Cart(2,i) *pts1Cart(1,i), pts2Cart(2,i) *pts1Cart(2,i), pts2Cart(2,i)]; 
A(2*i,:)= [pts1Cart(1,i),pts1Cart(2,i),1,0, 0,0, -pts2Cart(1,i) *pts1Cart(1,i), -pts2Cart(1,i) *pts1Cart(2,i), -pts2Cart(1,i)];

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

%****TO DO **** Write this routine 



%QUESTIONS TO THINK ABOUT...

%Do the results look realistic?
%If not, then what factors do you think might be causing this?

