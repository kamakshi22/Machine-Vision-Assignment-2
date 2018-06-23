function HW2_TrackingAndHomographies


LLs = HW2_Practical9c( 'll' );
LRs = HW2_Practical9c( 'lr' );
ULs = HW2_Practical9c( 'ul' );
URs = HW2_Practical9c( 'ur' );

close all;

% Load frames from the whole video into Imgs{}.
% This is really wasteful of memory, but makes subsequent rendering faster.
LoadVideoFrames

% Coordinates of the known target object (a dark square on a plane) in 3D:
XCart = [-50 -50  50  50;...
    50 -50 -50  50;...
    0   0   0   0];

% These are some approximate intrinsics for this footage.
K = [640  0    320;...
    0    512  256;
    0    0    1];

% Define 3D points of wireframe object.
XWireFrameCart = [-50 -50  50  50 -50 -50  50  50;...
    50 -50 -50  50  50 -50 -50  50;...
    0   0   0   0 -100 -100 -100 -100];

hImg = figure;

% ================================================
for iFrame = 1:numFrames
    xImCart = [LLs(iFrame,:)' ULs(iFrame,:)' URs(iFrame,:)' LRs(iFrame,:)'];
    xImCart = circshift( xImCart, 1);
    
    % To get a frame from footage
    im = Imgs{iFrame};
    
    % Draw image and 2d points
    set(0,'CurrentFigure',hImg);
    set(gcf,'Color',[1 1 1]);
    imshow(im); axis off; axis image; hold on;
    plot(xImCart(1,:),xImCart(2,:),'r.','MarkerSize',15);
    
    
    %TO DO Use your routine to calculate TEst the extrinsic matrix relating the
    %plane position to the camera position.
    TEst = estimatePlanePose(xImCart, XCart, K);
    
    
    
    %TO DO Draw a wire frame cube, by projecting the vertices of a 3D cube
    %through the projective camera, and drawing lines betweeen the
    %resulting 2d image points
  
 
    
    XHom = [XWireFrameCart; ones(1,size(XWireFrameCart,2))];
    mul= TEst* XHom;
     K = [640, 0, 320,0;
     0,640,240,0;
     0, 0, 1,0];
    
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
    
    
    
    % TO DO: Draw a wire frame cube using data XWireFrameCart. You need to
    % 1) project the vertices of a 3D cube through the projective camera;
    % 2) draw lines betweeen the resulting 2d image points.
    % Note: CONDUCT YOUR CODE FOR DRAWING XWireFrameCart HERE
    
    hold off;
    drawnow;
    
%     pngFileName = sprintf( '%s_%.5d.eps', 'myOutput14', iFrame );
%     print( gcf, '-depsc', '-r80', pngFileName );
    
    
    %     Optional code to save out figure
         
end % End of loop over all frames.
% ================================================

% TO DO: QUESTIONS TO THINK ABOUT...

% Q: Do the results look realistic?
% If not then what factors do you think might be causing this


% TO DO: your routines for computing a homography and extracting a
% valid rotation and translation GO HERE. Tips:
%
% - you may define functions for T and H matrices respectively.
% - you may need to turn the points into homogeneous form before any other
% computation.
% - you may need to solve a linear system in Ah = 0 form. Write your own
% routines or using the MATLAB builtin function 'svd'.
% - you may apply the direct linear transform (DLT) algorithm to recover the
% best homography H.
% - you may explain what & why you did in the report.

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

A= zeros(8,9);
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


