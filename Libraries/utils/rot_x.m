function R = rot_x( A )
% Returns a rotation matrix corresponding to rotation of A radians about x-axis

R = [   1      0         0  ;
        0    cos(A)  -sin(A);
        0    sin(A)   cos(A)];
       
       