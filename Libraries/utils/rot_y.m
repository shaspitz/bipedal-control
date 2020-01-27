function R = rot_y( A )
% Returns a rotation matrix corresponding to rotation of A radians about y-axis

R = [ cos(A)  0 sin(A);
         0    1    0 ;
      -sin(A) 0 cos(A)];
       