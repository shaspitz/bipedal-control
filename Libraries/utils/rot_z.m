function R = rot_z( A )
% Returns a rotation matrix corresponding to rotation of A radians about z-axis

R = [ cos(A) -sin(A) 0;
      sin(A) cos(A)  0;
         0      0    1];