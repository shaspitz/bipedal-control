% rotz  spatial coordinate transform (arbitrary-axis rotation).
function  X = rot_axis( axis, theta )

c = cos(theta);
s = sin(theta);
t = 1 - c ;
ux = axis(1) ;
uy = axis(2) ;
uz = axis(3) ;

X = zeros(6) ;
X(1:3,1:3) = [t*ux^2+c    t*ux*uy-s*uz    t*ux*uz+s*uy ;
              t*ux*uy+s*uz   t*uy^2+c     t*uy*uz-s*ux ;
              t*ux*uz-s*uy   t*uy*uz+s*ux   t*uz^2+c] ;
          
X(4:6,4:6) = X(1:3,1:3) ;

end