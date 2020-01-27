function  vcross = crm3( v )

% crm3  spatial cross-product operator (motion).
% crm3(r) calculates the 3x3 matrix such that the expression crm(v)*w is the
% cross product of the vectors v and w.

vcross = [  0    -v(3)  v(2) ;
	    v(3)  0    -v(1);
	   -v(2)  v(1)  0  ;
	 ];
