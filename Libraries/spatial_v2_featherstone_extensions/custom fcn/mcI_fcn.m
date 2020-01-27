% Function to convert mass, com, inertia to spatial inertial
function I_sp = mcI_fcn(m, c_xyz, c_rpy, I)
    if(norm(c_rpy) > eps)
        disp('Error in mcI_fcn') ; %keyboard ;
    end
    
    I_sp = mcI(m, c_xyz, I) ;
end