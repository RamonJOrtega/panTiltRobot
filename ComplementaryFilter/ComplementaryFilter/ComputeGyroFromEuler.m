function [ dGyro] = ComputeGyroFromEuler(Angles,dAngles)
    a=sind(Angles(1,1));
    b=cosd(Angles(1,1));
    c=sind(Angles(2,1));
    d=cosd(Angles(2,1));
    
    k=d^(-1);
    
    dGyro(3,1)=(dAngles(3,1)-k*a/b*(dAngles(2,1)))/(k*b+k*a^2/b);
    
    dGyro(2,1)=(dAngles(2,1)+a*dGyro(3,1))/b;
    
    dGyro(1,1)=dAngles(1,1)-k*a*c*dGyro(2,1)-b*dGyro(3,1);
end

