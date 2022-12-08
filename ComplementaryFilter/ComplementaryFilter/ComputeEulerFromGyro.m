function [dAngles] = ComputeEulerFromGyro(GyroRate, Angles)
    M=[0 sind(Angles(1,1)) cosd(Angles(1,1));0 cosd(Angles(1,1))*cosd(Angles(2,1)) -sind(Angles(1,1))*cosd(Angles(2,1));cosd(Angles(2,1)) sind(Angles(1,1))*sind(Angles(2,1)) cosd(Angles(1,1))*sind(Angles(2,1))];
    dAngles=(1/cosd(Angles(2,1)))*M*GyroRate;
    dAngles1=dAngles;
    dAngles(1,1)=dAngles1(3,1);
    dAngles(3,1)=dAngles1(1,1);
end

