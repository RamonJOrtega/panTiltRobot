function [ AnglesGyro] = GyroIntegration(OldAnglesGyro, GyroRate,OldGyroRate, dt)
    %Compute average rotational speed
    GyroRate(1,1) = (GyroRate(1,1)*0.5 + OldGyroRate(1,1)*0.5);
    GyroRate(2,1) = (GyroRate(2,1)*0.5 + OldGyroRate(2,1)*0.5);
    GyroRate(3,1)= (GyroRate(3,1)*0.5 + OldGyroRate(3,1)*0.5);
    %Compute gyro integration on the time variation
    AnglesGyro(1,1)=OldAnglesGyro(1,1)+GyroRate(1,1)*dt;
    AnglesGyro(2,1)=OldAnglesGyro(2,1)+GyroRate(2,1)*dt;
    AnglesGyro(3,1)=OldAnglesGyro(3,1)+GyroRate(3,1)*dt;
    %Avoid rotation over (-180,180]
    %AnglesGyro(1,1)=Abs_180_Reduction(AnglesGyro(1,1));
    %AnglesGyro(2,1)=Abs_180_Reduction(AnglesGyro(2,1));
    %AnglesGyro(3,1)=Abs_180_Reduction(AnglesGyro(3,1));
end

