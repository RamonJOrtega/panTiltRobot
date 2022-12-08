clear all;
close all;

%Connect to the INEMO device
[handle_dev pFD]=INEMO_Connection();
%End connection

%VARIABLES INITIALIZATION
%acquisitions length - edit only this variable when the size change
acqSize=400;
%Gyroscope Offset 
GyroOffset=[-3.5617,-3.4097,-2.4889];
%Magnetometer compesation
MagnCompens=zeros(3,1);
%Flag that indicates starting point to estimation based also on magnetometer
FlagMagn=zeros(3,1);

%wx,wy,wz
GyroRate=zeros(3,acqSize);

%R_Acc_X,R_Acc_Y,R_Acc_Z
R_Acc=zeros(3,acqSize);
R_AccF=zeros(3,acqSize);
%R_Magn_X,R_Magn_Y,R_Magn_Z
R_Magn=zeros(3,acqSize);
R_MagnF=zeros(3,acqSize);
%Roll,Pitch,Yaw
AnglesEuler=zeros(3,acqSize);
AnglesGyro=zeros(3,acqSize);
AnglesEulerGyro=zeros(3,acqSize);
AnglesEulerGyroFiltered=zeros(3,acqSize);
AnglesFiltered=zeros(3,acqSize);
%LP filter coefficients and related length
[bAcc,aAcc] = butter(3,0.0075,'low');
[bMagn,aMagn] = butter(2,0.06,'low');
%Accelerometer calibration matrix
Load=load('Calibration.mat');
X=Load.X;



magnF_Length=13;
accF_Length=13;

%Flag that indicates wich axis is parallel to the k versor
AxisParallel=zeros(3,1);

ExtAcceleration=0;
OldGyroFilteredRate=0;

t=[0];
i=1;
PsiMagnOld=0;
while(i<=acqSize)
    %Calculate time variation
    if(i>1)
        dt = toc(t0);
        t=[t t(length(t))+dt];
    end
    %-----------ACQUISITION AND VARIABLES SET UP--------------
    [errre pFD]=calllib('iNEMO2_SDK','INEMO2_GetDataSample',handle_dev,pFD);
    t0 = tic;
    %----------
    pause(0.01)
    %---------
    
    R_Acc(1,i)=pFD.Accelerometer.X;
    R_Acc(2,i)=pFD.Accelerometer.Y;
    R_Acc(3,i)=pFD.Accelerometer.Z;
    
    R_Acc(:,i)=GetRealAccValues(R_Acc(:,i),X);
    
    %External acceleration flag
    if(sqrt((R_Acc(1,i)/1000)^2+(R_Acc(2,i)/1000)^2+(R_Acc(3,i)/1000)^2)>1.3)
        ExtAcceleration=1;
    else
        ExtAcceleration=0;
    end
    
    R_Acc(:,i)=MyNorm(R_Acc(:,i));
    %normalization after filtering
    if(i<=accF_Length)
        R_AccF(:,i)=MyFilter(bAcc,aAcc,R_Acc(:,:));
    else
        R_AccF(:,i)=MyFilter(bAcc,aAcc,R_Acc(:,i-accF_Length:i));
    end
    %Accelerometer filtered normalization
    R_AccF(:,i)=MyNorm(R_AccF(:,i));
    
    %Magnetometer acquisition
    R_Magn(1,i)=pFD.Magnetometer.X;
    R_Magn(2,i)=pFD.Magnetometer.Y;
    R_Magn(3,i)=pFD.Magnetometer.Z;
    R_Magn(:,i)=MyNorm(R_Magn(:,i));
    if(i<=magnF_Length)
        R_MagnF(:,i)=MyFilter(bMagn,aMagn,R_Magn(:,:));
    else
        R_MagnF(:,i)=MyFilter(bMagn,aMagn,R_Magn(:,i-magnF_Length:i));
    end
    R_MagnF(:,i)=MyNorm(R_MagnF(:,i));
    
    %Gyroscope acquisition and substraction of offset
    GyroRate(1,i)=pFD.Gyroscope.X-GyroOffset(1);
    GyroRate(2,i)=pFD.Gyroscope.Y-GyroOffset(2);
    GyroRate(3,i)=-pFD.Gyroscope.Z+GyroOffset(3);
    
    %-----------END ACQUISITION AND VARIABLES SET UP---------------
    
    %-----------FIRST STEP: WAIT FOR FILTERS STABILIZATION-----
    if(i<accF_Length+5)
        %----Assuming that the IMU is with z axis perpendicular to terrain
        %and Yaw rotation doesn't exist
        AnglesEuler(:,i)=ComputeAnglesByMagnAcc(R_Acc(:,i),R_Magn(:,i));
        AnglesFiltered(:,i)=AnglesEuler(:,i);
        AnglesGyro(:,i)=AnglesEuler(:,i);
        AnglesEulerGyro(:,i)=AnglesEuler(:,i);
        AnglesEulerGyroFiltered(:,i)=AnglesEuler(:,i);
    else
    %-----------FILTERS STABILIZED-----------------------------
    
    %----START EXPERIMENT
        
        %switch on the IMU led
        if(i==accF_Length+5)
                [erroristrtr]=calllib('iNEMO2_SDK','INEMO2_Command',handle_dev,7);
        end
    
        %Compute Angles by mean of the gyroscope
        AnglesGyro(:,i)=GyroIntegration(AnglesGyro(:,i-1),GyroRate(:,i),GyroRate(:,i-1),dt);
        %Euler transformation from gyro
        dEulerGyro=ComputeEulerFromGyro(GyroRate(:,i), AnglesEulerGyro(:,i-1))*dt;
        AnglesEulerGyro(:,i)=AnglesEulerGyro(:,i-1)+dEulerGyro;
        dEulerGyroFiltered=ComputeEulerFromGyro(GyroRate(:,i), AnglesFiltered(:,i-1))*dt;
        AnglesEulerGyroFiltered(:,i)=AnglesFiltered(:,i-1)+dEulerGyro;
        
        AnglesEulerGyro(1,i)=Abs_180_Reduction(AnglesEulerGyro(1,i));
        AnglesEulerGyro(2,i)=Abs_180_Reduction(AnglesEulerGyro(2,i));
        AnglesEulerGyro(3,i)=Abs_180_Reduction(AnglesEulerGyro(3,i));
        
        %Observations computing
        [AnglesEuler(:,i)]=ComputeAnglesByMagnAcc(R_AccF(:,i),R_MagnF(:,i));
        %Filter updating
        [AnglesEuler(:,i) AnglesEulerGyroFiltered(:,i)]=AnglesSetUp(AnglesEuler(:,i), AnglesEulerGyroFiltered(:,i));
        AnglesFiltered(:,i)=0.98*AnglesEulerGyroFiltered(:,i)+0.02*AnglesEuler(:,i);
         
        AnglesFiltered(1,i)=Abs_180_Reduction(AnglesFiltered(1,i));
        AnglesFiltered(2,i)=Abs_180_Reduction(AnglesFiltered(2,i));
        AnglesFiltered(3,i)=Abs_180_Reduction(AnglesFiltered(3,i));
        
        AnglesEulerGyroFiltered(1,i)=Abs_180_Reduction(AnglesEulerGyroFiltered(1,i));
        AnglesEulerGyroFiltered(2,i)=Abs_180_Reduction(AnglesEulerGyroFiltered(2,i));
        AnglesEulerGyroFiltered(3,i)=Abs_180_Reduction(AnglesEulerGyroFiltered(3,i));
        
        AnglesGyro(1,i)=Abs_180_Reduction(AnglesGyro(1,i));
        AnglesGyro(2,i)=Abs_180_Reduction(AnglesGyro(2,i));
        AnglesGyro(3,i)=Abs_180_Reduction(AnglesGyro(3,i));
    end
    
    i=i+1;
end

INEMO_Disconnection(handle_dev);

%figure;
%    subplot(3,1,1);plot(t,R_Acc(1,:),'b',t,R_AccF(1,:),'r');legend('X Acc', 'X Acc Filt');grid;
%   subplot(3,1,2);plot(t,R_Acc(2,:),'b',t,R_AccF(2,:),'r');legend('Y Acc', 'Y Acc Filt');grid;
%   subplot(3,1,3);plot(t,R_Acc(3,:),'b',t,R_AccF(3,:),'r');legend('Z Acc', 'Z Acc Filt');grid;
    
%figure;
%    subplot(3,1,1);plot(t,R_Magn(1,:),'b',t,R_MagnF(1,:),'r');legend('X Magn', 'X Magn Filt');grid;
 %   subplot(3,1,2);plot(t,R_Magn(2,:),'b',t,R_MagnF(2,:),'r');legend('Y MAgn', 'Y Magn Filt');grid;
  %  subplot(3,1,3);plot(t,R_Magn(3,:),'b',t,R_MagnF(3,:),'r');legend('Z Magn', 'Z Magn Filt');grid;

figure;
    subplot(3,1,1);plot(t,AnglesGyro(1,:),'r',t,AnglesEulerGyro(1,:),'b');legend('Roll Gyro','Roll Euler Gyro');grid;xlabel('time (sec)');ylabel('angle (deg)');
    subplot(3,1,2);plot(t,AnglesGyro(2,:),'r',t,AnglesEulerGyro(2,:),'b');legend('Pitch Gyro','Pitch Euler Gyro');grid;xlabel('time (sec)');ylabel('angle (deg)');
    subplot(3,1,3);plot(t,AnglesGyro(3,:),'r',t,AnglesEulerGyro(3,:),'b');legend('Yaw Gyro','Yaw Euler Gyro');grid;xlabel('time (sec)');ylabel('angle (deg)');
   
figure;
    subplot(3,1,1);plot(t,AnglesEuler(1,:),'b',t,AnglesEulerGyro(1,:),'r',t,AnglesFiltered(1,:),'k',t,AnglesEulerGyroFiltered(1,:),'c');legend('Euler','Euler Gyro','Euler Filtered','Euler Gyro Filt');grid;
    subplot(3,1,2);plot(t,AnglesEuler(2,:),'b',t,AnglesEulerGyro(2,:),'r',t,AnglesFiltered(2,:),'k',t,AnglesEulerGyroFiltered(2,:),'c');legend('Euler','Euler Gyro','Euler Filtered','Euler Gyro Filt');grid;
    subplot(3,1,3);plot(t,AnglesEuler(3,:),'b',t,AnglesEulerGyro(3,:),'r',t,AnglesFiltered(3,:),'k',t,AnglesEulerGyroFiltered(3,:),'c');legend('Euler','Euler Gyro','Euler Filtered','Euler Gyro Filt');grid;