%Connect to the INEMO device
[handle_dev pFD]=INEMO_Connection();
%End connection

acqSize=10;
R_Acc=zeros(acqSize*6,4);

%Order:
%1:+X 2:-X 3:+Y 4:-Y 5:+Z 6:-Z


y=zeros(acqSize*6,3);
k=1;

i=1;


[error4]=calllib('iNEMO2_SDK','INEMO2_Command',handle_dev,7);
while(k<=6)
    while(i<=acqSize*k)
        [errre pFD]=calllib('iNEMO2_SDK','INEMO2_GetDataSample',handle_dev,pFD);
        %----------
        pause(0.01)
        %---------
        R_Acc(i,1)=pFD.Accelerometer.X/1000;
        R_Acc(i,2)=pFD.Accelerometer.Y/1000;
        R_Acc(i,3)=pFD.Accelerometer.Z/1000;
        R_Acc(i,4)=1;
        switch k
            case 1
                y(i,1)=1;
            case 2
                y(i,1)=-1;
            case 3
                y(i,2)=1;
            case 4
                y(i,2)=-1;
            case 5
                y(i,3)=1;
            case 6
                y(i,3)=-1;
        end
        i=i+1;
    end
    INEMO_Disconnection(handle_dev);
    pause(25);
    INEMO_Connection();
    [error4]=calllib('iNEMO2_SDK','INEMO2_Command',handle_dev,7);
    k=k+1;
end
INEMO_Disconnection(handle_dev);
X=(R_Acc'*R_Acc)^(-1)*R_Acc'*y;

