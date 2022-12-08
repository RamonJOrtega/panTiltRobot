function [AnglesEuler,AnglesGyroFilt] = AnglesSetUp(AnglesEuler,AnglesGyroFilt)
    for(j=1:3)
        if((180-abs(AnglesEuler(j,1)))<15 && (180-abs(AnglesGyroFilt(j,1))<15))
            if(sign(AnglesEuler(j,1))~=sign(AnglesGyroFilt(j,1)))
                AnglesGyroFilt(j,1)=sign(AnglesEuler(j,1))*(360-(abs(AnglesGyroFilt(j,1))));
            end
        end
    end


end

