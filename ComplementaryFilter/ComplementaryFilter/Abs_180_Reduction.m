function [Angle] = Abs_180_Reduction(Angle)
    %Reduce the angle between the (-180,180] interval
    if(Angle>=0)
        Angle=mod(Angle,360);
    else
        Angle=mod(Angle,-360);
    end
    if(Angle>180)
        Angle=Angle-360;
    else
        if(Angle<=-180)
            Angle=Angle+360;
        end
    end
end

