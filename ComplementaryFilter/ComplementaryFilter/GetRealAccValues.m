function [R_AccReturn] = GetRealAccValues(R_Acc,X)
    
    w=[R_Acc(1,1) R_Acc(2,1) R_Acc(3,1) 1];
    
    R_AccReal=w*X;
    
    R_AccReturn=[R_AccReal(1,1);R_AccReal(1,2);R_AccReal(1,3)];
end

