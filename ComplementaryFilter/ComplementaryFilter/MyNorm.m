function [VectorNorm] = MyNorm(Vector)
    %Normalize the Vector param and return a new vector
    Norm=norm(Vector);
    VectorNorm=Vector(:,1)/Norm;   
end