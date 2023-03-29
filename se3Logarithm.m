function [theta, S] = se3Logarithm(T)
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    I = eye(3);
    if R == I
        w = [0 0 0];
        v = (p/norm(p))';
        theta = norm(p);
        S = [w v];
    else
        [w, theta] = rot2AxisAngle(R);
        %disp(norm(w)); disp(w);                        
        wsk = skewSym(w);
        G_inv = 1/theta*I - (1/2)*wsk + (1/theta - 1/2*cot(theta/2)).*(wsk*wsk);
        v = G_inv*p;
        S = [w v'];
        %disp(theta);
    end
end