

function T = chaslesMozzi(S, theta, h)
    % One Safety CheckPoint
    if norm(S) < 1e-12
        disp("Screw Axis passed to ChaslesMozzi might be zero vector")
    end
    % normalize if input is not:
    th = theta; 
    w = S(1:3);  
    R = Rot(w,th);  
    I = eye(3);  
    v= S(4:6);
    %R  % uncomment for a check on R
    if h == inf
        v = v/norm(v);
        T = cat(1, [I th.*v'], [0 0 0 1]); % pure translation
    else  
        w = w/norm(w); 
        wsk = skewSym(w);
        G = I*th + (1 - cos(th))*wsk + (th - sin(th)).*wsk*wsk;
        p = G*v'; % translation along screw
        T = cat(1, [R p], [0 0 0 1]);
    end
end