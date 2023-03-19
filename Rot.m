function R = Rot(w, theta)
    if (size(w) ~= 3) | (isa(w,"double") == false)
        disp("Error: input axis was not a 3-dim vector. Returning Identity")
        R = eye(3); 
    end
    if (isUnit(w) == false) 
        w = (1/norm(w)).*w;
        theta = norm(w)*theta;
    end
    hat = skewSym(w); % Just apply Rodrigues Formula:
    R = eye(3) + sin(theta).*hat + (1-cos(theta)).*(hat*hat); 
end