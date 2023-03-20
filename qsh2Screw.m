function S = qsh2Screw(q,s,h)  % be sure to keep track of pitch as well
    if (length(q) ~= 3 | length(s) ~= 3 | length(h) ~= 1)
        disp('dimensins of input may be incorrect');
        S = [0 0 1 0 0 0]
    else
        if isUnit(s) == false
            disp('s hat was not a unit vector, angular velocity is assumed to be:')
            disp(norm(s))
        end
        if h == inf
            v = s/norm(s);
            w = [0 0 0];
            S = [w v];
        else
            v =  (skewSym(-1.*s)*q')' + (h.*s)
            %disp((skewSym(-1.*s)*q')')
            %disp((h.*s))
            % more fun to use skew symmetric but cross gives same
            w = s ;  %v = cross(s,q) + (h.*s)
            S = [w v]    %mod = norm(S)  %sanity check
            S = S./norm(w)
        end
    end
end