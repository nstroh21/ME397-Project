function [w,theta] = rot2AxisAngle(R)  % matrix logarithm
    if isRot(R) == false
        w = "undefined"; theta = 0;
        disp('Input to function was not a valid rotation matrix')
    else
        I = eye(3); t = trace(R);
        %disp(R) % Comment out if cluttering 
        if (R == I) theta = 0; w = "undefined";  % omega is arbitrary in this case (meaningless)
        elseif (t == -1) 
            theta = pi; 
            % 3 cases, choose the one where we do not divide by 0
            if  ( 1/(sqrt(2*(1+R(1,1)))) ~= Inf ) % Case 1
                alpha = 1/(sqrt(2*(1+R(1,1))));
                w = alpha.*[1+R(1,1), R(2,1), R(3,1)];
                %disp("here is case 1")
            elseif ( 1/(sqrt(2*(1+R(2,2)))) ~= Inf )  % Case 2
                alpha = 1/(sqrt(2*(1+R(2,2)))); 
                w = alpha.*[R(1,2), 1+R(2,2), R(3,2)];
                %disp("here is case 2 ")
            else                                        % Case 3
                alpha = 1/(sqrt(2*(1+R(3,3))));
                w = alpha .* [R(1,3), R(2,3), 1+R(3,3)];
            end
        else
            theta = acos(.5*(t-1));
            w_hat = 1/(2*sin(theta)).* (R - R');
            norm(w_hat);
            %disp("norm is here");   % Debug line
            w = [R(3,2) , R(1,3) , R(2,1)];
        end
    end
end