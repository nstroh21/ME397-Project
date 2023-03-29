classdef hw1
    % Functions from hw 1 could all be stored here ... maybe break down
    % into more classes after 

    properties
        Property1
    end

    methods (Static)
        function obj = hw1(obj)
            obj.Property1 = true;
            disp("object made")
        end
        function S = getScrew(q,s,h)  % be sure to keep track of pitch as well
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

        %%%% Given a screw axis and pitch , return Homogenous T %%%%%%%%%%%%%
        % since it is private, we will assume a valid screw axis is passed w/o checks%%%
        % 2 cases depending on pitch: pure rotation or case with translation
        function T = chaslesMozzi(S, theta, h)
            th = theta; w = S(1:3);  
            R = Rot(w,th);  I = eye(3);  v= S(4:6);
            %R  % uncomment for a check on R
            if h == inf
                v
                v = v/norm(v)
                T = cat(1, [I th.*v'], [0 0 0 1]); % pure translation
            else  
                w = w/norm(w); wsk = skewSym(w);
                norm(w)
                disp('norm')
                G = I*th + (1 - cos(th))*wsk + ...
                            (th - sin(th)).*wsk*wsk;
                p = G*v'; % translation along screw
                T = cat(1, [R p], [0 0 0 1]);
            end
        end
        
        function thetas = parse(theta)
            step = theta/4;
            for i = 0:3
                thetas(i+1) = step +i*step;
            end
        end
        
        function fig = plotFrame(T1)
            p = T1(:,4);
            c1 = T1(:,1); c1 = c1/norm(c1);
            c2 = T1(:,2); c2 = c2/norm(c2);
            c3 = T1(:,3); c3 = c3/norm(c3);
            figure;
            quiver3(p(1), p(2), p(3), c1(1), c1(2), c1(3) );
            hold on;
            quiver3(p(1), p(2), p(3), c2(1), c2(2), c2(3) );
            quiver3(p(1), p(2), p(3), c3(1), c3(2), c3(3) );
            hold off; %zlim([0,5]), xlim([0,5]), ylim([0,5])
            fig = figure;
        end
        
        function T_inv = invertSE3(T)
            R = T(1:3, 1:3); p = T(1:3 , 4);
            pinv = (-R')*p;
            T_inv = cat(1, [R' pinv], [0 0 0 1]);
        end
        
        function [theta, S] = se3Logarithm(T)
            R = T(1:3, 1:3)
            p = T(1:3, 4)
            I = eye(3);
            if R == I
                w = [0 0 0];
                v = (p/norm(p))';
                theta = norm(p)
                S = [w v]
            else
                [w, theta] = rot2AxisAngle(R);
                norm(w)
                w
                disp("norm")
                wsk = skewSym(w);
                G_inv = 1/theta*I - (1/2)*wsk + (1/theta - 1/2*cot(theta/2)).*(wsk*wsk);
                v = G_inv*p
                S = [w v']
                theta
            end
        end

        % ============= Q1:  Rot Matrix to Other ================ %
        function [w,theta] = rot2AxisAngle(R)  % matrix logarithm
            if isRot(R) == false
                w = "undefined"; theta = 0;
                disp('Input to function was not a valid rotation matrix')
            else
                I = eye(3); t = trace(R);
                disp(R) % Comment out if cluttering 
                if (R == I) theta = 0; w = "undefined";  % omega is arbitrary in this case (meaningless)
                elseif (t == -1) 
                    theta = pi; 
                    % 3 cases, choose the one where we do not divide by 0
                    if  ( 1/(sqrt(2*(1+R(1,1)))) ~= Inf ) % Case 1
                        alpha = 1/(sqrt(2*(1+R(1,1))))
                        w = alpha.*[1+R(1,1), R(2,1), R(3,1)];
                        disp("here 1")
                    elseif ( 1/(sqrt(2*(1+R(2,2)))) ~= Inf )  % Case 2
                        alpha = 1/(sqrt(2*(1+R(2,2)))); 
                        w = alpha.*[R(1,2), 1+R(2,2), R(3,2)];
                        %disp("sing is here 2 ")
                    else                                        % Case 3
                        alpha = 1/(sqrt(2*(1+R(3,3))));
                        w = alpha .* [R(1,3), R(2,3), 1+R(3,3)];
                    end
                else
                    theta = acos(.5*(t-1));
                    w_hat = 1/(2*sin(theta)).* (R - R');
                    norm(w_hat)
                    disp("norm is here")
                    w = [R(3,2) , R(1,3) , R(2,1)];
                end
            end
        end

        function isRot = isRot(R)
            r0 = R(1); r1 = R(2); r2 = R(3);
            i = [1,0,0]'; j = [0,1,0]'; k = [0,0,1]';
            check = dot(r0,r1) + dot(r0, r2) + dot(r1,r2); 
            if (det(R) ~= 1) isRot = false; disp('determinant')
            elseif (check > abs(1e-13)) isRot = false; disp('orthogonal') % numerical error possible, careful
            else isRot = true;
            end
        end

        %=================== Rotation Matrix to Quaternion Representation (Unique)=======%
        % Hard-code formulas given in class
        function Q = rot2Quat(R)
            Q = [1,0,0,0]; R  % display only
            if isRot(R) == false
                    disp('Input to function was not a valid rotation matrix');
            else
                Q(1) = sqrt(R(1,1) + R(2,2) + R(3,3) + 1)/2;
                Q(2) = sign(R(3,2) - R(2,3))*sqrt(R(1,1) - R(2,2) - R(3,3) + 1)/2;
                Q(3) = sign(R(1,3) - R(3,1))*sqrt(R(2,2) - R(3,3) - R(1,1) + 1)/2;
                Q(4) = sign(R(2,1) - R(1,2))*sqrt(R(3,3) - R(1,1) - R(2,2) + 1)/2;
            end
        end

        % Another way is to convert to axis-angle first and then to quaternion
        function Q = rot2Quat2(R)
            theta, w = Rot(R);
            Q = zeros(4); Q(1) = cos(theta)/2 ; Q(2:4) = sin(theta/2).*w
        end


        %=================== Rotation Matrices to Euler Angles ============================%
        % formulas come from comparisons ... watch out for singularities ... some other conditions if that happens
        % phi is the first rotation around z axis of the fixed frame ign{s} (body lines up with fixed)
        % theta is the 2nd rotation around y-axis of the body frame {b}
        % psi is the 3rd rotation about z-axis again but of body frame {b}

        %=================== Rotation Matrices to Euler Angles ============================%
        function [phi,theta, psi] = rot2Zyz(R)
            
            if isRot(R) == false
                    phi = 0; theta = 0; psi = 0;
                    disp('Input to function was not a valid rotation matrix');
            else
                % we don't use arc-cosine because it is important we get both value and quadrant correct
                if (R(2,3)~= 0) && (R(1,3) ~= 0) phi = atan2(R(2,3), R(1,3)); 
                else "idk -- use quaternion instead?"; end
                
                if  (R(3,3)~= 0) theta = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
                else theta = pi/2; end % but couldn't it also be - pi/2 ?
                
                if (R(3,1) ~= 0 ) 
                    psi = atan2(R(3,2), -R(3,1)); % also what about the 0 numerator ? what does atan2 choose?
                else "given theta is nonzero it implies"; psi = pi/2; end  % or -pi/2
            end
        end


        % Roll is about z-axis of the fixed frame {s}, Pitch is about y-axis of the fixed frame {s}
        % and Yaw is about z-axis of the fixed frame {s}
        function roll, pitch, yaw = ZYXRot(R)
            %r11,r21, r31, r32, r33 = R(1,1),  R(2,1),  R(3,1), R(3,2), R(3,3)
            if (R(1,1) ~= 0)
                roll = atan2(R(2,1), R(1,1))
            else
                thing = "idk exactly"
            end
            if (R(3,3) ~= 0)
                yaw = atan2(R(3,2), R(3,3));
                if (R(3,2) ~= 0)
                    pitch = atan2(-R(3,1), sqrt( R(3,2)^2 + R(3,3)^2 ));
                end
            end
        end

        % ========= Q2:  Other Back to Rot Mat =================================== %
        
        %================ Axis Angle to Rotation Matrix ============================%
        % note that this is unique in the forward direction, pass in w as a vector,
        % theta as a scalar. Returns unique SO3 matrix
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


        %================ Quaternions to Rotation Matrices =============================%
        % this way is hard-coding, here we are choosing 0 to pi
        function R = quat2Rot(obj,Q)
          if isa(Q, 'quaternion') % we can accept either format of quaternion
               [q0,q1,q2,q3] = parts(Q); 
               Q = [q0,q1,q2,q3];
          else 
              q0 = Q(1); q1 = Q(2); q2 = Q(3); q3 = Q(4);  
          end
          if (isUnit(Q) == false) % it must be a unit quaternion or else return err message and identity
            disp("Error: input must be a unit quaternion. Returning Identity")
            R = eye(3); 
          else
            R = [[(q0^2 + q1^2 - q2^2 - q3^2), 2*(q1*q2 - q0*q3)           ,          2*(q0*q2 + q1*q3) ]
                 [2*(q0*q3 + q1*q2)          , (q0^2 - q1^2 + q2^2 - q3^2) ,          2*(q2*q3 - q0*q1) ]
                 [2*(q1*q3 - q0*q2)          , 2*(q0*q1 + q2*q3)           , (q0^2 - q1^2 - q2^2 + q3^2)] ];
          end 
        end

        % this way ought to work as well, there is a double-cover. Here we are
        % choosing 0 to pi
        function R = quat2Rot2(Q) 
            if isa(Q, 'quaternion') % we can accept either format of quaternion
                [q0, q1,q2,q3] = parts(Q); 
            else 
                q0 = Q(1); q1 = Q(2); q2 = Q(3); q3 = Q(4);  
            end
            
            if (isUnit(Q) == false | (size(Q) ~= 4))
                disp("You must input a unit quaternion into this function")
                R = eye(3); 
            else
                theta = 2*acos(Q(1));
                if (theta == 0) w = [0,0,0]; else w = (1/sin(theta/2)).* Q(2:4); end
                R = Rot(w, theta);
            end
        end

        function Q = quatMult(p,q)
            q0 = q(1); q = q(2:4); p0 = p(1); p = p(2:4);
            vec = cross(p,q) + q0.*p  + p0.*q;
            Q = [q0*p0 - dot(p,q), vec];
        end
        
        function tf = isUnit(Q)  % have to be very precise since there is rounding error
            if abs(norm(Q) - 1) < 1e-15  % floating point precision (32 bit)
                tf = true;
            else 
                tf = false;
            end
        end


        %====================== Helper Functions ==============================%
        
        % always assume 3 dimensional vector input
        function w_hat = skewSym(w)
            w_hat = [[0    ,-w(3),  w(2)], 
                     [w(3) , 0   , -w(1)], 
                     [-w(2), w(1),  0   ]] ;
        end
        
        function s = sign(obj,x)
            if (x < 0) s = -1; else s = 1; end  % note: if x 0 then s is +1
        end

        % elementary basis rotations
        function Rz = RotZ(obj, theta)
            Rz = [ [cos(theta) -sin(theta) 0];
                   [sin(theta)  cos(theta) 0];
                   [0           0          1] ]
        end
        function Ry = RotY(obj,theta)
            Ry = [ [cos(theta)  0  sin(theta)];
                   [0           1           0];
                   [-sin(theta) 0  cos(theta)]]
        end
        function Rx = RotX(obj,theta)
            Rx = [ [1       0                    0];
                   [0      cos(theta)  -sin(theta)];
                   [0      sin(theta)   cos(theta)]]
        end

   end
end