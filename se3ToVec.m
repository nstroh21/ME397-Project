function vector = se3ToVec(A)
% Returns a column vector
A
wsk = A(1:3,1:3)
w1 = wsk(3,2); w2 = wsk(1,3); w3 = wsk(2,1)
v = A(1:3,4);
vector = [w1, w2, w3, v']'
end