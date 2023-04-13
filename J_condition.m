function condition = J_condition(J)
[vecs1, eigs] = eig(J*J');
eigs = diag(eigs);
if min(abs(eigs)) < 1e-11
    condition = inf;
else
    condition = max(abs(eigs))/min(abs(eigs));
end