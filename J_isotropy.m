function iso = J_isotropy(J)
condition = J_condition(J);
if condition == inf
    iso = inf;
else
    iso = sqrt(condition);
end