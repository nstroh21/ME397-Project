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