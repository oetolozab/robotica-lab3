function [xrec,vrec,arec] = poly3(ic,t)
%     x0 = ic(1);
%     v0 = ic(2);
%     xf = ic(3);
%     vf = ic(4);

    t0 = t(1);
    tf = t(length(t));

    trec = t';

    M = [t0^3    t0^2    t0    1;
         3*t0^2  2*t0    1     0;
         tf^3    tf^2    tf    1;
         3*tf^2  2*tf    1     0];

    coef = M\ic';

    xrec = coef(1)*trec.^3 + coef(2)*trec.^2 + coef(3)*trec + coef(4);
    vrec = 3*coef(1)*trec.^2 + 2*coef(2)*trec + coef(3);
    arec = 6*coef(1)*trec + 2*coef(2);
end