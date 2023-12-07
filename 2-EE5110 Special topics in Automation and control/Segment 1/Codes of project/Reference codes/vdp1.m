% function dydt = vdp1(t,y,A, B, u)
%     q = u;
%     dydt = A*y + B*q;
%     %dydt =  -a*y(1)*y(2)*t;
% end

function dydt = vdp1(t,y)

    dydt = [y(2); (1-y(1)^2)*y(2)-y(1)];
    %dydt =  -a*y(1)*y(2)*t;
end