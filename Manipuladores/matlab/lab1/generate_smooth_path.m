function [P, Q]=generate_smooth_path(P0, P1, P2, tau, T, t)
    % Function that calculates the transformation 
    % (P - position, and Q - orientation) from P1 to P3 
    % smoothing in P2 with Taylor method (quaternions)

    if (t<-T || t>T)
        % Out of allowed range
        disp('Parameter t out of range');

    elseif (t<=-tau) % First segment (lineal)
        [P, Q] = qpinter(P0, P1, (t+T)/T);
    
    elseif (t>=tau) % Third segment (lineal)
        [P, Q] = qpinter(P1, P2, t/T);
    
    else % Second segment (smoothing)

        % time factor
        tfactori = (tau-t)^2/(4 * tau * T);
        tfactord = (tau+t)^2/(4 * tau * T);
        
        % Position interpolation
        p0 = P0(1:3, 4);
        p1 = P1(1:3, 4);
        p2 = P2(1:3, 4);
        DeltaP0 = p1 - p0;
        DeltaP2 = p1 - p2;
        P = p1 - tfactori .* DeltaP0 - tfactord .* DeltaP2;
        %P = p1 + tfactor .* DeltaP1 - tfactor .* DeltaP2;
        
        % Orientation interpolation
        Q0 = tr2q(P0);
        Q1 = tr2q(P1);
        Q2 = tr2q(P2);
        Q01 = qqmul(qinv(Q0), Q1);
        Q12 = qqmul(qinv(Q1), Q2);
        w01 = Q01(1);
        w12 = Q12(1);
        v01 = Q01(2:4);
        v12 = Q12(2:4);
        theta01 = 2 * acos(w01);
        theta12 = 2 * acos(w12);
        n01 = v01 ./ sin(theta01/2);
        n12 = v12 ./ sin(theta12/2);
        thetak1 = -tfactori * theta01;
        thetak2 = +tfactord * theta12;
        Qk1 = [cos(thetak1/2), n01 .* sin(thetak1/2)];
        Qk2 = [cos(thetak2/2), n12 .* sin(thetak2/2)];
        Q = qqmul(Q1, qqmul(Qk1, Qk2));
        
    end
end