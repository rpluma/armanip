function [pr,qr]=qpinter(Pa,Pb,lambda)
    % Interpolate the position (fórmula 2)
    pr = (Pa(1:3, 4)+(Pb(1:3,4)-Pa(1:3,4)).*lambda);

    % Interpolate the orientation 
    Qa = tr2q(Pa);
    Qb = tr2q(Pb);
    Qc = qqmul(qinv(Qa), Qb);
    w = Qc(1);
    v = Qc(2:4);
    theta = 2*acos(w);
    n = v ./ sin(theta/2);
    lambda_theta = lambda * theta;
    wrot = cos(lambda_theta / 2);
    vrot = n .* sin(lambda_theta / 2);
    qrot = [wrot vrot];
    qr =  qqmul(Qa, qrot);    
end
