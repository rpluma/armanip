function [v, c]= prueba(Ganancia, LA, camino, poseG)
    
    % Comprobar si llegamos al final del camino y parar el robot
    persistent paso;
    if isempty(paso)
        paso = 1;
    end
    if paso >= length(camino)
        c = 0;
        v = 0;
    else
        % Cálculo de distancia hasta siguiente punto del camino
        deltaX = camino(paso, 1) - poseG(1);
        deltaY = camino(paso, 2) - poseG(2);
        dist = sqrt(deltaX^2 + deltaY^2);

        % Si la distancia es menor que el look ahead, cambiar de paso
        if (dist < LA)
            paso = paso + 1;
        end

        % Calcular la velocidad y curvatura
        a1 = poseG(3);
        TRot = [cos(a1), sin(a1);-sin(a1), cos(a1)];
        pos_p = TRot * [deltaX; deltaY];
        a2 = atan2(pos_p(2), pos_p(1));

        % Devolver las consignas de velocidad y curvatura
        v = 1.2; % Velocidad lineal del motor (metros/segundo)
        c = a2*Ganancia
    end