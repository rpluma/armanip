function [llegada] = nav_autonoma(bRobusta, bAestrella)

%% PARAMETROS DE CAMPOS POTENCIALES
%addpath("../P4");
v = 0.4;        % Velocidad del robot
alfa = 100;     % Constante de atracción
beta = 100;     % Constante de repulsión
D = 4;          % Radio de influencia de obstáculos
MAX_ITER = 1000; % Máximo de iteraciones intentando llegar al destino
llegada = true; % Por defecto indicamos que el robot llega a su destino

%% SENSOR LIDAR CONFIGURADO
max_rango = 10;
angulos = -pi/2:(pi/180):pi/2;


%% CARGA DEL MAPA Y DATOS
mapa = imread("mapa2.pgm");
map_bin = imbinarize(imcomplement(mapa));  % Obstáculos a negro, espacio libre blanco
m = binaryOccupancyMap(map_bin);
mapa2;  % Carga 'nodos' y 'costes'

%% PETICIÓN DE ORIGEN Y DESTINO
origen = input("Introduce el nodo de origen: ");
destino = input("Introduce el nodo de destino: ");

%% PLANIFICACION GLOBAL CON DIJKSTRA

if bAestrella % Se calcula la ruta óptima mediante el algoritmo de A*
    n = length(nodos);
    costes2 = zeros(n);
    heur = zeros(n);
    for f = 1: n 
        for c = 1:n 
            dist = norm(nodos(f, 2:3)-nodos(c, 2:3));
            heur(f, c) = dist;
            if costes(f, c) > 0
                costes2(f, c) = dist;
            end
        end  
    end
    [coste, camino] = aestrella(costes2, heur, origen, destino)


else % Se calcula la ruta óptima mediante el algoritmo de Dijkstra
    [coste, camino] = Dijkstra(costes, origen, destino);  
end

%% VISUALIZACION DEL ENTORNO
figure;
show(m); hold on;
plot(nodos(:,2), nodos(:,3), 'bo');
plot(nodos(origen,2), nodos(origen,3), 'go', 'MarkerFaceColor', 'g');
plot(nodos(destino,2), nodos(destino,3), 'ro', 'MarkerFaceColor', 'r');
title(sprintf("Ruta: %s%0.0f [Coste=%0.0f]", ...
    sprintf("%0.0f, ", camino(1:end-1)), destino, coste));
axis equal; xlim([0 150]); ylim([0 150]);



%% MOVIMIENTO ENTRE CADA PAR DE NODOS CON CAMPOS POTENCIALES
for k = 2:length(camino)
    origen_pos = nodos(camino(k-1),2:3);
    destino_pos = nodos(camino(k),2:3);
    robot = [origen_pos, 0];
    path = robot;
    iter = 0;

    while norm(destino_pos - robot(1:2)) > 0.5 && iter < MAX_ITER
        % Fuerza atractiva que empuja al robot hacia el nodo destino local
        Fattr = alfa * (destino_pos - robot(1:2));

        % Se simula un sensor LIDAR que detecta obstáculos cercanos en el entorno
        obs = SimulaLidar(robot, m, angulos, max_rango);
        Frep = [0, 0];

        % Fuerza repulsiva que evita que el robot se acerque demasiado a los obstáculos detectados
        for i = 1:size(obs, 1)
            if ~isnan(obs(i,1))
                d = norm(robot(1:2) - obs(i,:));
                if d < D
                    u = (robot(1:2) - obs(i,:)) / d;
                    Frep = Frep + beta * (1/d - 1/D) * u / d^2;
                end
            end
        end

        % --- Memoria de posiciones visitadas ---
        % Guardamos todas las posiciones recorridas por el robot para generar una fuerza repulsiva
        % adicional que lo empuje fuera de zonas ya visitadas (evita estancamientos)
        if ~exist('historial','var')
            historial = [];
        end
        historial = [historial; robot(1:2)];

        % Inicializa la fuerza de memoria. Por cada punto del historial, si el robot está demasiado cerca,
        % se le aplica una fuerza que lo aleja proporcionalmente (inversa al cuadrado de la distancia)
        Fmem = [0, 0];
        for h = 1:size(historial,1)
            d_mem = norm(robot(1:2) - historial(h,:));
            if d_mem < 5 && d_mem > 0.1
                Fmem = Fmem + 20 * (robot(1:2) - historial(h,:)) / (d_mem^2);
            end
        end

        % Fuerza total resultante que combina atracción al objetivo, repulsión de obstáculos y memoria de posiciones pasadas
        F = Fattr + Frep + Fmem * bRobusta;
        if norm(F) == 0
            break;
        end
        F = F / norm(F);

        % Movimiento
        robot(1) = robot(1) + v * cos(atan2(F(2), F(1)));
        robot(2) = robot(2) + v * sin(atan2(F(2), F(1)));
        path = [path; robot];

        % Se dibuja la trayectoria recorrida por el robot en cada paso del bucle
        plot(path(:,1), path(:,2), 'r-', 'LineWidth', 1);
        drawnow;
        iter = iter + 1;
    end % while del siguiente punto

    termina_bien = iter < MAX_ITER;
    if iter >= MAX_ITER
        llegada = false;
        disp('No se pudo completar el camino');
        return;
    end    
end % for del camino
disp('Simulación completada con éxito');
end

%% FUNCION DE SIMULACION LIDAR
function obs = SimulaLidar(robot, mapa, angulos, max_rango)
    obs = rayIntersection(mapa, robot, angulos, max_rango);
end