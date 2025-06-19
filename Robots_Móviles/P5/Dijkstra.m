function [ruta,coste] = Dijkstra(G,Origen,Fin)
    % G: matriz de adyacencia NxN con los costes entre nodos
    % Origen: nodo de inicio
    % Fin: nodo de destino
    % ruta: vector con la secuencia de nodos del camino más corto
    % coste: coste total del camino más corto desde Origen hasta Fin

    n = size(G,1);               % Número de nodos del grafo (asumiendo G es cuadrada)
    dist = Inf(n,1);             % Vector con las distancias mínimas conocidas desde Origen a cada nodo
    nodo_ant = NaN(n,1);         % Vector para almacenar el nodo anterior en el camino más corto
    visitados = false(n,1);      % Vector lógico que marca si un nodo ha sido visitado

    dist(Origen) = 0;            % La distancia desde el Origen a sí mismo es 0

    for i = 1:n
        % Encuentra el nodo no visitado con la menor distancia conocida
        [~, u] = min(dist);

        % Marca este nodo como visitado
        visitados(u) = true;

        % Si se alcanza el nodo destino, se puede terminar el algoritmo
        if u == Fin
            break;
        end

        % Recorre todos los nodos vecinos del nodo actual
        for v = 1:n
            % Si hay conexión y el nodo no ha sido visitado
            if G(u,v) > 0 && ~visitados(v)
                alt = dist(u) + G(u,v); % Calcula la distancia alternativa
                if alt < dist(v)        % Si esta nueva distancia es menor
                    dist(v) = alt;      % Se actualiza la distancia
                    nodo_ant(v) = u;    % Y se guarda el nodo anterior
                end
            end
        end

        dist(u) = Inf;  % Se evita volver a seleccionar el mismo nodo
    end

    % Reconstrucción del camino más corto
    ruta = [];
    u = Fin;
    while ~isnan(nodo_ant(u))
        ruta = [u, ruta];  % Se inserta el nodo actual al principio de la ruta
        u = nodo_ant(u);   % Se retrocede al nodo anterior
    end
    if ~isempty(ruta)
        ruta = [Origen, ruta]; % Se añade el nodo origen al principio
    end

    % Evaluación del coste final
    if isinf(dist(Fin))
        coste = NaN;          % Si no hay camino, se devuelve NaN
    else
        coste = dist(Fin);    % De lo contrario, se devuelve el coste óptimo
    end
end
