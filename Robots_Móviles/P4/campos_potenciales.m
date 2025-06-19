%% Configuración inicial
factorescala=1;
mapa=imread('mapa1_hr_laplaciana150.png');   % Mapa de obstáculos
mapa_show=imread('mapa1_150.png');           % Mapa para mostrar en pantalla
mapa=imresize(mapa,factorescala);
mapa=binarize(mapa)*255;

i = [137 15];        % Coordenadas de inicio
d = [15 137];        % Coordenadas de destino
v = 1;               % Velocidad del robot

alfa = 0.01;         % Coeficiente de la componente de atracción
beta = 0.1;          % Coeficiente de la componente de repulsión
D = 10;              % Rango del efecto del campo de repulsión de los obstáculos

[y,x] = find(mapa==255);
obs = [x,y];
n_obs = length(obs);

%% Inicialización
figure
imshow(mapa_show,'InitialMagnification','fit'); % Muestra el mapa de obstáculos
hold on                                          % Para dibujar encima del mapa de obstáculos
i = ginput(1);
d = ginput(1);
r = i;

plot(i(1), i(2), 'go');     % Dibuja el inicio
plot(d(1), d(2), 'ro');     % Dibuja el destino

path = [];
path = [path; r];           % Añade la posición actual del robot

%% Cálculo de la trayectoria
while norm(d-r) > v         % Mientras la distancia del robot (r) al destino (d) sea mayor de lo que se puede mover

    % Fuerza de atracción
    f_a = alfa*(d - r);

    % Fuerza de repulsión
    f_r = 0;
    if n_obs ~= 0
        for i=1:n_obs
            dist_o = norm(r-obs(i,:));
            if dist_o <= D
                f_r = f_r + beta * ((1/dist_o)-1/D)*(r - obs(i,:))/(dist_o^3);
            end
        end
    end

    f_res = f_a + f_r;
    f_res = f_res/norm(f_res);

    r = r + v*f_res;

    path = [path; r];                    % Añade la posición actual del robot
    plot(path(:,1),path(:,2),'r');      % Dibuja la posición del robot encima del mapa de obstáculos
    drawnow                              % Fuerza a que se redibuje en tiempo real

end
