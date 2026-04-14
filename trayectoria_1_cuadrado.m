clear
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 20;             % Tiempo de simulacion en segundos (s)
ts = 0.1;           % Tiempo de muestreo en segundos (s)
t = 0: ts: tf;       % Vector de tiempo
N = length(t);       % Muestras

u = zeros(1,N); 
w = zeros(1,N);
%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1 = zeros(1,N+1);  % Posición en eje x
y1 = zeros(1,N+1);  % Posición en eje y
phi = zeros(1, N+1); % Orientacion del robot (rad)

x1(1) = 0;    
y1(1) = 0;   
phi(1) = pi/2;   

%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUNTO DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%
hx = zeros(1, N+1);  
hy = zeros(1, N+1);  
hx(1) = x1(1); 
hy(1) = y1(1); 

%%%%%%%%%%%%%%%%%%%%%% VELOCIDADES DE REFERENCIA %%%%%%%%%%%%%%%%%%%%%%%%%%
tramo = floor(N/4); 

for i = 1:4
    inicio = (i-1)*tramo + 1;
    fin = i*tramo;
    
    % En cada tramo, el robot avanza a 1 m/s
    u(inicio:fin) = 1; 
    
    % Al final de cada tramo (excepto el último), damos un "pulso" de giro
    % Para girar 90 grados (pi/2) instantáneamente en este modelo:
    if fin < N
        w(fin) = -(pi/2) / ts; 
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N 
    
    % --- MODELO CINEMÁTICO (EULER) ---
    % Usamos la orientación actual phi(k) para calcular el siguiente paso
    xp1 = u(k)*cos(phi(k)); 
    yp1 = u(k)*sin(phi(k));
    
    % Actualizamos posiciones
    x1(k+1) = x1(k) + xp1*ts; 
    y1(k+1) = y1(k) + yp1*ts; 
    
    % Actualizamos la orientación para el siguiente paso k+1
    phi(k+1) = phi(k) + w(k)*ts; 
    
    % Guardamos la trayectoria en el punto de control
    hx(k+1) = x1(k+1); 
    hy(k+1) = y1(k+1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% a) Configuracion de escena
scene = figure;  
set(scene,'Color','white'); 
set(gca,'FontWeight','bold');
sizeScreen = get(0,'ScreenSize'); 
set(scene,'position',sizeScreen); 
camlight('headlight'); 
axis equal; 
grid on; 
box on; 
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); 
view([25 25]); 
axis([-3 10 -3 10 0 2]); 

% b) Cargar y graficar robot inicial
scale = 4;
MobileRobot_5; % Carga los datos del robot
H1 = MobilePlot_4(x1(1),y1(1),phi(1),scale); hold on;

% c) Inicializar Trayectoria
H2 = plot3(hx(1),hy(1),0,'r','lineWidth',2);

% d) Bucle de animación (EL CAMBIO ESTÁ AQUÍ)
step = 1; 
for k=1:step:N
    % Borramos la versión anterior del robot para que no deje "fantasmas"
    if ishandle(H1)
        delete(H1);
    end
    
    % Dibujamos el robot en la posición actual k
    H1 = MobilePlot_4(x1(k), y1(k), phi(k), scale);
    
    % Actualizamos la línea de la trayectoria (desde el inicio hasta k)
    set(H2, 'XData', hx(1:k), 'YData', hy(1:k), 'ZData', zeros(1,k));
    
    % Forzamos el refresco de pantalla
    drawnow; 
    pause(ts);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas de Velocidades %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph = figure;  
set(graph,'position',sizeScreen); 
subplot(211)
plot(t,u,'b','LineWidth',2), grid('on'), xlabel('Tiempo [s]'), ylabel('u (m/s)'), legend('Vel. Lineal');
subplot(212)
plot(t,w,'r','LineWidth',2), grid('on'), xlabel('Tiempo [s]'), ylabel('w (rad/s)'), legend('Vel. Angular');