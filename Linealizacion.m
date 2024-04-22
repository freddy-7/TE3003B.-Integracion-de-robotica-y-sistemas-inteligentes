%Variables simbólicas
syms w1 w2 theta x y v_x v_y real
%Parámetros
b=0.5; % separación del centro del eje de tracción a las ruedas
e=1; %separación del punto de control al centro del eje
R=0.25; %radio de la rueda

%%Modelo cinemático
v=(w1*R + w2*R)/2;       %velocidad lineal
w=(w2*R -w1*R)/(2*b);    %velocidad angular

%%Ecuaciones de movimiento del robot 
dxdt=v*cos(theta);
dydt=v*sin(theta);
dthetadt=w;

%%Obtengo la representación de las ecuaciones de estado
estado= [x y theta]'; entradas = [w1 ; w2];
destadodt= [dxdt; dydt; dthetadt];

%Las ecuaciones de salida serán la posición 2D del punto descentrado
xp=x+e*cos(theta);
yp=y+e*sin(theta);
EcSalida= [xp;yp]

%Las derivadas de las salidas respecto al tiempo son:
dsalidadt= jacobian (EcSalida, estado)*destadodt

%dsalidadt es una función que devuelve las "velocidades del punto descentrado"
%en función de las velocidades de las ruedas y orientación"
%El objetivo es invertirla para tener una función que devuelva las
%"Velocidades de las ruedas" en función de las "velocidades del punto
% descentrado y su orientación"

%Podemos igualarlas a los valores deseados, e intentar resolver 
%para despejar w1 y w2

sol= solve(dsalidadt==[v_x ; v_y],{w1,w2});

simplify(sol.w1)
simplify(sol.w2)