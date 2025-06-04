% function T=zyz2tr(a)
%
% Conversión del vector fila 'a' de angulos de Euler ZYZ a
% transformación homogénea 'T' de 4x4
%
% Víctor F. Muñoz 2000

function T=zyz2tr(a)

s=sin(a); c=cos(a);
T=[c(1)*c(2)*c(3)-s(1)*s(3)  -c(1)*c(2)*s(3)-s(1)*c(3)  c(1)*s(2)  0
   s(1)*c(2)*c(3)+c(1)*s(3)  -s(1)*c(2)*s(3)+c(1)*c(3)  s(1)*s(2)  0
       -s(2)*c(3)                   s(2)*s(3)             c(2)     0
            0                           0                   0      1];