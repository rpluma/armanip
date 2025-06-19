% function q=qqmul(q1,q2)
%
% Multiplicación de cuaternios. 'q' es el cuaternio resultado de multiplicar
% q1 por q2. Tanto q1 como q2 deben ser dos vectores filas de cuatro componentes.
% El resultando tambi�n tendr� el mismo formato.

function q=qqmul(q1,q2)

q=[q1(1)*q2(1)-q1(2:4)*q2(2:4)', ...
   q2(1)*q1(2:4)+q1(1)*q2(2:4)+cross(q1(2:4),q2(2:4))];