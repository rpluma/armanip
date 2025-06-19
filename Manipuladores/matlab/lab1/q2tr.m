% function T=q2tr(q)
%
% Calcula la matriz homogénea T de dimensiones 4x4 correspondiente
% al cuaternio q.
%
% Víctor F. Muñoz 2000

function T=q2tr(q)

S=q(1); X=q(2); Y=q(3); Z=q(4);
T=[1-2*Y^2-2*Z^2    2*X*Y-2*S*Z     2*X*Z+2*S*Y   0
    2*X*Y+2*S*Z    1-2*X^2-2*Z^2    2*Y*Z-2*S*X   0
    2*X*Z-2*S*Y     2*Y*Z+2*S*X   1-2*X^2-2*Y^2   0
          0              0               0        1];
