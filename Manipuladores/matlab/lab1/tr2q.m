% function q=tr2q(T,m)
%
% Realiza la conversión a cuaternio q de la matriz homogénea T. Las dos
% soluciones se eligen mediante el parámetro m. El signo de  m elige la
% solución positiva o la negativa. Si m se omite se toma la positiva por defecto.
%
% Víctor F. Muñoz 2000

function q=tr2q(T,m)

if nargin==1, m=1; end
M=sign(m);
S=M*sqrt(T(1,1)+T(2,2)+T(3,3)+1)/2;
if abs(S)>1e-3,
    X=(T(3,2)-T(2,3))/4/S;
    Y=(T(1,3)-T(3,1))/4/S;
    Z=(T(2,1)-T(1,2))/4/S;
else
    S=0;
    X=M*sqrt((T(1,1)+1)/2);
    Y=M*sqrt((T(2,2)+1)/2);
    Z=M*sqrt((T(3,3)+1)/2);
end
q=[S,X,Y,Z];