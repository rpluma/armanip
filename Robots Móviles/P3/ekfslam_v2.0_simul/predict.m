function predict (v,g,Q,WB,dt) % included odom (by Ricardo Vázquez)
%function predict (v,g,Q,WB,dt)
%
% Inputs:
%   v, g - control inputs: velocity and gamma (steer angle)
%   Q - covariance matrix for velocity and gamma
%   WB - vehicle wheelbase
%   dt - timestep
%
% Outputs: 
%   odom - store vehicle odometry (by Ricardo Vázquez) - (global variable)
%   XX, PX - predicted state and covariance (global variables)
%
% Modified by Ricado Vazquez-Martin (2020/03/30) - EKFLOC version, changes:
%   - map part of the state is only used to store the map, not for estimate
%
% Tim Bailey 2004.
global XX PX odom % odom as global (by Ricardo Vázquez)

s= sin(g+XX(3)); c= cos(g+XX(3));
vts= v*dt*s; vtc= v*dt*c;

% PON AQUI TU CODIGO
% calcular e incluir los jacobianos de prediccion (vehiculo)
% jacobians   
Gv= [1             0          -vts;
     0             1           vtc;
     0             0           1   ];
Gu= [dt*c          -vts;
     dt*s           vtc;
    (dt*sin(g)/WB  v*dt*cos(g))/WB];
  
% predict covariance
PX(1:3,1:3)= Gv*PX(1:3,1:3)*Gv' + Gu*Q*Gu';
if size(PX,1)>3
    PX(1:3,4:end)= Gv*PX(1:3,4:end);
    PX(4:end,1:3)= PX(1:3,4:end)';
end    

% predict state
XX(1:3)= [XX(1) + vtc; 
          XX(2) + vts;
         pi_to_pi(XX(3)+ v*dt*sin(g)/WB)];
     
% store vehicle odometry (by Ricardo Vázquez)
odom = [odom(1) + vtc; 
          odom(2) + vts;
         pi_to_pi(odom(3)+ v*dt*sin(g)/WB)];
     