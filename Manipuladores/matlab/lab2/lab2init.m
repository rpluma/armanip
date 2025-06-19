clear all

q0 =  [0.785398; -0.785398];
qdot0 = [0; 0];
torques = [0; 0]; 
wrenches = [0; 0];


par = [3, 2, 1, 0.6, 5.0, 5.0, 9.81]; % 1=>base
runExperiment(1);
par = [6, 2, 1, 0.6, 5.0, 5.0, 9.81]; % 2=>m1=6;
runExperiment(2); 
par = [3, 4, 1, 0.6, 5.0, 5.0, 9.81]; % 3=>m2=4;
runExperiment(3);
par = [3, 2, 2, 0.6, 5.0, 5.0, 9.81]; % 4=>l1=2
runExperiment(4);
par = [3, 2, 1, 1.2, 5.0, 5.0, 9.81]; % 5=>l2=1.2
runExperiment(5);
par = [3, 2, 1, 0.6, 10.0, 5.0, 9.81]; % 6=>b1=10
runExperiment(6);
par = [3, 2, 1, 0.6, 5.0, 10.0, 9.81]; % 7=>b2=10
runExperiment(7);
par = [3, 2, 1, 0.6, 5.0, 5.0, 19.62]; % 8=>g=19.68
runExperiment(8);

