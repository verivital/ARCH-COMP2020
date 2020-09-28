%% Reachability analysis of Airplane Benchmark

%% load the controller
net = Load_nn('controller_airplane.mat');

% Specify the reach step, has to be smaller than the control period
reachStep = 0.001;
%% specify the control period as specified by the benchmark description
controlPeriod = 0.1;

% define the plant as specified by nnv
plant = NonLinearODE(12,6,@dynamics, reachStep, controlPeriod, eye(12));
plant.set_taylorTerms(10)
plant.set_zonotopeOrder(10);

%% Reachability analysis
% Initial set
lb = [0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0];
% ub = [0.0;0.0;0.0;1.0;1.0;1.0;1.0;1.0;1.0;0.0;0.0;0.0];
ub = [0.0;0.0;0.0;0.01;0.01;0.01;0.01;0.01;0.01;0.0;0.0;0.0];
% ub = [0.0;0.0;0.0;0.001;0.001;0.001;0.001;0.001;0.001;0.0;0.0;0.0];
init_set = Star(lb,ub);
% Input set
lb = [0;0;0;0;0;0];
ub = [0;0;0;0;0;0];
input_set = Star(lb,ub);
% Store all reachable sets
reachAll = init_set;
% Execute reachabilty analysis
num_steps = 13;
t = tic;
for i=1:num_steps
    % Compute plant reachable set
    init_set = plant.stepReachStar(init_set(1),input_set(1));
%     init_set = plantReach(plant,init_set,input_set);
    reachAll = [reachAll init_set];
    % Compute controller output set
    input_set = net.reach(init_set,'approx-star');
end
timing = toc(t);
% save('../../results/airplaneReach.mat','-v7.3');
%% Visualize results
t = tic;
f1 = figure;
Star.plotBoxes_2D_noFill(plant.intermediate_reachSet,1,4,'b');
grid;hold on;
Star.plotBoxes_2D_noFill(reachAll,1,4,'m');
title('Airplane x_1 vs. x_4');
xlabel('x');
ylabel('u');

f2 = figure;
Star.plotBoxes_2D_noFill(reachAll,2,5,'b');
grid;hold on;
Star.plotBoxes_2D_noFill(reachAll,2,5,'m');
plot([-0.5 -0.5],[-2 2],'r');
plot([0.5 0.5],[-2 2],'r');
title('Airplane x_2 vs x_5');
xlabel('y');
ylabel('v');
% saveas(f2,'../../results/reachAirplane_plot2vs5_cP.jpg');

f3 = figure;
Star.plotBoxes_2D_noFill(plant.intermediate_reachSet,3,6,'b');
grid;hold on;
Star.plotBoxes_2D_noFill(reachAll,3,6,'m');
title('Airplane');
xlabel('z');
ylabel('w');

f4 = figure;
Star.plotBoxes_2D_noFill(plant.intermediate_reachSet,7,10,'b');
grid;hold on;
Star.plotBoxes_2D_noFill(reachAll,7,10,'m');
plot([-1 -1],[-0.2 0.2],'r');
plot([1 1],[-0.2 0.2],'r');
title('Airplane x_7 vs. x_{10}');
xlabel('x_7');
ylabel('x_{10}');
% saveas(f4,'../../results/reachAirplane_plot7vs10.jpg');

f5 = figure;
Star.plotBoxes_2D_noFill(plant.intermediate_reachSet,8,11,'b');
grid;hold on;
Star.plotBoxes_2D_noFill(reachAll,8,11,'m');
plot([-1 -1],[-0.2 0.2],'r');
plot([1 1],[-0.2 0.2],'r');
title('Airplane x_8 vs. x_{11}');
xlabel('x_8');
ylabel('x_{11}');
% saveas(f5,'../../results/reachAirplane_plotvs11.jpg');

f6 = figure;
Star.plotBoxes_2D_noFill(plant.intermediate_reachSet,9,12,'b');
grid;hold on;
Star.plotBoxes_2D_noFill(reachAll,9,12,'m');
plot([-1 -1],[-0.2 0.2],'r');
plot([1 1],[-0.2 0.2],'r');
title('Airplane x_9 vs. x_{12}');
xlabel('x_9');
ylabel('x_{12}');
% saveas(f6,'../../results/reachAirplane_plot9vs12.jpg');
toc(t);

%% Helper function
function init_set = plantReach(plant,init_set,input_set)
    nS = length(init_set);
    nL = length(input_set);
    ss = [];
    for k=1:nS
        for l=1:nL
            ss =[ss plant.stepReachStar(init_set(k), input_set(l))];
        end
    end
    init_set = ss;
end