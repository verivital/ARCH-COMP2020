%% Reachability analysis of Double Pendulum Benchmark

%% load the controller
net = Load_nn('controller_airplane.mat');

% Specify the reach step, has to be smaller than the control period
reachStep = 0.1;
%% specify the control period as specified by the benchmark description
controlPeriod = 0.05;

% define the plant as specified by nnv
plant = NonLinearODE(12,6,@dynamics, reachStep, controlPeriod, eye(12));
%plant.set_zonotopeOrder(50);
% plant.set_polytopeOrder(20);
%error = 0.01;
%plant.options.maxError = [error; error;error;error];

%% Reachability analysis
% Initial set
lb = [0.0;0.0;0.0;0.0;0.0; 0.0;0.0;0.0;0.0;0.0;0.0;0.0];
ub = [0.0;0.0;0.0;0.0;0.0; 0.0;1.0;1.0;1.0;1.0;1.0;1.0];
init_set = Star(lb,ub);
% Input set
lb = [0;0;0;0;0;0];
ub = [0;0;0;0;0;0];
input_set = Star(lb,ub);
% Store all reachable sets
reachAll = init_set;
% Execute reachabilty analysis
% for i =1:steps
num_steps =10;
for i=1:num_steps
    % Compute plant reachable set
    init_set = plant.stepReachStar(init_set, input_set);
    reachAll = [reachAll init_set];
    % Compute controller output set
    input_set = net.reach(init_set,'approx-star');
end


%times = reachStep:reachStep:(num_steps*controlPeriod);
%Star.plotRanges_2D(plant.intermediate_reachSet,1,times,'r')

% times = 0:controlPeriod:(num_steps*controlPeriod);
% Star.plotRanges_2D(reachAll,1,times,'r')
% 
% 
% f = figure;
% Star.plotBoxes_2D_noFill(reachAll,1,2,'b');
% grid;
% title('Single Pendulum reachable sets');
% xlabel('x1');
% ylabel('x2');
% 
% f1 = figure;
% Star.plotBoxes_2D_noFill(plant.intermediate_reachSet,1,2,'b');
% grid;
% title('Single Pendulum reachable sets');
% xlabel('x1');
% ylabel('x2');