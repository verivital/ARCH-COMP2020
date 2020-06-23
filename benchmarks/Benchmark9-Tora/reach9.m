%% Reachability analysis of TORA (benchmark 9)
% Load components and set reachability parameters
net = Load_nn('controllerTora.onnx');
reachStep = 0.001;
controlPeriod = 1;
plant = NonLinearODE(4,1,@dynamics9, reachStep, controlPeriod, eye(4));
% noise = Star(-0.0001, 0.0001);
plant.set_taylorTerms(5);
plant.set_zonotopeOrder(200);
% plant.set_polytopeOrder(5);% error = 0.001;
error = 0.1;
plant.options.maxError = [error; error; error; error];
time = 0:controlPeriod:20;
steps = length(time);
% Initial set
lb = [0.6; -0.7; -0.4; 0.5];
ub = [0.7; -0.6; -0.3; 0.6];
% ub = [0.61; -0.69; -0.39; 0.51];
% init_set = Box(lb,ub);
offset = 10;
scale_factor = 1;

%% Simulate system
% nncs = NonlinearNNCS(net,plant);
% [sT,simTrace, controlTrace, sx0,srfi] = nncs.sample(controlPeriod,40, init_set,[],20);
% figure;
% hold on;
% for i=1:length(simTrace)
%     traj = simTrace{i};
%     plot(traj(1,:),traj(2,:),'r');
% end

%% Reachability analysis
init_set = Star(lb,ub);
% Input set
lb = 0;
ub = 0;
input_set = Star(lb,ub);
% Store all reachable sets
reachAll = init_set;
% Execute reachabilty analysis
% for i =1:steps
t = tic;
for i =1:4
    % Compute plant reachable set
    init_set = plant.stepReachStar(init_set, input_set);
    reachAll = [reachAll init_set];
    % Compute controller output set
    input_set = net.reach(init_set,'approx-star');
    input_set = input_set.affineMap(1,-offset);
end
disp(' ');
toc(t);

f = figure;
Star.plotBoxes_2D_noFill(reachAll,1,2,'b');
grid;
title('Reachable set dimensions 1 and 2')
xlabel('x1');
ylabel('x2');

f2 = figure;
Star.plotBoxes_2D_noFill(plant.intermediate_reachSet,1,2,'m');
grid;
title('All Reachable sets for dimensions 1 and 2')
xlabel('x3');
ylabel('x4');