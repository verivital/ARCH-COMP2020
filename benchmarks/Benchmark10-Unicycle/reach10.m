%% Reachability analysis of the Unicycle (benchmark 10)
% net = Load_nn('controllerB.onnx');
net = Load_nn('controllerB_nnv.mat');
controlPeriod = 0.2;
reachstep = 0.001;
plant = NonLinearODE(4,2,@dynamics10, reachstep, controlPeriod, eye(4));
% noise = Star(-0.0001, 0.0001);
plant.set_taylorTerms(2);
plant.set_zonotopeOrder(50);
% plant.set_polytopeOrder(20);
error = 0.01;
plant.options.maxError = [error; error; error; error];
tF = 10;
time = 0:controlPeriod:tF;
steps = length(time);
offset = 20;
offsetM = offset*ones(2,1);
scale_factor = 1;

%% Simulate system
% There in an offset in the outputs, cannot simulate it like this
% nncs = NonlinearNNCS(net,plant);
% [simTrace, controlTrace] = nncs.evaluate(controlPeriod,40,[9.5;-4.5;2.2;1.5],[]);
% figure;
% plot(simTrace(1,:),simTrace(2,:));

%% Reachability analysis
% Initial set
lb = [9.5; -4.5; 2.1; 1.5];
ub = [9.55; -4.45; 2.11; 1.51];
init_set = Star(lb,ub);
% Input set
lb = [0;0];
ub = [0;0];
input_set = Star(lb,ub);
% Store all reachable sets
reachAll = init_set;
% Execute reachabilty analysis
% for i =1:steps
for i=1:10
    % Compute plant reachable set
    init_set = plant.stepReachStar(init_set, input_set);
    reachAll = [reachAll init_set];
    % Compute controller output set
    input_set = net.reach(init_set,'approx-star');
    input_set = input_set.affineMap(eye(2),-offsetM);
end


f = figure;
Star.plotBoxes_2D_noFill(reachAll,1,2,'b');
grid;
title('Benchmark 10 reachable sets');
xlabel('x1');
ylabel('x2');

f1 = figure;
Star.plotBoxes_2D_noFill(plant.intermediate_reachSet,1,2,'b');
grid;
title('Benchmark 10 reachable sets');
xlabel('x1');
ylabel('x2');