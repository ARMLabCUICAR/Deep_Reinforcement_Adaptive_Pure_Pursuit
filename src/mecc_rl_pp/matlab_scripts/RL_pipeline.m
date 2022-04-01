rosshutdown;clear;clc;clear global;
ipaddress = "http://localhost:11311";
rosinit(ipaddress)

%% Load Rosbag and Pure Pursuit

% bag = rosbag('/home/ajoglek/husky_ws/src/huskynick/bags/waypoints_2022-03-16-18-14-25.bag');
% msgs = readMessages(bag);
% r=size(msgs,1);
% path = zeros(r,2);
% for i = 1:r
%     x = msgs{i,1}.Pose.Pose.Position.X;
%     y = msgs{i,1}.Pose.Pose.Position.Y;
%     path(i,1) = x;
%     path(i,2) = y;
% end
path = load ('/home/ajoglek/Downloads/updated_rectangle_path.mat');
path = double(path.path); 
path = [path;path;path];

% path = [path;path;path;path];

% controller = controllerPurePursuit;
% controller.Waypoints = path;
% controller.DesiredLinearVelocity = 0.5;
% controller.MaxAngularVelocity = 0.3;
% controller.LookaheadDistance = 0.2;

%% Visualization marker for lookahead
[viz_pub,viz_msg] = rospublisher('/visualization_marker',"visualization_msgs/Marker");
point = rosmessage('geometry_msgs/Point');
point.X = 1.0;
point.Y = 2.0;
viz_msg.Header.FrameId = 'map';
viz_msg.Type = 8;
viz_msg.Action = 0;
viz_msg.Scale.X = 0.2;
viz_msg.Scale.Y = 0.2;
viz_msg.Points = point;
color_msg = rosmessage('std_msgs/ColorRGBA');
color_msg.B = 1.0;
color_msg.A = 1.0;
viz_msg.Colors = color_msg;

%% RL setup
%Observations space
ObservationInfo = rlNumericSpec([8 1]);
%                   "LowerLimit",zeros(2,1),...
%                   "UpperLimit",ones(2,1)*10);
ObservationInfo.Name = 'pose X, pose Y,eul Z, imu_vel_z,cross_track_error,lookahead_x,lookahead_y,lin_vel';
%X, Y, Vel X, Vel Y, cross_track_error, imu z vel,
% ObservationInfo.Description = 'r, l';

numActions = 2;
% Action space
ActionInfo = rlNumericSpec([numActions 1],...
    "LowerLimit",[0.2 ;0.1],...
    "UpperLimit",[1.5 ;2]);
ActionInfo.Description = 'forward vel,lookahead_dist';


%Publishers and subscribers
[vel_pub,vel_msg] = rospublisher("/husky_velocity_controller/cmd_vel", "geometry_msgs/Twist");
pose_sub = rossubscriber('/amcl_pose');
imu_sub = rossubscriber('/imu/data','DataFormat','struct');
odom_sub = rossubscriber('husky_velocity_controller/odom','DataFormat','struct');

%Rosservice setup
% Reset model_state service
model_state_client = rossvcclient('/gazebo/set_model_state');
model_state_req = rosmessage(model_state_client);
model_state_req.ModelState.ModelName = 'husky';
model_state_req.ModelState.ReferenceFrame = 'map';

%Reset pose service
pose_reset_client = rossvcclient('/reset_caller');
pose_reset_req = rosmessage(pose_reset_client);

%Pause physics
gazebo_pause_client = rossvcclient('/gazebo/pause_physics');
gazebo_pause_req = rosmessage(gazebo_pause_client);

%Unpause physics
gazebo_unpause_client = rossvcclient('/gazebo/unpause_physics');
gazebo_unpause_req = rosmessage(gazebo_unpause_client);


% Environment and ROS paramters
envConstants.SampleTime = 0.01;
envConstants.PauseTime = 0.2;
envConstants.Cross_Track_Threshold = 0.8;
envConstants.Dist_to_Goal_Threshold = 0.1;
envConstants.vel_pub = vel_pub;
envConstants.vel_msg = vel_msg;
envConstants.pose_sub = pose_sub;
envConstants.dist_covered = 0;
envConstants.is_training = 1; % 1 for training mode; 0 for testing
envConstants.save_data = 0;
envConstants.model_state_client = model_state_client;
envConstants.model_state_req = model_state_req;
envConstants.pose_reset_client = pose_reset_client;
envConstants.pose_reset_req = pose_reset_req;
envConstants.gazebo_pause_client = gazebo_pause_client;
envConstants.gazebo_pause_req = gazebo_pause_req;
envConstants.gazebo_unpause_client = gazebo_unpause_client;
envConstants.gazebo_unpause_req = gazebo_unpause_req;
envConstants.Path = path;
envConstants.controller = controllerPurePursuit;
envConstants.controller.Waypoints = envConstants.Path;
envConstants.controller.DesiredLinearVelocity = 0.5;
envConstants.controller.MaxAngularVelocity = 0.8;
envConstants.controller.LookaheadDistance = 0.2;
envConstants.robotGoal = path(end,:);
envConstants.viz_pub = viz_pub;
envConstants.viz_msg = viz_msg;
envConstants.imu_sub = imu_sub;
envConstants.odom_sub = odom_sub;

disp('Entering loop')
StepHandle = @(Action,LoggedSignals) myStepFunctionCustom(Action,LoggedSignals,envConstants);
ResetHandle = @() myResetFunctionCustom(envConstants);
% [InitialObservation,LoggedSignals] = myResetFunction(envConstants);
env = rlFunctionEnv(ObservationInfo,ActionInfo,StepHandle,ResetHandle);
% validateEnvironment(env)


%% Critic
L = 100; % number of neurons

statePath = [
    featureInputLayer(8,'Normalization','none','Name','observation')
    fullyConnectedLayer(L,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(L,'Name','fc2')
    additionLayer(2,'Name','add')
    reluLayer('Name','relu2')
    fullyConnectedLayer(L,'Name','fc3')
    reluLayer('Name','relu3')
    fullyConnectedLayer(1,'Name','fc4')];

actionPath = [
    featureInputLayer(2,'Normalization','none','Name','action')
    fullyConnectedLayer(L,'Name','fc5')];

criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork,actionPath);    
criticNetwork = connectLayers(criticNetwork,'fc5','add/in2');
criticOptions = rlRepresentationOptions('LearnRate',1e-3,'GradientThreshold',1,'L2RegularizationFactor',1e-4);
critic = rlQValueRepresentation(criticNetwork,ObservationInfo,ActionInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);

%% Actor

scale = [0.15 0.95]';
bias = [0.35 1.05 ]';
actorNetwork = [
    featureInputLayer(8,'Normalization','none','Name','observation')
    fullyConnectedLayer(L,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(L,'Name','fc2')
    reluLayer('Name','relu2')
    fullyConnectedLayer(L,'Name','fc3')
    reluLayer('Name','relu3')
    fullyConnectedLayer(2,'Name','fc4')
    tanhLayer('Name','tanh1')
    scalingLayer('Name','ActorScaling1','Scale',scale,'Bias',bias)];
actorOptions = rlRepresentationOptions('LearnRate',1e-3,'GradientThreshold',1,'L2RegularizationFactor',1e-4);
actor = rlDeterministicActorRepresentation(actorNetwork,ObservationInfo,ActionInfo,'Observation',{'observation'},'Action',{'ActorScaling1'},actorOptions);

%% Agent

agentOptions = rlTD3AgentOptions(...
    'SampleTime',envConstants.SampleTime,...
    'TargetSmoothFactor',1e-3,...
    'ExperienceBufferLength',1e6,...
    'DiscountFactor',0.99,...
    'MiniBatchSize',64);
% agentOptions.NoiseOptions.StandardDeviation = 1e-1;
% agentOptions.NoiseOptions.StandardDeviationDecayRate = 1e-3;

agent = rlTD3Agent(actor,critic,agentOptions);
maxepisodes = 50;
maxsteps = 1500;
trainingOpts = rlTrainingOptions('MaxEpisodes',maxepisodes,'MaxStepsPerEpisode',maxsteps,'Verbose',false,'Plots','training-progress','StopTrainingCriteria','EpisodeReward','StopTrainingValue',50);

if envConstants.is_training == 1
    trainingStats = train(agent,env,trainingOpts);
%     save("/home/ajoglek/Mecc_22_ws/src/mecc_rl_pp/agents/eureka_world_simple","agent")
else
    load("/home/ajoglek/Mecc_22_ws/src/mecc_rl_pp/agents/eureka.mat",'agent');
    simOpts = rlSimulationOptions('MaxSteps',2500);
    sim(env,agent,simOpts)
end
data_agent = getGlobal_data;
save('/home/ajoglek/Mecc_22_ws/src/mecc_rl_pp/agents/bumpy_world_data_updated','data_agent')
disp('Saved')
pause(inf) 
exit
   

%% Step function

function [NextObs,Reward,IsDone,LoggedSignals] = myStepFunctionCustom(Action,LoggedSignals,envConstants)
%This function applies the actions to the environment and gets rewards

%Get action from the DDPG actor
VelCommand = Action(1);
Lookahead = Action(2);

%Get old state,pose
old_state = LoggedSignals;
old_pose = old_state.State(1:3);

%Send action to the env
% envConstants.controller.LookaheadDistance = Lookahead;
envConstants.controller.LookaheadDistance  = double(Action(2));
[v, omega,lookahead_pt] = envConstants.controller(old_pose);
% lookahead_pt

%View the lookahead point
point = rosmessage('geometry_msgs/Point');
point.X = lookahead_pt(1);
point.Y = lookahead_pt(2);
envConstants.viz_msg.Points =point;
send(envConstants.viz_pub,envConstants.viz_msg)

envConstants.vel_msg.Linear.X = VelCommand;
envConstants.vel_msg.Angular.Z = omega;
send(envConstants.vel_pub,envConstants.vel_msg);

% n = 0.25;
% if envConstants.is_training == 1
%     pause(n) % Pause to facilitate training
%     call(envConstants.gazebo_pause_client,envConstants.gazebo_pause_req,"Timeout",3);
% end

% Get the new state 
pose = get_pose(envConstants);
imu = receive(envConstants.imu_sub);
odom = receive(envConstants.odom_sub);
call(envConstants.gazebo_pause_client,envConstants.gazebo_pause_req,"Timeout",3);
cross_track_error = cte(pose,envConstants.controller);

if envConstants.save_data == 1 %To save the data
    setGlobaldata(pose(1),pose(2),pose(3),imu.AngularVelocity.Z,cross_track_error,lookahead_pt(1),lookahead_pt(2),odom.Twist.Twist.Linear.X)
end

temp = [pose(1);pose(2);pose(3);imu.AngularVelocity.Z;cross_track_error;lookahead_pt(1);lookahead_pt(2);VelCommand];
LoggedSignals.State = double(temp);
NextObs = LoggedSignals.State;
    
%Obtain reward
dist_travel = (old_state.State(1) - pose(1))^2 + (old_state.State(2) - pose(2))^2;

get_dist = getGlobal_dist_covered;
total_distance = get_dist + dist_travel;
% distanceToGoal = norm(pose(1:2) - envConstants.robotGoal(:));
setGlobal_dist_covered(total_distance)

IsDone =   cross_track_error > envConstants.Cross_Track_Threshold;
if ~IsDone 
%     Reward = -lateral_error^2 + 2*pose.Twist.Twist.Linear.X^2 + 4*dist_travel - 2*pose.Twist.Twist.Angular.Z^2;
%     Reward = -2*lateral_error^2 -1.5*(5-VelCommand)^2 + 0.5*total_distance - 0.5*pose.Twist.Twist.Angular.Z^2;
% -0.5x^2 - 0.2y^2 - 3*c^2 + 0.08*d
    Reward = -0.5*(1.5-VelCommand)^2 - 0.2*(cross_track_error)^2 -3*(imu.AngularVelocity.Z)^2 + 0.08*total_distance;
else
    Reward = -1000;
end
call(envConstants.gazebo_unpause_client,envConstants.gazebo_unpause_req,"Timeout",3); 
end





%% Reset function
function [InitialObservation, LoggedSignal] = myResetFunctionCustom(envConstants)
    reset(envConstants.controller)
    release(envConstants.controller)
    clear envConstants.controller
    envConstants.controller = controllerPurePursuit;
    envConstants.controller.Waypoints = envConstants.Path;
    envConstants.controller.DesiredLinearVelocity = 0.5;
    envConstants.controller.MaxAngularVelocity = 0.8;
    envConstants.controller.LookaheadDistance = 0.2;
    call(envConstants.model_state_client,envConstants.model_state_req,"Timeout",3);
    for i = 1:2
    call(envConstants.pose_reset_client,envConstants.pose_reset_req,'Timeout',3);
    pause(2)
    end
    pose = get_pose(envConstants);
    imu = receive(envConstants.imu_sub);
    cross_track_error = cte(pose,envConstants.controller);
    [v, omega,lookahead] = envConstants.controller(pose');
    temp = [pose(1);pose(2);pose(3);imu.AngularVelocity.Z;cross_track_error;lookahead(1);lookahead(2);0];
    LoggedSignal.State = double(temp);
    InitialObservation = LoggedSignal.State;
    setGlobal_dist_covered(0)
end


%% Supporting functions
function pose = get_pose(envConstants)
    first_msg = receive(envConstants.pose_sub,10);
    x = first_msg.Pose.Pose.Position.X;
    y = first_msg.Pose.Pose.Position.Y;
    robotInitialLocation = [x y];
    quat = [first_msg.Pose.Pose.Orientation.W first_msg.Pose.Pose.Orientation.X first_msg.Pose.Pose.Orientation.Y first_msg.Pose.Pose.Orientation.Z];
    eul = quat2eul(quat);
    initialOrientation = eul(1,1);
    pose = [robotInitialLocation initialOrientation];
end

function cross_track_error = cte(pose,controller)
cross_track_error = 2; %Random high value initialization
for i=1:size(controller.Waypoints,1)
    dist = norm(pose(1:2) - controller.Waypoints(i,:));
    if dist < cross_track_error
        cross_track_error = dist;
    end
end
end


function setGlobal_dist_covered(val)
global dist_covered
dist_covered = val;
end

function get_dist = getGlobal_dist_covered
global dist_covered
get_dist = dist_covered;
end

function setGlobaldata(X,Y,Z,imu_vel_z,cross_track_error,lookahead_x,lookahead_y,lin_vel)
    global data_X; global data_Y; global data_Z; global data_imu_vel_z; global data_cross_track_error; global data_lookahead_x; global data_lookahead_y;global data_lin_vel
    data_X = [data_X X];
    data_Y = [data_Y Y];
    data_Z = [data_Z Z];
    data_imu_vel_z = [data_imu_vel_z imu_vel_z];
    data_cross_track_error = [data_cross_track_error cross_track_error];
    data_lookahead_x = [data_lookahead_x lookahead_x];
    data_lookahead_y = [data_lookahead_y lookahead_y];
    data_lin_vel = [data_lin_vel lin_vel];
end

function data_agent = getGlobal_data
    global data_X; global data_Y; global data_Z; global data_imu_vel_z; global data_cross_track_error; global data_lookahead_x; global data_lookahead_y;global data_lin_vel
    data_agent.data_X  = data_X;
    data_agent.data_Y = data_Y;
    data_agent.data_Z= data_Z;
    data_agent.data_imu_vel_z = data_imu_vel_z;
    data_agent.data_cross_track_error = data_cross_track_error;
    data_agent.data_lookahead_x =  data_lookahead_x;  
    data_agent.data_lookahead_y =  data_lookahead_y; 
    data_agent.data_lin_vel =  data_lin_vel; 
end