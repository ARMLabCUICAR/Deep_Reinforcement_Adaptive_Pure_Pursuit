%% Initialize ROS
rosshutdown;clear;clc;clear global;
ipaddress = "http://localhost:11311";
rosinit(ipaddress);


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
send(viz_pub,viz_msg)




%% Pub, Sub, Controller
amcl_pose = rossubscriber('/amcl_pose','DataFormat','struct');
imu_data = rossubscriber('/imu/data','DataFormat','struct');
[vel_pub,msg] = rospublisher("/husky_velocity_controller/cmd_vel", "geometry_msgs/Twist");

path = load ('/home/ajoglek/Downloads/updated_rectangle_path.mat');
path = double(path.path); 
path = [path;path;path];

controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.2;
controller.MaxAngularVelocity = 0.3;
controller.LookaheadDistance = 0.2;

%%
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
% pose_reset_client = rossvcclient('/reset_caller');
% pose_reset_req = rosmessage(pose_reset_client);

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
envConstants.is_training = 0; % 1 for training mode; 0 for testing
envConstants.save_data = 1;
envConstants.model_state_client = model_state_client;
envConstants.model_state_req = model_state_req;
% envConstants.pose_reset_client = pose_reset_client;
% envConstants.pose_reset_req = pose_reset_req;
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
%%

% bag = rosbag('/home/ajoglek/husky_ws/src/huskynick/bags/waypoints_2022-03-16-18-14-25.bag');

% bag = rosbag('/home/ajoglek/Mecc_22_ws/src/mecc_rl_pp/bags/rectangle2.bag');
% % /home/ajoglek/Mecc_22_ws/src/mecc_rl_pp/bags
% msgs = readMessages(bag);
% r=size(msgs,1);
% path = zeros(r,2);
% for i = 1:r
%     x = msgs{i,1}.Pose.Pose.Position.X;
%     y = msgs{i,1}.Pose.Pose.Position.Y;
%     path(i,1) = x;
%     path(i,2) = y;
% end


pose = get_pose(amcl_pose);
robotInitialLocation = pose;
robotGoal = path(end,:);
distanceToGoal = norm(robotInitialLocation - robotGoal);
goalRadius = 0.0001;

figure(1)
% plot(path(:,1), path(:,2),'k--d');
% xlim([-4 9])
% ylim([-7 3])

% lookahead_old = [0 0];


k = 1;
i = 0;
while k < 3100
%    controller.LookaheadDistance = randi([1,5],1,1);
   [v, omega,lookahead] = controller(pose);
   point.X = lookahead(1);
   point.Y = lookahead(2);
   send(viz_pub,viz_msg)
   cross_track_error = cte(pose,controller);
   msg.Linear.X = v;
   msg.Angular.Z = omega;
   send(vel_pub, msg);
   pose = get_pose(amcl_pose);
   imu = receive(envConstants.imu_sub);
   odom = receive(envConstants.odom_sub);
   distanceToGoal = norm(pose(1:2) - robotGoal(:));
   if envConstants.save_data == 1 %To save the data
    setGlobaldata(pose(1),pose(2),pose(3),imu.AngularVelocity.Z,cross_track_error,lookahead(1),lookahead(2),odom.Twist.Twist.Linear.X)
   end
   k = k+1
   
end
msg.Linear.X = 0;
msg.Angular.Z = 0;
send(vel_pub, msg);
disp('Done')
data_agent = getGlobal_data;
% j =1;
% reset(controller)
% release(controller)
% clear controller
% 
% a = 'Controller reset'
% 
% controller = controllerPurePursuit;
% controller.Waypoints = path;
% controller.DesiredLinearVelocity = 0.5;
% controller.MaxAngularVelocity = 0.3;
% controller.LookaheadDistance = 5;
% while  j<5
% %    controller.LookaheadDistance = randi([1,5],1,1);
%    [v, omega,lookahead] = controller(pose);
%    point.X = lookahead(1);
%    point.Y = lookahead(2);
%    send(viz_pub,viz_msg)
% %    cross_track_error = calc_cte(lookahead_old,lookahead,pose)
%    lookahead_old = lookahead;
%    msg.Linear.X = v;
%    msg.Angular.Z = omega;
%    send(vel_pub, msg);
%    pose = get_pose(amcl_pose);
%    plot(pose(1),pose(2),'r:s') 
%    distanceToGoal = norm(pose(1:2) - robotGoal(:));
% end

%% 

function pose = get_pose(amcl_pose)
    first_msg = receive(amcl_pose,10);
    x = first_msg.Pose.Pose.Position.X;
    y = first_msg.Pose.Pose.Position.Y;
    robotInitialLocation = [x y];
    quat = [first_msg.Pose.Pose.Orientation.W first_msg.Pose.Pose.Orientation.X first_msg.Pose.Pose.Orientation.Y first_msg.Pose.Pose.Orientation.Z];
    eul = quat2eul(quat);
    initialOrientation = eul(1,1);
    pose = [robotInitialLocation initialOrientation]';
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