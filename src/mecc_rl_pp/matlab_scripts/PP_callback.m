rosshutdown;clear;clc;
ipaddress = "http://localhost:11311";
rosinit(ipaddress);
global robot_pose
amcl_pose = rossubscriber('/amcl_pose','DataFormat','struct');
[vel_pub,msg] = rospublisher("/husky_velocity_controller/cmd_vel", "geometry_msgs/Twist");

bag = rosbag('/home/ajoglek/husky_ws/src/huskynick/bags/waypoints_2022-03-16-18-14-25.bag');
msgs = readMessages(bag);
r=size(msgs,1);
path = zeros(r,2);
for i = 1:r
    x = msgs{i,1}.Pose.Pose.Position.X;
    y = msgs{i,1}.Pose.Pose.Position.Y;
    path(i,1) = x;
    path(i,2) = y;
end

% B = [];
% path_ = single(path); path_ = round(path_,3);
% for i = 1:size(path_,1) -1
%     if path_(i,:) ~= path_(i+1,:) 
%         B = [B;path_(i,:)];
%     end
% end

% B = [B];

figure
plot(path(:,1), path(:,2),'k--d');
xlim([-4 9])
ylim([-7 3])


controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.5;
controller.MaxAngularVelocity = 0.3;
controller.LookaheadDistance = 0.2;

lookahead_vec = zeros(350,2);
i=1;
hold on;
tic;
while toc<15000000
   robot_position = robot_pose;
   [v, omega,lookahead] = controller(robot_position);
   msg.Linear.X = v;
   msg.Angular.Z = omega;
   send(vel_pub, msg);
   plot(robot_position(1),robot_position(2),'r:s') 
end

%% 
function current_pose = amcl_callback(~,message)
    global robot_pose
    x = message.Pose.Pose.Position.X;
    y = message.Pose.Pose.Position.Y;
    robotInitialLocation = [x y];
    quat = [message.Pose.Pose.Orientation.W message.Pose.Pose.Orientation.X message.Pose.Pose.Orientation.Y message.Pose.Pose.Orientation.Z];
    eul = quat2eul(quat);
    initialOrientation = eul(1,1);
    robot_pose = [robotInitialLocation initialOrientation]';
end
