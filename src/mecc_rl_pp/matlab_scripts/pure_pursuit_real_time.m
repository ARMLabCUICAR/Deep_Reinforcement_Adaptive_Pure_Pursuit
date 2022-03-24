%% Initialize ROS
rosshutdown;clear;clc;
ipaddress = "http://localhost:11311";
rosinit(ipaddress);

%% Pub, Sub, Controller
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

controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.5;
controller.MaxAngularVelocity = 0.3;
controller.LookaheadDistance = 0.2;

pose = get_pose(amcl_pose);
robotInitialLocation = pose;
robotGoal = path(end,:);
distanceToGoal = norm(robotInitialLocation - robotGoal);
goalRadius = 0.2;

figure(1)
plot(path(:,1), path(:,2),'k--d');
xlim([-4 9])
ylim([-7 3])
hold all;


while (distanceToGoal > goalRadius)
   [v, omega] = controller(pose);
   msg.Linear.X = v;
   msg.Angular.Z = omega;
   send(vel_pub, msg);
   pose = get_pose(amcl_pose);
   plot(pose(1),pose(2),'r:s') 
   distanceToGoal = norm(pose(1:2) - robotGoal(:))
end
msg.Linear.X = 0;
msg.Angular.Z = 0;
send(vel_pub, msg);

%% 
% function current_pose = amcl_callback(~,message)
%     global robot_pose
%     x = message.Pose.Pose.Position.X;
%     y = message.Pose.Pose.Position.Y;
%     robotInitialLocation = [x y];
%     quat = [message.Pose.Pose.Orientation.W message.Pose.Pose.Orientation.X message.Pose.Pose.Orientation.Y message.Pose.Pose.Orientation.Z];
%     eul = quat2eul(quat);
%     initialOrientation = eul(1,1);
%     robot_pose = [robotInitialLocation initialOrientation]';
% end

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