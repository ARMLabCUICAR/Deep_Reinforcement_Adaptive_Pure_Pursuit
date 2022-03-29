%% Initialize ROS
rosshutdown;clear;clc;
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
[vel_pub,msg] = rospublisher("/husky_velocity_controller/cmd_vel", "geometry_msgs/Twist");

% bag = rosbag('/home/ajoglek/husky_ws/src/huskynick/bags/waypoints_2022-03-16-18-14-25.bag');

bag = rosbag('/home/ajoglek/Mecc_22_ws/src/mecc_rl_pp/bags/rectangle2.bag');
% /home/ajoglek/Mecc_22_ws/src/mecc_rl_pp/bags
msgs = readMessages(bag);
r=size(msgs,1);
path = zeros(r,2);
for i = 1:r
    x = msgs{i,1}.Pose.Pose.Position.X;
    y = msgs{i,1}.Pose.Pose.Position.Y;
    path(i,1) = x;
    path(i,2) = y;
end

path = [path;path;path];

controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.5;
controller.MaxAngularVelocity = 0.3;
controller.LookaheadDistance = 0.2;

pose = get_pose(amcl_pose);
robotInitialLocation = pose;
robotGoal = path(end,:);
distanceToGoal = norm(robotInitialLocation - robotGoal);
goalRadius = 0.01;

figure(1)
plot(path(:,1), path(:,2),'k--d');
xlim([-4 9])
ylim([-7 3])
hold all;
lookahead_old = [0 0];


while distanceToGoal > goalRadius
%    controller.LookaheadDistance = randi([1,5],1,1);
   [v, omega,lookahead] = controller(pose);
   point.X = lookahead(1);
   point.Y = lookahead(2);
   send(viz_pub,viz_msg)
%    cross_track_error = calc_cte(lookahead_old,lookahead,pose)
   lookahead_old = lookahead;
   msg.Linear.X = v;
   msg.Angular.Z = omega;
   send(vel_pub, msg);
   pose = get_pose(amcl_pose);
   plot(pose(1),pose(2),'r:s') 
   distanceToGoal = norm(pose(1:2) - robotGoal(:));
end
msg.Linear.X = 0;
msg.Angular.Z = 0;
send(vel_pub, msg);
disp('Done')
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

function cross_track_error = calc_cte(lookahead_old,lookahead,pose)
    d1 = norm(pose(1:2) - lookahead_old);
    d2 = norm(pose(1:2) - lookahead);
    d3 = norm(lookahead_old - lookahead);
    ang_d2 = rad2deg(acos(abs(d1^2 + d3^2 - d2^2)/(2*d3*d1)));
    cross_track_error = d1*sin(ang_d2);
end