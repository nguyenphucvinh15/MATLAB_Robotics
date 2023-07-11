%% Initialize the Robot Simulator
rosinit
sim = ExampleHelperRobotSimulator('complexMap');% emptyMap, simpleMap, borderMap, complexMap
setRobotPose(sim, [2 2 2/3*pi]);
enableROSInterface(sim, true);
sim.LaserSensor.NumReadings = 100;% 50
% sim.LaserSensor.MaxRange = 10;
%% Setup ROS Interface
scanSub = rossubscriber('scan');
[velPub, velMsg] = rospublisher('/mobile_base/commands/velocity');
tftree = rostf;
pause(1);
%% Create a Path Controller
% [x,y] = ginput(30); x = [2;x]; y = [2;y];
% path = [x,y];
% save path
path = [2, 3;3.25 6.25;2 11;6 7; 11 11;8 6; 10 5;7 3;11 1.5];
% Visualize the path in the robot simulator
% plot(path(:,1), path(:,2),'k--d');
controller = robotics.PurePursuit('Waypoints', path);
controller.DesiredLinearVelocity = 0.4;
controlRate = robotics.Rate(100);
%%
goalRadius = 0.1;
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);
distanceToGoal = norm(robotCurrentLocation - robotGoal);
%% Define an Empty Map
% Creates a map with 14 m X 13 m size and a resolution of 20 cells per meter.
map = robotics.OccupancyGrid(25,20,20);
% Visualize the map in the figure window.
figureHandle = figure('Name', 'Map');
axesHandle = axes('Parent', figureHandle);
mapHandle = show(map, 'Parent', axesHandle);
title(axesHandle, 'OccupancyGrid: Update 0');
pause(1);
%%
% * Visualize the map after every 50 updates.
updateCounter = 1;
while( distanceToGoal > goalRadius )
    % Receive a new laser sensor reading
    scanMsg = receive(scanSub);
    % Get robot pose at the time of sensor reading
    pose = getTransform(tftree, 'map', 'robot_base', scanMsg.Header.Stamp, 'Timeout', 2);
    % Convert robot pose to 1x3 vector [x y yaw]
    position = [pose.Transform.Translation.X, pose.Transform.Translation.Y];
    orientation =  quat2eul([pose.Transform.Rotation.W, pose.Transform.Rotation.X, ...
        pose.Transform.Rotation.Y, pose.Transform.Rotation.Z], 'ZYX');
    robotPose = [position, orientation(1)];
    % Extract the laser scan
    scan = lidarScan(scanMsg);
    ranges = scan.Ranges;
    ranges(isnan(ranges)) = sim.LaserSensor.MaxRange;
    modScan = lidarScan(ranges, scan.Angles);
    % Insert the laser range observation in the map
    insertRay(map, robotPose, modScan, sim.LaserSensor.MaxRange);
    % Compute the linear and angular velocity of the robot and publish it
    % to drive the robot.
    %[v, w] = controller(robotPose);
    [v,w]= getKeyBoardAndControl;
    velMsg.Linear.X = v;
    velMsg.Angular.Z = w;
    send(velPub, velMsg);
    % Visualize the map after every 50th update.
    if ~mod(updateCounter,20)
        mapHandle.CData = occupancyMatrix(map);
        title(axesHandle, ['OccupancyGrid: Update ' num2str(updateCounter)]);
    end
    % Update the counter and distance to goal
    updateCounter = updateCounter+1;
    distanceToGoal = norm(robotPose(1:2) - robotGoal);
    % Wait for control rate to ensure 10 Hz rate
    waitfor(controlRate);
end
%% Display the final map, which has incorporated all the sensor readings.
show(map, 'Parent', axesHandle);
title(axesHandle, 'OccupancyGrid: Final Map');
% <<final_map.png>>
% <<simulator_end_image.png>>
%% Shutdown ROS Network
rosshutdown
displayEndOfDemoMessage(mfilename)
