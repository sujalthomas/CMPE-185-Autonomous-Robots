% https://www.mathworks.com/help/robotics/ug/path-following-for-differential-drive-robot.html
% Define the desired path for the robot
path = [2.00    1.00;
        12.00   10.00];

% Set the current location and the goal location of the robot as defined by
% the path.
robotInitialLocation = path(1,:);
robotGoal = path(end,:);

% Assume an initial robot orientation (the robot orientation is the angle
% between the robot heading and the positive X-axis, measured
% couterclockwise). 
initialOrientation = pi/2;

% Define the current pose of the robot [x y theta]
robotCurrentPose = [robotInitialLocation initialOrientation]';

% Create a kinematic robot model
% Initialize the robot model and assign an initial pose.
% The simulated robot has kinematic equations for the motion of a
% two-wheeled differential drive robot.
% The inputs to this simulated robot are linear and angular velocities. 
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");

% Visualize the desired path
figure
plot(path(:,1), path(:,2),'k--d')
xlim([0 13])
ylim([0 13])

% Define a goal radius, which is the desired distance threshold between the
% robot's final location and the goal location. Once the robot is within
% this distance from the goal, it will stop. Also you compute the current
% distance between the robot location and the goal location. This distance
% is continuously checked against the goal radius and the robot stops when
% this distance is less than the goal radius. 

goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);


% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
%frameSize = robot.TrackWidth/0.8;


while( distanceToGoal > goalRadius )
    
    % Implement a PID go-to-goal controller to compute the inputs to the
    % robot v and omega 
    % Hint: you can set v = constant, and use the function atan2 to find
    % the desired orientation
    
    % Implement the controller here. 

    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 13])
    ylim([0 13])
    
    waitfor(vizRate);
end

