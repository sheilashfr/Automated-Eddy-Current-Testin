%% Load library
import zaber.motion.ascii.Connection;
import zaber.motion.Units;
import zaber.motion.Library;

%% Open connection and find devices

% % For virtual device
% Library.enableDeviceDbStore();
% try
%   connection = Connection.openIot('f69285aa-50ed-4041-ae7a-116e7c629b49', 'apjg3YQq3f1UtQrKmArZKu04iMeSrqxC'); 
%   deviceList = connection.detectDevices();
%   fprintf('Found %d devices.\n', deviceList.length);
% end

%% For physical device
%Open a serial port
connection = Connection.openSerialPort('/dev/tty.usbserial-AL00BUEA');
%% Detect devices
devices = connection.detectDevices();
device1 = devices(1);
device2 = devices(2);
device3 = devices(3);

%% home all axes of device
device1.getAllAxes().home();
device2.getAllAxes().home();
device3.getAllAxes().home();

%% Get all axis
axis1 = device1.getAxis(1);
axis2 = device2.getAxis(1);
axis3 = device3.getAxis(1);

%% set hard home pos
axis3.moveAbsolute(10, Units.LENGTH_MILLIMETRES);
absolutePos = axis3.getPosition(Units.LENGTH_MILLIMETRES);
%axis2.moveRelative(100, Units.LENGTH_MILLIMETRES);
%% Get current position 
import zaber.motion.Units;
home_x = axis1.getPosition(Units.LENGTH_MILLIMETRES);
home_y = axis2.getPosition(Units.LENGTH_MILLIMETRES);
home_z = axis3.getPosition(Units.LENGTH_MILLIMETRES);

%% Loading the positions
waypoints = readtable("adaptive_D.csv"); % Load waypoints
num_points = height(waypoints); % Get number of points
% Extract the column data as arrays
col1 = waypoints(:,1).Variables;
col2 = waypoints(:,2).Variables;
col3 = waypoints(:,3).Variables;
% Initialize variables for storing the relative differences
diff1 = zeros(size(col1));
diff2 = zeros(size(col2));
diff3 = zeros(size(col3));
% Compute the relative differences for each row
for i = 2:height(waypoints)
    diff1(i) = col1(i) - col1(i-1);
    diff2(i) = col2(i) - col2(i-1);
    diff3(i) = col3(i) - col3(i-1);
end

%%  Loop over each waypoint
import zaber.motion.Units;
for i = 1:height(waypoints)
    x = diff1(i);
    y = diff2(i);
    z = diff3(i);
 
    %axis1.moveRelative(x, Units.LENGTH_MILLIMETRES, false);
    %axis2.moveRelative(y, Units.LENGTH_MILLIMETRES, false);
    axis3.moveRelative(z, Units.LENGTH_MILLIMETRES);
    
    position_x(i) = axis1.getPosition(Units.LENGTH_MILLIMETRES) %- home_x;
    position_y(i) = axis2.getPosition(Units.LENGTH_MILLIMETRES) %- home_y;
    position_z(i) = axis3.getPosition(Units.LENGTH_MILLIMETRES) % - home_z;

    pause(2)
end

%     % 1. Set first point as origin / home - Get position of the first point
%     % relative to the home position, then it moves to the first point by
%     % that amount of distance, record current position
%     % 2. Move to next set of points - Get position of the second point
%     % relative to the current position, then it moves to the next point by
%     % that amount of distance, record current position

%     %Display values
% 
%     % [x, y, z] = pid_controller(current_position, desired_position, Kp, Ki, Kd);
%     % error_x = desired_position(1) - current_position(1);
%     % error_y = desired_position(2) - current_position(2);
%     % error_z = desired_position(3) - current_position(3);
%     % current_time = now;
%     % delta_t = (current_time - prev_time) * 24 * 3600; % Convert from days to seconds
%     % prev_time = current_time;

%% Create position table

getpos = [position_x; position_y]'

%% Visualize
figure
scatter(col1,col2,10,'blue','filled');
hold on
scatter(position_x+col1(1),position_y+col2(1),10, 'red','filled');
axis on
axis equal
xlabel('x');
ylabel('y');
zlabel('z');
grid on
grid(gca,'minor');
legend('Waypoints coordinates','Position recorded by the stages');
title('Position recorded by the stages vs waypoints coordinates');
hold off

newposx=position_x+col1(1)
newposy=position_y+col2(1)
%% Close connection
connection.close();
