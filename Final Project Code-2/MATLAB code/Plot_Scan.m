
clear; clc;


% SERIAL SETUP
s = serialport("COM6", 115200);
flush(s);

disp("Press PJ1 on board to send data...");

% STORAGE
raw_data = [];   % [scan, step, dist, x]

% READ UNTIL "END"
while true
    line = readline(s);
    line = strtrim(line);

    if line == "END"
        break;
    end

    % Skip header lines
    if startsWith(line, "SCANS")
        continue;
    end

    vals = sscanf(line, "%d, %d, %d, %d");

    if length(vals) == 4
        raw_data(end+1, :) = vals'; 
        fprintf("RAW -> Scan:%d Step:%d Dist:%d X:%d\n", vals);
    end
end

disp("Data received!");

% EXTRACT COLUMNS
scan = raw_data(:,1);
step = raw_data(:,2);
dist_raw = raw_data(:,3);
x = raw_data(:,4)/10;

measurements = 32;
angles = linspace(0, 2*pi, measurements+1);
angles(end) = [];
theta = angles(step+1)';

% RAW COORDS
y_raw = dist_raw .* cos(theta);
z_raw = dist_raw .* sin(theta);

% DATA CLEANING 
dist_clean = dist_raw;

for i = 2:length(dist_raw)-1
    
    val = dist_raw(i);
    prev = dist_raw(i-1);
    next = dist_raw(i+1);
    
    % invalid values
    if val < 40 || val > 4000 || val == 0
        dist_clean(i) = (prev + next)/2;
        continue;
    end
    
    % spike vs neighbors
    if scan(i) == scan(i-1) && scan(i) == scan(i+1)
        
        if abs(val - prev) > 500 && abs(val - next) > 500
            % replace with average of neighbors
            dist_clean(i) = (prev + next)/2;
        end
        
    end
end
y_clean = dist_clean .* cos(theta)/10;
z_clean = dist_clean .* sin(theta)/10;
unique_scans = unique(scan);

% PLOTTING
figure;
hold on;

for k = 1:length(unique_scans)
    idx = find(scan == unique_scans(k));
    
    % close the loop
    idx = [idx; idx(1)];
    
    plot3(x(idx), y_clean(idx), z_clean(idx), '-o', 'MarkerSize', 4);
end

title("PLOTTED DATA");
xlabel("X (cm)"); ylabel("Y (cm)"); zlabel("Z (cm)");
grid on;
view(3);