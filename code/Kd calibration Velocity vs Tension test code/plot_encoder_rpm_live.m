function plot_encoder_rpm_live()
workDir = 'C:\Users\Nathan\matlab\Wire_Tension';
if exist(workDir,'dir')
    cd(workDir)
end

baudRate = 115200;
sampleWindow = 200;
countsPerRev = 500;
useX4 = false;

if useX4
    effCountsPerRev = countsPerRev * 4;
else
    effCountsPerRev = countsPerRev;
end

preferredPorts = ["COM13","COM16"];
availablePorts = string(serialportlist("available"));

disp("Available ports:")
disp(availablePorts')

if isempty(availablePorts)
    error('MATLAB does not currently see any serial ports.')
end

portsToTry = [preferredPorts, setdiff(availablePorts, preferredPorts, 'stable')];

s = [];
connectedPort = "";

for k = 1:numel(portsToTry)
    thisPort = portsToTry(k);
    if any(strcmpi(thisPort, availablePorts))
        try
            s = serialport(thisPort, baudRate);
            configureTerminator(s, "LF");
            flush(s);
            connectedPort = thisPort;
            break
        catch
        end
    end
end

if isempty(s)
    error('Could not open any available COM port. Close Arduino Serial Monitor and try again.')
end

disp("Connected to " + connectedPort)

f = figure( ...
    'Name', sprintf('Live Encoder RPM (%s)', connectedPort), ...
    'NumberTitle', 'off', ...
    'Position', [100 100 700 350], ...
    'Color', 'w');

ax = axes(f);
h = animatedline(ax, 'LineWidth', 1.5);
grid(ax, 'on');
xlabel(ax, 'Time (s)');
ylabel(ax, 'RPM');
title(ax, sprintf('Instantaneous RPM - %s', connectedPort));

tData = [];
rpmData = [];
countPrev = [];
timePrev = [];
t0 = tic;

cleanupObj = onCleanup(@() localCleanup(s));

while isvalid(f)
    if s.NumBytesAvailable > 0
        line = strtrim(readline(s));
        vals = sscanf(line, '%f,%f,%f,%f');

        if numel(vals) >= 3
            countNow = vals(1);
            tNow = toc(t0);

            if isempty(countPrev)
                countPrev = countNow;
                timePrev = tNow;
            else
                dt = tNow - timePrev;
                dc = countNow - countPrev;

                if dt > 0
                    rpmUse = (dc / effCountsPerRev) / dt * 60;
                else
                    rpmUse = 0;
                end

                tData(end+1) = tNow;
                rpmData(end+1) = rpmUse;

                if numel(tData) > sampleWindow
                    tData = tData(end-sampleWindow+1:end);
                    rpmData = rpmData(end-sampleWindow+1:end);
                end

                clearpoints(h);
                addpoints(h, tData, rpmData);

                xlim(ax, [max(0, tNow-10), max(10, tNow)]);
                drawnow limitrate

                countPrev = countNow;
                timePrev = tNow;
            end
        end
    else
        drawnow limitrate
    end
end
end

function localCleanup(s)
try
    if ~isempty(s)
        clear s
    end
catch
end
end