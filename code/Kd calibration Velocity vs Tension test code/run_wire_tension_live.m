function T = run_wire_tension_live(targetRPM, duration_s)
if nargin < 1, targetRPM = 100; end
if nargin < 2, duration_s = 30; end

workDir = 'C:\Users\Nathan\matlab\Wire_Tension';
if exist(workDir,'dir')
    cd(workDir);
end

baud = 115200;
preferredPorts = ["COM13","COM16"];
avail = string(serialportlist("available"));
portsToTry = [preferredPorts, setdiff(avail, preferredPorts, 'stable')];

s = [];
portUsed = "";

for k = 1:numel(portsToTry)
    if any(strcmpi(portsToTry(k), avail))
        try
            s = serialport(portsToTry(k), baud);
            configureTerminator(s, "LF");
            flush(s);
            portUsed = portsToTry(k);
            break
        catch
        end
    end
end

if isempty(s)
    error('Could not open any COM port.');
end

cleanupObj = onCleanup(@() localCleanup(s)); %#ok<NASGU>

pause(2.0);
flush(s);

writeline(s, sprintf('RPM=%.3f', targetRPM));

f = figure('Name', sprintf('Wire Tension Live - %s', portUsed), ...
           'NumberTitle', 'off', ...
           'Color', 'w', ...
           'Position', [100 100 900 600]);

ax1 = subplot(2,1,1);
h1_meas = animatedline(ax1, 'LineWidth', 1.5);
h1_tgt  = animatedline(ax1, 'LineStyle', '--', 'LineWidth', 1.0);
grid(ax1, 'on');
ylabel(ax1, 'RPM');
title(ax1, 'Motor Speed');
ylim(ax1, [0 200]);

ax2 = subplot(2,1,2);
h2 = animatedline(ax2, 'LineWidth', 1.5);
grid(ax2, 'on');
xlabel(ax2, 'Time (s)');
ylabel(ax2, 'Load (kg)');
title(ax2, 'Load Cell Reading');

time_s     = [];
target_r   = [];
meas_r     = [];
cmd_r      = [];
load_g     = [];
load_kg    = [];
tension_g  = [];
tension_kg = [];
tension_N  = [];
enc_count  = [];

tStart = tic;

while isvalid(f) && toc(tStart) <= duration_s
    if s.NumBytesAvailable > 0
        line = strtrim(readline(s));
        vals = sscanf(line, '%f,%f,%f,%f,%f,%f,%f,%f');

        if numel(vals) == 8
            t = vals(1) / 1000.0;

            time_s(end+1,1)     = t;
            target_r(end+1,1)   = vals(2);
            meas_r(end+1,1)     = vals(3);
            cmd_r(end+1,1)      = vals(4);
            load_g(end+1,1)     = vals(5);
            load_kg(end+1,1)    = vals(5) / 1000;
            tension_g(end+1,1)  = vals(6);
            tension_kg(end+1,1) = vals(6) / 1000;
            tension_N(end+1,1)  = vals(7);
            enc_count(end+1,1)  = vals(8);

            addpoints(h1_meas, t, vals(3));
            addpoints(h1_tgt,  t, vals(2));
            addpoints(h2,      t, vals(5) / 1000);

            if t > 10
                xlim(ax1, [t-10 t]);
                xlim(ax2, [t-10 t]);
            else
                xlim(ax1, [0 10]);
                xlim(ax2, [0 10]);
            end

            ylim(ax1, [0 200]);

            drawnow limitrate
        end
    else
        drawnow limitrate
    end
end

writeline(s, "STOP");

T = table(time_s, target_r, meas_r, cmd_r, load_g, load_kg, tension_g, tension_kg, tension_N, enc_count, ...
    'VariableNames', {'time_s','target_rpm','meas_rpm','cmd_rpm','load_g','load_kg','tension_g','tension_kg','tension_N','enc_count'});

stamp = datestr(now, 'yyyy_mm_dd_HHMMSS');
csvName = fullfile(workDir, sprintf('wire_tension_run_%s.csv', stamp));
writetable(T, csvName);

disp(['Saved: ', csvName]);
end

function localCleanup(s)
try
    if ~isempty(s)
        writeline(s, "STOP");
    end
catch
end
try
    clear s
catch
end
end