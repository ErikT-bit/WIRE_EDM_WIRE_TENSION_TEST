function [T, S] = run_loadcell_calibration()
workDir = 'C:\Users\Nathan\matlab\Wire_Tension';
if ~exist(workDir,'dir')
    error('Work directory not found: %s', workDir);
end
cd(workDir);

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

disp(" ");
disp("============================================================");
disp("LOAD CELL CALIBRATION CAPTURE");
disp("This will run:");
disp("  - 3 captures at 2 kg");
disp("  - 3 captures at 4 kg");
disp("Start each run: press Enter in Command Window");
disp("Stop each run: click the figure, then press Enter");
disp("Outputs (overwritten every time):");
disp("  - calibration_data.csv");
disp("  - calibration_run_summary.csv");
disp("============================================================");
disp(" ");

% Delete old outputs so only the newest ones remain
dataPath = fullfile(workDir, 'calibration_data.csv');
summaryPath = fullfile(workDir, 'calibration_run_summary.csv');

if exist(dataPath, 'file')
    delete(dataPath);
end
if exist(summaryPath, 'file')
    delete(summaryPath);
end

pause(2.0);
flush(s);

writeline(s, "PING");
pause(0.2);

input('Remove all weight from the load cell, then press Enter to tare: ', 's');
flush(s);
writeline(s, "TARE");
pause(1.0);
flush(s);

weights_kg = [2 2 2 4 4 4]';
run_number = (1:6)';
weight_group = strings(6,1);
replicate_number = [1;2;3;1;2;3];

for i = 1:6
    if weights_kg(i) == 2
        weight_group(i) = "2kg";
    else
        weight_group(i) = "4kg";
    end
end

captureTables = cell(6,1);

stopFlag = false;

f = figure( ...
    'Name', sprintf('Load Cell Calibration - %s', portUsed), ...
    'NumberTitle', 'off', ...
    'Color', 'w', ...
    'Position', [100 100 1100 750], ...
    'KeyPressFcn', @keyStopCallback, ...
    'CloseRequestFcn', @closeFigureCallback);

for runIdx = 1:6
    if ~isvalid(f)
        break
    end

    stopFlag = false;
    clf(f);

    ax1 = subplot(2,1,1);
    h1 = animatedline(ax1,'LineWidth',1.2);
    grid(ax1,'on');
    title(ax1, sprintf('Raw Load Cell Output | Run %d | %s', runIdx, weight_group(runIdx)));
    xlabel(ax1,'Time within run (s)');
    ylabel(ax1,'Raw g');

    ax2 = subplot(2,1,2);
    h2 = animatedline(ax2,'LineWidth',1.6);
    grid(ax2,'on');
    title(ax2, sprintf('Filtered Load Cell Output | Run %d | %s', runIdx, weight_group(runIdx)));
    xlabel(ax2,'Time within run (s)');
    ylabel(ax2,'Filtered g');

    statusText = annotation('textbox',[0.10 0.93 0.82 0.05], ...
        'String', sprintf('Prepare Run %d of 6 | Place %.1f kg on load cell', runIdx, weights_kg(runIdx)), ...
        'FitBoxToText','off', ...
        'EdgeColor','none', ...
        'HorizontalAlignment','left', ...
        'FontSize',10, ...
        'Interpreter','none');

    liveText = annotation('textbox',[0.70 0.82 0.22 0.08], ...
        'String', sprintf('Raw: -- g\nFiltered: -- g'), ...
        'FitBoxToText','off', ...
        'BackgroundColor','w', ...
        'HorizontalAlignment','left', ...
        'FontSize',11, ...
        'Interpreter','none');

    fprintf('\n------------------------------------------------------------\n');
    fprintf('Run %d of 6\n', runIdx);
    fprintf('Target calibration mass: %.1f kg\n', weights_kg(runIdx));
    fprintf('Replicate number: %d\n', replicate_number(runIdx));
    fprintf('Place the weight, let it settle, then press Enter to START capture.\n');
    input('', 's');

    flush(s);
    writeline(s, "START");
    pause(0.1);

    figure(f);
    drawnow;

    fprintf('Capture is running.\n');
    fprintf('To STOP: click the figure window, then press Enter.\n');

    runData = zeros(0,4);  % [time_ms, elapsed_ms, raw_g, filtered_g]

    while ~stopFlag && isvalid(f)
        while s.NumBytesAvailable > 0
            line = strtrim(readline(s));

            if isempty(line)
                continue
            end

            if startsWith(line, "EVENT")
                statusText.String = sprintf('Run %d | %s | %s', runIdx, weight_group(runIdx), line);
                continue
            end

            if startsWith(lower(line), "time_ms")
                continue
            end

            vals = sscanf(line, '%f,%f,%f,%f');
            if numel(vals) == 4
                runData(end+1,:) = vals.'; %#ok<AGROW>

                tsec = vals(2) / 1000.0;
                raw_g = vals(3);
                filt_g = vals(4);

                addpoints(h1, tsec, raw_g);
                addpoints(h2, tsec, filt_g);

                if tsec <= 10
                    xlim(ax1,[0 10]);
                    xlim(ax2,[0 10]);
                else
                    xlim(ax1,[tsec-10 tsec]);
                    xlim(ax2,[tsec-10 tsec]);
                end

                statusText.String = sprintf('Run %d | %s | samples captured: %d | Press Enter in figure to stop', ...
                    runIdx, weight_group(runIdx), size(runData,1));

                liveText.String = sprintf('Raw: %.2f g\nFiltered: %.2f g\nRaw: %.4f kg\nFiltered: %.4f kg', ...
                    raw_g, filt_g, raw_g/1000, filt_g/1000);
            end
        end

        drawnow limitrate
        pause(0.02);
    end

    writeline(s, "STOP");
    pause(0.25);
    flush(s);

    if isempty(runData)
        warning('Run %d captured no data.', runIdx);
        captureTables{runIdx} = emptyRunTable(runIdx, weights_kg(runIdx), weight_group(runIdx), replicate_number(runIdx), portUsed);
        continue
    end

    thisTable = table;
    n = size(runData,1);

    captureTime = string(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));

    thisTable.run_number = repmat(runIdx, n, 1);
    thisTable.weight_group = repmat(weight_group(runIdx), n, 1);
    thisTable.nominal_weight_kg = repmat(weights_kg(runIdx), n, 1);
    thisTable.replicate_number = repmat(replicate_number(runIdx), n, 1);
    thisTable.sample_index = (1:n).';
    thisTable.port_used = repmat(string(portUsed), n, 1);
    thisTable.capture_timestamp = repmat(captureTime, n, 1);

    thisTable.time_ms = runData(:,1);
    thisTable.elapsed_ms = runData(:,2);
    thisTable.elapsed_s = runData(:,2) / 1000.0;
    thisTable.raw_g = runData(:,3);
    thisTable.filtered_g = runData(:,4);
    thisTable.raw_kg = runData(:,3) / 1000.0;
    thisTable.filtered_kg = runData(:,4) / 1000.0;

    captureTables{runIdx} = thisTable;

    fprintf('Run %d complete. Samples captured: %d\n', runIdx, n);

    % Save current progress after every run, always overwriting the same files
    Tpartial = vertcat(captureTables{1:runIdx});
    Spartial = buildCalibrationSummary(Tpartial);
    writetable(Tpartial, dataPath);
    writetable(Spartial, summaryPath);

    if runIdx < 6
        fprintf('Prepare the next weight and continue.\n');
    end
end

validTables = captureTables(~cellfun(@isempty, captureTables));
if isempty(validTables)
    T = table;
    S = table;
else
    T = vertcat(validTables{:});
    S = buildCalibrationSummary(T);
end

writetable(T, dataPath);
writetable(S, summaryPath);

fprintf('\nSaved calibration data to:\n%s\n', dataPath);
fprintf('Saved calibration run summary to:\n%s\n', summaryPath);
disp('Done.');

    function keyStopCallback(~, evt)
        if strcmp(evt.Key, 'return') || strcmp(evt.Key, 'enter')
            stopFlag = true;
        end
    end

    function closeFigureCallback(src, ~)
        stopFlag = true;
        delete(src);
    end
end

function T = emptyRunTable(runIdx, weightKg, weightGroup, repNum, portUsed)
T = table;
T.run_number = zeros(0,1) + runIdx;
T.weight_group = strings(0,1) + weightGroup;
T.nominal_weight_kg = zeros(0,1) + weightKg;
T.replicate_number = zeros(0,1) + repNum;
T.sample_index = zeros(0,1);
T.port_used = strings(0,1) + string(portUsed);
T.capture_timestamp = strings(0,1);
T.time_ms = zeros(0,1);
T.elapsed_ms = zeros(0,1);
T.elapsed_s = zeros(0,1);
T.raw_g = zeros(0,1);
T.filtered_g = zeros(0,1);
T.raw_kg = zeros(0,1);
T.filtered_kg = zeros(0,1);
end

function S = buildCalibrationSummary(T)
if isempty(T)
    S = table;
    return
end

runs = unique(T.run_number);
nRuns = numel(runs);

run_number = zeros(nRuns,1);
weight_group = strings(nRuns,1);
nominal_weight_kg = zeros(nRuns,1);
replicate_number = zeros(nRuns,1);
port_used = strings(nRuns,1);
capture_timestamp = strings(nRuns,1);

samples = zeros(nRuns,1);
duration_s = zeros(nRuns,1);

mean_raw_g = zeros(nRuns,1);
std_raw_g = zeros(nRuns,1);
min_raw_g = zeros(nRuns,1);
max_raw_g = zeros(nRuns,1);
range_raw_g = zeros(nRuns,1);

mean_filtered_g = zeros(nRuns,1);
std_filtered_g = zeros(nRuns,1);
min_filtered_g = zeros(nRuns,1);
max_filtered_g = zeros(nRuns,1);
range_filtered_g = zeros(nRuns,1);

mean_raw_kg = zeros(nRuns,1);
std_raw_kg = zeros(nRuns,1);
mean_filtered_kg = zeros(nRuns,1);
std_filtered_kg = zeros(nRuns,1);

mean_error_filtered_g = zeros(nRuns,1);
mean_error_filtered_kg = zeros(nRuns,1);
percent_error_filtered = zeros(nRuns,1);

for i = 1:nRuns
    idx = (T.run_number == runs(i));
    Tr = T(idx,:);

    run_number(i) = Tr.run_number(1);
    weight_group(i) = Tr.weight_group(1);
    nominal_weight_kg(i) = Tr.nominal_weight_kg(1);
    replicate_number(i) = Tr.replicate_number(1);
    port_used(i) = Tr.port_used(1);
    capture_timestamp(i) = Tr.capture_timestamp(1);

    samples(i) = height(Tr);

    if ~isempty(Tr.elapsed_s)
        duration_s(i) = max(Tr.elapsed_s, [], 'omitnan');
    else
        duration_s(i) = NaN;
    end

    mean_raw_g(i) = mean(Tr.raw_g, 'omitnan');
    std_raw_g(i) = std(Tr.raw_g, 'omitnan');
    min_raw_g(i) = min(Tr.raw_g, [], 'omitnan');
    max_raw_g(i) = max(Tr.raw_g, [], 'omitnan');
    range_raw_g(i) = max_raw_g(i) - min_raw_g(i);

    mean_filtered_g(i) = mean(Tr.filtered_g, 'omitnan');
    std_filtered_g(i) = std(Tr.filtered_g, 'omitnan');
    min_filtered_g(i) = min(Tr.filtered_g, [], 'omitnan');
    max_filtered_g(i) = max(Tr.filtered_g, [], 'omitnan');
    range_filtered_g(i) = max_filtered_g(i) - min_filtered_g(i);

    mean_raw_kg(i) = mean(Tr.raw_kg, 'omitnan');
    std_raw_kg(i) = std(Tr.raw_kg, 'omitnan');
    mean_filtered_kg(i) = mean(Tr.filtered_kg, 'omitnan');
    std_filtered_kg(i) = std(Tr.filtered_kg, 'omitnan');

    mean_error_filtered_g(i) = mean_filtered_g(i) - nominal_weight_kg(i) * 1000.0;
    mean_error_filtered_kg(i) = mean_filtered_kg(i) - nominal_weight_kg(i);

    if nominal_weight_kg(i) ~= 0
        percent_error_filtered(i) = 100.0 * mean_error_filtered_kg(i) / nominal_weight_kg(i);
    else
        percent_error_filtered(i) = NaN;
    end
end

S = table(run_number, weight_group, nominal_weight_kg, replicate_number, ...
          port_used, capture_timestamp, samples, duration_s, ...
          mean_raw_g, std_raw_g, min_raw_g, max_raw_g, range_raw_g, ...
          mean_filtered_g, std_filtered_g, min_filtered_g, max_filtered_g, range_filtered_g, ...
          mean_raw_kg, std_raw_kg, mean_filtered_kg, std_filtered_kg, ...
          mean_error_filtered_g, mean_error_filtered_kg, percent_error_filtered);
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