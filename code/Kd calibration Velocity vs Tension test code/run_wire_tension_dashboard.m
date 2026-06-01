function T = run_wire_tension_dashboard(targetLinear_mpm, kd_value)
% RUN_WIRE_TENSION_DASHBOARD
%   Collects wire tension data for kd tuning on a wire EDM brake system.
%   Ramps stepper to target speed over 5 s, holds for 10 s, then stops.
%   Appends a one-row summary to a master CSV after each saved run.

workDir = 'C:\Users\Nathan\matlab\Wire_Tension';
if ~exist(workDir,'dir')
    error('Work directory not found: %s', workDir);
end
cd(workDir);

% ===================== FIXED PARAMETERS =====================
moteusPulleyDia_mm   = 25.4;            % moteus brake pulley [mm]
driverPulleyDia_in   = 1.5;             % stepper puller pulley [in]
loadToWireDivider    = 2;               % load-cell geometry divider
tarePause_s          = 0.75;            % tare settle time [s]
rampTime_s           = 5;               % ramp duration [s]
runTime_s            = 10;              % time at full speed [s]
duration_s           = rampTime_s + runTime_s;  % total test [s]
moteusId             = 1;
moteusTau            = 0.60;            % max torque [Nm]
moteusRate           = 200;             % sample rate [Hz]
baud                 = 115200;
arduinoPort          = "COM16";

% ===================== USER PROMPTS =====================
if nargin < 1 || isempty(targetLinear_mpm)
    targetLinear_mpm = inputDefault('Target linear speed [m/min] (default 15): ', 15);
end
if nargin < 2 || isempty(kd_value)
    kd_value = inputDefault('Moteus kd value (default 0.25): ', 0.25);
end

saveAns  = lower(strtrim(input('Save this run? (y/n): ','s')));
runLabel = strtrim(input('Run label (optional): ','s'));
if isempty(runLabel), runLabel = "run"; end

% ===================== DERIVED CONSTANTS =====================
moteusRadius_m = (moteusPulleyDia_mm / 1000) / 2;
driverDia_m    = driverPulleyDia_in * 0.0254;
targetRPM      = targetLinear_mpm / (pi * driverDia_m);

% ===================== SERIAL PORT =====================
avail = string(serialportlist("available"));
if ~any(strcmpi(arduinoPort, avail))
    error('Arduino not found on %s.', arduinoPort);
end

s = serialport(arduinoPort, baud);
configureTerminator(s, "LF");
flush(s);
portUsed = arduinoPort;

cleanupObj = onCleanup(@() localCleanup(s)); %#ok<NASGU>

% ===================== MOTEUS LOGGER =====================
moteusCsv = fullfile(workDir, 'moteus_latest_torque.csv');
pyScript  = fullfile(workDir, 'damper_kd_csv_logger.py');

if exist(moteusCsv,'file')
    try delete(moteusCsv); catch, end
end

if ~exist(pyScript,'file')
    error('Python logger not found: %s', pyScript);
end

launchMoteusLogger(workDir, pyScript, moteusId, kd_value, moteusTau, moteusRate);
pause(2.5);

% ===================== INIT ARDUINO =====================
pause(2.0);
flush(s);

writeline(s, "STOP");
pause(0.10);

writeline(s, "TARE");
pause(tarePause_s);

writeline(s, sprintf('TDIV=%.6f', loadToWireDivider));
pause(0.05);

writeline(s, "STATE=2");
pause(0.02);
writeline(s, "REC=0");
pause(0.02);
writeline(s, "RPM=0");
pause(0.05);

% ===================== DASHBOARD FIGURE =====================
f = figure('Name', sprintf('Wire Tension Dashboard - %s', portUsed), ...
           'NumberTitle', 'off', ...
           'Color', 'w', ...
           'Position', [100 50 1350 900]);

ax1 = subplot(2,2,1);
h1_raw  = animatedline(ax1,'LineWidth',1.0,'Color',[0.7 0.7 0.7]);
h1_filt = animatedline(ax1,'LineWidth',1.8,'Color',[0.0 0.45 0.74]);
grid(ax1,'on');
title(ax1,'Load Cell (Tension)');
xlabel(ax1,'Time (s)');
ylabel(ax1,'kgf');
legend(ax1, {'Raw','Filtered'}, 'Location','best');

ax2 = subplot(2,2,2);
h2_meas = animatedline(ax2,'LineWidth',1.6,'Color',[0.85 0.33 0.10]);
h2_tgt  = animatedline(ax2,'LineStyle','--','LineWidth',1.0,'Color',[0.3 0.3 0.3]);
grid(ax2,'on');
title(ax2,'Linear Wire Speed');
xlabel(ax2,'Time (s)');
ylabel(ax2,'m/min');
legend(ax2, {'Measured','Target'}, 'Location','best');

ax3 = subplot(2,2,3);
h3 = animatedline(ax3,'LineWidth',1.5,'Color',[0.47 0.67 0.19]);
grid(ax3,'on');
title(ax3,'Moteus Torque (from current)');
xlabel(ax3,'Time (s)');
ylabel(ax3,'Nm');

ax4 = subplot(2,2,4);
h4 = animatedline(ax4,'LineWidth',1.5,'Color',[0.64 0.08 0.18]);
grid(ax4,'on');
title(ax4,'Load-Cell-Inferred Torque');
xlabel(ax4,'Time (s)');
ylabel(ax4,'Nm');

statusText = annotation('textbox',[0.52 0.92 0.45 0.07], ...
    'String','Initializing...', ...
    'FitBoxToText','off', ...
    'EdgeColor','none', ...
    'HorizontalAlignment','left', ...
    'FontSize',10, ...
    'Interpreter','none');

% ===================== DATA ARRAYS =====================
time_s = []; state_code = []; meas_rpm = []; cmd_rpm = [];
load_kg_raw = []; load_kg_filt = [];
wire_force_N = []; linear_mpm = [];
loadcell_torque_nm = []; moteus_torque_nm = [];

lastMoteusTorque = NaN;
lastMoteusKd     = NaN;
lastReadStamp    = "";

firstArduinoTimeRaw = NaN;

% ===================== MAIN LOOP =====================
tStart           = tic;
lastRampCmdTime  = -inf;
lastStateSent    = -1;
lastRecSent      = -1;
lastCmdSentRPM   = NaN;
rampCmdInterval_s = 0.10;

while isvalid(f) && toc(tStart) <= duration_s
    tNow = toc(tStart);

    % --- Ramp fraction ---
    rampFrac = min(max(tNow / rampTime_s, 0), 1);

    if rampFrac < 1
        desiredRPM   = targetRPM * rampFrac;
        desiredState  = 2;   % RAMP
        desiredRec    = 0;
    else
        desiredRPM   = targetRPM;
        desiredState  = 3;   % RUN
        desiredRec    = 1;
    end

    % --- Send commands at interval ---
    if (tNow - lastRampCmdTime) >= rampCmdInterval_s
        if desiredState ~= lastStateSent
            writeline(s, sprintf('STATE=%d', desiredState));
            lastStateSent = desiredState;
            pause(0.005);
        end

        if desiredRec ~= lastRecSent
            writeline(s, sprintf('REC=%d', desiredRec));
            lastRecSent = desiredRec;
            pause(0.005);
        end

        if isnan(lastCmdSentRPM) || abs(desiredRPM - lastCmdSentRPM) >= 0.5 || desiredState == 3
            writeline(s, sprintf('RPM=%.3f', desiredRPM));
            lastCmdSentRPM = desiredRPM;
        end

        lastRampCmdTime = tNow;
    end

    % --- Read moteus CSV ---
    [tauMoteus, ~, ~, ~, kdMoteus, lastReadStamp] = ...
        readLatestMoteusTorque(moteusCsv, lastReadStamp);
    if ~isnan(tauMoteus)
        lastMoteusTorque = tauMoteus;
        lastMoteusKd     = kdMoteus;
    end

    % --- Read Arduino serial ---
    while s.NumBytesAvailable > 0
        line = strtrim(readline(s));
        if isempty(line) || startsWith(lower(line), "time_ms"), continue; end

        vals = sscanf(line, '%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');
        if numel(vals) ~= 12, continue; end

        tRaw = vals(1) / 1000.0;
        if isnan(firstArduinoTimeRaw), firstArduinoTimeRaw = tRaw; end
        t = tRaw - firstArduinoTimeRaw;

        thisStateCode  = vals(2);
        thisMeasRPM    = vals(6);
        thisCmdRPM     = vals(7);
        thisLoadRaw_kg = vals(8) / 1000;
        thisLoadFilt_kg = vals(9) / 1000;
        thisForce_N    = thisLoadFilt_kg * 9.80665;
        thisWireForce  = thisForce_N / loadToWireDivider;
        thisTorque_Nm  = thisWireForce * moteusRadius_m;
        thisLinear_mpm = thisMeasRPM * (pi * driverDia_m);

        % Store all data (for plotting)
        time_s(end+1,1)              = t;
        state_code(end+1,1)          = thisStateCode;
        meas_rpm(end+1,1)            = thisMeasRPM;
        cmd_rpm(end+1,1)             = thisCmdRPM;
        load_kg_raw(end+1,1)         = thisLoadRaw_kg;
        load_kg_filt(end+1,1)        = thisLoadFilt_kg;
        wire_force_N(end+1,1)        = thisWireForce;
        linear_mpm(end+1,1)          = thisLinear_mpm;
        loadcell_torque_nm(end+1,1)  = thisTorque_Nm;
        moteus_torque_nm(end+1,1)    = lastMoteusTorque;

        % Update plots
        addpoints(h1_raw,  t, thisLoadRaw_kg);
        addpoints(h1_filt, t, thisLoadFilt_kg);
        addpoints(h2_meas, t, thisLinear_mpm);
        addpoints(h2_tgt,  t, targetLinear_mpm);
        if ~isnan(lastMoteusTorque)
            addpoints(h3, t, lastMoteusTorque);
        end
        addpoints(h4, t, thisTorque_Nm);

        % Scrolling x-axis (10 s window)
        if t > 10
            xlim(ax1,[t-10 t]); xlim(ax2,[t-10 t]);
            xlim(ax3,[t-10 t]); xlim(ax4,[t-10 t]);
        else
            xlim(ax1,[0 10]); xlim(ax2,[0 10]);
            xlim(ax3,[0 10]); xlim(ax4,[0 10]);
        end

        % Status bar
        stateTxt = stateCodeName(thisStateCode);
        if isnan(lastMoteusTorque)
            moteusTxt = 'Moteus: waiting...';
        else
            moteusTxt = sprintf('Moteus torque %.4f Nm | kd %.3f', lastMoteusTorque, lastMoteusKd);
        end
        statusText.String = sprintf('State: %s | Speed: %.1f m/min | %s', ...
            stateTxt, thisLinear_mpm, moteusTxt);
    end

    drawnow limitrate
    pause(0.01)
end

% ===================== SHUTDOWN =====================
writeline(s, "REC=0");  pause(0.02);
writeline(s, "STATE=0"); pause(0.02);
writeline(s, "STOP");

% ===================== BUILD TABLE =====================
T = table(time_s, state_code, meas_rpm, cmd_rpm, ...
          load_kg_raw, load_kg_filt, wire_force_N, ...
          linear_mpm, loadcell_torque_nm, moteus_torque_nm);

% ===================== SAVE SUMMARY =====================
if saveAns == "y"
    runsRoot = fullfile(workDir, 'Wire_Tension_Runs');
    if ~exist(runsRoot,'dir'), mkdir(runsRoot); end

    summaryPath = fullfile(runsRoot, 'run_summary_log.csv');

    % Determine run number
    if exist(summaryPath,'file')
        oldT = readtable(summaryPath, 'TextType', 'string');
        runNum = max(oldT.run_number) + 1;
    else
        runNum = 1;
    end

    % Extract only the RUN phase data (state == 3) for summary stats
    idxRun = (state_code == 3);

    if any(idxRun)
        runLoad    = load_kg_filt(idxRun);
        runSpeed   = linear_mpm(idxRun);
        runLcTorq  = loadcell_torque_nm(idxRun);
        runMtTorq  = moteus_torque_nm(idxRun);
        runForce   = wire_force_N(idxRun);

        speedErr   = runSpeed - targetLinear_mpm;

        S = table;
        S.run_number                 = runNum;
        S.timestamp                  = string(datestr(now,'yyyy-mm-dd HH:MM:SS'));
        S.run_label                  = string(runLabel);
        S.target_speed_mpm           = targetLinear_mpm;
        S.kd_value                   = kd_value;
        S.moteus_pulley_dia_mm       = moteusPulleyDia_mm;
        S.stepper_pulley_dia_in      = driverPulleyDia_in;
        S.ramp_time_s                = rampTime_s;
        S.run_time_s                 = runTime_s;
        S.samples_at_speed           = nnz(idxRun);

        % Tension (kgf)
        S.mean_tension_kgf           = mean(runLoad,'omitnan');
        S.std_tension_kgf            = std(runLoad,'omitnan');
        S.max_tension_kgf            = max(runLoad,[],'omitnan');
        S.min_tension_kgf            = min(runLoad,[],'omitnan');

        % Wire force (N)
        S.mean_wire_force_N          = mean(runForce,'omitnan');
        S.std_wire_force_N           = std(runForce,'omitnan');

        % Speed
        S.mean_speed_mpm             = mean(runSpeed,'omitnan');
        S.std_speed_mpm              = std(runSpeed,'omitnan');
        S.rmse_speed_mpm             = sqrt(mean(speedErr.^2,'omitnan'));
        S.mean_speed_error_pct       = mean(100*speedErr./targetLinear_mpm,'omitnan');

        % Load-cell inferred torque
        S.mean_lc_torque_nm          = mean(runLcTorq,'omitnan');
        S.std_lc_torque_nm           = std(runLcTorq,'omitnan');

        % Moteus measured torque
        S.mean_moteus_torque_nm      = mean(runMtTorq,'omitnan');
        S.std_moteus_torque_nm       = std(runMtTorq,'omitnan');

        % Correlation between the two torque measurements
        goodIdx = isfinite(runLcTorq) & isfinite(runMtTorq);
        if nnz(goodIdx) >= 2
            S.torque_correlation     = corr(runLcTorq(goodIdx), runMtTorq(goodIdx));
        else
            S.torque_correlation     = NaN;
        end

        % Append to master CSV
        appendSummaryCsv(summaryPath, S);

        fprintf('\n=== Run #%d saved to %s ===\n', runNum, summaryPath);
    else
        fprintf('\nWarning: no samples at full speed — nothing saved.\n');
    end

    % Save the figure
    figPath = fullfile(runsRoot, sprintf('run_%03d_dashboard.png', runNum));
    exportgraphics(f, figPath, 'Resolution', 150);
    fprintf('Figure saved: %s\n', figPath);
end
end

% =====================================================================
%  HELPER FUNCTIONS
% =====================================================================

function appendSummaryCsv(csvPath, Snew)
if exist(csvPath,'file')
    Told = readtable(csvPath, 'TextType', 'string');
    % Ensure columns match — add missing columns as NaN
    newVars = Snew.Properties.VariableNames;
    oldVars = Told.Properties.VariableNames;
    for i = 1:numel(newVars)
        if ~ismember(newVars{i}, oldVars)
            Told.(newVars{i}) = repmat({NaN}, height(Told), 1);
        end
    end
    for i = 1:numel(oldVars)
        if ~ismember(oldVars{i}, newVars)
            Snew.(oldVars{i}) = NaN;
        end
    end
    Tall = [Told; Snew];
else
    Tall = Snew;
end
writetable(Tall, csvPath);
end

function name = stateCodeName(code)
switch round(code)
    case 0, name = "IDLE";
    case 1, name = "WAIT_ZERO";
    case 2, name = "RAMP";
    case 3, name = "RUN";
    otherwise, name = "UNKNOWN";
end
end

function v = inputDefault(prompt, defaultVal)
txt = strtrim(input(prompt,'s'));
if isempty(txt)
    v = defaultVal;
else
    v = str2double(txt);
    if isnan(v), v = defaultVal; end
end
end

function launchMoteusLogger(workDir, pyScript, moteusId, kd, tauMax, rateHz)
cmd = sprintf(['start "moteus_logger" cmd /k "cd /d "%s" && py "%s" --id %d ' ...
               '--kd %.6f --tau_max %.6f --rate %.6f ' ...
               '--watchdog 0.1 --plot ' ...
               '--csv moteus_latest_torque.csv"'], ...
               workDir, pyScript, round(moteusId), kd, tauMax, rateHz);
system(cmd);
end

function [tauEst, velRPM, elapsed_s, unixTime, kdApplied, stamp] = readLatestMoteusTorque(csvPath, prevStamp)
tauEst = NaN; velRPM = NaN; elapsed_s = NaN; unixTime = NaN; kdApplied = NaN;
stamp = prevStamp;

if ~exist(csvPath,'file'), return; end

d = dir(csvPath);
thisStamp = sprintf('%0.12f_%d', d.datenum, d.bytes);
if strcmp(thisStamp, prevStamp), return; end

try
    TT = readtable(csvPath, 'TextType', 'string');
    if height(TT) < 1, return; end

    vars = string(TT.Properties.VariableNames);
    n = height(TT);

    idx = find(strcmpi(vars,'measured_torque_nm'),1);
    if isempty(idx), return; end
    tauEst = tableValNum(TT{n,idx});

    idx = find(strcmpi(vars,'velocity_rpm'),1);
    if ~isempty(idx), velRPM = tableValNum(TT{n,idx}); end

    idx = find(strcmpi(vars,'elapsed_s'),1);
    if ~isempty(idx), elapsed_s = tableValNum(TT{n,idx}); end

    idx = find(strcmpi(vars,'timestamp_unix'),1);
    if ~isempty(idx), unixTime = tableValNum(TT{n,idx}); end

    idx = find(strcmpi(vars,'kd_applied'),1);
    if ~isempty(idx), kdApplied = tableValNum(TT{n,idx}); end

    stamp = thisStamp;
catch
end
end

function x = tableValNum(v)
if isnumeric(v), x = double(v);
elseif isstring(v) || ischar(v), x = str2double(v);
else, try x = double(v); catch, x = NaN; end
end
if isempty(x) || ~isscalar(x), x = NaN; end
end

function localCleanup(s)
try
    if ~isempty(s)
        writeline(s, "REC=0");  pause(0.02);
        writeline(s, "STATE=0"); pause(0.02);
        writeline(s, "STOP");
    end
catch
end
try clear s; catch, end
end