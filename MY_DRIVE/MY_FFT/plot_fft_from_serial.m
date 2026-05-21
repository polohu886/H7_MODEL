%% STM32H7 FFT serial viewer (harmonic-aware)
% Read FFT frames from USART and plot magnitude spectrum in real time.
% Frame format from firmware (Phase.c):
%   FFT_BEGIN,Fs=xxxxx.xx,N=xxxx
%   idx,mag
%   ...
%   FFT_END
% Controls:
%   Space : pause/resume live update
%   p     : print detected harmonic peaks

clear; clc;

portName = "COM11";      % TODO: change to your serial port
baudRate = 115200;
timeoutSec = 2;

% Auto waveform classification from harmonic pattern.
maxOddHarmonics = 9;           % internal detection depth
searchBinsAroundHarmonic = 2;  % +- bins around expected harmonic bin
minPeakDbOverNoise = 8;        % minimum prominence over noise floor (dB)
classifyMinOddDb = -40;        % 3rd/5th lower than this (dBc) is treated as near-noise

sp = serialport(portName, baudRate, "Timeout", timeoutSec);
configureTerminator(sp, "CR/LF");
flush(sp);

fprintf("Connected: %s @ %d\n", portName, baudRate);

fig = figure("Name", "STM32H7 FFT Viewer", "Color", "w");
ax = axes(fig);
fig.WindowKeyPressFcn = @onKeyPress;
grid(ax, "on");
xlabel(ax, "Frequency (Hz)");
ylabel(ax, "Magnitude");
title(ax, "Real-time FFT (SPACE pause/resume)");

hLine = plot(ax, 0, 0, "LineWidth", 1.2);
hold(ax, "on");
hFund = plot(ax, nan, nan, "o", "MarkerSize", 8, "LineWidth", 1.5, ...
    "Color", [0.15 0.25 0.85], "DisplayName", "Fundamental");
hHarm = plot(ax, nan, nan, "ro", "MarkerSize", 7, "LineWidth", 1.2, ...
    "DisplayName", "Odd Harmonics");
hText = gobjects(maxOddHarmonics, 1);
for i = 1:maxOddHarmonics
    hText(i) = text(ax, nan, nan, "", "Color", [0.75 0 0], "FontSize", 10, "VerticalAlignment", "bottom");
end
hold(ax, "off");
legend(ax, "show", "Location", "northeast");

% Use axes toolbar interactions instead of legacy modes,
% so keyboard callbacks (space/p) remain available.
axtoolbar(ax, {"zoomin", "zoomout", "pan", "datacursor", "restoreview"});
fprintf("Hotkeys: SPACE pause/resume, P print classification/markers\n");

state.paused = false;
state.lastF = [];
state.lastMag = [];
state.lastHarmonics = [];
state.lastFundamental = [];
state.lastWaveType = "UNKNOWN";
state.lastMarkers = [];
state.lastDiag = struct();
setappdata(fig, "viewerState", state);

fs = 200000;
N = 1024;

while isvalid(fig)
    drawnow limitrate;

    if sp.NumBytesAvailable <= 0
        pause(0.005);
        continue;
    end

    line = strtrim(readline(sp));

    if startsWith(line, "FFT_BEGIN")
        tokFs = regexp(line, "Fs=([0-9]+\.?[0-9]*)", "tokens", "once");
        tokN = regexp(line, "N=([0-9]+)", "tokens", "once");
        if ~isempty(tokFs), fs = str2double(tokFs{1}); end
        if ~isempty(tokN), N = str2double(tokN{1}); end

        halfN = floor(N/2);
        mag = nan(halfN, 1);

        while true
            if ~isvalid(fig)
                break;
            end

            drawnow limitrate;

            if sp.NumBytesAvailable <= 0
                pause(0.002);
                continue;
            end

            d = strtrim(readline(sp));
            if strcmp(d, "FFT_END")
                break;
            end

            parts = split(d, ",");
            if numel(parts) == 2
                k = str2double(parts{1});
                v = str2double(parts{2});
                if ~isnan(k) && ~isnan(v)
                    idx = k + 1;
                    if idx >= 1 && idx <= halfN
                        mag(idx) = v;
                    end
                end
            end
        end

        f = (0:halfN-1).' * (fs / N);

        state = getappdata(fig, "viewerState");
        state.lastF = f;
        state.lastMag = mag;

        result = detectOddHarmonics(f, mag, maxOddHarmonics, searchBinsAroundHarmonic, minPeakDbOverNoise, classifyMinOddDb);
        state.lastFundamental = result.fundamental;
        state.lastHarmonics = result.harmonics;
        state.lastWaveType = result.waveType;
        state.lastMarkers = result.markers;
        state.lastDiag = result.diag;
        setappdata(fig, "viewerState", state);

        if ~state.paused
            set(hLine, "XData", f, "YData", mag);
            if ~isempty(result.fundamental)
                set(hFund, "XData", result.fundamental(1), "YData", result.fundamental(2));
            else
                set(hFund, "XData", nan, "YData", nan);
            end

            if ~isempty(result.markers)
                set(hHarm, "XData", result.markers(:, 2), "YData", result.markers(:, 3));
            else
                set(hHarm, "XData", nan, "YData", nan);
            end

            for i = 1:numel(hText)
                if ~isempty(result.markers) && i <= size(result.markers, 1)
                    ord = result.markers(i, 1);
                    pf = result.markers(i, 2);
                    pm = result.markers(i, 3);
                    set(hText(i), "Position", [pf, pm, 0], ...
                        "String", sprintf("%dth: %.1f Hz", ord, pf), "Visible", "on");
                else
                    set(hText(i), "Visible", "off");
                end
            end

            title(ax, sprintf("Real-time FFT | Type=%s | Fs=%.2f Hz  N=%d  [LIVE]", result.waveType, fs, N));
            drawnow limitrate;
        else
            title(ax, sprintf("Real-time FFT | Type=%s | Fs=%.2f Hz  N=%d  [PAUSED]", result.waveType, fs, N));
            drawnow limitrate;
        end
    end
end

if exist("sp", "var")
    clear sp;
end

function result = detectOddHarmonics(f, mag, maxOddHarmonics, searchBinsAround, minPeakDbOverNoise, classifyMinOddDb)
result.fundamental = [];
result.harmonics = [];
result.waveType = "UNKNOWN";
result.markers = [];
result.harmonicsRaw = [];
result.diag = struct("r3db", nan, "r5db", nan, "fundScore", nan);

valid = ~isnan(mag) & ~isnan(f);
f = f(valid);
mag = mag(valid);
if numel(mag) < 8
    return;
end

% Remove DC bin for peak detection.
f2 = f(2:end);
m2 = mag(2:end);
if numel(m2) < 8
    return;
end

df = f(2) - f(1);
if df <= 0
    return;
end

noiseFloor = median(m2);
noiseFloor = max(noiseFloor, 1e-12);

% Find local maxima as candidate peaks.
cand = findLocalMaxima(m2);
if isempty(cand)
    return;
end

% Select fundamental by harmonic-consistency score instead of pure max peak.
fundIdx = selectFundamentalCandidate(cand, m2, noiseFloor, minPeakDbOverNoise);

if fundIdx < 0
    return;
end

[fFund, mFund] = refinePeakByParabola(f2, m2, fundIdx);
result.fundamental = [fFund, mFund];

nyquist = f(end);
harm = zeros(maxOddHarmonics, 3);
harmRaw = zeros(maxOddHarmonics, 3);
count = 0;
countRaw = 0;

for odd = 1:2:(2 * maxOddHarmonics - 1)
    fExp = odd * fFund;
    if fExp >= nyquist
        break;
    end

    binExp = round(fExp / df) + 1;
    binExp2 = binExp - 1;
    if binExp2 < 1 || binExp2 > numel(m2)
        continue;
    end

    span = max(searchBinsAround, max(2, round(0.015 * binExp2)));
    lo = max(1, binExp2 - span);
    hi = min(numel(m2), binExp2 + span);
    [localMag, rel] = max(m2(lo:hi));
    localIdx = lo + rel - 1;

    [pf, pm] = refinePeakByParabola(f2, m2, localIdx);
    countRaw = countRaw + 1;
    harmRaw(countRaw, :) = [odd, pf, pm];

    if localMag <= noiseFloor
        continue;
    end

    localDb = 20 * log10(localMag / noiseFloor);
    if localDb < minPeakDbOverNoise
        continue;
    end

    count = count + 1;
    harm(count, :) = [odd, pf, pm];
end

if count > 0
    result.harmonics = harm(1:count, :);
end

if countRaw > 0
    result.harmonicsRaw = harmRaw(1:countRaw, :);
end

result.waveType = classifyWaveform(result.harmonicsRaw, noiseFloor, classifyMinOddDb);
result.markers = selectMarkersByWaveType(result.waveType, result.harmonicsRaw);

m1d = getHarmonicMag(result.harmonicsRaw, 1);
m3d = getHarmonicMag(result.harmonicsRaw, 3);
m5d = getHarmonicMag(result.harmonicsRaw, 5);
if isempty(m1d), m1d = noiseFloor; end
if isempty(m3d), m3d = noiseFloor; end
if isempty(m5d), m5d = noiseFloor; end
result.diag.r3db = 20 * log10(max(m3d, 1e-12) / max(m1d, 1e-12));
result.diag.r5db = 20 * log10(max(m5d, 1e-12) / max(m1d, 1e-12));
result.diag.fundScore = m1d + 0.7*m3d + 0.5*m5d;
end

function fundIdx = selectFundamentalCandidate(cand, m2, noiseFloor, minPeakDbOverNoise)
fundIdx = -1;

if isempty(cand)
    return;
end

candMag = m2(cand);
[~, sidx] = sort(candMag, "descend");
topN = min(24, numel(sidx));

bestScore = -inf;
bestIdx = -1;
for k = 1:topN
    idx = cand(sidx(k));
    pk = m2(idx);
    pkDb = 20 * log10(max(pk, 1e-12) / noiseFloor);
    if pkDb < minPeakDbOverNoise
        continue;
    end

    s1 = pk;
    s3 = localPeakAtRatio(m2, idx, 3);
    s5 = localPeakAtRatio(m2, idx, 5);
    lowFreqBias = 1.0 / (1.0 + 0.01 * idx);
    score = (s1 + 0.7 * s3 + 0.5 * s5) * lowFreqBias;

    if score > bestScore
        bestScore = score;
        bestIdx = idx;
    end
end

fundIdx = bestIdx;
end

function pk = localPeakAtRatio(m2, idxBase, ratio)
pk = 0;
idxExp = round(double(idxBase) * double(ratio));
if idxExp < 1 || idxExp > numel(m2)
    return;
end

span = max(2, round(0.02 * idxExp));
lo = max(1, idxExp - span);
hi = min(numel(m2), idxExp + span);
pk = max(m2(lo:hi));
end

function waveType = classifyWaveform(harmonics, noiseFloor, classifyMinOddDb)
waveType = "UNKNOWN";

if isempty(harmonics)
    return;
end

m1 = getHarmonicMag(harmonics, 1);
m3 = getHarmonicMag(harmonics, 3);
m5 = getHarmonicMag(harmonics, 5);
m7 = getHarmonicMag(harmonics, 7);
m9 = getHarmonicMag(harmonics, 9);
m11 = getHarmonicMag(harmonics, 11);
m2 = getHarmonicMag(harmonics, 2);
m4 = getHarmonicMag(harmonics, 4);

if isempty(m1)
    return;
end

if isempty(m3), m3 = noiseFloor; end
if isempty(m5), m5 = noiseFloor; end
if isempty(m7), m7 = noiseFloor; end
if isempty(m9), m9 = noiseFloor; end
if isempty(m11), m11 = noiseFloor; end
if isempty(m2), m2 = noiseFloor; end
if isempty(m4), m4 = noiseFloor; end

r3 = m3 / max(m1, 1e-12);
r5 = m5 / max(m1, 1e-12);
r7 = m7 / max(m1, 1e-12);
r9 = m9 / max(m1, 1e-12);
r11 = m11 / max(m1, 1e-12);
r2 = m2 / max(m1, 1e-12);
r4 = m4 / max(m1, 1e-12);

r3db = 20 * log10(max(r3, 1e-12));
r5db = 20 * log10(max(r5, 1e-12));
r7db = 20 * log10(max(r7, 1e-12));
r9db = 20 * log10(max(r9, 1e-12));
r11db = 20 * log10(max(r11, 1e-12));

% ====== STEP 1: SINE DETECTION ======
% Key: Check 7th/9th/11th harmonics - sine wave has almost NO high-order harmonics
noiseThreshold = noiseFloor * 3;
if m7 < noiseThreshold && m9 < noiseThreshold && m11 < noiseThreshold
    if r3db < -20
        waveType = "SINE";
        return;
    end
end

% Backup: all odd harmonics very weak
sineThreshold = -35;
if r3db < sineThreshold && r5db < sineThreshold && r7db < sineThreshold
    waveType = "SINE";
    return;
end

% ====== STEP 2: SQUARE vs TRIANGLE ======
sq3db = -9.54; sq5db = -13.98; sq7db = -16.90; sq9db = -19.08;
tri3db = -20.0; tri5db = -27.96; tri7db = -33.0; tri9db = -36.99;

significantDb = noiseFloor * 2;
errSquare = 0; errTriangle = 0; wSq = 0; wTri = 0;

% 3rd (highest weight)
if m3 > significantDb
    errSquare = errSquare + abs(r3db - sq3db) * 1.5;
    errTriangle = errTriangle + abs(r3db - tri3db) * 1.5;
    wSq = wSq + 1.5; wTri = wTri + 1.5;
end

% 5th
if m5 > significantDb
    errSquare = errSquare + abs(r5db - sq5db);
    errTriangle = errTriangle + abs(r5db - tri5db);
    wSq = wSq + 1; wTri = wTri + 1;
end

% 7th (key differentiator)
if m7 > significantDb
    errSquare = errSquare + abs(r7db - sq7db) * 0.8;
    errTriangle = errTriangle + abs(r7db - tri7db) * 0.8;
    wSq = wSq + 0.8; wTri = wTri + 0.8;
end

if wSq == 0
    waveType = "SINE";
    return;
end

errSquare = errSquare / wSq;
errTriangle = errTriangle / wTri;

% ====== STEP 3: Final decision ======
margin = 3;

if errSquare < errTriangle - margin
    waveType = "SQUARE";
elseif errTriangle < errSquare - margin
    waveType = "TRIANGLE";
else
    % Close call: use 7th harmonic for final decision
    if m7 > significantDb
        if r7db > -25
            waveType = "SQUARE";
        else
            waveType = "TRIANGLE";
        end
    else
        if errSquare <= errTriangle
            waveType = "SQUARE";
        else
            waveType = "TRIANGLE";
        end
    end
end

end

function [ok, slopeVal] = oddSlope(harmonics)
ok = false;
slopeVal = 0.0;

if isempty(harmonics)
    return;
end

ord = harmonics(:,1);
mag = harmonics(:,3);
v = (ord >= 1) & (mag > 0);
ord = ord(v);
mag = mag(v);

if numel(ord) < 3
    return;
end

x = log10(double(ord));
y = log10(double(mag));
p = polyfit(x, y, 1);
slopeVal = p(1);
ok = true;
end

function markers = selectMarkersByWaveType(waveType, harmonics)
markers = [];
if isempty(harmonics)
    return;
end

switch upper(waveType)
    case "SINE"
        markers = pickHarmonics(harmonics, [1]);
    case "SQUARE"
        markers = pickHarmonics(harmonics, [1 3 5]);
    case "TRIANGLE"
        markers = pickHarmonics(harmonics, [1 3 5]);
    otherwise
        markers = pickHarmonics(harmonics, [1 3 5]);
end
end

function out = pickHarmonics(harmonics, orders)
out = [];
for i = 1:numel(orders)
    ord = orders(i);
    idx = find(harmonics(:,1) == ord, 1, "first");
    if ~isempty(idx)
        out = [out; harmonics(idx, :)];
    end
end
end

function m = getHarmonicMag(harmonics, order)
idx = find(harmonics(:,1) == order, 1, "first");
if isempty(idx)
    m = [];
else
    m = harmonics(idx, 3);
end
end

function [pf, pm] = refinePeakByParabola(f, m, idx)
pf = f(idx);
pm = m(idx);

if idx < 2 || idx >= numel(f)
    return;
end

y1 = m(idx-1);
y2 = m(idx);
y3 = m(idx+1);

denom = y1 - 2*y2 + y3;
if abs(denom) < 1e-12
    return;
end

delta = 0.5 * (y1 - y3) / denom;
if abs(delta) > 1
    return;
end

pf = f(idx) + delta * (f(idx+1) - f(idx-1)) / 2;
pm = y2 - 0.25 * (y1 - y3) * delta;
end

function cand = findLocalMaxima(m)
cand = [];
n = numel(m);
if n < 3
    return;
end

for i = 2:(n-1)
    if m(i) > m(i-1) && m(i) > m(i+1)
        cand = [cand, i]; %#ok<AGROW>
    end
end
end

function onKeyPress(src, event)
fig = src;
state = getappdata(fig, "viewerState");

if event.Key == "space"
    state.paused = ~state.paused;
    setappdata(fig, "viewerState", state);
elseif event.Key == "p"
    fprintf("=== Current State ===\n");
    fprintf("Wave Type: %s\n", state.lastWaveType);
    if ~isempty(state.lastFundamental)
        fprintf("Fundamental: %.1f Hz, Mag: %.2f\n", state.lastFundamental(1), state.lastFundamental(2));
    end
    if ~isempty(state.lastHarmonics)
        fprintf("Harmonics:\n");
        disp(state.lastHarmonics);
    end
    if isfield(state.lastDiag, 'r3db')
        fprintf("r3=%.2f dBc, r5=%.2f dBc\n", state.lastDiag.r3db, state.lastDiag.r5db);
    end
end
end