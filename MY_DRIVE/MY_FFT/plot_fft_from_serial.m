%% STM32H7 FFT serial viewer (real-time spectrum only)
% Read FFT frames from USART and plot magnitude spectrum in real time.
% Controls: Space = pause/resume

clear; clc;

portName = "COM9";
baudRate = 115200;
timeoutSec = 10;

sp = serialport(portName, baudRate, "Timeout", timeoutSec);
configureTerminator(sp, "CR/LF");
flush(sp);

fprintf("Connected: %s @ %d\n", portName, baudRate);

fig = figure("Name", "STM32H7 FFT Viewer", "Color", "w");
ax = axes(fig);
fig.WindowKeyPressFcn = @onKeyPress;
grid(ax, "on");
xlabel(ax, "Frequency (Hz)");
ylabel(ax, "Magnitude (V)");
title(ax, "Real-time FFT (SPACE pause/resume)");

hLine = plot(ax, 0, 0, "LineWidth", 1.2);
hold(ax, "on");
hPeak = plot(ax, nan, nan, "ro", "MarkerSize", 8, "LineWidth", 1.5);
hText = text(ax, nan, nan, "", "Color", "r", "FontSize", 11, "FontWeight", "bold", ...
    "VerticalAlignment", "bottom", "HorizontalAlignment", "left");
hold(ax, "off");

axtoolbar(ax, {"zoomin", "zoomout", "pan", "datacursor", "restoreview"});
fprintf("Hotkeys: SPACE pause/resume\n");

paused = false;
store = struct("paused", false);
setappdata(fig, "store", store);

fs = 512000;
N = 4096;

while isvalid(fig)
    drawnow limitrate;

    if sp.NumBytesAvailable <= 0
        pause(0.005);
        continue;
    end

    line = strtrim(readline(sp));

    if startsWith(line, "FFT_BEGIN")
        tokFs = regexp(line, "Fs=([0-9]+\.?[0-9]*)", "tokens", "once");
        tokN  = regexp(line, "N=([0-9]+)", "tokens", "once");
        if ~isempty(tokFs), fs = str2double(tokFs{1}); end
        if ~isempty(tokN),  N  = str2double(tokN{1});  end

        halfN = floor(N/2);
        mag = nan(halfN, 1);

        while true
            if ~isvalid(fig), break; end
            drawnow limitrate;

            if sp.NumBytesAvailable <= 0
                pause(0.002);
                continue;
            end

            d = strtrim(readline(sp));
            if strcmp(d, "FFT_END"), break; end

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

        s = getappdata(fig, "store");
        if ~s.paused
            set(hLine, "XData", f, "YData", mag);

            % find peak (skip DC bin 0)
            [peakMag, peakIdx] = max(mag(2:end));
            peakFreq = f(peakIdx + 1);
            set(hPeak, "XData", peakFreq, "YData", peakMag);
            set(hText, "Position", [peakFreq, peakMag, 0], ...
                "String", sprintf("%.1f Hz", peakFreq));

            title(ax, sprintf("Fs=%.2f Hz  N=%d  Peak=%.1f Hz  [LIVE]", fs, N, peakFreq));
            drawnow limitrate;
        else
            title(ax, sprintf("Fs=%.2f Hz  N=%d  [PAUSED]", fs, N));
            drawnow limitrate;
        end
    end
end

if exist("sp", "var"), clear sp; end

function onKeyPress(src, event)
s = getappdata(src, "store");
if event.Key == "space"
    s.paused = ~s.paused;
    setappdata(src, "store", s);
end
end
