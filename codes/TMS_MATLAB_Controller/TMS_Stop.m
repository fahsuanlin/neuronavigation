function TMS_Stop()
% TMS_Stop  Stop MATLAB-side periodic command monitoring and TCP client.
% This does NOT stop the LabVIEW motor control VI or physical motors.
    KEY = 'TMS_TCP_SESSION';
    assignin('base','stop',1);
    if ~isappdata(0,KEY)
        disp('TMS TCP 尚未啟動或已停止。');
        return;
    end
    s = getappdata(0,KEY);
    % Remove state first to prevent nested callbacks accessing stale client
    rmappdata(0,KEY);
    try
        if ~isempty(s.timer) && isvalid(s.timer)
            stop(s.timer);
            delete(s.timer);
        end
    catch ME
        warning('停止 Timer 時發生問題：%s',ME.message);
    end
    try
        configureCallback(s.client,'off');
    catch
    end
    try
        if ~isempty(s.ui) && isvalid(s.ui.fig)
            delete(s.ui.fig);
        end
    catch
    end
    % When session/timer/handles go out of scope, the tcpclient is released.
    clear s;
    disp('MATLAB TCP 控制與監控已停止。');
    disp('注意：LabVIEW Motor Loops / 實體馬達不會因此自動停止。');
end
