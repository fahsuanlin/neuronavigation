function TMS_Status()
% TMS_Status  Print latest LabVIEW 44-byte status from async receiver.
    KEY = 'TMS_TCP_SESSION';
    if ~isappdata(0,KEY)
        disp('TMS TCP 尚未啟動。');
        return;
    end
    s = getappdata(0,KEY);
    fprintf('TX 已傳送 %d 筆，RX 已接收 %d 筆。\n',s.txCount,s.rxCount);
    if isempty(s.latest)
        disp('尚未收到 LabVIEW 完整的 44 Bytes 回傳。');
        return;
    end
    f = s.latest;
    fprintf('X     : %9.3f | Moving %d\n',f(2),round(f(1)));
    fprintf('Y     : %9.3f | Moving %d\n',f(4),round(f(3)));
    fprintf('Z     : %9.3f | Moving %d\n',f(6),round(f(5)));
    fprintf('Psi   : %9.3f | Moving %d\n',f(8),round(f(7)));
    fprintf('Theta : %9.3f\n',f(10));
    fprintf('Phi   : %9.3f | Curve Moving %d\n',f(11),round(f(9)));
    disp('(Moving 定義依你目前 LV：1=停止、0=移動)');
end
