function TMS_Start()
% TMS_Start  Interactive MATLAB <-> LabVIEW TCP controller
% TX: 18 single (72 bytes); RX: 11 single (44 bytes)
% Command Window: x_pos = 2;  or  y_pos = 1;
% Stop with: stop = 1;  or  TMS_Stop
%
% IMPORTANT: Edit HOME values below to match your own LabVIEW HOME
% settings BEFORE running with hardware enabled. These are startup
% command targets, not a physical homing routine.

    KEY = 'TMS_TCP_SESSION';
    if isappdata(0, KEY)
        error('TMS TCP 已啟動。請先執行 TMS_Stop。');
    end

    %% ===== 請先確認這一區的設定 =====
    ip = '172.22.11.2';
    port = 6340;

    % 初始目標值：先沿用前次 TCP 測試值，不代表實際機械歸零值
    home_x = 0;
    home_y = 0;
    home_z = 0;
    home_psi = 0;
    home_theta = 0;
    home_phi = 53.1916;   % 特別確認！這只是先前測試值，非已知 HOME

    default_speed = 4;
    default_curve_speed = 4;
    default_tab_mode = 0;  % 0 = Azimuth/Theta; 1 = Elevation/Phi

    show_panel = true;
    poll_period = 0.15; % 每 150 ms 檢查 Command Window 變數是否改變
    %% =================================

    % 變數放在 Base Workspace，可在 Command Window 直接更改
    initNames = {'x_pos','y_pos','z_pos','psi_pos','theta_pos','phi_pos', ...
                 'x_speed','y_speed','z_speed','psi_speed', ...
                 'curve_speed','tab_mode','stop'};
    initValues = {home_x,home_y,home_z,home_psi,home_theta,home_phi, ...
                  default_speed,default_speed,default_speed,default_speed, ...
                  default_curve_speed,default_tab_mode,0};
    for k = 1:numel(initNames)
        assignin('base', initNames{k}, initValues{k});
    end

    fprintf('連線到 LabVIEW %s:%d ...\n', ip, port);
    client = tcpclient(ip, port, 'Timeout', 3);

    session = struct();
    session.client = client;
    session.timer = [];
    session.ui = [];
    session.lastSent = single([]);
    session.latest = [];
    session.rxCount = 0;
    session.txCount = 0;
    session.lastRxAt = NaN;
    session.beginTime = tic;
    setappdata(0, KEY, session);

    try
        % 每收到至少 44 bytes，觸發一次回傳解析
        configureCallback(client, 'byte', 44, @onFeedback);

        if show_panel
            panel = createPanel();
            session = getappdata(0, KEY);
            session.ui = panel;
            setappdata(0, KEY, session);
        end

        % 啟動時傳一次完整 72 bytes，Jog 全部為 0
        sendIfChanged(true);

        % Timer 只檢查 Command Window 數值改變；不是持續重送封包
        watcher = timer('ExecutionMode','fixedSpacing', ...
                        'Period',poll_period, ...
                        'TimerFcn',@onTimer, ...
                        'ErrorFcn',@onTimerError);
        session = getappdata(0, KEY);
        session.timer = watcher;
        setappdata(0, KEY, session);
        start(watcher);

        fprintf(['\nTMS TCP 已啟動。\n' ...
                 'Command Window 輸入：x_pos = 2   （自動送 72 Bytes）\n' ...
                 '其他軸：y_pos, z_pos, psi_pos, theta_pos, phi_pos\n' ...
                 '停止通訊：stop = 1   或   TMS_Stop\n' ...
                 '查看最近回傳：TMS_Status\n\n']);

    catch ME
        TMS_Stop();
        rethrow(ME);
    end

    %% ===== Nested callbacks =====

    function onTimer(~, ~)
        if ~isappdata(0, KEY)
            return;
        end
        try
            if evalin('base','exist(''stop'',''var'')') && ...
                    isequal(evalin('base','stop'),1)
                TMS_Stop();
                return;
            end
            sendIfChanged(false);
            refreshPanel();
        catch ME
            % 不讓一次無效的輸入終止整個 TCP 接收
            persistent lastTimerMessage
            if isempty(lastTimerMessage) || ~strcmp(lastTimerMessage,ME.message)
                warning('TMS 監控：%s', ME.message);
                lastTimerMessage = ME.message;
            end
        end
    end

    function sendIfChanged(force)
        if ~isappdata(0, KEY)
            return;
        end
        names = {'x_speed','x_pos','y_speed','y_pos', ...
                 'z_speed','z_pos','psi_speed','psi_pos', ...
                 'tab_mode','curve_speed','theta_pos','phi_pos'};
        v = zeros(1,numel(names));
        for j = 1:numel(names)
            if ~evalin('base',sprintf('exist(''%s'',''var'')',names{j}))
                error('找不到變數 %s，請勿 clear 單一控制參數。',names{j});
            end
            value = evalin('base',names{j});
            if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) || ...
                    ~isfinite(value)
                error('%s 必須為有限的純量數字。',names{j});
            end
            v(j) = double(value);
        end
        if ~ismember(v(9),[0 1])
            error('tab_mode 只能是 0（Theta）或 1（Phi）。');
        end

        % 固定 18 SGL，完全符合現有 LabVIEW Main VI 的 byte offsets
        % Offset 48=tab, 52=curve speed, 56=theta, 60=0,
        %        64=phi, 68=0. 全部 Jog 固定 0。
        packet = single([ ...
            v(1),v(2),0, v(3),v(4),0, v(5),v(6),0, ...
            v(7),v(8),0, v(9),v(10),v(11),0,v(12),0]);
        if numel(packet) ~= 18
            error('封包建立錯誤：應為 18 個 single。');
        end

        s = getappdata(0, KEY);
        if ~force && isequal(packet,s.lastSent)
            return;
        end

        bytes = typecast(swapbytes(packet),'uint8');
        write(s.client,bytes,'uint8');
        s.lastSent = packet;
        s.txCount = s.txCount + 1;
        setappdata(0, KEY, s);
        fprintf('TX #%d | X=%.3f Y=%.3f Z=%.3f Psi=%.3f Theta=%.3f Phi=%.3f Mode=%d\n', ...
            s.txCount,v(2),v(4),v(6),v(8),v(11),v(12),round(v(9)));
    end

    function onFeedback(src, ~)
        if ~isappdata(0, KEY)
            return;
        end
        try
            s = getappdata(0, KEY);
            while src.NumBytesAvailable >= 44
                rx = read(src,44,'uint8');
                f = double(swapbytes(typecast(uint8(rx(:).'),'single')));
                if numel(f) ~= 11
                    error('RX 資料長度異常。');
                end
                % LabVIEW 的現行 Build Array：Moving 再 Real Output
                % [XMoving XReal YMoving YReal ZMoving ZReal
                %  PsiMoving PsiReal CurveMoving ThetaReal PhiReal]
                s.latest = f(:);
                s.rxCount = s.rxCount + 1;
                s.lastRxAt = toc(s.beginTime);
            end
            setappdata(0, KEY, s);
        catch ME
            warning('TCP 收到資料但無法解析：%s',ME.message);
        end
    end

    function onTimerError(~,event)
        warning('MATLAB Timer callback 錯誤，請檢查 Command Window。');
        if isfield(event,'Data') && isfield(event.Data,'message')
            disp(event.Data.message);
        end
    end

    function ui = createPanel()
        fig = uifigure('Name','TMS TCP 控制與監控', ...
                       'Position',[110 90 900 630], ...
                       'CloseRequestFcn',@(~,~)TMS_Stop());
        uilabel(fig,'Position',[25 588 820 28], ...
            'Text','TMS MATLAB ↔ LabVIEW  |  72B TX / 44B RX', ...
            'FontSize',18,'FontWeight','bold');
        status = uilabel(fig,'Position',[25 555 840 24], ...
            'Text','TCP 已連線，等待 LabVIEW 回傳...', 'FontSize',12);
        uilabel(fig,'Position',[25 520 120 22], ...
            'Text','軸別','FontWeight','bold');
        uilabel(fig,'Position',[155 520 120 22], ...
            'Text','目標位置（可編輯）','FontWeight','bold');
        uilabel(fig,'Position',[365 520 120 22], ...
            'Text','LV 回傳數值','FontWeight','bold');
        uilabel(fig,'Position',[600 520 210 22], ...
            'Text','Moving：1=停、0=動','FontWeight','bold');

        axesNames = {'X','Y','Z','Psi','Theta','Phi'};
        fieldNames = {'x_pos','y_pos','z_pos','psi_pos','theta_pos','phi_pos'};
        target = cell(6,1);
        realText = cell(6,1);
        moveText = cell(6,1);
        for i = 1:6
            y = 480-(i-1)*48;
            uilabel(fig,'Position',[30 y 120 28], ...
                'Text',axesNames{i},'FontSize',14);
            f = uieditfield(fig,'numeric','Position',[150 y 130 30], ...
                'Value',evalin('base',fieldNames{i}));
            f.UserData = fieldNames{i};
            f.ValueChangedFcn = @(src,~)assignin('base',src.UserData,src.Value);
            target{i} = f;
            realText{i} = uilabel(fig,'Position',[370 y 150 30], ...
                'Text','—','FontSize',13);
            moveText{i} = uilabel(fig,'Position',[605 y 175 30], ...
                'Text','—','FontSize',12);
        end

        uilabel(fig,'Position',[25 181 520 24], ...
            'Text','Step/s（變更後自動傳送完整封包）', ...
            'FontWeight','bold');
        speedNames = {'x_speed','y_speed','z_speed','psi_speed','curve_speed'};
        speedLabels = {'X','Y','Z','Psi','Curve'};
        speed = cell(5,1);
        for i = 1:5
            x = 30+(i-1)*172;
            uilabel(fig,'Position',[x 149 58 22], ...
                'Text',speedLabels{i});
            f = uieditfield(fig,'numeric','Position',[x+50 145 105 29], ...
                'Value',evalin('base',speedNames{i}));
            f.UserData = speedNames{i};
            f.ValueChangedFcn = @(src,~)assignin('base',src.UserData,src.Value);
            speed{i} = f;
        end

        uilabel(fig,'Position',[30 103 115 25], ...
            'Text','Curve Tab Mode');
        mode = uidropdown(fig,'Position',[155 101 220 30], ...
            'Items',{'Theta / Azimuth (0)','Phi / Elevation (1)'}, ...
            'ItemsData',[0 1], 'Value',evalin('base','tab_mode'));
        mode.ValueChangedFcn = @(src,~)assignin('base','tab_mode',src.Value);

        uibutton(fig,'push','Position',[600 100 120 34], ...
            'Text','重新送出', 'ButtonPushedFcn',@(~,~)sendIfChanged(true));
        uibutton(fig,'push','Position',[740 100 120 34], ...
            'Text','停止 TCP', 'ButtonPushedFcn',@(~,~)TMS_Stop());
        uilabel(fig,'Position',[30 55 830 24], ...
            'Text','Command Window：x_pos = 2、y_pos = 3 ... ；stop = 1 停止通訊', ...
            'FontSize',12);
        uilabel(fig,'Position',[30 27 840 20], ...
            'Text','注意：停止 TCP 不等於停止實體馬達。操作前須完成 LV 原點復歸與安全限制檢查。');

        ui = struct('fig',fig,'status',status,'target',{target}, ...
                    'realText',{realText},'moveText',{moveText}, ...
                    'speed',{speed},'mode',mode);
    end

    function refreshPanel()
        if ~isappdata(0, KEY)
            return;
        end
        s = getappdata(0, KEY);
        if isempty(s.ui) || ~isvalid(s.ui.fig)
            return;
        end
        ui = s.ui;
        fieldNames = {'x_pos','y_pos','z_pos','psi_pos','theta_pos','phi_pos'};
        for i = 1:6
            v = evalin('base',fieldNames{i});
            if isnumeric(v) && isscalar(v) && isfinite(v) && ui.target{i}.Value ~= v
                ui.target{i}.Value = v;
            end
        end
        speedNames = {'x_speed','y_speed','z_speed','psi_speed','curve_speed'};
        for i = 1:5
            v = evalin('base',speedNames{i});
            if isnumeric(v) && isscalar(v) && isfinite(v) && ui.speed{i}.Value ~= v
                ui.speed{i}.Value = v;
            end
        end
        m = evalin('base','tab_mode');
        if ismember(m,[0 1]) && ui.mode.Value ~= m
            ui.mode.Value = m;
        end
        if isempty(s.latest)
            ui.status.Text = sprintf('TCP 已連線  |  TX %d  |  尚無完整 44 Bytes 回傳',s.txCount);
            return;
        end
        fb = s.latest;
        actual = fb([2,4,6,8,10,11]);
        moving = fb([1,3,5,7,9,9]);
        for i = 1:6
            ui.realText{i}.Text = sprintf('%.3f',actual(i));
            if moving(i) == 1
                ui.moveText{i}.Text = '1  停止 / Idle';
            elseif moving(i) == 0
                ui.moveText{i}.Text = '0  移動 / Moving';
            else
                ui.moveText{i}.Text = sprintf('未知：%.2f',moving(i));
            end
        end
        ui.status.Text = sprintf('TCP 已連線  |  TX %d  |  RX %d  |  最近 RX %.2f s', ...
            s.txCount,s.rxCount,s.lastRxAt);
    end
end
