function varargout=nav_robot_tab(app,action,varargin)
%
%   nav_robot_tab    the 'Robot' tab of nav_camera_terminal
%
%   nav_robot_tab(app,'init')                 startupFcn: style the tab, read-only marker fields, empty state
%   nav_robot_tab(app,'ensure')               from nav_camera_graphic_update on every frame: runs 'init' and
%                                             wires the Robot tab callbacks if that has not happened, and
%                                             refills MarkerDropDown when the tool list changes. The tab
%                                             therefore works even if the .mlapp lost its hook lines
%                                             (App Designer saves an older copy over outside edits).
%   nav_robot_tab(app,'populate')             after Tool check: fill app.MarkerDropDown with the tools
%   nav_robot_tab(app,'marker_changed')       MarkerDropDown callback: reset marker readout and its 3D dot
%   nav_robot_tab(app,'target_changed')       TargetX/Y/ZEditField callback: (re)define the target point
%   nav_robot_tab(app,'frame',idx,p)          from nav_camera_graphic_update for every tracked tool and
%                                             frame (p: 3x1 display coordinates, mm)
%   [p,valid]=nav_robot_tab(app,'marker_pos') current position of the chosen marker and whether it is valid
%   nav_robot_tab(app,'cmd_selected')         CommCmdDropDown callback: send a command without parameter at once;
%                                             for one with a parameter, wait for CommParamEditField
%   nav_robot_tab(app,'param_entered')        CommParamEditField callback: send '<cmd> <param>'
%   nav_robot_tab(app,'log',kind,str)         add a line to app.RobotCommTextArea (newest line on top)
%                                             (kind: '>' sent, '<' received, '*' status, '!' problem)
%
%   Marker panel : the tool chosen in MarkerDropDown; its live x, y, z (mm, same display
%                  coordinates as the Control tab: x right, y up, z distance from the camera)
%                  are shown in the read-only MarkerX/Y/ZEditField (greyed out, keeping the
%                  last value, while the marker is not valid).
%   Target panel : x, y, z typed into TargetX/Y/ZEditField define the target point (the
%                  first edit sets it; unset fields count as 0). While the marker is valid,
%                  TargetUITable shows marker - target: dx, dy, dz and the distance
%                  d = sqrt(dx^2+dy^2+dz^2).
%   Goto         : TargetGotoButton, nav_robot_goto: moves the robot (SET X/Y/Z, closed loop on
%                  the tracked marker) until the marker reaches the target. Press again to abort.
%   cmd. / param : CommCmdDropDown + CommParamEditField: manual commands to the robot PC
%                  (PING, HELP, GET STATUS, GET DELTA, GET DOC <name>, SET <axis> <value>,
%                  CALIBRATE <step mm>: camera/robot calibration used by Goto; never STOP).
%   Comm.        : nav_camera_tool_comm; while connected the same dx, dy, dz, d are sent
%                  as 'DELTA dx dy dz d' (~5 Hz) while the marker is valid and a target is set.
%   RobotUIAxes  : a dot in the tool's color for the marker (reset when the marker changes;
%                  grey while the marker is not seen) and a magenta dot for the target.
%
%   A marker is 'valid' when its latest pose arrived less than valid_s ago
%   (the tracking loop only reports tools it can see).
%
% fhlin@Nov 18, 2026
%

valid_s=0.5;          % marker pose older than this -> not valid
ui_period_s=0.1;      % fields / table / dots refresh (10 Hz)
send_period_s=0.2;    % DELTA to the robot (5 Hz)
target_color=[0.85 0 0.85];
lost_color=[0.6 0.6 0.6];
max_log_lines=300;
cmd_items={ ...   % CommCmdDropDown: command, parameter hint ('' = none). Never STOP.
    '(choose)',''; 'PING',''; 'GET STATUS',''; 'GET DELTA',''; 'HELP',''; ...
    'SET X','mm, -25..25'; 'SET Y','mm, -25..25'; 'SET Z','mm, -25..25'; 'SET PSI','mm, -25..25'; ...
    'SET THETA','absolute'; 'SET PHI','absolute'; 'SET L','relative steps'; 'SET R','relative steps'; ...
    'GET DOC','BRIEF|PROTOCOL|GUIDE'; 'CALIBRATE','step mm, e.g. 2'};

if(nargin<2 || isempty(action))
    action='init';
end;

switch lower(action)
    case 'ensure'
        if(~isfield(app.nav_camera,'robot_tab_ready') || ~app.nav_camera.robot_tab_ready)
            nav_robot_tab(app,'init');
        end;
        if(isfield(app.nav_camera,'tool') && ~isempty(app.nav_camera.tool))
            names=string({app.nav_camera.tool.name});
            if(~isequal(names,app.nav_camera.robot_tool_names))
                nav_robot_tab(app,'populate');   % new Tool check result
            end;
        end;

    case 'init'
        app.nav_camera.robot_tab_ready=true;
        app.nav_camera.robot_tool_names=strings(1,0);
        % wire the callbacks here as well, so they work even when the .mlapp stubs are empty
        app.MarkerDropDown.ValueChangedFcn=@(src,event) nav_robot_tab(app,'marker_changed');
        app.TargetXEditField.ValueChangedFcn=@(src,event) nav_robot_tab(app,'target_changed');
        app.TargetYEditField.ValueChangedFcn=@(src,event) nav_robot_tab(app,'target_changed');
        app.TargetZEditField.ValueChangedFcn=@(src,event) nav_robot_tab(app,'target_changed');
        app.RobotCommButton.ButtonPushedFcn=@(src,event) nav_camera_tool_comm(app,'toggle');
        app.TargetGotoButton.ButtonPushedFcn=@(src,event) nav_robot_goto(app,'toggle');
        app.CommCmdDropDown.ValueChangedFcn=@(src,event) nav_robot_tab(app,'cmd_selected');
        app.CommParamEditField.ValueChangedFcn=@(src,event) nav_robot_tab(app,'param_entered');
        app.TargetGotoButton.Tooltip='Move the robot (X/Y/Z) until the marker reaches the target; press again to abort';
        app.CommCmdDropDown.Items=cmd_items(:,1).';
        app.CommCmdDropDown.Value=cmd_items{1,1};
        app.CommCmdDropDown.Position(3)=85;     % room for 'GET STATUS'
        app.CommParamEditField.Value='';
        app.CommParamEditField.Enable='off';
        app.nav_camera.robot_marker_idx=[];
        app.nav_camera.robot_target=nan(3,1);
        app.nav_camera.robot_marker_h=gobjects(0);
        app.nav_camera.robot_target_h=gobjects(0);
        app.nav_camera.robot_marker_valid=false;
        app.nav_camera.robot_ui_t=[];
        app.nav_camera.robot_send_t=[];
        app.nav_camera.robot_view_box=[];

        % marker readout: live values from the camera, not editable
        for f={'MarkerXEditField','MarkerYEditField','MarkerZEditField'}
            app.(f{1}).Editable='off';
            app.(f{1}).ValueDisplayFormat='%.1f';
            app.(f{1}).Position(3)=70;   % room for e.g. -1234.5
        end;
        for f={'TargetXEditField','TargetYEditField','TargetZEditField'}
            app.(f{1}).ValueDisplayFormat='%.1f';
            app.(f{1}).Position(3)=60;
        end;
        app.DropDownLabel.Text='Marker';
        app.MarkerDropDown.Position(3)=110;    % room for a tool name
        app.MarkerDropDown.Items={'(run Tool check)'};
        app.MarkerDropDown.ItemsData=0;          % 0 = no marker
        clear_marker_readout(app);

        app.TargetUITable.ColumnName={'dx (mm)';'dy (mm)';'dz (mm)';'d (mm)'};
        app.TargetUITable.ColumnFormat={'bank','bank','bank','bank'};   % fixed 2 decimals, not 16.5000
        app.TargetUITable.Data=zeros(0,4);

        app.RobotCommTextArea.Editable='off';
        app.RobotCommTextArea.Value={''};

        ax=app.RobotUIAxes;
        cla(ax); hold(ax,'on');
        box(ax,'on'); grid(ax,'on'); ax.GridAlpha=0.25;
        daspect(ax,[1 1 1]);
        xlabel(ax,'right (mm)'); ylabel(ax,'up (mm)'); zlabel(ax,'distance from camera (mm)');
        view(ax,[45 30]);

        app.RobotCommButton.Text='Comm.';
        app.nav_camera_terminalUIFigure.CloseRequestFcn=@(src,event) nav_camera_tool_comm(app,'close_app');   % BYE before closing
        nav_robot_tab(app,'log','*','Robot tab ready: Tool check -> choose a marker -> type a target -> Comm.');

    case 'populate'
        if(~isfield(app.nav_camera,'tool') || isempty(app.nav_camera.tool))
            return;
        end;
        if(~isfield(app.nav_camera,'robot_tab_ready') || ~app.nav_camera.robot_tab_ready)
            nav_robot_tab(app,'init');
        end;
        old=app.nav_camera.robot_marker_idx;
        app.nav_camera.robot_tool_names=string({app.nav_camera.tool.name});
        app.MarkerDropDown.Items=app.nav_camera.robot_tool_names;
        app.MarkerDropDown.ItemsData=1:length(app.nav_camera.tool);
        if(~isempty(old) && old<=length(app.nav_camera.tool))
            app.MarkerDropDown.Value=old;
        end;
        nav_robot_tab(app,'marker_changed');

    case 'marker_changed'
        idx=app.MarkerDropDown.Value;
        if(~isnumeric(idx) || isempty(idx) || idx<1) idx=[]; end;
        if(~isequal(idx,app.nav_camera.robot_marker_idx))
            nav_robot_goto(app,'abort','aborted: the marker changed');
        end;
        app.nav_camera.robot_marker_idx=idx;
        app.nav_camera.robot_marker_valid=false;
        if(~isempty(app.nav_camera.robot_marker_h) && isgraphics(app.nav_camera.robot_marker_h))
            delete(app.nav_camera.robot_marker_h);   % reset the marker dot
        end;
        app.nav_camera.robot_marker_h=gobjects(0);
        app.nav_camera.robot_view_box=[];
        clear_marker_readout(app);
        for f={'MarkerXEditField','MarkerYEditField','MarkerZEditField'}
            app.(f{1}).Value=0;   % no stale numbers from the previous marker (greyed until seen)
        end;
        if(~isempty(idx))
            nav_robot_tab(app,'log','*',sprintf('marker: %s',tool_name(app,idx)));
        end;

    case 'target_changed'
        t=[app.TargetXEditField.Value; app.TargetYEditField.Value; app.TargetZEditField.Value];
        app.nav_camera.robot_target=t;
        ax=app.RobotUIAxes;
        if(isempty(app.nav_camera.robot_target_h) || ~isgraphics(app.nav_camera.robot_target_h))
            hold(ax,'on');
            app.nav_camera.robot_target_h=plot3(ax,t(1),t(2),t(3),'.','MarkerSize',30,'Color',target_color,'DisplayName','target');
        else
            set(app.nav_camera.robot_target_h,'XData',t(1),'YData',t(2),'ZData',t(3));
        end;
        update_view(app);
        nav_robot_tab(app,'log','*',sprintf('target set: x=%.1f y=%.1f z=%.1f mm',t));
        update_diff(app);

    case 'frame'
        idx_frame=varargin{1};
        idx=app.nav_camera.robot_marker_idx;
        if(isempty(idx))
            return;
        end;
        if(~isempty(app.nav_camera.robot_ui_t) && toc(app.nav_camera.robot_ui_t)<ui_period_s && idx_frame~=idx)
            return;   % frames of other tools are only used to notice a lost marker, at the UI rate
        end;
        [p,valid]=marker_position(app,idx,valid_s);

        % state change: found / lost
        if(valid~=app.nav_camera.robot_marker_valid)
            app.nav_camera.robot_marker_valid=valid;
            if(valid)
                nav_robot_tab(app,'log','*',sprintf('marker %s visible',tool_name(app,idx)));
            else
                nav_robot_tab(app,'log','!',sprintf('marker %s not visible',tool_name(app,idx)));
                clear_marker_readout(app);
                if(~isempty(app.nav_camera.robot_marker_h) && isgraphics(app.nav_camera.robot_marker_h))
                    set(app.nav_camera.robot_marker_h,'Color',lost_color);
                end;
            end;
        end;
        if(~valid)
            return;
        end;

        % stream to the robot (5 Hz), independent of the UI refresh
        if(isempty(app.nav_camera.robot_send_t) || toc(app.nav_camera.robot_send_t)>=send_period_s)
            if(all(isfinite(app.nav_camera.robot_target)))
                app.nav_camera.robot_send_t=tic;
                nav_camera_tool_comm(app,'send_delta',p-app.nav_camera.robot_target);
            end;
        end;

        % UI refresh (10 Hz)
        if(~isempty(app.nav_camera.robot_ui_t) && toc(app.nav_camera.robot_ui_t)<ui_period_s)
            return;
        end;
        app.nav_camera.robot_ui_t=tic;
        if(strcmp(app.MarkerXEditField.Enable,'off'))
            set([app.MarkerXEditField app.MarkerYEditField app.MarkerZEditField],'Enable','on');
        end;
        app.MarkerXEditField.Value=p(1);
        app.MarkerYEditField.Value=p(2);
        app.MarkerZEditField.Value=p(3);
        ax=app.RobotUIAxes;
        if(isempty(app.nav_camera.robot_marker_h) || ~isgraphics(app.nav_camera.robot_marker_h))
            hold(ax,'on');
            app.nav_camera.robot_marker_h=plot3(ax,p(1),p(2),p(3),'.','MarkerSize',30, ...
                'Color',tool_color(app,idx),'DisplayName',tool_name(app,idx));
        else
            set(app.nav_camera.robot_marker_h,'XData',p(1),'YData',p(2),'ZData',p(3),'Color',tool_color(app,idx));
        end;
        update_view(app);
        update_diff(app,p);

    case 'marker_pos'
        p=nan(3,1); valid=false;
        if(~isempty(app.nav_camera.robot_marker_idx))
            [p,valid]=marker_position(app,app.nav_camera.robot_marker_idx,valid_s);
        end;
        varargout={p,valid};

    case 'cmd_selected'
        cmd=app.CommCmdDropDown.Value;
        k=find(strcmp(cmd_items(:,1),cmd),1);
        if(isempty(k) || k==1)
            app.CommParamEditField.Enable='off';
            return;
        end;
        hint=cmd_items{k,2};
        if(isempty(hint))
            app.CommParamEditField.Enable='off';
            nav_camera_tool_comm(app,'cmd',cmd,'');
            app.CommCmdDropDown.Value=cmd_items{1,1};   % back to '(choose)', so the same command can be picked again
        else
            app.CommParamEditField.Enable='on';
            app.CommParamEditField.Value='';
            try app.CommParamEditField.Placeholder=hint; catch; end;
            focus_param(app);
            nav_robot_tab(app,'log','*',sprintf('%s: type the value (%s) in param and press Enter.',cmd,hint));
        end;

    case 'param_entered'
        cmd=app.CommCmdDropDown.Value;
        param=strtrim(app.CommParamEditField.Value);
        k=find(strcmp(cmd_items(:,1),cmd),1);
        if(isempty(k) || k==1 || isempty(param))
            return;
        end;
        nav_camera_tool_comm(app,'cmd',cmd,param);
        app.CommParamEditField.Value='';   % empty again, so the next value (even the same one) triggers a send

    case 'log'
        kind=varargin{1};
        str=varargin{2};
        line=sprintf('%s %s %s',datestr(now,'HH:MM:SS.FFF'),kind,str);
        fprintf('Robot: %s %s\n',kind,str);
        try
            v=app.RobotCommTextArea.Value;
            if(ischar(v)) v={v}; end;
            if(numel(v)==1 && isempty(v{1})) v={}; end;
            v=[{line}; v(:)];   % newest first: always visible without scrolling
            if(numel(v)>max_log_lines) v=v(1:max_log_lines); end;
            app.RobotCommTextArea.Value=v;
        catch
        end;

    otherwise
        fprintf('nav_robot_tab: unknown action [%s]\n',action);
end;

return;


function focus_param(app)
try
    focus(app.CommParamEditField);   % R2022b+
catch
end;


function [p,valid]=marker_position(app,idx,valid_s)
p=nan(3,1); valid=false;
if(~isfield(app.nav_camera,'last_p') || size(app.nav_camera.last_p,2)<idx)
    return;
end;
p=app.nav_camera.last_p(:,idx);
if(~all(isfinite(p)))
    return;
end;
if(isfield(app.nav_camera,'last_R_t') && numel(app.nav_camera.last_R_t)>=idx && isfield(app.nav_camera,'clock0'))
    valid=(toc(app.nav_camera.clock0)-app.nav_camera.last_R_t(idx))<valid_s;
else
    valid=true;
end;


function update_diff(app,p)
if(nargin<2)
    [p,valid]=marker_position(app,app.nav_camera.robot_marker_idx,0.5);
    if(~valid) p=nan(3,1); end;
end;
t=app.nav_camera.robot_target;
if(isempty(app.nav_camera.robot_marker_idx) || ~all(isfinite(t)) || ~all(isfinite(p)))
    app.TargetUITable.Data=zeros(0,4);
    return;
end;
d=p-t;
app.TargetUITable.Data=round([d(:).' norm(d)]*10)/10;   % 0.1 mm


function clear_marker_readout(app)
% no valid marker: grey out the readout (it keeps the last value, never a fake 0) and empty the table
for f={'MarkerXEditField','MarkerYEditField','MarkerZEditField'}
    app.(f{1}).Enable='off';
end;
app.TargetUITable.Data=zeros(0,4);


function update_view(app)
% keep the marker and the target in view; only re-frame when a point leaves the box
pts=[];
if(~isempty(app.nav_camera.robot_marker_h) && isgraphics(app.nav_camera.robot_marker_h))
    h=app.nav_camera.robot_marker_h; pts=[pts [h.XData;h.YData;h.ZData]];
end;
if(~isempty(app.nav_camera.robot_target_h) && isgraphics(app.nav_camera.robot_target_h))
    h=app.nav_camera.robot_target_h; pts=[pts [h.XData;h.YData;h.ZData]];
end;
pts=pts(:,all(isfinite(pts),1));
if(isempty(pts))
    return;
end;
box_now=app.nav_camera.robot_view_box;
inside=~isempty(box_now) && all(pts(:)>=repmat(box_now(:,1),size(pts,2),1)) && all(pts(:)<=repmat(box_now(:,2),size(pts,2),1));
if(inside)
    return;
end;
ctr=(min(pts,[],2)+max(pts,[],2))/2;
half=max((max(pts,[],2)-min(pts,[],2))/2*1.5,50);   % at least +/-50 mm, 50% margin
half=max(half)*ones(3,1);                          % cube: same scale on every axis
box_new=[ctr-half ctr+half];
app.nav_camera.robot_view_box=box_new;
set(app.RobotUIAxes,'XLim',box_new(1,:),'YLim',box_new(2,:),'ZLim',box_new(3,:));


function name=tool_name(app,idx)
name=sprintf('Tool %d',idx);
if(isfield(app.nav_camera,'tool') && numel(app.nav_camera.tool)>=idx && ~isempty(app.nav_camera.tool(idx).name))
    name=char(app.nav_camera.tool(idx).name);
end;


function c=tool_color(app,idx)
c=[0 0.447 0.741];
if(isfield(app.nav_camera,'tool') && numel(app.nav_camera.tool)>=idx && isfield(app.nav_camera.tool,'color') && ~isempty(app.nav_camera.tool(idx).color))
    c=app.nav_camera.tool(idx).color;
end;
