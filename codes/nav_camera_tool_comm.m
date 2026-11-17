function nav_camera_tool_comm(app,action,varargin)
%
%   nav_camera_tool_comm    stream a tool's offset from its marked reference to the robot PC
%
%   nav_camera_tool_comm(app,'populate')  fill app.ToolDropDown with the known tools (after Tool check)
%   nav_camera_tool_comm(app,'toggle')    'Comm.' button: connect + start streaming / stop + disconnect
%   nav_camera_tool_comm(app,'send')      send the latest offset of the selected tool (called from
%                                         nav_camera_graphic_update at ~5 Hz)
%   nav_camera_tool_comm(app,'set',axis,value)  send 'SET <axis> <value>' on the open connection
%                                         (axis X|Y|Z|PSI|THETA|PHI absolute, L|R relative steps).
%                                         MOVES THE ROBOT; never called automatically. Poll the
%                                         m* flags with GET STATUS to see when the move is done.
%   nav_camera_tool_comm(app,'close')     send BYE and close the connection
%   nav_camera_tool_comm(app,'close_app') figure CloseRequestFcn: 'close', then delete the app
%
%   After 'Mark ref.', the offset of the tool selected in app.ToolDropDown
%   from its reference (dx right, dy up, dz distance from camera, |d|; mm)
%   is sent to the TMS-robot server on the Windows PC as one ASCII line:
%
%       DELTA <dx> <dy> <dz> <d>\r\n        e.g.  DELTA 1.2 -0.4 3.1 3.4
%
%   The server acknowledges each line by echoing it back:
%
%       OK DELTA <dx> <dy> <dz> <d>\r\n     (same text as sent)
%
%   Replies are read without blocking the camera loop and matched in order
%   against the sent lines; a missing or different ack is reported in
%   app.infoTextArea, and the counts are reported on disconnect. Nothing is
%   sent while the tool is not visible or has no reference. This function
%   never sends STOP, and SET only through the explicit 'set' action.
%
%   DELTA is part of protocol v1 (doc/tms_robot/PROTOCOL.md) since Nov 17 2026;
%   it only stores the offset on the robot side (GET DELTA reads it back) and
%   never moves the robot. An older server answers 'ERR UNKNOWN_CMD DELTA',
%   reported once. The server drops a client after 10 s of silence, so a
%   timer sends PING whenever nothing was sent for keepalive_s seconds (e.g.
%   tool hidden, camera stopped). Only one client is allowed (a second one
%   gets ERR BUSY instead of the greeting). When the panel's Remote switch
%   is off, SET is answered 'ERR BUSY operator has control'.
%
%   Robot PC address: 169.254.30.121:6340 by default; for the simulator
%   (tms_robot_sim.py) run setenv('TMS_ROBOT_HOST','127.0.0.1') first.
%
% fhlin@Nov 17, 2026
%

robot_host='169.254.30.121';   % Windows/LabVIEW PC on the Mac's en23 Ethernet port
robot_port=6340;
if(~isempty(getenv('TMS_ROBOT_HOST')))
    robot_host=getenv('TMS_ROBOT_HOST');   % e.g. setenv('TMS_ROBOT_HOST','127.0.0.1') for tms_robot_sim.py
end;
msg_format='DELTA %.1f %.1f %.1f %.1f';
keepalive_s=4;   % PING after this much silence (server timeout: 10 s)

if(nargin<2 || isempty(action))
    action='toggle';
end;

switch lower(action)
    case 'populate'
        if(~isfield(app.nav_camera,'tool') || isempty(app.nav_camera.tool))
            return;
        end;
        app.ToolDropDown.Items=string({app.nav_camera.tool.name});
        app.ToolDropDown.ItemsData=1:length(app.nav_camera.tool);

    case 'toggle'
        if(is_connected(app))
            nav_camera_tool_comm(app,'close');
            return;
        end;
        if(~isnumeric(app.ToolDropDown.Value) || isempty(app.ToolDropDown.Value))
            uialert(app.nav_camera_terminalUIFigure,'Run "Tool check" first and select a tool.','Tool comm.');
            return;
        end;
        try
            c=tcpclient(robot_host,robot_port,'Timeout',2,'ConnectTimeout',3);
            configureTerminator(c,"CR/LF");
            greeting=strtrim(string(readline(c)));   % server greets first: HELLO TMS-ROBOT 1 [...]
            tok=split(greeting);
            if(numel(tok)<3 || tok(1)~="HELLO" || tok(2)~="TMS-ROBOT" || tok(3)~="1")   % protocol version 1 only
                clear c;
                if(startsWith(greeting,"ERR BUSY"))
                    msg=sprintf('The robot at %s:%d is already in use by another client.\n(%s)',robot_host,robot_port,greeting);
                else
                    msg=sprintf('Unexpected greeting from %s:%d: "%s" (expected HELLO TMS-ROBOT 1)',robot_host,robot_port,greeting);
                end;
                uialert(app.nav_camera_terminalUIFigure,msg,'Tool comm.');
                return;
            end;
            c.Timeout=0.2;
        catch ME
            uialert(app.nav_camera_terminalUIFigure,sprintf('Cannot connect to %s:%d\n%s',robot_host,robot_port,ME.message),'Tool comm.');
            return;
        end;
        app.nav_camera.robot=c;
        app.nav_camera.robot_tool_idx=app.ToolDropDown.Value;
        app.nav_camera.robot_last_reply="";
        app.nav_camera.robot_pending=strings(1,0);   % requests sent, reply not yet read (FIFO)
        app.nav_camera.robot_expect=strings(1,0);    % the reply each of them should get
        app.nav_camera.robot_delta_unsupported=false;
        app.nav_camera.robot_last_tx=tic;
        app.nav_camera.robot_timer=timer('ExecutionMode','fixedSpacing','Period',1,'BusyMode','drop', ...
            'Name','nav_camera_tool_comm keepalive','TimerFcn',@(~,~) nav_camera_tool_comm(app,'keepalive'));
        app.nav_camera.robot_keepalive_s=keepalive_s;
        start(app.nav_camera.robot_timer);
        app.nav_camera.robot_n_sent=0;
        app.nav_camera.robot_n_acked=0;
        app.nav_camera.robot_n_bad=0;
        app.ToolCommButton.Text='Stop comm.';
        app.ToolDropDown.Enable='off';   % keep the streamed tool fixed while connected
        info(app,sprintf('Robot: connected to %s:%d (%s), streaming %s',robot_host,robot_port,greeting,app.ToolDropDown.Items{app.ToolDropDown.Value==app.ToolDropDown.ItemsData}));

    case 'send'
        if(~is_connected(app))
            return;
        end;
        idx=app.nav_camera.robot_tool_idx;
        if(~isfield(app.nav_camera,'ref_p') || size(app.nav_camera.ref_p,2)<idx || ~all(isfinite(app.nav_camera.ref_p(:,idx))))
            return;   % no reference marked for this tool yet
        end;
        if(~isfield(app.nav_camera,'last_p') || size(app.nav_camera.last_p,2)<idx || ~all(isfinite(app.nav_camera.last_p(:,idx))))
            return;   % tool not visible
        end;
        d=app.nav_camera.last_p(:,idx)-app.nav_camera.ref_p(:,idx);
        try
            drain_replies(app);
            msg=string(sprintf(msg_format,d(1),d(2),d(3),norm(d)));
            request(app,msg,"OK "+msg);   % ack echoes the line
        catch ME
            info(app,sprintf('Robot: send failed (%s); disconnected.',ME.message));
            close_link(app);
        end;

    case 'set'   % explicit move request, e.g. nav_camera_tool_comm(app,'set','X',0.5)
        if(~is_connected(app))
            info(app,'Robot: not connected; press Comm. first.');
            return;
        end;
        if(numel(varargin)<2)
            error('usage: nav_camera_tool_comm(app,''set'',axis,value)');
        end;
        ax=upper(string(varargin{1}));
        if(~any(ax==["X","Y","Z","PSI","THETA","PHI","L","R"]))
            error('nav_camera_tool_comm: unknown axis [%s]',ax);
        end;
        v=varargin{2};
        if(any(ax==["L","R"]))
            v_str=sprintf('%d',round(v));
        else
            v_str=sprintf('%.6g',v);
        end;
        msg="SET "+ax+" "+v_str;
        request(app,msg,"OK "+msg);   % reply arrives asynchronously; ERR BUSY etc. reported in drain_replies

    case 'keepalive'   % timer: read replies that arrived, PING if the link has been quiet
        if(~is_connected(app))
            close_link(app);
            return;
        end;
        try
            drain_replies(app);
            if(toc(app.nav_camera.robot_last_tx)>app.nav_camera.robot_keepalive_s)
                request(app,"PING","OK PONG");
            end;
        catch ME
            info(app,sprintf('Robot: link lost (%s); disconnected.',ME.message));
            close_link(app);
        end;

    case 'close'
        if(is_connected(app))
            try
                % collect acks still in flight (up to 1 s), then say BYE
                t_wait=tic;
                while(~isempty(app.nav_camera.robot_pending) && toc(t_wait)<1)
                    pause(0.02);
                    drain_replies(app);
                end;
                writeline(app.nav_camera.robot,"BYE");
                app.nav_camera.robot.Timeout=1;
                readline(app.nav_camera.robot);   % OK BYE
            catch
            end;
            n_lost=sum(startsWith(app.nav_camera.robot_pending,"DELTA"));
            info(app,sprintf('Robot: disconnected. DELTA sent %d, acknowledged %d, wrong reply %d, no reply %d.', ...
                app.nav_camera.robot_n_sent+n_lost,app.nav_camera.robot_n_acked,app.nav_camera.robot_n_bad,n_lost));
        end;
        close_link(app);

    case 'close_app'   % figure CloseRequestFcn: say BYE, then close the app as usual
        try
            nav_camera_tool_comm(app,'close');
        catch
        end;
        delete(app);

    otherwise
        fprintf('nav_camera_tool_comm: unknown action [%s]\n',action);
end;

return;


function tf=is_connected(app)
tf=isfield(app.nav_camera,'robot') && ~isempty(app.nav_camera.robot) && isvalid(app.nav_camera.robot);


function request(app,msg,expect)
% send one request line and queue the reply it should get
drain_replies(app);
writeline(app.nav_camera.robot,msg);
app.nav_camera.robot_pending(end+1)=msg;
app.nav_camera.robot_expect(end+1)=expect;
app.nav_camera.robot_last_tx=tic;


function drain_replies(app)
% read whatever reply lines have arrived, without waiting. One reply per
% request, in order: each reply answers the oldest pending request.
c=app.nav_camera.robot;
while(c.NumBytesAvailable>0)
    line=readline(c);
    if(isempty(line)) break; end;
    reply=strtrim(string(line));
    app.nav_camera.robot_last_reply=reply;
    if(isempty(app.nav_camera.robot_pending))
        info(app,sprintf('Robot: unexpected reply "%s"',reply));
        continue;
    end;
    sent=app.nav_camera.robot_pending(1);
    expect=app.nav_camera.robot_expect(1);
    app.nav_camera.robot_pending(1)=[];
    app.nav_camera.robot_expect(1)=[];
    is_delta=startsWith(sent,"DELTA");
    if(is_delta)
        app.nav_camera.robot_n_sent=app.nav_camera.robot_n_sent+1;
    end;
    if(reply==expect)
        if(is_delta) app.nav_camera.robot_n_acked=app.nav_camera.robot_n_acked+1; end;
        continue;
    end;
    if(is_delta)
        app.nav_camera.robot_n_bad=app.nav_camera.robot_n_bad+1;
        if(startsWith(reply,"ERR UNKNOWN_CMD DELTA"))
            if(~app.nav_camera.robot_delta_unsupported)   % report once, not at 5 Hz
                app.nav_camera.robot_delta_unsupported=true;
                info(app,'Robot: this server does not know DELTA (ERR UNKNOWN_CMD DELTA); offsets are sent but not stored.');
            end;
            continue;
        end;
    end;
    if(startsWith(sent,"SET") && startsWith(reply,"ERR BUSY"))
        info(app,sprintf('Robot: "%s" refused, the operator has control (panel Remote switch is off).',sent));
        continue;
    end;
    info(app,sprintf('Robot: sent "%s", expected "%s", got "%s"',sent,expect,reply));
end;


function close_link(app)
if(isfield(app.nav_camera,'robot_timer') && ~isempty(app.nav_camera.robot_timer))
    try
        stop(app.nav_camera.robot_timer);
        delete(app.nav_camera.robot_timer);
    catch
    end;
    app.nav_camera.robot_timer=[];
end;
if(isfield(app.nav_camera,'robot'))
    app.nav_camera.robot=[];   % deleting the tcpclient closes the socket
end;
app.ToolCommButton.Text='Comm.';
app.ToolDropDown.Enable='on';


function info(app,str)
fprintf('%s\n',str);
try
    app.infoTextArea.Value=str;
catch
end;
