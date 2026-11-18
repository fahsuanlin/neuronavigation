function nav_camera_tool_comm(app,action,varargin)
%
%   nav_camera_tool_comm    TCP link from the Robot tab to the TMS-robot server (Windows/LabVIEW PC)
%
%   nav_camera_tool_comm(app,'toggle')            'Comm.' button (app.RobotCommButton): connect / disconnect
%   nav_camera_tool_comm(app,'send_delta',d)      send 'DELTA dx dy dz |d|' for d = marker - target (mm,
%                                                 3x1); called by nav_robot_tab at ~5 Hz while the marker
%                                                 is valid and a target is set
%   nav_camera_tool_comm(app,'set',axis,value)    send 'SET <axis> <value>' (X|Y|Z|PSI|THETA|PHI absolute,
%                                                 L|R relative steps). MOVES THE ROBOT; never called
%                                                 automatically. Poll GET STATUS m* flags for completion.
%   nav_camera_tool_comm(app,'keepalive')         timer: read arrived replies, PING after keepalive_s silence
%   nav_camera_tool_comm(app,'close')             send BYE and close the connection
%   nav_camera_tool_comm(app,'close_app')         figure CloseRequestFcn: 'close', then delete the app
%
%   Protocol: TMS-robot protocol v1, doc/tms_robot/PROTOCOL.md (contract) and
%   doc/tms_robot/MAC_ROBOT_TCP.md (Mac side). ASCII lines ending CRLF, one
%   reply line per request, server greets 'HELLO TMS-ROBOT 1 ...'.
%
%       DELTA <dx> <dy> <dz> <d>    ->  OK DELTA <the same four tokens>   (stored on the robot side, no motion)
%       PING                        ->  OK PONG
%       SET <axis> <value>          ->  OK SET <axis> <value> | ERR BUSY operator has control | ERR ...
%       BYE                         ->  OK BYE
%
%   Replies are read without blocking the tracking loop and matched in order
%   against the requests sent. Traffic and status go to app.RobotCommTextArea
%   (through nav_robot_tab(app,'log',...)); DELTA traffic is logged at most
%   once per second, every error is logged. The server drops a client after
%   10 s of silence, so a timer sends PING whenever nothing was sent for
%   keepalive_s seconds. Never sends STOP.
%
%   Robot PC address: 169.254.30.121:6340 by default; for the simulator
%   (codes/tms_robot_sim.py) run setenv('TMS_ROBOT_HOST','127.0.0.1') first.
%
% fhlin@Nov 17, 2026 (Robot tab: Nov 18, 2026)
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
    case 'toggle'
        if(is_connected(app))
            nav_camera_tool_comm(app,'close');
            return;
        end;
        log_msg(app,'*',sprintf('connecting to %s:%d ...',robot_host,robot_port));
        drawnow;
        try
            c=tcpclient(robot_host,robot_port,'Timeout',2,'ConnectTimeout',3);
            configureTerminator(c,"CR/LF");
            greeting=strtrim(string(readline(c)));   % server greets first: HELLO TMS-ROBOT 1 [...]
            log_msg(app,'<',greeting);
            tok=split(greeting);
            if(numel(tok)<3 || tok(1)~="HELLO" || tok(2)~="TMS-ROBOT" || tok(3)~="1")   % protocol version 1 only
                clear c;
                if(startsWith(greeting,"ERR BUSY"))
                    log_msg(app,'!','the robot is already in use by another client; not connected.');
                else
                    log_msg(app,'!','unexpected greeting (expected HELLO TMS-ROBOT 1); not connected.');
                end;
                return;
            end;
            c.Timeout=0.2;
        catch ME
            log_msg(app,'!',sprintf('cannot connect to %s:%d (%s)',robot_host,robot_port,ME.message));
            return;
        end;
        app.nav_camera.robot=c;
        app.nav_camera.robot_host=sprintf('%s:%d',robot_host,robot_port);
        app.nav_camera.robot_last_reply="";
        app.nav_camera.robot_rx_buf='';               % received bytes not yet forming a full line
        app.nav_camera.robot_pending=strings(1,0);   % requests sent, reply not yet read (FIFO)
        app.nav_camera.robot_expect=strings(1,0);    % the reply each of them should get
        app.nav_camera.robot_delta_unsupported=false;
        app.nav_camera.robot_last_tx=tic;
        app.nav_camera.robot_delta_log_t=[];          % last time DELTA traffic was logged
        app.nav_camera.robot_n_sent=0;
        app.nav_camera.robot_n_acked=0;
        app.nav_camera.robot_n_bad=0;
        app.nav_camera.robot_keepalive_s=keepalive_s;
        app.nav_camera.robot_timer=timer('ExecutionMode','fixedSpacing','Period',1,'BusyMode','drop', ...
            'Name','nav_camera_tool_comm keepalive','TimerFcn',@(~,~) nav_camera_tool_comm(app,'keepalive'));
        start(app.nav_camera.robot_timer);
        app.RobotCommButton.Text='Stop comm.';
        log_msg(app,'*',sprintf('connected to %s; DELTA (marker - target) is sent while the marker is valid and a target is set.',app.nav_camera.robot_host));

    case 'send_delta'
        if(~is_connected(app) || isempty(varargin))
            return;
        end;
        d=varargin{1};
        if(numel(d)~=3 || ~all(isfinite(d(:))))
            return;
        end;
        try
            msg=string(sprintf(msg_format,d(1),d(2),d(3),norm(d)));
            request(app,msg,"OK "+msg);   % ack echoes the line
        catch ME
            log_msg(app,'!',sprintf('send failed (%s); disconnected.',ME.message));
            close_link(app);
        end;

    case 'set'   % explicit move request, e.g. nav_camera_tool_comm(app,'set','X',0.5)
        if(~is_connected(app))
            log_msg(app,'!','not connected; press Comm. first.');
            return;
        end;
        if(numel(varargin)<2 || ~isnumeric(varargin{2}) || ~isscalar(varargin{2}) || ~isfinite(varargin{2}))
            log_msg(app,'!','usage nav_camera_tool_comm(app,''set'',axis,value); nothing sent.');
            return;
        end;
        ax=upper(string(varargin{1}));
        if(~any(ax==["X","Y","Z","PSI","THETA","PHI","L","R"]))
            log_msg(app,'!',sprintf('unknown axis [%s] (X|Y|Z|PSI|THETA|PHI|L|R); nothing sent.',ax));
            return;
        end;
        v=varargin{2};
        if(any(ax==["L","R"]))
            v_str=sprintf('%d',round(v));
        else
            v_str=sprintf('%.6g',v);
        end;
        msg="SET "+ax+" "+v_str;
        try
            request(app,msg,"OK "+msg);   % reply arrives asynchronously; ERR BUSY etc. reported in drain_replies
        catch ME
            log_msg(app,'!',sprintf('send failed (%s); disconnected.',ME.message));
            close_link(app);
        end;

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
            log_msg(app,'!',sprintf('link lost (%s); disconnected.',ME.message));
            close_link(app);
        end;

    case 'close'
        if(is_connected(app))
            try
                % collect replies still in flight (up to 1 s), then say BYE
                t_wait=tic;
                while(~isempty(app.nav_camera.robot_pending) && toc(t_wait)<1)
                    pause(0.02);
                    drain_replies(app);
                end;
                writeline(app.nav_camera.robot,"BYE");
                log_msg(app,'>',"BYE");
                app.nav_camera.robot.Timeout=1;
                log_msg(app,'<',strtrim(string(readline(app.nav_camera.robot))));   % OK BYE
            catch
            end;
            n_lost=sum(startsWith(app.nav_camera.robot_pending,"DELTA"));
            log_msg(app,'*',sprintf('disconnected. DELTA sent %d, acknowledged %d, wrong reply %d, no reply %d.', ...
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
if(~startsWith(msg,"DELTA"))
    log_msg(app,'>',msg);
elseif(isempty(app.nav_camera.robot_delta_log_t) || toc(app.nav_camera.robot_delta_log_t)>1)
    app.nav_camera.robot_delta_log_t=tic;   % DELTA runs at ~5 Hz: log one per second
    log_msg(app,'>',msg);
    app.nav_camera.robot_log_next_ack=true;
end;


function drain_replies(app)
% read whatever reply lines have arrived, without waiting. One reply per
% request, in order: each reply answers the oldest pending request.
% Bytes are buffered here, so a reply split across TCP packets is never read
% half (readline would block for Timeout and warn).
c=app.nav_camera.robot;
if(c.NumBytesAvailable>0)
    app.nav_camera.robot_rx_buf=[app.nav_camera.robot_rx_buf char(read(c,c.NumBytesAvailable,"uint8"))];
end;
while(true)
    k=find(app.nav_camera.robot_rx_buf==newline,1);
    if(isempty(k)) break; end;
    line=app.nav_camera.robot_rx_buf(1:k-1);
    app.nav_camera.robot_rx_buf(1:k)=[];
    reply=strtrim(string(line));   % strips the CR of CRLF
    app.nav_camera.robot_last_reply=reply;
    if(isempty(app.nav_camera.robot_pending))
        log_msg(app,'<',reply+"   (unexpected: no request pending)");
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
        if(is_delta)
            app.nav_camera.robot_n_acked=app.nav_camera.robot_n_acked+1;
            if(isfield(app.nav_camera,'robot_log_next_ack') && app.nav_camera.robot_log_next_ack)
                app.nav_camera.robot_log_next_ack=false;
                log_msg(app,'<',sprintf('%s   (%d/%d acknowledged)',reply,app.nav_camera.robot_n_acked,app.nav_camera.robot_n_sent));
            end;
        else
            log_msg(app,'<',reply);
        end;
        continue;
    end;
    log_msg(app,'<',reply);
    if(is_delta)
        app.nav_camera.robot_n_bad=app.nav_camera.robot_n_bad+1;
        if(startsWith(reply,"ERR UNKNOWN_CMD DELTA"))
            if(~app.nav_camera.robot_delta_unsupported)   % explain once, not at 5 Hz
                app.nav_camera.robot_delta_unsupported=true;
                log_msg(app,'!','this server does not know DELTA; offsets are sent but not stored.');
            end;
            continue;
        end;
    end;
    if(startsWith(sent,"SET") && startsWith(reply,"ERR BUSY"))
        log_msg(app,'!',sprintf('"%s" refused: the operator has control (panel Remote switch is off).',sent));
        continue;
    end;
    log_msg(app,'!',sprintf('sent "%s", expected "%s"',sent,expect));
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
try
    app.RobotCommButton.Text='Comm.';
catch
end;


function log_msg(app,kind,str)
% kind: '>' sent, '<' received, '*' status, '!' problem
nav_robot_tab(app,'log',kind,char(str));
