function varargout=nav_camera_tool_comm(app,action,varargin)
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
%   nav_camera_tool_comm(app,'cmd',cmd,param)     manual command from CommCmdDropDown / CommParamEditField:
%                                                 PING | HELP | GET STATUS | GET DELTA | GET DOC <name> |
%                                                 SET <axis> <value> | CALIBRATE <step mm> (nav_robot_goto).
%                                                 Never STOP, never DELTA by hand.
%   nav_camera_tool_comm(app,'status')            send GET STATUS quietly (not logged); the reply is parsed into
%                                                 app.nav_camera.robot_status and robot_status_n is incremented
%   n=nav_camera_tool_comm(app,'status_target')   the robot_status_n value at which the newest GET STATUS
%                                                 already sent has been answered
%   tf=nav_camera_tool_comm(app,'connected')      true while the link is open
%   nav_camera_tool_comm(app,'poll')              read replies that arrived (no sending)
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
%       GET STATUS                  ->  STATUS t=..;x=..;y=..;z=..;...;mx=..;my=..;mz=..;...
%       GET DOC <name>              ->  DOC <name> <nbytes> + nbytes raw (saved to tempdir, path logged)
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
        app.nav_camera.robot_expect=strings(1,0);    % the reply each of them should get ('...*' = prefix)
        app.nav_camera.robot_quiet=false(1,0);       % do not log this request / its normal reply
        app.nav_camera.robot_status=[];               % last parsed STATUS reply (struct of numbers)
        app.nav_camera.robot_status_n=0;              % STATUS replies received
        app.nav_camera.robot_set_err="";              % last SET that was refused: '<request> -> <reply>'
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

    case 'connected'
        varargout{1}=is_connected(app);

    case 'poll'
        if(is_connected(app))
            try
                drain_replies(app);
            catch ME
                log_msg(app,'!',sprintf('link lost (%s); disconnected.',ME.message));
                close_link(app);
            end;
        end;

    case 'status'
        if(~is_connected(app)) return; end;
        try
            request(app,"GET STATUS","STATUS *",true);
        catch ME
            log_msg(app,'!',sprintf('send failed (%s); disconnected.',ME.message));
            close_link(app);
        end;

    case 'status_target'
        n=0;
        if(is_connected(app))
            n=app.nav_camera.robot_status_n+sum(app.nav_camera.robot_pending=="GET STATUS");
        end;
        varargout{1}=n;

    case 'cmd'   % manual command from the Robot tab: nav_camera_tool_comm(app,'cmd','SET X','0.5')
        if(~is_connected(app))
            log_msg(app,'!','not connected; press Comm. first.');
            return;
        end;
        cmd=upper(strtrim(string(varargin{1})));
        param="";
        if(numel(varargin)>=2) param=strtrim(string(varargin{2})); end;
        try
            switch cmd
                case "PING"
                    request(app,"PING","OK PONG");
                case "HELP"
                    request(app,"HELP","*");
                case "GET STATUS"
                    request(app,"GET STATUS","STATUS *");
                case "GET DELTA"
                    request(app,"GET DELTA","DELTA *");
                case "GET DOC"
                    name=upper(param);
                    if(~any(name==["BRIEF","PROTOCOL","GUIDE"]))
                        log_msg(app,'!','GET DOC needs param BRIEF, PROTOCOL or GUIDE; nothing sent.');
                        return;
                    end;
                    request(app,"GET DOC "+name,"DOC "+name+" *");
                case "CALIBRATE"   % moves X, Y, Z by <param> mm in turn and measures the marker (nav_robot_goto)
                    v=str2double(param);
                    if(isempty(param) || ~isfinite(v))
                        log_msg(app,'!','CALIBRATE needs the step in mm (0.5..5) in param; nothing sent.');
                        return;
                    end;
                    nav_robot_goto(app,'calibrate',v);
                otherwise
                    if(startsWith(cmd,"SET "))
                        v=str2double(param);
                        if(isempty(param) || ~isfinite(v))
                            log_msg(app,'!',sprintf('%s needs a number in param; nothing sent.',cmd));
                            return;
                        end;
                        nav_camera_tool_comm(app,'set',extractAfter(cmd,"SET "),v);
                    else
                        log_msg(app,'!',sprintf('unknown command [%s]; nothing sent.',cmd));
                    end;
            end;
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


function request(app,msg,expect,quiet)
% send one request line and queue the reply it should get
% (expect ending in '*': any reply starting with the text before the '*')
if(nargin<4) quiet=false; end;
drain_replies(app);
if(startsWith(msg,"SET"))
    app.nav_camera.robot_set_err="";
end;
% queue first: another timer callback (camera frame, keepalive, Goto) can run while
% writeline is busy and read the reply; it must then find this request pending
app.nav_camera.robot_pending(end+1)=msg;
app.nav_camera.robot_expect(end+1)=expect;
app.nav_camera.robot_quiet(end+1)=quiet;
writeline(app.nav_camera.robot,msg);
app.nav_camera.robot_last_tx=tic;
if(quiet)
    % not logged
elseif(~startsWith(msg,"DELTA"))
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
    reply=strtrim(string(line));   % strips the CR of CRLF
    doc='';
    if(startsWith(reply,"DOC ") && ~isempty(app.nav_camera.robot_pending) && startsWith(app.nav_camera.robot_pending(1),"GET DOC"))
        % the only multi-line reply: 'DOC <name> <nbytes>' then exactly nbytes raw bytes
        tok=split(reply);
        nb=str2double(tok(end));
        if(numel(app.nav_camera.robot_rx_buf)-k<nb)
            break;   % body not complete yet; keep everything buffered
        end;
        doc=app.nav_camera.robot_rx_buf(k+1:k+nb);
        app.nav_camera.robot_rx_buf(1:k+nb)=[];
    else
        app.nav_camera.robot_rx_buf(1:k)=[];
    end;
    app.nav_camera.robot_last_reply=reply;
    if(isempty(app.nav_camera.robot_pending))
        log_msg(app,'<',reply+"   (unexpected: no request pending)");
        continue;
    end;
    sent=app.nav_camera.robot_pending(1);
    expect=app.nav_camera.robot_expect(1);
    quiet=app.nav_camera.robot_quiet(1);
    app.nav_camera.robot_pending(1)=[];
    app.nav_camera.robot_expect(1)=[];
    app.nav_camera.robot_quiet(1)=[];
    is_delta=startsWith(sent,"DELTA");
    if(is_delta)
        app.nav_camera.robot_n_sent=app.nav_camera.robot_n_sent+1;
    end;
    if(startsWith(reply,"STATUS "))
        app.nav_camera.robot_status=parse_status(reply);
        app.nav_camera.robot_status_n=app.nav_camera.robot_status_n+1;
    end;
    if(endsWith(expect,"*"))
        ok=startsWith(reply,extractBefore(expect,strlength(expect))) && ~startsWith(reply,"ERR");
    else
        ok=(reply==expect);
    end;
    if(ok && ~isempty(doc))
        fn=fullfile(tempdir,sprintf('tms_robot_%s.md',char(tok(2))));
        fid=fopen(fn,'w'); fwrite(fid,uint8(doc)); fclose(fid);
        log_msg(app,'<',sprintf('%s   (saved to %s)',reply,fn));
        continue;
    end;
    if(ok && quiet)
        continue;
    end;
    if(ok)
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
    if(startsWith(sent,"SET"))
        app.nav_camera.robot_set_err=sent+" -> "+reply;   % nav_robot_goto checks this
    end;
    if(startsWith(sent,"SET") && startsWith(reply,"ERR BUSY"))
        if(contains(reply,"operator"))
            log_msg(app,'!',sprintf('"%s" refused: the operator has control (Remote switch / bridge "Allow motion" is off).',sent));
        else
            log_msg(app,'!',sprintf('"%s" refused: %s',sent,extractAfter(reply,"ERR BUSY ")));
        end;
        continue;
    end;
    log_msg(app,'!',sprintf('sent "%s", expected "%s"',sent,expect));
end;


function s=parse_status(reply)
% 'STATUS t=..;x=..;...' -> struct with numeric fields
s=struct();
kv=split(extractAfter(reply,"STATUS "),";");
for i=1:numel(kv)
    p=split(strtrim(kv(i)),"=");
    if(numel(p)==2 && isvarname(char(p(1))))
        s.(char(p(1)))=str2double(p(2));
    end;
end;


function close_link(app)
try
    nav_robot_goto(app,'abort','link closed');
catch
end;
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
