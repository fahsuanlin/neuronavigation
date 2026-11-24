function nav_robot_goto(app,action,varargin)
%
%   nav_robot_goto    'Goto' button of the Robot tab: move the robot (SET X/Y/Z) until the
%                     tracked marker reaches the target typed in the Target panel
%
%   nav_robot_goto(app,'toggle')        Goto button: start, or abort a running Goto
%   nav_robot_goto(app,'abort',why)     stop issuing moves (the robot finishes the move in progress;
%                                       STOP is never sent because it ends the robot program)
%   nav_robot_goto(app,'calibrate',step)  CALIBRATE command (cmd. dropdown): only the calibration below, with
%                                       step mm per axis (default cal_step_mm), then back to the start
%                                       position. No target needed. The result is kept for Goto.
%   nav_robot_goto(app,'recalibrate')   forget the camera/robot calibration (the next Goto measures it again)
%   nav_robot_goto(app,'tick')          timer (20 Hz) while a Goto runs
%
%   Protocol v1 only moves robot axes (SET X/Y/Z, absolute mm, -25..25), and the robot axes
%   are not aligned with the camera axes (marker x, y, z). So Goto works in closed loop:
%
%   1. calibrate (CALIBRATE command, or a Goto without a calibration for this marker): move X, Y and Z by cal_step_mm one
%      after another, measuring the marker after each move. The marker displacements give the
%      3x3 Jacobian J (camera mm per robot mm): dp = J*dr. Kept for later Gotos.
%   2. servo: e = marker - target; if |e| <= tol_mm stop. Otherwise dr = -J\e (at most
%      max_step_mm per move), SET X/Y/Z to r+dr (clamped to the robot range), wait until the
%      STATUS moving flags mx/my/mz are 0, let the marker settle, measure it, repeat.
%      Stops after max_iter moves, or if the error grows twice in a row (then J is dropped
%      so that the next Goto recalibrates).
%
%   Every SET is logged in the Robot tab log. A refused SET (ERR BUSY: the operator has
%   control, ERR OUT_OF_RANGE, ...) aborts the Goto.
%
% fhlin@Nov 19, 2026
%

cal_step_mm=2;        % calibration move per axis
tol_mm=0.5;           % done when |marker - target| <= tol_mm
max_step_mm=10;       % largest move per iteration (norm of dr)
max_iter=8;           % servo moves before giving up
r_lim=25;             % robot X/Y/Z range (mm)
mm_per_step=2/52;     % robot resolution (26 steps/mm)
settle_s=0.5;         % wait after the motion flags clear, before measuring the marker
sample_s=0.5;         % average the marker over this long
status_period_s=0.25; % GET STATUS polling while moving (protocol: <= 20 Hz)
steps_per_s=4;        % panel default motor speed, used for the move timeout
tick_s=0.05;

if(nargin<2 || isempty(action)) action='toggle'; end;

switch lower(action)
    case 'toggle'
        if(is_running(app))
            nav_robot_goto(app,'abort','aborted by the user');
            return;
        end;
        start_goto(app,tick_s,"goto",cal_step_mm);

    case 'calibrate'
        if(is_running(app))
            log_msg(app,'!','CALIBRATE: a Goto/calibration is running; abort it first (Abort button).');
            return;
        end;
        step=cal_step_mm;
        if(~isempty(varargin) && ~isempty(varargin{1})) step=varargin{1}; end;
        if(~isnumeric(step) || ~isscalar(step) || ~isfinite(step) || step<0.5 || step>5)
            log_msg(app,'!','CALIBRATE: step must be 0.5..5 mm; nothing sent.');
            return;
        end;
        start_goto(app,tick_s,"calibrate",step);

    case 'abort'
        why='aborted';
        if(~isempty(varargin)) why=varargin{1}; end;
        if(~is_running(app)) return; end;
        finish(app,'!',sprintf('%s %s. The robot completes the move in progress.',upper_who(app),why));

    case 'recalibrate'
        app.nav_camera.robot_goto_J=[];
        log_msg(app,'*','Goto calibration cleared; the next Goto measures it again.');

    case 'tick'
        if(~is_running(app)) return; end;
        try
            tick(app);
        catch ME
            finish(app,'!',sprintf('Goto failed: %s',ME.message));
        end;
end;

    % ------------------------------------------------------------------
    function tick(app)
        nav_camera_tool_comm(app,'poll');
        if(~nav_camera_tool_comm(app,'connected'))
            finish(app,'!','Goto stopped: not connected.');
            return;
        end;
        g=app.nav_camera.robot_goto;
        if(strlength(app.nav_camera.robot_set_err)>0 && g.phase~="first_status")
            finish(app,'!',sprintf('Goto stopped: %s',app.nav_camera.robot_set_err));
            return;
        end;
        switch g.phase
            case "first_status"   % robot position at the start
                if(app.nav_camera.robot_status_n<g.status_wait)
                    if(toc(g.t)>3) finish(app,'!','Goto stopped: no STATUS reply.'); end;
                    return;
                end;
                st=app.nav_camera.robot_status;
                if(any([st.mx st.my st.mz]))
                    g.phase="moving"; g.t=tic; g.t_poll=[]; g.status_wait=0; g.timeout=60;   % let the current motion finish first
                    g.r=[st.x; st.y; st.z];
                else
                    g.r=[st.x; st.y; st.z];
                    g=begin_measure(g);
                end;

            case "moving"   % wait for mx = my = mz = 0 in a STATUS sent after the SET
                if(app.nav_camera.robot_status_n>=g.status_wait && g.status_wait>0)
                    st=app.nav_camera.robot_status;
                    if(~any([st.mx st.my st.mz]))
                        g=begin_measure(g);
                        app.nav_camera.robot_goto=g;
                        return;
                    end;
                    g.status_wait=0;
                end;
                if(toc(g.t)>g.timeout)
                    finish(app,'!',sprintf('Goto stopped: the robot did not finish the move within %.0f s.',g.timeout));
                    return;
                end;
                if(g.status_wait==0 && (isempty(g.t_poll) || toc(g.t_poll)>=status_period_s))
                    nav_camera_tool_comm(app,'status');
                    g.status_wait=nav_camera_tool_comm(app,'status_target');
                    g.t_poll=tic;
                end;

            case "measure"   % settle, then average the marker
                if(toc(g.t)<settle_s) return; end;
                [p,valid]=nav_robot_tab(app,'marker_pos');
                if(valid) g.samples(:,end+1)=p; end;
                if(toc(g.t)<settle_s+sample_s) app.nav_camera.robot_goto=g; return; end;
                if(size(g.samples,2)<2)
                    finish(app,'!','Goto stopped: the marker is not visible.');
                    return;
                end;
                p=mean(g.samples,2);
                app.nav_camera.robot_goto=g;
                measured(app,p);
                return;
        end;
        app.nav_camera.robot_goto=g;
    end

    function g=begin_measure(g)
        g.phase="measure"; g.t=tic; g.samples=zeros(3,0);
    end

    % ------------------------------------------------------------------
    function measured(app,p)
        % called with the settled marker position p after each move
        g=app.nav_camera.robot_goto;
        t=app.nav_camera.robot_target;
        if(g.mode=="return")   % CALIBRATE finished and the robot is back at the start
            finish(app,'*',sprintf('Calibration done; robot back at X=%g Y=%g Z=%g.',g.r));
            return;
        end;
        if(isempty(app.nav_camera.robot_goto_J))   % calibrating
            g.cal_p(:,end+1)=p;
            g.cal_r(:,end+1)=g.r;
            k=size(g.cal_p,2);   % 1 = start, 2..4 = after moving X, Y, Z
            if(k<=3)
                dr=zeros(3,1);
                dr(k)=g.cal_step;
                if(g.r(k)+dr(k)>r_lim) dr(k)=-g.cal_step; end;   % stay inside the range
                axn='XYZ';
                log_msg(app,'*',sprintf('Calibration %d/3: %s %+g mm',k,axn(k),dr(k)));
                app.nav_camera.robot_goto=g;
                move_to(app,g.r+dr);
                return;
            end;
            J=diff(g.cal_p,1,2)./diag(diff(g.cal_r,1,2)).';   % column k: marker motion per mm of axis k
            moved=sqrt(sum(J.^2,1));
            if(any(moved<0.3) || rcond(J)<1e-3)
                finish(app,'!',sprintf(['Calibration failed, marker moved [%.2f %.2f %.2f] mm per mm of X/Y/Z. ' ...
                    'Is the marker attached to the robot?'],moved));
                return;
            end;
            app.nav_camera.robot_goto_J=J;
            app.nav_camera.robot_goto_J_marker=app.nav_camera.robot_marker_idx;
            log_msg(app,'*',sprintf('Calibrated: marker mm per robot mm  X->[%.2f %.2f %.2f]  Y->[%.2f %.2f %.2f]  Z->[%.2f %.2f %.2f]',J));
            if(g.mode=="calibrate")   % CALIBRATE command: go back to where it started
                g.mode="return";
                app.nav_camera.robot_goto=g;
                log_msg(app,'*','Calibration: returning to the start position.');
                move_to(app,g.cal_r(:,1));
                return;
            end;
        end;

        e=p-t;
        en=norm(e);
        g.err(end+1)=en;
        log_msg(app,'*',sprintf('Goto: marker - target = [%.1f %.1f %.1f], |d| = %.2f mm',e,en));
        if(en<=tol_mm)
            app.nav_camera.robot_goto=g;
            finish(app,'*',sprintf('Goto done: |d| = %.2f mm after %d move(s).',en,g.iter));
            return;
        end;
        if(numel(g.err)>=3 && g.err(end)>g.err(end-1) && g.err(end-1)>g.err(end-2))
            app.nav_camera.robot_goto_J=[];
            app.nav_camera.robot_goto=g;
            finish(app,'!','Goto stopped: the error grew twice in a row; calibration dropped (the next Goto recalibrates).');
            return;
        end;
        if(g.iter>=max_iter)
            app.nav_camera.robot_goto=g;
            finish(app,'!',sprintf('Goto stopped after %d moves: |d| = %.2f mm.',g.iter,en));
            return;
        end;
        dr=-app.nav_camera.robot_goto_J\e;
        if(norm(dr)>max_step_mm) dr=dr*max_step_mm/norm(dr); end;
        r_new=min(max(g.r+dr,-r_lim),r_lim);
        if(any(abs(g.r+dr)>r_lim+mm_per_step))
            g.n_clamped=g.n_clamped+1;   % the move wanted to leave the robot range
        else
            g.n_clamped=0;
        end;
        if(g.n_clamped>=2)
            axn='XYZ';
            app.nav_camera.robot_goto=g;
            finish(app,'!',sprintf('Goto stopped: the target is out of reach (robot %s at its +/-%g mm limit); |d| = %.2f mm.', ...
                axn(abs(g.r+dr)>r_lim),r_lim,en));
            return;
        end;
        r_new=round(r_new/mm_per_step)*mm_per_step;
        if(all(abs(r_new-g.r)<mm_per_step/2))
            app.nav_camera.robot_goto=g;
            if(any(abs(g.r+dr)>r_lim))
                finish(app,'!',sprintf('Goto stopped: the target is outside the robot range (+/-%g mm); |d| = %.2f mm.',r_lim,en));
            else
                finish(app,'*',sprintf('Goto done: |d| = %.2f mm (within one robot step).',en));
            end;
            return;
        end;
        g.iter=g.iter+1;
        app.nav_camera.robot_goto=g;
        move_to(app,r_new);
    end

    function move_to(app,r_new)
        g=app.nav_camera.robot_goto;
        ax=["X","Y","Z"];
        n_steps=0;
        for i=1:3
            if(abs(r_new(i)-g.r(i))>=mm_per_step/2)
                nav_camera_tool_comm(app,'set',ax(i),r_new(i));
                n_steps=max(n_steps,abs(r_new(i)-g.r(i))/mm_per_step);
            end;
        end;
        g.r=r_new;
        g.phase="moving"; g.t=tic; g.t_poll=[]; g.status_wait=0;
        g.timeout=n_steps/steps_per_s*1.5+5;
        app.nav_camera.robot_goto=g;
    end

end


% ======================================================================
function start_goto(app,tick_s,mode,cal_step)
% mode "goto": Goto button (calibrates first if needed); "calibrate": CALIBRATE command only
who='Goto';
if(mode=="calibrate") who='CALIBRATE'; end;
if(~nav_camera_tool_comm(app,'connected'))
    log_msg(app,'!',[who ': not connected; press Comm. first.']);
    return;
end;
if(isempty(app.nav_camera.robot_marker_idx))
    log_msg(app,'!',[who ': choose a marker first.']);
    return;
end;
if(mode=="goto" && ~all(isfinite(app.nav_camera.robot_target)))
    log_msg(app,'!','Goto: type a target first.');
    return;
end;
[~,valid]=nav_robot_tab(app,'marker_pos');
if(~valid)
    log_msg(app,'!',[who ': the marker is not visible.']);
    return;
end;
if(mode=="calibrate" || ~isfield(app.nav_camera,'robot_goto_J') || ~isfield(app.nav_camera,'robot_goto_J_marker') || ...
        ~isequal(app.nav_camera.robot_goto_J_marker,app.nav_camera.robot_marker_idx))
    app.nav_camera.robot_goto_J=[];   % CALIBRATE, none yet, or measured with another marker
end;
g=struct('phase',"first_status",'t',tic,'t_poll',[],'status_wait',0,'timeout',0, ...
    'r',nan(3,1),'samples',zeros(3,0),'cal_p',zeros(3,0),'cal_r',zeros(3,0),'err',zeros(1,0),'iter',0,'n_clamped',0,'mode',mode,'cal_step',cal_step);
app.nav_camera.robot_set_err="";
nav_camera_tool_comm(app,'status');
g.status_wait=nav_camera_tool_comm(app,'status_target');
app.nav_camera.robot_goto=g;
t=app.nav_camera.robot_target;
if(mode=="calibrate")
    log_msg(app,'*',sprintf('CALIBRATE: moving X, Y, Z by %g mm in turn, then back to the start.',cal_step));
elseif(isempty(app.nav_camera.robot_goto_J))
    log_msg(app,'*',sprintf('Goto target x=%.1f y=%.1f z=%.1f: calibrating first (moves X, Y, Z in turn).',t));
else
    log_msg(app,'*',sprintf('Goto target x=%.1f y=%.1f z=%.1f.',t));
end;
app.nav_camera.robot_goto_timer=timer('ExecutionMode','fixedSpacing','Period',tick_s,'BusyMode','drop', ...
    'Name','nav_robot_goto','TimerFcn',@(~,~) nav_robot_goto(app,'tick'));
start(app.nav_camera.robot_goto_timer);
try app.TargetGotoButton.Text='Abort'; catch; end;
end


function tf=is_running(app)
tf=isfield(app.nav_camera,'robot_goto_timer') && ~isempty(app.nav_camera.robot_goto_timer) && isvalid(app.nav_camera.robot_goto_timer);
end


function finish(app,kind,str)
if(is_running(app))
    stop(app.nav_camera.robot_goto_timer);
    delete(app.nav_camera.robot_goto_timer);
end;
app.nav_camera.robot_goto_timer=[];
try app.TargetGotoButton.Text='Goto'; catch; end;
log_msg(app,kind,str);
end


function log_msg(app,kind,str)
nav_robot_tab(app,'log',kind,char(str));
end


function w=upper_who(app)
w='Goto';
if(isfield(app.nav_camera,'robot_goto') && isfield(app.nav_camera.robot_goto,'mode') && app.nav_camera.robot_goto.mode~="goto")
    w='Calibration';
end;
end
