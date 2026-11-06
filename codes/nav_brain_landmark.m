function nav_brain_landmark(app,action,varargin)
%
%   nav_brain_landmark      anatomical landmarks: load, sample with the pointer, register
%
%   nav_brain_landmark(app,'load')      choose a landmark .mat file (variable 'sensor' with
%                                       fields 'name' and 'coords' (MRI mm), as made by
%                                       prep_landmark.m) and list it in app.LandmarkListBox
%   nav_brain_landmark(app,'sample')    start sampling the landmark highlighted in the list box:
%                                       the hot spot of the tool in app.LandmarkToolDropDown,
%                                       expressed in the frame of the head tracker in
%                                       app.BrainToolDropDown, averaged over n_sample frames
%   nav_brain_landmark(app,'draw')      (re)draw the landmark markers in app.UIAxes2
%   nav_brain_landmark(app,'register')  fit MRI -> head-tracker transform from the sampled landmarks
%   nav_brain_landmark(app,'update_button')   enable 'Landmark sample' only while the landmark
%                                       tool and the head tracker both give valid poses
%                                       (called by nav_camera_graphic_update every frame)
%   nav_brain_landmark(app,'stream_stopped')  disable 'Landmark sample' when streaming ends
%   nav_brain_landmark(app,'sample_frame',R,component_idx)
%                                       called by nav_camera_graphic_update for every tool pose
%                                       while a sample is being collected
%
%   Colours (list box and markers): red = not sampled, orange = sampling, green = sampled.
%   When every landmark is sampled, the registration is computed automatically; from then on
%   nav_camera_graphic_update moves the brain with the head tracker.
%
%   app.nav_camera.landmark_tip (3x1, mm, default [0;0;0]) is the hot spot in the landmark
%   tool's own frame. For NDI probes the .rom file puts the tool origin at the tip, so 0.
%
% fhlin@Nov 5, 2026
%

n_sample=30;        % frames averaged per landmark (~0.5 s at 60 Hz)
max_spread=2;       % mm; reject the sample if the hot spot moved more than this
max_head_age=0.05;  % s; head pose must be this fresh to pair with a pointer pose
sample_timeout=5;   % s

switch lower(action)
    case 'load'
        [file,path]=uigetfile('*.mat','Select landmark file');
        if(isequal(file,0)) return; end;
        S=load(fullfile(path,file));
        if(~isfield(S,'sensor') || ~isfield(S.sensor,'name') || ~isfield(S.sensor,'coords'))
            uialert(app.nav_camera_terminalUIFigure,sprintf('[%s] has no variable "sensor" with fields "name" and "coords".',file),'Landmark file');
            return;
        end;
        name=cellstr(string(S.sensor.name(:)));
        mri=double(S.sensor.coords);
        if(size(mri,2)~=3 && size(mri,1)==3) mri=mri.'; end;
        if(size(mri,1)~=numel(name))
            uialert(app.nav_camera_terminalUIFigure,sprintf('%d names but %d coordinates in [%s].',numel(name),size(mri,1),file),'Landmark file');
            return;
        end;

        lm.file=fullfile(path,file);
        lm.name=name;
        lm.mri=mri;                         % n x 3, MRI (surface) coordinates, mm
        lm.head=nan(numel(name),3);         % n x 3, hot spot in the head-tracker frame, mm
        lm.spread=nan(numel(name),1);       % mm
        lm.h=gobjects(0);
        app.nav_camera.landmark=lm;
        app.nav_camera.landmark_sampling=[];
        app.nav_camera.reg=[];              % old registration no longer valid
        app.nav_camera.R0=[];

        app.LandmarkListBox.Items=name;
        app.LandmarkListBox.ItemsData=1:numel(name);
        app.LandmarkListBox.Value=1;

        nav_brain_landmark(app,'draw');
        nav_brain_landmark(app,'update_button');
        show_msg(app,sprintf('%d landmarks loaded from %s. Highlight one, hold the pointer on it, click ''Landmark sample''.',numel(name),file));

    case 'draw'
        if(~has_landmark(app)) return; end;
        lm=app.nav_camera.landmark;
        delete(lm.h(isgraphics(lm.h)));
        parent=brain_parent(app);           % hgtransform of the brain (moves with it) or the axes
        hold(app.UIAxes2,'on');
        h=gobjects(0);
        for k=1:numel(lm.name)
            h(end+1)=line('Parent',parent,'XData',lm.mri(k,1),'YData',lm.mri(k,2),'ZData',lm.mri(k,3), ...
                'LineStyle','none','Marker','.','MarkerSize',30,'Tag',sprintf('landmark%d',k));
            h(end+1)=text('Parent',parent,'Position',lm.mri(k,:),'String',['  ' lm.name{k}],'FontSize',12,'Tag',sprintf('landmark%d',k));
        end;
        app.nav_camera.landmark.h=h;
        refresh_colors(app);

    case 'sample'
        if(~has_landmark(app))
            show_msg(app,'Load a landmark file first.');
            return;
        end;
        fr=app.nav_camera.flag_repeat;
        is_streaming=(ischar(fr)&&strcmpi(fr,'On'))||(isnumeric(fr)&&~isempty(fr)&&fr~=0);
        if(~is_streaming)
            show_msg(app,'Start streaming (BX2, repeat On, Send) before sampling landmarks.');
            return;
        end;
        [head_idx,ptr_idx]=tool_idx(app);
        if(head_idx==ptr_idx)
            show_msg(app,'The landmark tool and the brain (head) tool must be different tools.');
            return;
        end;
        [R_head,age_head]=last_pose(app,head_idx);
        if(isempty(R_head) || age_head>0.1)
            show_msg(app,sprintf('Head tracker #%d (%s, Brain tool dropdown) is not visible to the camera; it is needed to sample a landmark.',head_idx,tool_name(app,head_idx)));
            return;
        end;
        k=app.LandmarkListBox.Value;
        if(~isnumeric(k) || isempty(k)) return; end;
        app.nav_camera.landmark_sampling=struct('k',k,'buf',zeros(3,0),'t_start',tic, ...
            'head_idx',head_idx,'ptr_idx',ptr_idx);
        refresh_colors(app);
        nav_brain_landmark(app,'update_button');     % disable while sampling
        show_msg(app,sprintf('Sampling [%s]: hold the pointer still...',app.nav_camera.landmark.name{k}));

    case 'sample_frame'
        s=app.nav_camera.landmark_sampling;
        if(isempty(s)) return; end;
        R=varargin{1};
        component_idx=varargin{2};
        lm=app.nav_camera.landmark;

        if(toc(s.t_start)>sample_timeout)
            app.nav_camera.landmark_sampling=[];
            refresh_colors(app);
            show_msg(app,sprintf('Sampling [%s] timed out: only %d of %d frames had both the pointer and the head tracker visible.',lm.name{s.k},size(s.buf,2),n_sample));
            return;
        end;
        if(component_idx~=s.ptr_idx || any(~isfinite(R(:)))) return; end;

        [R_head,age]=last_pose(app,s.head_idx);
        if(isempty(R_head) || age>max_head_age) return; end;     % head not visible in this frame

        tip=landmark_tip(app);
        x=rigid_inv(R_head)*R*[tip;1];          % hot spot in the head-tracker frame
        s.buf(:,end+1)=x(1:3);
        app.nav_camera.landmark_sampling=s;
        if(size(s.buf,2)<n_sample) return; end;

        % enough frames: check the pointer was held still
        m=mean(s.buf,2);
        spread=max(sqrt(sum((s.buf-m).^2,1)));
        app.nav_camera.landmark_sampling=[];
        if(spread>max_spread)
            refresh_colors(app);
            show_msg(app,sprintf('[%s] rejected: hot spot moved %.1f mm during sampling (max %.1f). Hold still and sample again.',lm.name{s.k},spread,max_spread));
            return;
        end;
        app.nav_camera.landmark.head(s.k,:)=m.';
        app.nav_camera.landmark.spread(s.k)=spread;
        app.nav_camera.reg=[];

        done=all(isfinite(app.nav_camera.landmark.head),2);
        refresh_colors(app);
        if(all(done))
            nav_brain_landmark(app,'register');
        else
            % move the highlight to the next landmark still to be sampled
            nxt=find(~done);
            nxt=[nxt(nxt>s.k); nxt];
            app.LandmarkListBox.Value=nxt(1);
            show_msg(app,sprintf('[%s] sampled (spread %.1f mm). %d of %d done; next: [%s].',lm.name{s.k},spread,sum(done),numel(done),lm.name{nxt(1)}));
        end;

    case 'update_button'
        % enable 'Landmark sample' only while the chosen landmark tool gives valid data:
        % landmarks loaded, no sample in progress, and the landmark tool has reported a
        % valid pose within max_valid_age (the head tracker is checked when you click)
        max_valid_age=0.1;   % s
        if(~isprop(app,'LandmarkSampleButton')) return; end;
        [~,ptr_idx]=tool_idx(app);
        [R_ptr,age_ptr]=last_pose(app,ptr_idx);
        if(~has_landmark(app))
            ok=false; why='Load a landmark file first.';
        elseif(isfield(app.nav_camera,'landmark_sampling') && ~isempty(app.nav_camera.landmark_sampling))
            ok=false; why='Sampling in progress...';
        elseif(~seen(app,ptr_idx))
            ok=false; why=sprintf('Landmark tool #%d (%s) has not reported any pose since streaming started; check that the dropdown matches the tool order in the stream.',ptr_idx,tool_name(app,ptr_idx));
        elseif(isempty(R_ptr) || age_ptr>max_valid_age)
            ok=false; why=sprintf('Landmark tool #%d (%s) not visible to the camera.',ptr_idx,tool_name(app,ptr_idx));
        else
            ok=true; why=sprintf('Landmark tool #%d (%s) valid: hold the hot spot on the highlighted landmark and click.',ptr_idx,tool_name(app,ptr_idx));
        end;
        % only touch the UI when the reason changes (this runs for every tool on every frame)
        if(~isfield(app.nav_camera,'lm_button_why') || ~strcmp(app.nav_camera.lm_button_why,why))
            app.nav_camera.lm_button_why=why;
            app.LandmarkSampleButton.Enable=matlab.lang.OnOffSwitchState(ok);
            app.LandmarkSampleButton.Tooltip=why;
            fprintf('Landmark sample button %s: %s\n',char(matlab.lang.OnOffSwitchState(ok)),why);
        end;

    case 'stream_stopped'
        % no more frames will arrive: poses go stale without anyone noticing, so disable now
        if(~isprop(app,'LandmarkSampleButton')) return; end;
        app.nav_camera.lm_button_why='Start streaming (BX2, repeat On, Send) to sample landmarks.';
        app.LandmarkSampleButton.Enable='off';
        app.LandmarkSampleButton.Tooltip=app.nav_camera.lm_button_why;
        if(isfield(app.nav_camera,'lm_ptr_h') && isgraphics(app.nav_camera.lm_ptr_h))
            app.nav_camera.lm_ptr_h.Visible='off';     % hot-spot dot would be stale
        end;

    case 'register'
        if(~has_landmark(app)) return; end;
        lm=app.nav_camera.landmark;
        ok=all(isfinite(lm.head),2);
        if(sum(ok)<3)
            show_msg(app,'At least 3 sampled landmarks are needed for registration.');
            return;
        end;
        A=lm.mri(ok,:);  B=lm.head(ok,:);     % B ~ Rr*A + t
        ca=mean(A,1);    cb=mean(B,1);
        [U,~,V]=svd((A-ca).'*(B-cb));
        Rr=V*diag([1 1 sign(det(V*U.'))])*U.';
        t=cb.'-Rr*ca.';
        T=[Rr t; 0 0 0 1];                   % MRI -> head-tracker frame

        err=sqrt(sum((Rr*A.'+t-B.').^2,1));
        reg.T=T;
        reg.err=nan(numel(lm.name),1);
        reg.err(ok)=err(:);
        reg.rms=sqrt(mean(err.^2));
        app.nav_camera.reg=reg;
        app.nav_camera.R0=[];                % brain motion restarts from the next head frame

        names=lm.name(ok);
        str=sprintf('Registered with %d landmarks, RMS %.1f mm (',sum(ok),reg.rms);
        for k=1:numel(names)
            str=[str sprintf('%s %.1f',names{k},err(k))];
            if(k<numel(names)) str=[str ', ']; end;
        end;
        show_msg(app,[str ' mm). The brain now follows the head tracker.']);

    otherwise
        fprintf('nav_brain_landmark: unknown action [%s]\n',action);
end;

return;


function ok=has_landmark(app)
ok=isfield(app.nav_camera,'landmark') && ~isempty(app.nav_camera.landmark);


function refresh_colors(app)
% list box items and markers: red = to do, orange = sampling, green = sampled
lm=app.nav_camera.landmark;
n=numel(lm.name);
col=repmat([0.85 0 0],n,1);
col(all(isfinite(lm.head),2),:)=repmat([0 0.6 0],sum(all(isfinite(lm.head),2)),1);
if(isfield(app.nav_camera,'landmark_sampling') && ~isempty(app.nav_camera.landmark_sampling))
    col(app.nav_camera.landmark_sampling.k,:)=[1 0.5 0];
end;
try
    removeStyle(app.LandmarkListBox);
    for k=1:n
        addStyle(app.LandmarkListBox,uistyle('FontColor',col(k,:),'FontWeight','bold'),'item',k);
    end;
catch
end;
for k=1:n
    h=findobj(lm.h(isgraphics(lm.h)),'flat','Tag',sprintf('landmark%d',k));   % marker and its label
    set(h,'Color',col(k,:));
end;


function parent=brain_parent(app)
global etc_render_fsbrain;
parent=app.UIAxes2;
if(isstruct(etc_render_fsbrain) && isfield(etc_render_fsbrain,'tr') && isgraphics(etc_render_fsbrain.tr) ...
        && ancestor(etc_render_fsbrain.tr,'axes')==app.UIAxes2)
    parent=etc_render_fsbrain.tr;
end;


function [head_idx,ptr_idx]=tool_idx(app)
head_idx=1; ptr_idx=2;
if(isprop(app,'BrainToolDropDown') && isnumeric(app.BrainToolDropDown.Value) && ~isempty(app.BrainToolDropDown.Value))
    head_idx=app.BrainToolDropDown.Value;
end;
if(isprop(app,'LandmarkToolDropDown') && isnumeric(app.LandmarkToolDropDown.Value) && ~isempty(app.LandmarkToolDropDown.Value))
    ptr_idx=app.LandmarkToolDropDown.Value;
end;


function [R,age]=last_pose(app,idx)
R=[]; age=inf;
if(~isfield(app.nav_camera,'last_R') || size(app.nav_camera.last_R,3)<idx) return; end;
R=app.nav_camera.last_R(:,:,idx);
if(any(~isfinite(R(:)))) R=[]; return; end;
age=toc(app.nav_camera.clock0)-app.nav_camera.last_R_t(idx);


function ok=seen(app,idx)
% has this tool slot received any pose (valid or missing) since streaming started?
ok=isfield(app.nav_camera,'last_R_t') && numel(app.nav_camera.last_R_t)>=idx && isfinite(app.nav_camera.last_R_t(idx));


function str=tool_name(app,idx)
str='?';
if(isfield(app.nav_camera,'tool') && numel(app.nav_camera.tool)>=idx && isfield(app.nav_camera.tool,'name') && ~isempty(app.nav_camera.tool(idx).name))
    str=char(app.nav_camera.tool(idx).name);
end;


function tip=landmark_tip(app)
tip=[0;0;0];
if(isfield(app.nav_camera,'landmark_tip') && numel(app.nav_camera.landmark_tip)==3)
    tip=double(app.nav_camera.landmark_tip(:));
end;


function Ri=rigid_inv(R)
Ri=[R(1:3,1:3).', -R(1:3,1:3).'*R(1:3,4); 0 0 0 1];


function show_msg(app,str)
fprintf('%s\n',str);
title(app.UIAxes2,str,'FontSize',11,'FontWeight','normal','Interpreter','none');
