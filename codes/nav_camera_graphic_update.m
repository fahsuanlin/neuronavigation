function [varargout]=nav_camera_graphic_update(app,R,component_idx,varargin)

global etc_render_fsbrain;

color=[];
for i=1:length(varargin)/2
    option=varargin{i*2-1};
    option_value=varargin{i*2};
    switch lower(option)
        case 'color'
            color=option_value;
        otherwise
            fprintf('unknown option [%s]! error!\n',option);
            return;
    end;
end;

marker_color={      % live tool dots; blue is reserved for marked reference points
    [1 0 0];
    [0 0.6 0];
    [1 0.5 0];};

C = [ 0 -1  0;
    -1  0  0;
    0  0 -1];          % det(C) = +1, so this is a pure rotation, no mirroring

p = C * R(1:3,4);   % display coordinates

hold(app.UIAxes,"on");

% style the tracking axes once (box, grid, true proportions, meaningful labels)
if(~isfield(app.nav_camera,'axes_styled') || ~app.nav_camera.axes_styled)
    box(app.UIAxes,'on');
    app.UIAxes.BoxStyle='full';      % all 12 edges of the 3D box
    grid(app.UIAxes,'on');
    app.UIAxes.GridAlpha=0.25;
    daspect(app.UIAxes,[1 1 1]);     % same mm scale on every axis
    xlabel(app.UIAxes,'right (mm)');
    ylabel(app.UIAxes,'up (mm)');
    zlabel(app.UIAxes,'distance from camera (mm)');
    app.nav_camera.axes_styled=true;
end;

if(isfield(app.nav_camera,'dot_h'))
    if(length(app.nav_camera.dot_h)>=component_idx)
        try
            set(app.nav_camera.dot_h(component_idx),'XData',p(1),'YData',p(2),'ZData',p(3));

            %             set(app.nav_camera.dot_h(component_idx),'XData',R(1,4));
            %             set(app.nav_camera.dot_h(component_idx),'YData',R(2,4));
            %             set(app.nav_camera.dot_h(component_idx),'ZData',R(3,4));
        catch
            app.nav_camera.dot_h(component_idx)=plot3(app.UIAxes,p(1),p(2),p(3),'.');
            %app.nav_camera.dot_h(component_idx)=plot3(app.UIAxes,R(1,4), R(2,4), R(3,4), '.');
        end;
    else
        %app.nav_camera.dot_h(component_idx)=plot3(app.UIAxes,R(1,4), R(2,4), R(3,4), '.');
        app.nav_camera.dot_h(component_idx)=plot3(app.UIAxes,p(1),p(2),p(3),'.');
    end;
else
    %app.nav_camera.dot_h(component_idx)=plot3(app.UIAxes,R(1,4), R(2,4), R(3,4), '.');
    app.nav_camera.dot_h(component_idx)=plot3(app.UIAxes,p(1),p(2),p(3),'.');
end;

% dot colour = the tool's colour in the tool table (set by Tool check); same tool numbering
% as the Brain/Landmark tool dropdowns. Fallback before Tool check: marker_color.
if(isempty(color))
    if(isfield(app.nav_camera,'tool') && numel(app.nav_camera.tool)>=component_idx ...
            && isfield(app.nav_camera.tool,'color') && ~isempty(app.nav_camera.tool(component_idx).color))
        color=app.nav_camera.tool(component_idx).color;
    else
        color=marker_color{mod(component_idx-1,numel(marker_color))+1};
    end;
end;
set(app.nav_camera.dot_h(component_idx),'MarkerSize',24,'Color',color);

% auto-fit app.UIAxes to where the tools actually are:
% keep a bounding box of all tool positions seen so far (it only grows) and
% show it with a margin; limits are only re-set when the box has grown by
% more than view_step (so tracking jitter doesn't re-set them every frame).
% Reset with app.nav_camera.view_box=[] (e.g. on connect) to fit again from scratch.
view_margin = 150;   % mm around the tools
view_min_span = 400; % mm; minimum width of each axis so a single tool isn't zoomed in too far
view_step = 20;      % mm; growth needed before the limits are updated
if(all(isfinite(p)))
    if(~isfield(app.nav_camera,'view_box') || isempty(app.nav_camera.view_box))
        bbox = [p(:) p(:)];                          % 3x2: [min max] per display axis
        box_shown = [];
    else
        bbox = app.nav_camera.view_box;
        bbox = [min(bbox(:,1),p(:)) max(bbox(:,2),p(:))];
        box_shown = app.nav_camera.view_box_shown;
    end;
    app.nav_camera.view_box = bbox;
    if(isempty(box_shown) || any(abs(bbox(:)-box_shown(:))>view_step))
        app.nav_camera.view_box_shown = bbox;
        ctr  = mean(bbox,2);
        half = max((bbox(:,2)-bbox(:,1))/2 + view_margin, view_min_span/2);
        set(app.UIAxes,'XLim',ctr(1)+[-1 1]*half(1),'YLim',ctr(2)+[-1 1]*half(2),'ZLim',ctr(3)+[-1 1]*half(3));
    end;
end;

% connect the 'Mark ref' button if the app did not give it a callback
if(isprop(app,'MarkrefButton') && isempty(app.MarkrefButton.ButtonPushedFcn))
    app.MarkrefButton.ButtonPushedFcn=@(src,event) nav_camera_mark_ref(app);
end;

% remember the latest position of each tool (display coordinates, NaN when missing);
% nav_camera_mark_ref uses it when the 'Mark ref' button is pressed
if(~isfield(app.nav_camera,'last_p'))
    app.nav_camera.last_p=nan(3,0);
end;
if(size(app.nav_camera.last_p,2)<component_idx)
    app.nav_camera.last_p(:,end+1:component_idx)=nan;   % new columns as NaN, not 0
end;
app.nav_camera.last_p(:,component_idx)=p(:);

% remember the latest full pose (camera frame) of each tool and when it arrived;
% nav_brain_landmark pairs pointer and head-tracker poses from the same moment
if(~isfield(app.nav_camera,'clock0') || isempty(app.nav_camera.clock0))
    app.nav_camera.clock0=tic;
end;
if(~isfield(app.nav_camera,'last_R'))
    app.nav_camera.last_R=nan(4,4,0);
    app.nav_camera.last_R_t=zeros(1,0);
end;
if(size(app.nav_camera.last_R,3)<component_idx)
    app.nav_camera.last_R(:,:,end+1:component_idx)=nan;
    app.nav_camera.last_R_t(end+1:component_idx)=-inf;
end;
app.nav_camera.last_R(:,:,component_idx)=R;
app.nav_camera.last_R_t(component_idx)=toc(app.nav_camera.clock0);

% landmark sampling in progress -> hand this pose to the sampler
if(isfield(app.nav_camera,'landmark_sampling') && ~isempty(app.nav_camera.landmark_sampling))
    nav_brain_landmark(app,'sample_frame',R,component_idx);
end;

% enable/disable 'Landmark sample' according to whether both tools are currently valid
nav_brain_landmark(app,'update_button');

% offset of each tool from its marked reference -> MarkrefUITable (throttled to ~5 Hz)
if(isfield(app.nav_camera,'ref_p') && size(app.nav_camera.ref_p,2)>=component_idx && all(isfinite(app.nav_camera.ref_p(:,component_idx))))
    if(~isfield(app.nav_camera,'ref_table_t') || toc(app.nav_camera.ref_table_t)>0.2)
        app.nav_camera.ref_table_t=tic;
        nav_camera_mark_ref(app,'update_table');
    end;
end;


% tool that is attached to the head: the item selected in the Brain tab's dropdown
% (Value is the tool index once ToolcheckButtonPushed sets ItemsData=1:n)
head_tool_idx = 1;   % fallback before tool check has filled the dropdown
if(isprop(app,'BrainToolDropDown') && isnumeric(app.BrainToolDropDown.Value) && ~isempty(app.BrainToolDropDown.Value))
    head_tool_idx = app.BrainToolDropDown.Value;
end;

% selection changed -> take a new reference on this tool's next valid frame
if(~isfield(app.nav_camera,'head_tool_idx') || app.nav_camera.head_tool_idx~=head_tool_idx)
    app.nav_camera.head_tool_idx = head_tool_idx;
    app.nav_camera.R0 = [];
end;

% how the brain in the Brain tab moves (see nav_brain_landmark):
%   no landmark file loaded           -> follows the head tool relative to its first pose,
%                                        rotating about the brain centroid (no registration)
%   landmarks loaded, not all sampled -> held still until the registration is done
%   registered (app.nav_camera.reg.T) -> follows the head tracker in anatomical (MRI) axes,
%                                        and the landmark tool's hot spot is shown as a dot
has_lm   = isfield(app.nav_camera,'landmark') && ~isempty(app.nav_camera.landmark);
has_reg  = isfield(app.nav_camera,'reg') && ~isempty(app.nav_camera.reg);
brain_ok = isfield(app.nav_camera,'etc_render_fsbrain') && ~isempty(app.nav_camera.etc_render_fsbrain) ...
    && isstruct(etc_render_fsbrain) && isfield(etc_render_fsbrain,'tr') && isgraphics(etc_render_fsbrain.tr);

if(brain_ok && all(isfinite(R(:))) && component_idx==head_tool_idx)
    try
        if(has_lm && ~has_reg)
            if(~isequal(etc_render_fsbrain.tr.Matrix,eye(4)))
                etc_render_fsbrain.tr.Matrix = eye(4);       % wait for the registration
            end;
            app.nav_camera.R0 = [];
        elseif(~isfield(app.nav_camera,'R0') || isempty(app.nav_camera.R0))
            app.nav_camera.R0 = R;                           % head pose at the start of following

            % centre the axes on the brain, with room for it to move
            V0 = double(etc_render_fsbrain.h.Vertices);
            c  = mean(V0,1).';                               % 3x1 centroid
            app.nav_camera.brain_center = c;
            r = max(sqrt(sum((V0 - c.').^2,2)));             % brain radius (mm)
            app.nav_camera.brain_radius = r;
            app.nav_camera.brain_lim = [c c] + [-2 2]*r;     % 3x2 [min max]; grows if the brain moves out
            set(app.UIAxes2,'xlim',app.nav_camera.brain_lim(1,:),'ylim',app.nav_camera.brain_lim(2,:),'zlim',app.nav_camera.brain_lim(3,:));
            etc_render_fsbrain.tr.Matrix = eye(4);
        else
            D = rigid_inv(app.nav_camera.R0) * R;            % head motion in the head tool's initial frame
            if(has_reg)
                T = app.nav_camera.reg.T;                    % MRI -> head-tracker frame
                M = rigid_inv(T) * D * T;                    % the same motion, in MRI coordinates
            else
                c = app.nav_camera.brain_center;
                M = makehgtform('translate',c) * D * makehgtform('translate',-c);
            end;
            [U,~,V] = svd(M(1:3,1:3));                       % snap to an exact rotation (avoids hgtransform warnings)
            M(1:3,1:3) = U*V.';
            M(4,:) = [0 0 0 1];
            etc_render_fsbrain.tr.Matrix = M;

            % keep the whole brain inside the axes: every vertex stays within brain_radius of the
            % moved centroid (whatever the rotation), so grow the limits (never shrink, so the view
            % does not jump around) whenever that sphere would cross the axes box
            if(isfield(app.nav_camera,'brain_lim') && ~isempty(app.nav_camera.brain_lim))
                cm  = M * [app.nav_camera.brain_center; 1];
                need = [cm(1:3) cm(1:3)] + [-1.2 1.2]*app.nav_camera.brain_radius;
                lim = app.nav_camera.brain_lim;
                if(any(need(:,1)<lim(:,1)) || any(need(:,2)>lim(:,2)))
                    span = max(lim(:,2)-lim(:,1));
                    lim  = [min(lim(:,1),need(:,1)-0.25*span) max(lim(:,2),need(:,2)+0.25*span)];   % grow with spare room
                    app.nav_camera.brain_lim = lim;
                    set(app.UIAxes2,'xlim',lim(1,:),'ylim',lim(2,:),'zlim',lim(3,:));
                end;
            end;
        end;
    catch ME
        fprintf('graphic update error: %s\n', ME.message);
    end;
end;

% registered: show the landmark tool's hot spot in the Brain tab (same frame as the brain display)
% as a magenta dot -- touch a landmark and the dot should sit on that landmark of the mesh.
% The dot is hidden whenever the landmark tool is not currently tracked, so it never
% lingers at a stale position.
if(isfield(app.nav_camera,'lm_ptr_h') && isgraphics(app.nav_camera.lm_ptr_h) && (~brain_ok || ~has_reg))
    delete(app.nav_camera.lm_ptr_h);              % registration gone (new landmark file / brain)
end;
if(brain_ok && has_reg && isfield(app.nav_camera,'R0') && ~isempty(app.nav_camera.R0))
    ptr_idx = 2;
    if(isprop(app,'LandmarkToolDropDown') && isnumeric(app.LandmarkToolDropDown.Value) && ~isempty(app.LandmarkToolDropDown.Value))
        ptr_idx = app.LandmarkToolDropDown.Value;
    end;
    have_dot = isfield(app.nav_camera,'lm_ptr_h') && isgraphics(app.nav_camera.lm_ptr_h);
    if(component_idx==ptr_idx && ptr_idx~=head_tool_idx && all(isfinite(R(:))))
        tip = [0;0;0];
        if(isfield(app.nav_camera,'landmark_tip') && numel(app.nav_camera.landmark_tip)==3)
            tip = double(app.nav_camera.landmark_tip(:));
        end;
        x = rigid_inv(app.nav_camera.reg.T) * rigid_inv(app.nav_camera.R0) * R * [tip;1];
        if(~have_dot)
            app.nav_camera.lm_ptr_h = line('Parent',app.UIAxes2,'XData',x(1),'YData',x(2),'ZData',x(3), ...
                'LineStyle','none','Marker','.','MarkerSize',36,'Color',[1 0 1]);
        else
            set(app.nav_camera.lm_ptr_h,'XData',x(1),'YData',x(2),'ZData',x(3),'Visible','on');
        end;
    elseif(have_dot)
        % pointer reported missing (BX2: NaN pose) or not reported for a while (BX) -> hide
        stale = component_idx==ptr_idx || ~isfield(app.nav_camera,'last_R_t') || numel(app.nav_camera.last_R_t)<ptr_idx ...
            || toc(app.nav_camera.clock0)-app.nav_camera.last_R_t(ptr_idx)>0.1;
        if(stale && logical(app.nav_camera.lm_ptr_h.Visible))
            app.nav_camera.lm_ptr_h.Visible = 'off';
        end;
    end;
end;

return;


function Ri=rigid_inv(R)
% exact inverse of a rigid 4x4 transform
Ri=[R(1:3,1:3).', -R(1:3,1:3).'*R(1:3,4); 0 0 0 1];
