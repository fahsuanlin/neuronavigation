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

set(app.nav_camera.dot_h(component_idx),'markersize',24)
if(isempty(color))
    set(app.nav_camera.dot_h(component_idx),'Color',marker_color{mod(component_idx-1,numel(marker_color))+1});
else
    set(app.nav_camera.dot_h(component_idx),'Color',color);
end;

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

if(isempty(find(isnan(R(:))))&isempty(find(isinf(R(:)))) & component_idx==head_tool_idx)
    if(isfield(app.nav_camera,'etc_render_fsbrain'))
        if(~isempty(app.nav_camera.etc_render_fsbrain))
            try
                %[M,euler]=nav_quat_rot(q);

%                 if(~isfield(app.nav_camera,'R0') || isempty(app.nav_camera.R0))
%                     app.nav_camera.R0 = R;
%                     app.nav_camera.brain_coords = cat(2,etc_render_fsbrain.h.Vertices,ones(size(etc_render_fsbrain.h.Vertices,1),1)).';
% 
%                     % set brain-axes limits once, on the axes that actually holds the brain surface
%                     brain_ax = ancestor(etc_render_fsbrain.h,'axes');
%                     if(~isempty(brain_ax) && isgraphics(brain_ax))
%                         set(brain_ax,'xlim',[-100 100],'ylim',[-100 100],'zlim',[-100 100]);
%                         drawnow;
%                     end;
%                 else
%                     tmp2 = (R / app.nav_camera.R0) * app.nav_camera.brain_coords;   % R*inv(R0)
%                     etc_render_fsbrain.h.Vertices = tmp2(1:3,:).';
%                 end;

                if(~isfield(app.nav_camera,'R0') || isempty(app.nav_camera.R0))
                    app.nav_camera.R0 = R;

                    % centre the brain at its own centroid
                    V0 = double(etc_render_fsbrain.h.Vertices);
                    c  = mean(V0,1).';                                   % 3x1 centroid
                    app.nav_camera.brain_center = c;
                    %app.nav_camera.brain_coords = cat(2, V0 - c.', ones(size(V0,1),1)).';   % 4xN, centred

                    % auto-scale the axes once around the brain, with room for it to move
                    r = max(sqrt(sum((V0 - c.').^2,2)));                 % brain radius (mm)
                    brain_ax = ancestor(etc_render_fsbrain.h,'axes');
                    if(~isempty(brain_ax) && isgraphics(brain_ax))
                        set(brain_ax,'xlim',c(1)+[-2 2]*r,'ylim',c(2)+[-2 2]*r,'zlim',c(3)+[-2 2]*r);
                        %drawnow;
                    end;
                    etc_render_fsbrain.tr.Matrix = eye(4);
                else
                    c = app.nav_camera.brain_center;
                    %D    = app.nav_camera.R0 \ R;                        %
                    %inv(R0)*R: head motion in the head tool's initial
                    %frame; can give warnings.


                    R0 = app.nav_camera.R0;
                    R0inv = [R0(1:3,1:3).', -R0(1:3,1:3).'*R0(1:3,4); 0 0 0 1];   % exact inverse of a rigid transform
                    D = R0inv * R;                                                % bottom row is exactly [0 0 0 1]

                    [U,~,V] = svd(D(1:3,1:3));                                    % snap the rotation to the nearest orthonormal matrix
                    D(1:3,1:3) = U*V.';
                    D(4,:) = [0 0 0 1];


                    etc_render_fsbrain.tr.Matrix = makehgtform('translate',c) * D * makehgtform('translate',-c);
                    %tmp2 = D * app.nav_camera.brain_coords;              % rotate about brain centre, then translate
                    %etc_render_fsbrain.h.Vertices = tmp2(1:3,:).' + app.nav_camera.brain_center.';
                end;


       
            catch ME
                fprintf('graphic update error: %s\n', ME.message);  
            end;
        end;
    end;
end;