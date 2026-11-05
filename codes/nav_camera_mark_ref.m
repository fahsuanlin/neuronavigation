function nav_camera_mark_ref(app,action)
%
%   nav_camera_mark_ref     mark the current tool positions as reference origins
%
%   nav_camera_mark_ref(app)                 mark (called by the 'Mark ref' button)
%   nav_camera_mark_ref(app,'update_table')  refresh MarkrefUITable (called by nav_camera_graphic_update)
%   nav_camera_mark_ref(app,'clear')         remove all reference markers and offsets
%
%   Mark: for every tool that is currently visible,
%     - its current position (display coordinates: right, up, distance from
%       camera; mm) becomes that tool's reference origin,
%     - the current dot is frozen in place and turned blue (the reference marker;
%       any previous reference marker of that tool is removed),
%     - a new dot is created to continue live tracking.
%   The offset (dx, dy, dz, |d|) of each tool from its reference is shown in
%   app.MarkrefUITable.
%
% fhlin@Nov 5, 2026
%

if(nargin<2 || isempty(action))
    action='mark';
end;

ref_color=[0 0 1];

switch lower(action)
    case 'mark'
        if(~isfield(app.nav_camera,'last_p') || isempty(app.nav_camera.last_p))
            fprintf('Mark ref: no tool position received yet.\n');
            return;
        end;
        n_tool=size(app.nav_camera.last_p,2);
        if(~isfield(app.nav_camera,'ref_p'))
            app.nav_camera.ref_p=nan(3,0);
        end;
        if(size(app.nav_camera.ref_p,2)<n_tool)
            app.nav_camera.ref_p(:,end+1:n_tool)=nan;
        end;
        if(~isfield(app.nav_camera,'ref_h'))
            app.nav_camera.ref_h=gobjects(1,0);
        end;
        if(numel(app.nav_camera.ref_h)<n_tool)
            app.nav_camera.ref_h(end+1:n_tool)=gobjects(1,n_tool-numel(app.nav_camera.ref_h));
        end;

        n_marked=0;
        for idx=1:n_tool
            p=app.nav_camera.last_p(:,idx);
            if(~all(isfinite(p)))
                continue;   % tool not visible right now: keep its previous reference (if any)
            end;
            if(~isfield(app.nav_camera,'dot_h') || numel(app.nav_camera.dot_h)<idx || ~isgraphics(app.nav_camera.dot_h(idx)))
                continue;
            end;

            % remove this tool's previous reference marker
            if(isgraphics(app.nav_camera.ref_h(idx)))
                delete(app.nav_camera.ref_h(idx));
            end;

            % freeze the current dot as the (blue) reference marker
            live_h=app.nav_camera.dot_h(idx);
            set(live_h,'XData',p(1),'YData',p(2),'ZData',p(3),'Color',ref_color);
            app.nav_camera.ref_h(idx)=live_h;
            app.nav_camera.ref_p(:,idx)=p;

            % new dot for continued tracking (same look as the live dot had)
            app.nav_camera.dot_h(idx)=plot3(app.UIAxes,p(1),p(2),p(3),'.','MarkerSize',24);
            n_marked=n_marked+1;
        end;

        if(n_marked==0)
            fprintf('Mark ref: no visible tool to mark.\n');
        end;
        nav_camera_mark_ref(app,'update_table');

    case 'update_table'
        if(~isprop(app,'MarkrefUITable') || ~isfield(app.nav_camera,'ref_p'))
            return;
        end;
        idx_ref=find(all(isfinite(app.nav_camera.ref_p),1));
        if(isempty(idx_ref))
            app.MarkrefUITable.Data=table();
            return;
        end;
        name=strings(numel(idx_ref),1);
        d=nan(numel(idx_ref),3);
        for k=1:numel(idx_ref)
            idx=idx_ref(k);
            if(isfield(app.nav_camera,'tool') && numel(app.nav_camera.tool)>=idx && isfield(app.nav_camera.tool,'name') && ~isempty(app.nav_camera.tool(idx).name))
                name(k)=string(app.nav_camera.tool(idx).name);
            else
                name(k)=sprintf('Tool %d',idx);
            end;
            if(isfield(app.nav_camera,'last_p') && size(app.nav_camera.last_p,2)>=idx)
                d(k,:)=(app.nav_camera.last_p(:,idx)-app.nav_camera.ref_p(:,idx)).';   % NaN while the tool is missing
            end;
        end;
        d=round(d*10)/10;   % 0.1 mm
        col_name={'tool','dx right (mm)','dy up (mm)','dz distance (mm)','|d| (mm)'};
        app.MarkrefUITable.Data=table(name,d(:,1),d(:,2),d(:,3),round(sqrt(sum(d.^2,2))*10)/10, ...
            'VariableNames',col_name);
        if(~iscell(app.MarkrefUITable.ColumnName) || numel(app.MarkrefUITable.ColumnName)~=numel(col_name))
            app.MarkrefUITable.ColumnName=col_name;   % keep the designer's names if there are 5 of them
        end;

    case 'clear'
        if(isfield(app.nav_camera,'ref_h'))
            delete(app.nav_camera.ref_h(isgraphics(app.nav_camera.ref_h)));
        end;
        app.nav_camera.ref_h=gobjects(1,0);
        app.nav_camera.ref_p=nan(3,0);
        if(isprop(app,'MarkrefUITable'))
            app.MarkrefUITable.Data=table();
        end;

    otherwise
        fprintf('nav_camera_mark_ref: unknown action [%s]\n',action);
end;

return;
