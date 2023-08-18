function [varargout]=nav_camera_graphic_update(app,R,compoent_idx,varargin)

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

marker_color={
    [0 0 1];
    [1 0 0];
    [0 1 0];};


hold(app.UIAxes,"on");
if(isfield(app.nav_camera,'dot_h'))
    if(length(app.nav_camera.dot_h)>=compoent_idx)
        try
            set(app.nav_camera.dot_h(compoent_idx),'XData',R(1,4));
            set(app.nav_camera.dot_h(compoent_idx),'YData',R(2,4));
            set(app.nav_camera.dot_h(compoent_idx),'ZData',R(3,4));
        catch
            app.nav_camera.dot_h(compoent_idx)=plot3(app.UIAxes,R(1,4), R(2,4), R(3,4), '.');
        end;
    else
        app.nav_camera.dot_h(compoent_idx)=plot3(app.UIAxes,R(1,4), R(2,4), R(3,4), '.');
    end;
else
    app.nav_camera.dot_h(compoent_idx)=plot3(app.UIAxes,R(1,4), R(2,4), R(3,4), '.');
end;

set(app.nav_camera.dot_h(compoent_idx),'markersize',24)
if(isempty(color))
    set(app.nav_camera.dot_h(compoent_idx),'Color',marker_color{compoent_idx});
else
    set(app.nav_camera.dot_h(compoent_idx),'Color',color);
end;
drawnow;


if(isempty(find(isnan(R(:))))&isempty(find(isinf(R(:)))))
    if(isfield(app.nav_camera,'etc_render_fsbrain'))
        if(~isempty(app.nav_camera.etc_render_fsbrain))
            try
                %[M,euler]=nav_quat_rot(q);

                if(isfield(app.nav_camera,'R'))
                    if(isempty(app.nav_camera.R))
                        app.nav_camera.R=R;
                    else
                        global etc_render_fsbrain;
                        if(isfield(app.nav_camera,'brain_coords'))
                        else
                            app.nav_camera.brain_coords=cat(2,etc_render_fsbrain.h.Vertices,ones(size(etc_render_fsbrain.h.Vertices,1),1)).';
                        end;
                        tmp2=(R*inv(app.nav_camera.R))*app.nav_camera.brain_coords;

                        double(R)

                        double(app.nav_camera.R)

                        etc_render_fsbrain.h.Vertices=tmp2(1:3,:).';
                        app.nav_camera.R=R;
                        %etc_render_fsbrain_handle('redraw');
                        set(gca,'xlim',[-100 100],'ylim',[-100 100],'zlim',[-100 100]);
%                        axis vis3d equal;
                    end;
                else
                    app.nav_camera.R=R;
                end;

                fprintf('euler=%s\n',mat2str(euler.*180./pi,2));

            catch

            end;
        end;
    end;
end;