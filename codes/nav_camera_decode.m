function [str,varargout]=nav_camera_decode(buffer,command,varargin)

varargout{1}=[];

str='';

command_orig=command;
command_split=split(command,' ');
command=command_split{1};
param='';
if(length(command_split)>1)
    param=command_split{2};
end;




switch lower(command) %some commands; not an exhaustive set
    case 'bx'
        try
            buffer_counter=1;

            start_sequence          = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
            reply_length            = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
            header_crc              = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
            number_of_handles       = typecast(uint8(buffer(buffer_counter:buffer_counter+1-1)), 'uint8'); buffer_counter=buffer_counter+1;
            handle_n                = typecast(uint8(buffer(buffer_counter:buffer_counter+1-1)), 'uint8'); buffer_counter=buffer_counter+1;
            handle_status           = typecast(uint8(buffer(buffer_counter:buffer_counter+1-1)), 'uint8'); buffer_counter=buffer_counter+1;

            str{1}=sprintf('BX: [%04d] bytes read; reply length = [%04d] bytes \n',length(buffer), reply_length);

            component=[];
            switch handle_status
                case 1 %valid
                    component.q0 = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;
                    component.qx = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;
                    component.qy = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;
                    component.qz = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;
                    component.tx = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;
                    component.ty = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;
                    component.tz = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;
                    component.error = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;
                    component.port_status = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;
                    component.frame_number = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;

                    str{2}=sprintf('\tstatus 1: q0=%2.2f; qx=%2.2f; qy=%2.2f; qz=%2.2f; tx=%2.2f; ty=%2.2f; tz=%2.2f; error=%2.2f;', component.q0, component.qx, component.qy, component.qz, component.tx, component.ty, component.tz, component.error);
                    
                case 2 %missing
                    component.port_status = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;
                    component.frame_number = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;
                case 4 %disabled

            end;

            system_status           = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;

            varargout{1}=component;

        catch
            varargout{1}=component;

            fprintf('error here BX...\n');
        end;

    case 'bx2'
        try
            buffer_counter=1;

            start_sequence          = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
            reply_length            = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
            header_crc              = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
            gbf_version             = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
            component_count         = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;

            str='';
            str{1}=sprintf('%04d components; [%04d] bytes read; reply length = [%04d] bytes \n',component_count, length(buffer), reply_length);

            component=[];
            switch(dec2hex(start_sequence))
                case 'A5C4' %data...

                    for component_idx=1:component_count
                        component(component_idx).component_type     = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
                        component(component_idx).component_size     = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;
                        component(component_idx).item_format_opion  = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
                        component(component_idx).item_count         = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;

                        component(component_idx).payload            = typecast(uint8(buffer(buffer_counter:buffer_counter+component(component_idx).component_size-10-1)), 'uint8'); buffer_counter=buffer_counter+component(component_idx).component_size-10;
                        str{end+1}=sprintf('\t[%d]:: [%02d, %02d, %02d, %02d]\n',...
                            component_idx, ...
                            component(component_idx).component_type,...
                            component(component_idx).component_size,...
                            component(component_idx).item_format_opion,...
                            component(component_idx).item_count);


                        component_counter=1;

                        for frame_idx=1:component(component_idx).item_count
                            component(component_idx).frame(frame_idx).frame_type        =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+1-1)), 'uint8'); component_counter=component_counter+1;
                            component(component_idx).frame(frame_idx).sequence_index    =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+1-1)), 'uint8'); component_counter=component_counter+1;
                            component(component_idx).frame(frame_idx).frame_status      =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+2-1)), 'uint16'); component_counter=component_counter+2;
                            component(component_idx).frame(frame_idx).frame_number      =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'uint32'); component_counter=component_counter+4;
                            component(component_idx).frame(frame_idx).timestamp_s       =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'uint32'); component_counter=component_counter+4;
                            component(component_idx).frame(frame_idx).timestamp_ns      =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'uint32'); component_counter=component_counter+4;
                            component(component_idx).frame(frame_idx).gbf_version       =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+2-1)), 'uint16'); component_counter=component_counter+2;
                            component(component_idx).frame(frame_idx).component_count   =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+2-1)), 'uint16'); component_counter=component_counter+2;

                            str{end+1}=sprintf('---->> %02d, %02d, %02d, %02d, %02d, %02d, %02d\n',component(component_idx).frame(frame_idx).frame_type,component(component_idx).frame(frame_idx).sequence_index, component(component_idx).frame(frame_idx).frame_number , component(component_idx).frame(frame_idx).timestamp_s, component(component_idx).frame(frame_idx).timestamp_ns, component(component_idx).frame(frame_idx).gbf_version, component(component_idx).frame(frame_idx).component_count  );

                            for data_idx=1:component(component_idx).frame(frame_idx).component_count
                                component(component_idx).frame(frame_idx).data(data_idx).component_type      =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+2-1)), 'uint16'); component_counter=component_counter+2;
                                component(component_idx).frame(frame_idx).data(data_idx).component_size      =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'uint32'); component_counter=component_counter+4;
                                component(component_idx).frame(frame_idx).data(data_idx).item_format_option  =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+2-1)), 'uint16'); component_counter=component_counter+2;
                                component(component_idx).frame(frame_idx).data(data_idx).item_count          =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'uint32'); component_counter=component_counter+4;

                                str{end+1}=sprintf('---------->> %02d, %02d, %02d, %02d\n',component(component_idx).frame(frame_idx).data(data_idx).component_type, component(component_idx).frame(frame_idx).data(data_idx).component_size, component(component_idx).frame(frame_idx).data(data_idx).item_format_option, component(component_idx).frame(frame_idx).data(data_idx).item_count );

                                for data_6d_item_idx=1:component(component_idx).frame(frame_idx).data(data_idx).item_count
                                    try
                                        component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tool_handle  =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+2-1)), 'uint16'); component_counter=component_counter+2;
                                        component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).handle_status=   typecast(uint8(component(component_idx).payload(component_counter:component_counter+2-1)), 'uint16'); component_counter=component_counter+2;
                                        %if(strcmp(dec2hex(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).handle_status,'2000'))
                                        component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).q0           =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'single'); component_counter=component_counter+4;
                                        component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).qx           =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'single'); component_counter=component_counter+4;
                                        component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).qy           =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'single'); component_counter=component_counter+4;
                                        component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).qz           =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'single'); component_counter=component_counter+4;
                                        component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tx           =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'single'); component_counter=component_counter+4;
                                        component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).ty           =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'single'); component_counter=component_counter+4;
                                        component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tz           =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'single'); component_counter=component_counter+4;
                                        component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).error        =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'single'); component_counter=component_counter+4;

                                        str{end+1}=sprintf('---------------------{%02d}>> <<%2.2f, %2.2f, %2.2f>>\n',...
                                            component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tool_handle,...
                                            component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tx,...
                                            component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).ty,...
                                            component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tz);
                                    catch

                                        fprintf('error here 1...\n');
                                        varargout{1}=component;
                                        break;
                                        %                                        keyboard;
                                    end;
                                end;
                            end;
                        end;
                    end;


                    varargout{1}=component;
                otherwise
                    varargout{1}=component;

                    fprintf('error here 2...\n');

            end;

        catch
            varargout{1}=component;

            fprintf('error here 3...\n');
        end;

    case 'phrq' %get new port handel
        try
            buffer_counter=1;

            for idx=1:round(length(buffer)/7)
                port_handel{idx} = buffer((idx-1)*7+1:(idx-1)*7+2);
            end;
            varargout{1}=port_handel;
        catch
        end;


    case 'phsr' %report port handel status
        try

            n_port_handle=str2num(buffer(1:2));
            
            str{1}=sprintf('[%02d] port handle(s)...\n',n_port_handle);
            
            buffer_idx=3;
            for idx=1:n_port_handle
                port_handle(idx).number=str2num(buffer(buffer_idx:buffer_idx+1));
                buffer_idx=buffer_idx+2;

                status_str=buffer(buffer_idx:buffer_idx+2);
                buffer_idx=buffer_idx+3;

                port_handle(idx).status=str2num(status_str);
                port_handle(idx).status_bin=dec2bin(hex2dec(status_str),8);

                port_handle(idx).status_occupied=str2num(port_handle(idx).status_bin(8));
                port_handle(idx).status_switch1closed=str2num(port_handle(idx).status_bin(7));
                port_handle(idx).status_switch2closed=str2num(port_handle(idx).status_bin(6));
                port_handle(idx).status_switch3closed=str2num(port_handle(idx).status_bin(5));
                port_handle(idx).status_initialized=str2num(port_handle(idx).status_bin(4));
                port_handle(idx).status_enabled=str2num(port_handle(idx).status_bin(3));
                port_handle(idx).status_reserved=str2num(port_handle(idx).status_bin(2));
                port_handle(idx).status_tooldetected=str2num(port_handle(idx).status_bin(1));

                %str=strcat(str,sprintf('port handle [%02d]::%04d\t\n',port_handle(idx).number,port_handle(idx).status));
                str{idx}=sprintf('port handle [%02d]::%04d\t\n',port_handle(idx).number,port_handle(idx).status);

                varargout{1}=port_handle;
            end;
        catch
        end;
    case 'phinf'
        try
            port_handle=[];
            if(~isempty(param))
                if(length(param)==6)

                    switch(str2num(param(3:end)))

                        case 1 %0001; tool info
                            port_handle.tool_info.tool_type=buffer(1:8);
                            port_handle.tool_info.manufacturer_id=buffer(9:20);
                            port_handle.tool_info.tool_revision=buffer(21:23);
                            port_handle.tool_info.serial_number=buffer(24:31);
                            port_handle.tool_info.port_status=buffer(32:33);

                            str=sprintf('port handle [%02d]::%s, %s, %s, %s, %s',...
                                str2num(param(1:2)),port_handle.tool_info.tool_type,...
                                port_handle.tool_info.manufacturer_id,...
                                port_handle.tool_info.tool_revision,...
                                port_handle.tool_info.serial_number,...
                                port_handle.tool_info.port_status);
                        case 2 %0002; wired tool electrical info
                            port_handle.wired_tool_electrical_info=buffer(1:8);
                            str=sprintf('port handle [%02d]::%s',str2num(param(1:2)),port_handle.wired_tool_electrical_info);

                        case 4 %0004; tool part number
                            port_handle.tool_part_number=num2str(buffer(1:end-5));
                            str=sprintf('port handle [%02d]::%s',str2num(param(1:2)),num2str(buffer(1:end-5)));

                        case 8 %0008; switch and visible LED info
                            port_handle.switch_and_visible_led_info=buffer(1:2);
                            str=sprintf('port handle [%02d]::%s',str2num(param(1:2)),port_handle.switch_and_visible_led_info);
                        case 10 %0010; tool marker type and wavelength
                            port_handle.tool_marker_type_and_wavelength=buffer(1:2);
                            str=sprintf('port handle [%02d]::%s',str2num(param(1:2)),port_handle.tool_marker_type_and_wavelength);
                        case 20 %0020; physical port location
                            port_handle.physical_port_location.hardware_device=buffer(1:8);
                            port_handle.physical_port_location.system_type=buffer(9);
                            port_handle.physical_port_location.tool_type=buffer(10);
                            port_handle.physical_port_location.port_number=buffer(11:12);
                            port_handle.physical_port_location.reserved=buffer(13:14);
                            str=sprintf('port handle [%02d]::%s, %s, %s, %s, %s',...
                                str2num(param(1:2)),port_handle.physical_port_location.hardware_device,...
                                port_handle.physical_port_location.system_type,...
                                port_handle.physical_port_location.tool_type,...
                                port_handle.physical_port_location.port_number,...
                                port_handle.physical_port_location.reserved);

                        otherwise
                            str=sprintf('port handle [%02d] error param!',str2num(param(1:2)));
                    end;
                else

                end;

                varargout{1}=port_handle;
            end;
        catch
        end;
    otherwise

end;

if(~isempty(str))
    if(iscell(str))
        for idx=1:length(str)
            fprintf('%s\n',str{idx});
        end;
    else
        fprintf('%s\n',str);
    end;
end;

return;

