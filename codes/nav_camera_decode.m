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
        component=[];
        try
            % verify length and both CRC16s before parsing anything
            [reply_ok,reply_msg]=check_binary_reply(buffer);
            if(~reply_ok)
                str{1}=sprintf('BX: %s',reply_msg);
                error('nav_camera_decode:badReply','%s',reply_msg);
            end;

            buffer_counter=1;

            start_sequence          = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
            reply_length            = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
            header_crc              = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
            number_of_handles       = typecast(uint8(buffer(buffer_counter:buffer_counter+1-1)), 'uint8'); buffer_counter=buffer_counter+1;

            %str{1}=sprintf('BX: [%04d] bytes read; reply length = [%04d] bytes; [%d] handels; handel [%03d]; status:%d \n',length(buffer), reply_length,number_of_handles,handle_n,handle_status);
            str{1}=sprintf('BX: [%04d] bytes read; reply length = [%04d] bytes; [%d] handels; \n',length(buffer), reply_length,number_of_handles);

            component=[];
            for handle_idx=1:number_of_handles
                component(handle_idx).handle_n                = typecast(uint8(buffer(buffer_counter:buffer_counter+1-1)), 'uint8'); buffer_counter=buffer_counter+1;
                component(handle_idx).handle_status           = typecast(uint8(buffer(buffer_counter:buffer_counter+1-1)), 'uint8'); buffer_counter=buffer_counter+1;

                switch component(handle_idx).handle_status  
                    case 1 %valid
                        component(handle_idx).q0 = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'single'); buffer_counter=buffer_counter+4;
                        component(handle_idx).qx = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'single'); buffer_counter=buffer_counter+4;
                        component(handle_idx).qy = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'single'); buffer_counter=buffer_counter+4;
                        component(handle_idx).qz = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'single'); buffer_counter=buffer_counter+4;
                        component(handle_idx).tx = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'single'); buffer_counter=buffer_counter+4;
                        component(handle_idx).ty = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'single'); buffer_counter=buffer_counter+4;
                        component(handle_idx).tz = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'single'); buffer_counter=buffer_counter+4;
                        component(handle_idx).error = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'single'); buffer_counter=buffer_counter+4;
                        component(handle_idx).port_status = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;
                        component(handle_idx).frame_number = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;

                        str{1+handle_idx}=sprintf('\thandle [%d]::status[%d];  q0=%2.2f; qx=%2.2f; qy=%2.2f; qz=%2.2f; tx=%2.2f; ty=%2.2f; tz=%2.2f; error=%2.2f;', component(handle_idx).handle_n, component(handle_idx).handle_status, component(handle_idx).q0, component(handle_idx).qx, component(handle_idx).qy, component(handle_idx).qz, component(handle_idx).tx, component(handle_idx).ty, component(handle_idx).tz, component(handle_idx).error);
                    case 2 %missing
                        component(handle_idx).port_status = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;
                        component(handle_idx).frame_number = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;
                        
                        component(handle_idx).q0 = nan;
                        component(handle_idx).qx = nan;
                        component(handle_idx).qy = nan;
                        component(handle_idx).qz = nan;
                        component(handle_idx).tx = nan;
                        component(handle_idx).ty = nan;
                        component(handle_idx).tz = nan;
                        component(handle_idx).error = nan;

                        str{1+handle_idx}=sprintf('\thandle [%d]::status[%d]; missing!', component(handle_idx).handle_n, component(handle_idx).handle_status);
                    case 4 %disabled: no transform, port status or frame number follows
                        component(handle_idx).q0 = nan;
                        component(handle_idx).qx = nan;
                        component(handle_idx).qy = nan;
                        component(handle_idx).qz = nan;
                        component(handle_idx).tx = nan;
                        component(handle_idx).ty = nan;
                        component(handle_idx).tz = nan;
                        component(handle_idx).error = nan;

                        str{1+handle_idx}=sprintf('\thandle [%d]::status[%d]; disabled!', component(handle_idx).handle_n, component(handle_idx).handle_status);
                    otherwise %unknown status: the size of what follows is unknown, so stop here
                        component(handle_idx)=[];
                        str{end+1}=sprintf('\thandle [%d]::unknown status [%d]; remaining handles not parsed', double(buffer(buffer_counter-2)), double(buffer(buffer_counter-1)));
                        break;
                end;

            end;

            varargout{1}=component;

        catch ME
            varargout{1}=component;

            if(~strcmp(ME.identifier,'nav_camera_decode:badReply'))
                fprintf('error here BX: %s\n',ME.message);
            end;
        end;

    case 'bx2'
        component=[];
        try
            buffer_counter=1;

            start_sequence          = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
            switch(dec2hex(start_sequence))
                case 'A5C4' %data...
                    % verify length and both CRC16s before parsing anything
                    [reply_ok,reply_msg]=check_binary_reply(buffer);
                    if(~reply_ok)
                        str{1}=sprintf('BX2: %s',reply_msg);
                        varargout{1}=[];
                        return;
                    end;

                    reply_length            = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
                    header_crc              = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
                    gbf_version             = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
                    component_count         = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;

                    str='';
                    str{1}=sprintf('%04d components; [%04d] bytes read; reply length = [%04d] bytes \n',component_count, length(buffer), reply_length);


                    for component_idx=1:component_count
                        component(component_idx).component_type     = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
                        component(component_idx).component_size     = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;
                        component(component_idx).item_format_opion  = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
                        component(component_idx).item_count         = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;

                        %component(component_idx).payload            = typecast(uint8(buffer(buffer_counter:buffer_counter+component(component_idx).component_size-10-1)), 'uint8'); buffer_counter=buffer_counter+component(component_idx).component_size-10;
                        payload_len = double(component(component_idx).component_size) - 12;
                        component(component_idx).payload            = uint8(buffer(buffer_counter:buffer_counter+payload_len-1)); buffer_counter=buffer_counter+payload_len;

                        str{end+1}=sprintf('\t[%d]:: [%02d, %02d, %02d, %02d]',...
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

                            str{end+1}=sprintf('---->> %02d, %02d, %02d, %02d, %02d, %02d, %02d',component(component_idx).frame(frame_idx).frame_type,component(component_idx).frame(frame_idx).sequence_index, component(component_idx).frame(frame_idx).frame_number , component(component_idx).frame(frame_idx).timestamp_s, component(component_idx).frame(frame_idx).timestamp_ns, component(component_idx).frame(frame_idx).gbf_version, component(component_idx).frame(frame_idx).component_count  );

                            for data_idx=1:component(component_idx).frame(frame_idx).component_count
                                data_start = component_counter;          % <-- add: position of this component's header
                                
                                component(component_idx).frame(frame_idx).data(data_idx).component_type      =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+2-1)), 'uint16'); component_counter=component_counter+2;
                                component(component_idx).frame(frame_idx).data(data_idx).component_size      =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'uint32'); component_counter=component_counter+4;
                                component(component_idx).frame(frame_idx).data(data_idx).item_format_option  =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+2-1)), 'uint16'); component_counter=component_counter+2;
                                component(component_idx).frame(frame_idx).data(data_idx).item_count          =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'uint32'); component_counter=component_counter+4;

                                str{end+1}=sprintf('---------->> %02d, %02d, %02d, %02d',component(component_idx).frame(frame_idx).data(data_idx).component_type, component(component_idx).frame(frame_idx).data(data_idx).component_size, component(component_idx).frame(frame_idx).data(data_idx).item_format_option, component(component_idx).frame(frame_idx).data(data_idx).item_count );

                                if(component(component_idx).frame(frame_idx).data(data_idx).component_type==2)
                                    for data_6d_item_idx=1:component(component_idx).frame(frame_idx).data(data_idx).item_count %ony parsing  6D component
                                        try
                                            component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tool_handle  =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+2-1)), 'uint16'); component_counter=component_counter+2;
                                            component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).handle_status1=   typecast(uint8(component(component_idx).payload(component_counter:component_counter+1-1)), 'uint8'); component_counter=component_counter+1;
                                            component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).handle_status2=   typecast(uint8(component(component_idx).payload(component_counter:component_counter+1-1)), 'uint8'); component_counter=component_counter+1;
                                            
                                            item_missing = bitand(component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).handle_status2, uint8(1)) ~= 0;
                                            if(~item_missing)
                                            %if(strcmp(dec2hex(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).handle_status,'2000'))
                                            %if(component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).handle_status1==0&component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).handle_status2==0)
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).q0           =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'single'); component_counter=component_counter+4;
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).qx           =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'single'); component_counter=component_counter+4;
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).qy           =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'single'); component_counter=component_counter+4;
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).qz           =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'single'); component_counter=component_counter+4;
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tx           =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'single'); component_counter=component_counter+4;
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).ty           =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'single'); component_counter=component_counter+4;
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tz           =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'single'); component_counter=component_counter+4;
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).error        =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'single'); component_counter=component_counter+4;

                                                str{end+1}=sprintf('---------------------{%02d}>> [%02d|%s] <<%2.2f, %2.2f, %2.2f>>',...
                                                    component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tool_handle,...
                                                    component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).handle_status1,...
                                                    dec2bin(component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).handle_status2,8),...
                                                    component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tx,...
                                                    component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).ty,...
                                                    component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tz);
                                            else
                                                % missing: no transform bytes were sent; fill NaN so every item has the same fields
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).q0=nan;
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).qx=nan;
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).qy=nan;
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).qz=nan;
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tx=nan;
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).ty=nan;
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tz=nan;
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).error=nan;
                                                str{end+1}=sprintf('---------------------{%02d}>> missing', component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tool_handle);
                                            end;
                                        catch

                                            fprintf('6d data reading error...\n');
                                            varargout{1}=component;


                                            if(~isempty(str))
                                                if(iscell(str))
                                                    for idx=1:length(str)
                                                        fprintf('%s\n',str{idx});
                                                    end;
                                                else
                                                    fprintf('%s\n',str);
                                                end;
                                            end;

                                            fprintf('end of 6d data reading error...\n');

                                            break;
                                        end;
                                    end;
                                end;
                                
                                component_counter = data_start + double(component(component_idx).frame(frame_idx).data(data_idx).component_size);
                            end;
                        end;
                    end;


                    varargout{1}=component;

                case 'A5C8' %extended data: start(2) + reply length(4) + header CRC(2); payload not parsed yet
                    reply_length            = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;
                    header_crc              = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
                    gbf_version             = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
                    component_count         = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;

                    str='';
                    str{1}=sprintf('extended reply (A5C8), not parsed: %04d components; [%04d] bytes read; reply length = [%04d] bytes \n',component_count, length(buffer), reply_length);
                    varargout{1}=[];

                otherwise
                    varargout{1}=component;

                    fprintf('error  2...\n');
                    str{1}=sprintf('unknown start sequence [%s]',dec2hex(start_sequence));
                    if(~isempty(str))
                        if(iscell(str))
                            for idx=1:length(str)
                                fprintf('%s\n',str{idx});
                            end;
                        else
                            fprintf('%s\n',str);
                        end;
                    end;

                    fprintf('end of error 2...\n');
            end;

        catch
            varargout{1}=component;

            fprintf('error  3...\n');
            if(~isempty(str))
                if(iscell(str))
                    for idx=1:length(str)
                        fprintf('%s\n',str{idx});
                    end;
                else
                    fprintf('%s\n',str);
                end;
            end;
            fprintf('end of error 3...\n');


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
            % reply (all hex ASCII): <n port handles:2> { <handle:2><status:3> } x n <CRC16:4><CR>
            buffer=char(buffer(:).');
            if(strncmpi(buffer,'ERROR',5))
                str{1}=sprintf('PHSR error reply: %s',strtrim(buffer));
                error('nav_camera_decode:phsrError','%s',str{1});
            end;

            n_port_handle=hex2dec(buffer(1:2));

            str{1}=sprintf('[%02d] port handle(s)...\n',n_port_handle);

            port_handle=[];
            buffer_idx=3;
            for idx=1:n_port_handle
                port_handle(idx).number=hex2dec(buffer(buffer_idx:buffer_idx+1));
                buffer_idx=buffer_idx+2;

                status_str=buffer(buffer_idx:buffer_idx+2);
                buffer_idx=buffer_idx+3;

                % 12-bit status (API guide, PHSR): bit0 occupied, bit1-3 switch 1-3 closed,
                % bit4 initialized, bit5 enabled, bit6 reserved, bit7 tool detected, bit8-11 reserved
                status=hex2dec(status_str);
                port_handle(idx).status=status;
                port_handle(idx).status_bin=dec2bin(status,12);

                port_handle(idx).status_occupied=bitget(status,1);
                port_handle(idx).status_switch1closed=bitget(status,2);
                port_handle(idx).status_switch2closed=bitget(status,3);
                port_handle(idx).status_switch3closed=bitget(status,4);
                port_handle(idx).status_initialized=bitget(status,5);
                port_handle(idx).status_enabled=bitget(status,6);
                port_handle(idx).status_reserved=bitget(status,7);
                port_handle(idx).status_tooldetected=bitget(status,8);

                str{end+1}=sprintf('port handle [%02X]::%03X\toccupied:%d\tinitialized:%d\tenabled:%d\tdetected:%d\t\n',port_handle(idx).number,port_handle(idx).status,...
                    port_handle(idx).status_occupied,...
                    port_handle(idx).status_initialized,...
                    port_handle(idx).status_enabled,...
                    port_handle(idx).status_tooldetected);

            end;
            varargout{1}=port_handle;
        catch ME
            if(~strcmp(ME.identifier,'nav_camera_decode:phsrError'))
                fprintf('error in PHSR decoding: %s\n',ME.message);
            end;
        end;

    case 'pena' %report port handel status
        try

            buffer
        catch
        end;

    case 'phinf'
        try
            port_handle=[];
            if(~isempty(param))
                if(length(param)==6)
                %if(1)
                    switch(hex2dec(param(3:end)))   % reply option is hex: 0001,0002,0004,0008,0010,0020

                        case 1 %0001; tool info
                            port_handle.tool_info.tool_type=buffer(1:8);
                            port_handle.tool_info.manufacturer_id=buffer(9:20);
                            port_handle.tool_info.tool_revision=buffer(21:23);
                            port_handle.tool_info.serial_number=buffer(24:31);
                            port_handle.tool_info.port_status=buffer(32:33);

                            str=sprintf('port handle [%02X]::%s, %s, %s, %s, %s',...
                                hex2dec(param(1:2)),port_handle.tool_info.tool_type,...
                                port_handle.tool_info.manufacturer_id,...
                                port_handle.tool_info.tool_revision,...
                                port_handle.tool_info.serial_number,...
                                port_handle.tool_info.port_status);
                        case 2 %0002; wired tool electrical info
                            port_handle.wired_tool_electrical_info=buffer(1:8);
                            str=sprintf('port handle [%02X]::%s',hex2dec(param(1:2)),port_handle.wired_tool_electrical_info);

                        case 4 %0004; tool part number
                            port_handle.tool_part_number=num2str(buffer(1:end-5));
                            str=sprintf('port handle [%02X]::%s',hex2dec(param(1:2)),num2str(buffer(1:end-5)));

                        case 8 %0008; switch and visible LED info
                            port_handle.switch_and_visible_led_info=buffer(1:2);
                            str=sprintf('port handle [%02X]::%s',hex2dec(param(1:2)),port_handle.switch_and_visible_led_info);
                        case 16 %0010; tool marker type and wavelength
                            port_handle.tool_marker_type_and_wavelength=buffer(1:2);
                            str=sprintf('port handle [%02X]::%s',hex2dec(param(1:2)),port_handle.tool_marker_type_and_wavelength);
                        case 32 %0020; physical port location
                            port_handle.physical_port_location.hardware_device=buffer(1:8);
                            port_handle.physical_port_location.system_type=buffer(9);
                            port_handle.physical_port_location.tool_type=buffer(10);
                            port_handle.physical_port_location.port_number=buffer(11:12);
                            port_handle.physical_port_location.reserved=buffer(13:14);
                            str=sprintf('port handle [%02X]::%s, %s, %s, %s, %s',...
                                hex2dec(param(1:2)),port_handle.physical_port_location.hardware_device,...
                                port_handle.physical_port_location.system_type,...
                                port_handle.physical_port_location.tool_type,...
                                port_handle.physical_port_location.port_number,...
                                port_handle.physical_port_location.reserved);

                        otherwise
                            str=sprintf('port handle [%02X] error param!',hex2dec(param(1:2)));
                    end;
                else

                end;

                varargout{1}=port_handle;
            end;
        catch
        end;
    otherwise

end;

flag_verbose=0; %showing decoded info for EVERY frame
if(~isempty(str)&& flag_verbose)
    if(iscell(str))
        for idx=1:length(str)
            fprintf('%s\n',str{idx});
        end;
    else
        fprintf('%s\n',str);
    end;
end;

return;


function [ok,msg]=check_binary_reply(buffer)
% Validate a binary (A5C4) BX/BX2 reply:
%   start(2) | reply length(2) | header CRC16(2) | data(reply length) | data CRC16(2)
% Both CRCs use NDI's CRC16 (see doc/CombinedAPISample/library/src/SystemCRC.cpp).
ok=false;
bytes=uint8(buffer(:).');
if(length(bytes)<6)
    msg=sprintf('reply too short (%d bytes)',length(bytes));
    return;
end;
reply_length=double(typecast(bytes(3:4),'uint16'));
if(length(bytes)<6+reply_length+2)
    msg=sprintf('incomplete reply (%d of %d bytes)',length(bytes),6+reply_length+2);
    return;
end;
if(nav_crc16(bytes(1:4))~=typecast(bytes(5:6),'uint16'))
    msg='header CRC16 mismatch';
    return;
end;
if(nav_crc16(bytes(7:6+reply_length))~=typecast(bytes(7+reply_length:8+reply_length),'uint16'))
    msg='data CRC16 mismatch';
    return;
end;
ok=true;
msg='';
return;


function crc=nav_crc16(bytes)
% NDI CRC16: reflected polynomial 0xA001, initial value 0
persistent crc_table;
if(isempty(crc_table))
    crc_table=zeros(1,256,'uint16');
    for i=0:255
        c=uint32(i);
        for j=1:8
            if(bitand(c,1))
                c=bitxor(bitshift(c,-1),uint32(40961));   % 0xA001
            else
                c=bitshift(c,-1);
            end;
        end;
        crc_table(i+1)=uint16(c);
    end;
end;
crc=uint16(0);
for m=1:length(bytes)
    crc=bitxor(crc_table(double(bitxor(bitand(crc,uint16(255)),uint16(bytes(m))))+1),bitshift(crc,-8));
end;
return;

