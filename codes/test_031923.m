close all; clear all;
figure;
plot3(0,0,0,'k.');
set(gca,'xlim',[-600 600],'ylim',[-600 600],'zlim',[-2000 2000]);
axis vis3d;
hold on;
pause(0.1);

%fixed TCP and port
try
    tCmd=tcpip('P9-10219.local',8765);

    fopen(tCmd);
catch
    fprintf('error in connecting to the camera!\n');
    return;
end;

flag_cont=1;
counter=1;
hh=[];

try
    tic;
    while(flag_cont)
        fprintf(tCmd,'BX2'); %read tool info (by defaults)

        if(tCmd.BytesAvailable>0)
            buffer_counter=1;
            buffer=[];
            buffer=fscanf(tCmd,'%c',tCmd.BytesAvailable);
           
            try
                start_sequence          = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
                reply_length            = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
                header_crc              = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
                gbf_version             = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
                component_count         = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;

                fprintf('-----------------------------------------------------------------------------------------------------------------------------------------\n');
                fprintf('%04d components; [%04d] bytes read; reply length = [%04d] bytes \n',component_count, length(buffer), reply_length);
                component=[];
                switch(dec2hex(start_sequence))
                    case 'A5C4' %data...

                        for component_idx=1:component_count
                            component(component_idx).component_type     = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
                            component(component_idx).component_size     = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;
                            component(component_idx).item_format_opion  = typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;
                            component(component_idx).item_count         = typecast(uint8(buffer(buffer_counter:buffer_counter+4-1)), 'uint32'); buffer_counter=buffer_counter+4;

                            component(component_idx).payload            = typecast(uint8(buffer(buffer_counter:buffer_counter+component(component_idx).component_size-10-1)), 'uint8'); buffer_counter=buffer_counter+component(component_idx).component_size-10;
                            fprintf('\t[%d]:: [%02d, %02d, %02d, %02d]\n',...
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
                                    
                                fprintf('---->> %02d, %02d, %02d, %02d, %02d, %02d, %02d\n',component(component_idx).frame(frame_idx).frame_type,component(component_idx).frame(frame_idx).sequence_index, component(component_idx).frame(frame_idx).frame_number , component(component_idx).frame(frame_idx).timestamp_s, component(component_idx).frame(frame_idx).timestamp_ns, component(component_idx).frame(frame_idx).gbf_version, component(component_idx).frame(frame_idx).component_count  );

                                for data_idx=1:component(component_idx).frame(frame_idx).component_count
                                    component(component_idx).frame(frame_idx).data(data_idx).component_type      =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+2-1)), 'uint16'); component_counter=component_counter+2;
                                    component(component_idx).frame(frame_idx).data(data_idx).component_size      =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'uint32'); component_counter=component_counter+4;
                                    component(component_idx).frame(frame_idx).data(data_idx).item_format_option  =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+2-1)), 'uint16'); component_counter=component_counter+2;
                                    component(component_idx).frame(frame_idx).data(data_idx).item_count          =   typecast(uint8(component(component_idx).payload(component_counter:component_counter+4-1)), 'uint32'); component_counter=component_counter+4;

                                    fprintf('---------->> %02d, %02d, %02d, %02d\n',component(component_idx).frame(frame_idx).data(data_idx).component_type, component(component_idx).frame(frame_idx).data(data_idx).component_size, component(component_idx).frame(frame_idx).data(data_idx).item_format_option, component(component_idx).frame(frame_idx).data(data_idx).item_count );

                                    for data_6d_item_idx=1:component(component_idx).frame(frame_idx).data(data_idx).item_count
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

                                            fprintf('---------------------{%02d}>> <<%2.2f, %2.2f, %2.2f>>\n',...
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tool_handle,...
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tx,...
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).ty,...
                                                component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tz);


                                            tt=toc;
                                            if(tt>0.5)
                                                tic;
                                                if(~isempty(hh))
                                                    set(hh,'color',[1 1 1].*.9);
                                                    hh=plot3(...
                                                        component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tx, ...
                                                        component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).ty, ...
                                                        component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tz, 'b.');
                                                    drawnow;
                                                else
                                                    hh=plot3(...
                                                        component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tx, ...
                                                        component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).ty, ...
                                                        component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tz, 'b.');
                                                    drawnow;
                                                end;
                                            end;
                                        %end;
                                    end;
                                end;
                           end;
                        end;

                    otherwise

                end;

                %CRC16
                %typecast(uint8(buffer(buffer_counter:buffer_counter+2-1)), 'uint16'); buffer_counter=buffer_counter+2;

            catch
                fprintf('error in buffer reading!\n');
                
            end;



            counter=counter+1;

        end;
    end;
catch
    flag_cont=0;
    fprintf('error occurred!\n');
    fclose(tCmd);
    return;
end;