close all; clear all;
figure;
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
    while(flag_cont)
        tic;
        fprintf(tCmd,'BX2'); %read tool info (by defaults)
        
        if(tCmd.BytesAvailable>0)

            %typecast((uint32(hex2dec('BD37F076'))),'single')
        
            %ret=fscanf(tCmd,'%c',tCmd.BytesAvailable);


            start_sequence  = swapbytes(uint16(fread(tCmd, 1, 'uint16')));
            reply_length    = swapbytes(uint16(fread(tCmd, 1, 'uint16')));
            header_CRC      = swapbytes(uint16(fread(tCmd, 1, 'uint16')));

            gbf_version     = swapbytes(uint16(fread(tCmd, 1, 'uint16')));
            component_count = swapbytes(uint16(fread(tCmd, 1, 'uint16')));

            fprintf('%04d components\n',component_count);
            component=[];
            switch(dec2hex(start_sequence))
                case 'A5C4' %data...

                    for component_idx=1:component_count
                        try
                            component(component_idx).component_type     = swapbytes(uint16(fread(tCmd, 1, 'uint16')));
                            component(component_idx).component_size     = swapbytes(uint32(fread(tCmd, 1, 'uint32')));
                            component(component_idx).item_format_opion  = swapbytes(uint16(fread(tCmd, 1, 'uint16')));
                            component(component_idx).item_count         = swapbytes(uint16(fread(tCmd, 1, 'uint16')));

                            for frame_idx=1:component(component_idx).item_count
                                component(component_idx).frame(frame_idx).frame_type        =   swapbytes(uint8(fread(tCmd, 1, 'uint8')));
                                component(component_idx).frame(frame_idx).sequence_index    =   swapbytes(uint8(fread(tCmd, 1, 'uint8')));
                                component(component_idx).frame(frame_idx).frame_status      =   swapbytes(uint16(fread(tCmd, 1, 'uint16')));
                                component(component_idx).frame(frame_idx).frame_number      =   swapbytes(uint32(fread(tCmd, 1, 'uint32')));
                                component(component_idx).frame(frame_idx).timestamp_s       =   swapbytes(uint32(fread(tCmd, 1, 'uint32')));
                                component(component_idx).frame(frame_idx).timestamp_ns      =   swapbytes(uint32(fread(tCmd, 1, 'uint32')));
                                component(component_idx).frame(frame_idx).gbf_version       =   swapbytes(uint16(fread(tCmd, 1, 'uint16')));
                                component(component_idx).frame(frame_idx).component_count   =   swapbytes(uint16(fread(tCmd, 1, 'uint16')));

                                fprintf('---->> %02d, %02d, %04d\n',component(component_idx).frame(frame_idx).frame_type,component(component_idx).frame(frame_idx).sequence_index, component(component_idx).frame(frame_idx).component_count  );

                                for data_idx=1:component(component_idx).frame(frame_idx).component_count
                                    component(component_idx).frame(frame_idx).data(data_idx).component_type      =   swapbytes(uint16(fread(tCmd, 1, 'uint16')));
                                    component(component_idx).frame(frame_idx).data(data_idx).component_size      =   swapbytes(uint32(fread(tCmd, 1, 'uint32')));
                                    component(component_idx).frame(frame_idx).data(data_idx).item_format_option  =   swapbytes(uint16(fread(tCmd, 1, 'uint16')));
                                    component(component_idx).frame(frame_idx).data(data_idx).item_count          =   swapbytes(uint32(fread(tCmd, 1, 'uint32')));

                                    fprintf('-------->> %02d\n',component(component_idx).frame(frame_idx).data(data_idx).item_count );

                                    for data_6d_item_idx=1:component(component_idx).frame(frame_idx).data(data_idx).item_count
                                        component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tool_handle  =   swapbytes(uint16(fread(tCmd, 1, 'uint16')));
                                        component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).handle_status=   swapbytes(uint16(fread(tCmd, 1, 'uint16')));
                                        if(strcmp(dec2hex(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).handle_status,'2000'))
                                            component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).q0           =   swapbytes(single(fread(tCmd, 1, 'single')));
                                            component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).qx           =   swapbytes(single(fread(tCmd, 1, 'single')));
                                            component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).qy           =   swapbytes(single(fread(tCmd, 1, 'single')));
                                            component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).qz           =   swapbytes(single(fread(tCmd, 1, 'single')));
                                            component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tx           =   swapbytes(single(fread(tCmd, 1, 'single')));
                                            component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).ty           =   swapbytes(single(fread(tCmd, 1, 'single')));
                                            component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tz           =   swapbytes(single(fread(tCmd, 1, 'single')));

                                            if(~iempty(hh))
                                                delete(hh);
                                                hh=plot3(component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tx ,component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).ty, component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).tz, 'g.');
                                                drawnow;
                                            end;


                                            component(component_idx).frame(frame_idx).data(data_idx).data_6d_item(data_6d_item_idx).error           =   swapbytes(single(fread(tCmd, 1, 'single')));
                                        end;
                                    end;
                                end;
                                %tCmd.BytesAvailable
                                if(tCmd.BytesAvailable>0)
                                    %ret=fscanf(tCmd,'%c',tCmd.BytesAvailable);
                                end;

                            end;

                        catch
                            if(tCmd.BytesAvailable>0)
                                ret=fscanf(tCmd,'%c',tCmd.BytesAvailable);
                            end;

                            fprintf('frame error!\n');
                            break;
                        end;
                    end;

                otherwise

            end;

            %CRC16
            swapbytes(uint16(fread(tCmd, 1, 'uint16')));




            try
                if(tCmd.BytesAvailable>0)
                    ret=fscanf(tCmd,'%c',tCmd.BytesAvailable);
                end;
            catch
                fprintf('error in reading data...\n');
                break;
            end;

            lapsed_time=toc;

            fprintf('[%05d]: [%03d byets received (%2.2f s) <%s,%d>',counter,length(ret),lapsed_time, dec2hex(start_sequence), component_count);
            if(strcmp(dec2hex(start_sequence),'A5C4'))
                 for component_idx=1:component_count
                     fprintf(':<%02d> ',component(component_idx).component_size);
%                     for frame_idx=1:component(component_idx).item_count
%                         component(component_idx).frame(frame_idx)
%                         fprintf(':[%d:%d:%d] ',component(component_idx).frame(frame_idx).timestamp_s,component(component_idx).frame(frame_idx).timestamp_ns,component(component_idx).frame(frame_idx).component_count );
%                     end;
                 end;
            end;
            fprintf('\n');
             
            counter=counter+1;

        end;
    end;
catch
    flag_cont=0;
    fprintf('error occurred!\n');
    fclose(tCmd);
    return;
end;