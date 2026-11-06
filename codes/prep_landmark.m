close all; clear all;

output_file='fhlin_landmark.mat';

surfin_os='lh.seghead.tri';

electrodes_init={
    'LPA';
    'RPA';
    'Nasion';
    'Nose tip';
     };
 
 verts_electrode_idx_init=[];
 %%%%%%%%%
 
if(exist(output_file))
    fprintf('loading [%s] for  scalp topology rendering...\n',output_file);
    load(output_file);
else
    electrodes=electrodes_init;
    verts_electrode_idx=verts_electrode_idx_init;
end;

%load head model

[tmp,fn,stem]=fileparts(surfin_os);
switch lower(stem)
    case '.tri'
        [verts, faces] = inverse_read_tri(surfin_os);
end;
[nv,nf]=reducepatch(faces,verts,0.1);

%render head model
%etc_render_topo('vol_vertex',verts,'vol_face',faces-1);
e_idx=find(verts_electrode_idx);
etc_render_topo('vol_vertex',verts,'vol_face',faces-1,'topo_aux_point_coords',[verts(verts_electrode_idx(e_idx),1), verts(verts_electrode_idx(e_idx),2), verts(verts_electrode_idx(e_idx),3)],'topo_aux_point_name',electrodes(e_idx));

fprintf('saving [%s] for later scalp topology rendering...\n',output_file);
if(exist(output_file))
    answer = questdlg('over-write variable "verts_electrode_idx"?','existed topology mat file','No');
    if(strcmp(lower(answer),'yes'))
        fprintf('over-writing variable "verts_electrode_idx".\n');
        save(output_file,'faces','verts','verts_electrode_idx','electrodes');
    else
%        fprintf('variable "verts_electrode_idx" NOT over-written.\n');
%        save(output_file,'-append','faces','verts','electrodes');
    end;
else
    fprintf('create [%s] ...\n',output_file);
    save(output_file,'faces','verts','verts_electrode_idx','electrodes');
end;