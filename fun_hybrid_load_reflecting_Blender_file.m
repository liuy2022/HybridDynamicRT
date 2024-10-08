function [pixel_distance_vector,distan_all_array,amplitude_all] = fun_hybrid_load_reflecting_Blender_file(pixel_width,pixel_height,frame_start,frame_end,num_tx,num_rx,path_load)
%
pixel_intensity_vector = zeros(pixel_width*pixel_height, frame_end);
pixel_distance_vector = zeros(pixel_width*pixel_height, frame_end);

%     pixerl_angle_vector = ones(size(pixel_distance_vector));

distan_all_array = cell(num_tx,num_rx);
amplitude_all = ones(num_tx,num_rx,pixel_width*pixel_width,frame_end);

for frame_index= frame_start: 1:(frame_end-1)
     for index_tx = 1:num_tx
        for index_rx = 1: num_rx
            % amplitude
            raw_data_blender_Amp =  exrread(char(path_load+"Tx" + num2str(1-1) + "Rx" + num2str(1-1) +"/AmplitudeOutput" + num2str(frame_index,'%04d')+".exr")) ;
            % raw_data_blender_Amp = ones(size(raw_data_blender_Amp));

            % distance 
            raw_data_blender_Dist =   exrread(char(path_load+"Tx" + num2str(1-1) + "Rx" + num2str(1-1) +"/DistanceOutput" + num2str((frame_index),'%04d')+".exr"));
%             raw_data_blender_Dist =  hdrread(path_prefix + "Blender scenario/render/Tx" + num2str(1-1) + "Rx" + num2str(1-1) +"/DistanceOutput" + num2str((frame_index),'%04d')+".hdr");
            raw_data_blender_Dist(find(raw_data_blender_Dist>=100|raw_data_blender_Dist<=0))=0;

            pixel_intensity_vector(:, frame_index) = reshape(flip(raw_data_blender_Amp(1:end,1:end,1)),[],1);

            %  pixel_intensity_vector(find(pixel_intensity_vector<60)) = 2;
            pixel_distance_vector(:,frame_index) = reshape(flip(raw_data_blender_Dist(1:end,1:end,1)),[],1);
            distan_all_array{index_tx, index_rx}(:,frame_index) = reshape(flip(raw_data_blender_Dist(1:end,1:end,1)),[],1);
            amplitude_all(index_tx,index_rx,:,frame_index) = reshape(flip(raw_data_blender_Amp(1:end,1:end,1)),[],1);

        end
    end
end

% save 

end