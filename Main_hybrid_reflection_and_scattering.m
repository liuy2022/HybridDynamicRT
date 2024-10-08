%% This file is for joint scattering and reflection radar simulations
% close all
clear
clc

%% Settings

settings.consider_antenna_pattern = 0;
settings.Dis_THR =0.6;
settings.load_existing_blender_frames = 0;
settings.reflecting_coefficient = 0.6;
settings.scattering_coefficient = 0.2;
settings.range_bin_length = 10;
settings.scenario_index = 4; % 1 for walking; 2 for stepping; 3 for walking without arm-swing; 4 for walking in circle; 5 for upstairs
settings.scenario_consider_reflector = 1;
settings.simu_reflecting_paths = 1;
settings.simu_scattering_paths = 0;

settings.show_figure_60 = 0; % all 
settings.show_figure_50 = 0; % only scattering paths
settings.show_figure_40 = 1; % show target and reflected target 

% %
% if settings.scenario_consider_reflector
%     settings.simu_reflecting_paths = 1;
% else
%     settings.simu_reflecting_paths = 0;
% end
%% Blender data frame settings
% this is based on the number of frames setting in Blender
if settings.scenario_index == 1
    frame_start = 1;
    frame_end =130 - 1; % 52 - frame_step_for_velo; 97;
    camera_ang =  75; % According to the FoV setting in Blender; in degree
    pixel_width     = 500; % This can be rectangular; not limited to square;
    pixel_height    = 500;
    camera_ang_rotate = [0 0]; %azmuth and elevation
    path_prefix = "";
    if settings.scenario_consider_reflector
        path_load = path_prefix + "Blender scenario/render_for_different_walking_motion/scenario_1_with_reflectors/";
    else
        path_load = path_prefix + "Blender scenario/render_for_different_walking_motion/scenario_1_without_reflectors/";
    end
elseif settings.scenario_index == 2
    frame_start = 80;
    frame_end =210-1; % 52 - frame_step_for_velo; 97;
    camera_ang =  75; % According to the FoV setting in Blender; in degree
    pixel_width     = 500; % This can be rectangular; not limited to square;
    pixel_height    = 500;
    camera_ang_rotate = [0 0]; %azmuth and elevation
    path_prefix = "";
    if settings.scenario_consider_reflector
        path_load = path_prefix + "Blender scenario/render_for_different_walking_motion/scenario_2_with_reflectors/";
    else
        path_load = path_prefix + "Blender scenario/render_for_different_walking_motion/scenario_2_without_reflectors/";
    end
elseif settings.scenario_index == 3
    frame_start = 40;
    frame_end =170-1; % 52 - frame_step_for_velo; 97;
    camera_ang =  75; % According to the FoV setting in Blender; in degree
    pixel_width     = 500; % This can be rectangular; not limited to square;
    pixel_height    = 500;
    camera_ang_rotate = [0 0]; %azmuth and elevation
    path_prefix = "";
    if settings.scenario_consider_reflector
        path_load = path_prefix + "Blender scenario/render_for_different_walking_motion/scenario_3_with_reflectors/";
    else
        path_load = path_prefix + "Blender scenario/render_for_different_walking_motion/scenario_3_without_reflectors/";
    end
elseif settings.scenario_index == 4
    frame_start = 1;
    frame_end =215-1; % 52 - frame_step_for_velo; 97;
    camera_ang =  90; % According to the FoV setting in Blender; in degree
    pixel_width     = 500; % This can be rectangular; not limited to square;
    pixel_height    = 500;
    camera_ang_rotate = [0 0]; %azmuth and elevation
    path_prefix = "";
    if settings.scenario_consider_reflector
        path_load = path_prefix + "Blender scenario/render_for_different_walking_motion/scenario_4_with_reflectors/";
    else
        path_load = path_prefix + "Blender scenario/render_for_different_walking_motion/scenario_4_without_reflectors/";
    end
elseif settings.scenario_index == 5
    frame_start = 70;
    frame_end =220 - 1 ; % 52 - frame_step_for_velo; 97;
    camera_ang =  90; % According to the FoV setting in Blender; in degree
    pixel_width     = 500; % This can be rectangular; not limited to square;
    pixel_height    = 500;
    camera_ang_rotate = [0 0]; %azmuth and elevation
    path_prefix = "";
    if settings.scenario_consider_reflector
        path_load = path_prefix + "Blender scenario/render_for_different_walking_motion/scenario_5_with_reflectors/";
    else
        path_load = path_prefix + "Blender scenario/render_for_different_walking_motion/scenario_5_without_reflectors/";
    end
else
    frame_start = 50;
    frame_end =54; % 52 - frame_step_for_velo; 97;
    camera_ang =  90; % According to the FoV setting in Blender; in degree
    pixel_width     = 500; % This can be rectangular; not limited to square;
    pixel_height    = 500;
    camera_ang_rotate = [0 0]; %azmuth and elevation
    path_prefix = "";
    path_load = path_prefix + "Blender scenario/render/";
end

R_frame = 30;
num_frame = frame_end -frame_start;

% choose the frame-interval that used for calculating velocity
frame_step_for_velo = 1;
slow_time_delta_t = 1/R_frame; 1.0;  % we can design the speed;


%
[pixel_angle,angle_vec,angle_view_cali] = fun_hybrid_pixel_angle(camera_ang, camera_ang_rotate, pixel_width, pixel_height);
% test
if 0
    figure
    mesh(pixel_angle)
end

%% import the array configurations
% define the antenna configuration in radar_parameters_hybrid.json
% Tx
hybrid_read_parameters;
values = fieldnames(params.antenna_offsets_tx);
num_tx = numel(values);
% Rx
values = fieldnames(params.antenna_offsets_rx);
num_rx = numel(values);
% the followings are used for calculation of pixerl localtions and angles
camera_loc =[0 0 1.2]; % could be MIMO
for index_tx = 1:num_tx
    Antenna.loc_tx(index_tx,:) = camera_loc + eval(['params.antenna_offsets_tx.Tx' num2str(index_tx-1)]).';
end
for index_rx = 1: num_rx
    Antenna.loc_rx(index_rx,:) = camera_loc + eval(['params.antenna_offsets_rx.Rx' num2str(index_rx-1)]).';
end

% antenna parttern
if settings.consider_antenna_pattern
    % load('antenna_pattern_TI1648.mat')

else

end

%% Chirp data: FMCW settings
f0              = 77.00e+9;     % Radar center frequency (Hz)
Tr              = 6e-5;       % Chirp duration (s)
BW = 3.6e9;  % BW = 1.8e9;
Np = 128;  % number of chirps
Ns = 256; % number of frequency samples
Fs              = 10e+6;        % Radar sampling rate (Hz)
PRF             = Np/(slow_time_delta_t);  % PRF = 25000;         % Pulse Reputation Frequency (Hz)
PRT =  1/PRF;
Naz = length([frame_start:1:frame_end]);
c               = 2.9979e+8;    % Speed of light (m/s)
lambda          = c/f0;
% pre-computation
% numAnt = size(camera.loc,1);
t_fast=(0:(Ns-1)) * 1/Fs;
t_slow=(0:(Np-1)) * PRT;
rang_max = c*Fs/(2*BW/Tr);
range_sequency = (0: (Ns - 1))*rang_max/Ns;
% t_fast = range_sequency/c*2;
% radar signal generation

%% Loads Blender data
if settings.load_existing_blender_frames
    % load exsisting Blender files
    if settings.simu_scattering_paths
        %load scattering paths data
    else
    end
    if settings.simu_reflecting_paths
        %load reflection paths data
    else
    end

else  % load a new Blender data

    % add exrread to path
    addpath('openexr-matlab-master')
    digits(8)
    frm =5;

    if settings.simu_reflecting_paths
        %load reflection paths data
        %         path_prefix = "";
        %         path_load = path_prefix + "Blender scenario/render/";
        % path_save;

        [pixel_distance_vector_reflection,distan_all_array_reflection,amplitude_all_reflection] = fun_hybrid_load_reflecting_Blender_file(pixel_width,pixel_height,frame_start,frame_end,num_tx,num_rx,path_load);
        pixel_radial_velocity = ones(size(pixel_distance_vector_reflection)); % 
        % save the reflection data in path_save file
    else
    end

    if settings.simu_scattering_paths
        % load scattering paths
        [pixel_distance_vector_scattering,distan_all_array_scattering,amplitude_all_scattering] = fun_hybrid_load_scattering_Blender_file(pixel_width,pixel_height,frame_start,frame_end,num_tx,num_rx,path_load);
        pixel_radial_velocity = ones(size(pixel_distance_vector_scattering)); % 
    else
    end


end

%% Generate signal
% concatenated_fx_Ang = [];
concatenated_fx_Dop_mean_scattering = [];
concatenated_fx_Dop_spread_scattering = [];
concatenated_fx_Dop_max_scattering = [];
concatenated_fx_Dop_Ave_scattering =[];
concatenated_fx_Ang_scattering = [];
concatenated_fx_Range_scattering = [];

%
concatenated_fx_Dop_max_reflection = [];
concatenated_fx_Dop_Ave_reflection =[];
concatenated_fx_Dop_mean_reflection = [];
concatenated_fx_Dop_spread_reflection = [];
concatenated_fx_Ang_reflection = [];
concatenated_fx_Range_reflection = [];

for frame_index= frame_start: 1:(frame_end- frame_step_for_velo -1 )

    %% Reflection paths
    if settings.simu_reflecting_paths
        %% import
        temp_range_reflection = squeeze(distan_all_array_reflection{1,1}(:,frame_index))/2;
        temp_range_D_reflection = squeeze(distan_all_array_reflection{1,1}(:,frame_index + frame_step_for_velo))/2;
        tem_ang_reflection = pixel_angle;

        %% Calculate the amplitude based on distance and angles
        %         temp_s = squeeze(amplitude_all_reflection(1,1,:,frame_index));
        % G_t; G_r;
        P_t = 60 ; % dBm 40+20
        temp_s_reflection = fun_hybrid_calculate_reflecting_path_power(P_t, pixel_width, pixel_height, settings.reflecting_coefficient,lambda,temp_range_reflection);
        %% Visualize data
        % Construct 3D raw data
        target_locations_reflection = fun_hybrid_generate_point_cloud(temp_range_reflection,tem_ang_reflection,camera_loc);
        target_locations_reflection_D = fun_hybrid_generate_point_cloud(temp_range_D_reflection,tem_ang_reflection,camera_loc);
        %
        if settings.show_figure_60
            figure(60)
            subplot(4,4,1)
            pcolor(angle_vec,angle_vec, reshape(squeeze(distan_all_array_reflection{1,1}(:,frame_index)),pixel_width,pixel_height) )
            %title("Blender distance pictures of Tx" + num2str(index_tx-1) + " Rx" + num2str(index_rx - 1) + " frame index "+num2str(frame_index))
            title("Reflection distance image" + " frame index "+num2str(frame_index))
            caxis([0,10]);
            ax = gca;
            pos = get(gca,'pos');
            shading flat
            xlabel('Azimuth angles [deg]')
            ylabel('Elevation angles [deg]')
            colorbar
            %caxis([3,7])

            % plot antenna locations and the scatter locations
            figure(60)
            subplot(4,4,2)
            scatter3(Antenna.loc_tx(:,2),Antenna.loc_tx(:,1),Antenna.loc_tx(:,3),30,"*",'red')
            hold on
            scatter3(Antenna.loc_rx(:,2),Antenna.loc_rx(:,1),Antenna.loc_rx(:,3),30,"*",'yellow')
            hold on
            scatter3(target_locations_reflection(:,1),target_locations_reflection(:,2),target_locations_reflection(:,3),5,10*log10(temp_s_reflection.' ) )
            %         colorbar
            hold off
            % legend('Location of Txs','Locations of Rxs','Constructed target points')
            title("The output point-cloud with pixel migration" )
            xlabel('X-axis [m]')
            ylabel('Y-axis [m]')
            zlabel('Z-axis [m]')
            xlim([-4,4])
            ylim([-1,10])
            zlim([0,3])
        end

        %% filter wrong velocity based on 3D movement
        target_move_reflection =  temp_range_D_reflection - temp_range_reflection;
        target_move_filter_reflection = target_move_reflection;
        remo_index_dis = find(temp_range_reflection<1 | temp_range_reflection>30 | target_move_filter_reflection> settings.Dis_THR | target_move_filter_reflection < -settings.Dis_THR);
        temp_range_reflection(remo_index_dis) = [];
        temp_range_D_reflection(remo_index_dis) = [];
        temp_s_reflection(remo_index_dis) = [];
        tem_ang_reflection(remo_index_dis,:)=[];
        target_locations_reflection(remo_index_dis,:)=[];

        % plot antenna locations and the scatter locations
        if settings.show_figure_60
            figure(60)
            subplot(4,4,3)
            scatter3(Antenna.loc_tx(:,2),Antenna.loc_tx(:,1),Antenna.loc_tx(:,3),30,"*",'red')
            hold on
            scatter3(Antenna.loc_rx(:,2),Antenna.loc_rx(:,1),Antenna.loc_rx(:,3),30,"*",'yellow')
            hold on
            %         scatter3(target_locations(:,1),target_locations(:,2),target_locations(:,3),5,10*log10(temp_s.'/max(max(temp_s.')) ) )
            scatter3(target_locations_reflection(:,1),target_locations_reflection(:,2),target_locations_reflection(:,3),5,10*log10(temp_s_reflection.' ) )
            %         colorbar
            hold off
            % legend('Location of Txs','Locations of Rxs','Constructed target points')
            title("The output point-cloud denoise" )
            xlabel('X-axis [m]')
            ylabel('Y-axis [m]')
            zlabel('Z-axis [m]')
            xlim([-4,4])
            ylim([-1,10])
            zlim([0,3])
            %      view(-90,45)
            %     caxis([-30,0])
        end

        %% Generate virtual MIMO using plane steering vector or spherical
        distan_all_array_reflection = fun_hybrid_generate_virtual_MIMO(distan_all_array_reflection, lambda, pixel_angle, num_tx, num_rx, frame_index);

        %% Channel generation
        [H_reflection, Dop_mean_reflection, Dop_spread_reflection]= fun_hybrid_generate_channel(num_tx, num_rx, distan_all_array_reflection, frame_index,pixel_radial_velocity, target_move_reflection,...
            slow_time_delta_t,frame_step_for_velo,remo_index_dis,t_slow,temp_s_reflection,f0,c,Np,BW,Tr,t_fast);

        if settings.show_figure_60
            figure(60)
            subplot(4,4,4)
            plot(range_sequency,10*log10(abs(H_reflection(1,:))))
            title('CIR of Tx0 to Rx0')
            xlabel("Range [m]");
            ylabel("Received power in dBm");grid on;
            drawnow
            xlim([1,7])
        end

        %% estimation
        nfft = 144;

        % Dop
        [fx_Dop_reflection,fx_Dop_reflection_win] = fun_hybrid_Doppler_estimation(H_reflection,Np,nfft);
        [fx_Dop_max_reflection, fx_Dop_Ave_reflection] = fun_hybrid_generate_concatenated_Dop(fx_Dop_reflection,settings.range_bin_length);

        concatenated_fx_Dop_max_reflection = [concatenated_fx_Dop_max_reflection fx_Dop_max_reflection];
        concatenated_fx_Dop_Ave_reflection =[concatenated_fx_Dop_Ave_reflection fx_Dop_Ave_reflection];

        concatenated_fx_Dop_mean_reflection = [concatenated_fx_Dop_mean_reflection Dop_mean_reflection];
        concatenated_fx_Dop_spread_reflection = [concatenated_fx_Dop_spread_reflection Dop_spread_reflection];

        % Ang
        [fx_ang_reflection,capRangeAzimuth,Ncap] = fun_hybrid_Ang_estimation(H_reflection,Np,Ns,nfft,num_tx,num_rx,angle_view_cali);
        concatenated_fx_Ang_reflection = [concatenated_fx_Ang_reflection sum(fx_ang_reflection(:,:),2)];
        concatenated_fx_Range_reflection =[concatenated_fx_Range_reflection sum(fx_ang_reflection(:,:),1).'];
        % plot
        if settings.show_figure_40
            figure(40)
            subplot(2,3,1)
            imagesc(range_sequency,[-PRF/2,PRF/2]/f0*c, 10*log10(fx_Dop_reflection/max(max(fx_Dop_reflection))))
            xlim([1,7])
            ylim([-3.5,3.5])
            %zlim([0,max(max(abs(fx)))])
            set(gca,'YDir','normal')
            xlabel('range [m]',FontSize=10)
            ylabel('Doppler velocity [m/s]',FontSize=10)
            title("Scattering Doppler image" + " frame index "+num2str(frame_index))
            shading flat
            colorbar
            caxis([  - 30,-0  ]);

            concatenated_fx_Dop_Ave_reflection_2 = concatenated_fx_Dop_Ave_reflection;
%             concatenated_fx_Dop_Ave_scattering_2 = concatenated_fx_Dop_max_scattering;
            for ii = 1:size(concatenated_fx_Dop_Ave_reflection,2)
                concatenated_fx_Dop_Ave_reflection_2(:,ii) = concatenated_fx_Dop_Ave_reflection(:,ii)/max(concatenated_fx_Dop_Ave_reflection(:,ii));
            end
            figure(40)
            subplot(2,3,2)
            imagesc([1:(frame_index - frame_start+1)]*slow_time_delta_t,[-PRF/2,PRF/2]/f0*c, 10*log10([ concatenated_fx_Dop_Ave_reflection_2 ]))
            set(gca,'YDir','normal')
            xlabel('Time [s]','FontSize', 15)
            ylabel('Doppler velocity [m/s]','FontSize', 15)
            shading flat
%             ylim([-4.5,4.5])
            caxis([  -35, -0 ]);

            figure(40)
            subplot(2,3,3)
            plot([1:(frame_index - frame_start+1)]*slow_time_delta_t, (concatenated_fx_Dop_mean_reflection));
%             ylim([-3.5,3.5])
            xlabel('Time [s]','FontSize', 15)
            ylabel('Normalized Dop velocity PSD [m/s]','FontSize', 15)
            legend('Dop mean')

            figure(40)
            subplot(2,3,4)
            plot([1:(frame_index - frame_start+1)]*slow_time_delta_t, (concatenated_fx_Dop_spread_reflection));
%             ylim([-3.5,3.5])
             xlabel('Time [s]','FontSize', 15)
            ylabel('RMS Doppler spread [m/s]','FontSize', 15)
            legend('Dop spread')

            concatenated_fx_Ang_reflection_2 = concatenated_fx_Ang_reflection;
%             concatenated_fx_Dop_Ave_scattering_2 = concatenated_fx_Dop_max_scattering;
            for ii = 1:size(concatenated_fx_Ang_reflection,2)
                concatenated_fx_Ang_reflection_2(:,ii) = concatenated_fx_Ang_reflection(:,ii)/max(concatenated_fx_Ang_reflection(:,ii));
            end
            figure(40)
            subplot(2,3,5)
            imagesc([1:(frame_index - frame_start+1)]*slow_time_delta_t,linspace(angle_view_cali(1),angle_view_cali(2),nfft), 10*log10(abs(concatenated_fx_Ang_reflection_2 )) )
            set(gca,'YDir','normal')
            xlabel('Time [s]','FontSize', 15)
            ylabel('Ang [deg]','FontSize', 15)
            shading flat
            caxis([  -20, -0 ]);

            concatenated_fx_Range_reflection_2 = concatenated_fx_Range_reflection;
%             concatenated_fx_Dop_Ave_scattering_2 = concatenated_fx_Dop_max_scattering;
            for ii = 1:size(concatenated_fx_Range_reflection,2)
                concatenated_fx_Range_reflection_2(:,ii) = concatenated_fx_Range_reflection(:,ii)/max(concatenated_fx_Range_reflection(:,ii));
            end
            figure(40)
            subplot(2,3,6)
            imagesc([1:(frame_index - frame_start+1)]*slow_time_delta_t,range_sequency, 10*log10(abs( concatenated_fx_Range_reflection_2 )))
            set(gca,'YDir','normal')
            xlabel('Time [s]','FontSize', 15)
            ylabel('Range [m]','FontSize', 15)
            shading flat
            ylim([3,7])
            caxis([  -30, -0 ]);

        end 

        if settings.show_figure_60
            figure(60)
            subplot(4,4,5)
            imagesc(range_sequency,[-PRF/2,PRF/2]/f0*c, 10*log10(fx_Dop_reflection/max(max(fx_Dop_reflection))))
            xlim([1,7])
            ylim([-3,3])
            %zlim([0,max(max(abs(fx)))])
            set(gca,'YDir','normal')
            xlabel('range [m]',FontSize=20)
            ylabel('Doppler velocity [m/s]',FontSize=20)
            %     title('Doppler velocity  estimation')
            shading flat
            colorbar
            caxis([  - 30,-0  ]);
            % caxis([max(max(fx))-5,max(max(fx))]);

            figure(60)
            subplot(4,4,6)
            %pcolor(range_sequency,linspace(-90,90,nfft),fx)
            imagesc(range_sequency,linspace(angle_view_cali(1),angle_view_cali(2),nfft),10*log10(fx_ang_reflection/max(max(fx_ang_reflection))))
            %bimagesc(range_sequency,linspace(-90,90,nfft), 10*log10(fx_ang))
            xlim([1, 7])
            % ylim([-75,75])
            %zlim([0,max(max(abs(fx)))])
            set(gca,'YDir','normal')
            xlabel('range [m]')
            ylabel('angle [deg.]')
            %     title('Angle estimation')
            shading flat
            colorbar
            %     caxis([max(max(10*log10(fx_ang)))-50,max(max(10*log10(fx_ang)))])
            caxis([-30,-0]);
            %     concatenated_fx_Ang = [concatenated_fx_Ang max(fx_ang_reflection,[],2)];
            %         keyboard

            figure(60)
            subplot(4,4,7)
            imagesc(range_sequency,linspace(angle_view_cali(1),angle_view_cali(2),Ncap), 10*log10(abs(capRangeAzimuth.')))
            % ylim([-45,45])
            %zlim([0,max(max(abs(fx)))])
            set(gca,'YDir','normal')
            xlabel('Range [m]','FontSize', 20)
            ylabel('Azimuth angle [deg]','FontSize', 20)
            shading flat
            colorbar
            caxis([-40,-0])
            xlim([1,7])
        end
    else
    end

    %% Scattering paths
    if settings.simu_scattering_paths
        %% import
        temp_range_scattering = squeeze(distan_all_array_scattering{1,1}(:,frame_index))/2;
        temp_range_D_scattering = squeeze(distan_all_array_scattering{1,1}(:,frame_index + frame_step_for_velo))/2;
        tem_ang_scattering = pixel_angle;

        %% Calculate the amplitude based on distance and angles
        %         temp_s = squeeze(amplitude_all_reflection(1,1,:,frame_index));
        % G_t; G_r;
        P_t = 60 ; % dBm 40+20
        temp_s_scattering = fun_hybrid_calculate_scattering_path_power(P_t, pixel_width, pixel_height, settings.scattering_coefficient,lambda,temp_range_scattering,tem_ang_scattering);
        %% Visualize data
        % Construct 3D raw data
        target_locations_scattering = fun_hybrid_generate_point_cloud(temp_range_scattering,tem_ang_scattering,camera_loc);
        target_locations_scattering_D = fun_hybrid_generate_point_cloud(temp_range_D_scattering,tem_ang_scattering,camera_loc);
        %
        if settings.show_figure_60
            figure(60)
            subplot(4,4,8)
            pcolor(angle_vec,angle_vec, reshape(squeeze(distan_all_array_scattering{1,1}(:,frame_index)),pixel_width,pixel_height) )
            %title("Blender distance pictures of Tx" + num2str(index_tx-1) + " Rx" + num2str(index_rx - 1) + " frame index "+num2str(frame_index))
            title("Scattering distance image" + " frame index "+num2str(frame_index))
            caxis([0,10]);
            ax = gca;
            pos = get(gca,'pos');
            shading flat
            xlabel('Azimuth angles [deg]')
            ylabel('Elevation angles [deg]')
            colorbar
            %caxis([3,7])

            % plot antenna locations and the scatter locations
            figure(60)
            subplot(4,4,9)
            scatter3(Antenna.loc_tx(:,2),Antenna.loc_tx(:,1),Antenna.loc_tx(:,3),30,"*",'red')
            hold on
            scatter3(Antenna.loc_rx(:,2),Antenna.loc_rx(:,1),Antenna.loc_rx(:,3),30,"*",'yellow')
            hold on
            scatter3(target_locations_scattering(:,1),target_locations_scattering(:,2),target_locations_scattering(:,3),5,10*log10(temp_s_scattering.' ) )
            %         colorbar
            hold off
            % legend('Location of Txs','Locations of Rxs','Constructed target points')
            title("The output point-cloud with pixel migration" )
            xlabel('X-axis [m]')
            ylabel('Y-axis [m]')
            zlabel('Z-axis [m]')
            xlim([-4,4])
            ylim([-1,10])
            zlim([0,3])
        end

        %% filter wrong velocity based on 3D movement
        target_move_scattering =  temp_range_D_scattering - temp_range_scattering;
        target_move_filter_scattering = target_move_scattering;
        remo_index_dis = find(temp_range_scattering<2 | temp_range_scattering>30 | target_move_filter_scattering> settings.Dis_THR | target_move_filter_scattering < -settings.Dis_THR);
        temp_range_scattering(remo_index_dis) = [];
        temp_range_D_scattering(remo_index_dis) = [];
        temp_s_scattering(remo_index_dis) = [];
        tem_ang_scattering(remo_index_dis,:)=[];
        target_locations_scattering(remo_index_dis,:)=[];

        % plot antenna locations and the scatter locations
        if settings.show_figure_60
            figure(60)
            subplot(4,4,10)
            scatter3(Antenna.loc_tx(:,2),Antenna.loc_tx(:,1),Antenna.loc_tx(:,3),30,"*",'red')
            hold on
            scatter3(Antenna.loc_rx(:,2),Antenna.loc_rx(:,1),Antenna.loc_rx(:,3),30,"*",'yellow')
            hold on
            %         scatter3(target_locations_scattering(:,1),target_locations_scattering(:,2),target_locations_scattering(:,3),5,10*log10(temp_s_scattering.'/max(max(temp_s_scattering.')) ) )
            scatter3(target_locations_scattering(:,1),target_locations_scattering(:,2),target_locations_scattering(:,3),5,10*log10(temp_s_scattering.' ) )
            %         colorbar
            hold off
            % legend('Location of Txs','Locations of Rxs','Constructed target points')
            title("The output point-cloud denoise" )
            xlabel('X-axis [m]')
            ylabel('Y-axis [m]')
            zlabel('Z-axis [m]')
            xlim([-4,4])
            ylim([-1,10])
            zlim([0,3])
            %      view(-90,45)
            %             caxis([-30,0])
        end

        %% Generate virtual MIMO using plane steering vector or spherical
        distan_all_array_scattering = fun_hybrid_generate_virtual_MIMO(distan_all_array_scattering, lambda, pixel_angle, num_tx, num_rx, frame_index);

        %% Channel generation
        [H_scattering, Dop_mean_scattering, Dop_spread_scattering]= fun_hybrid_generate_channel(num_tx, num_rx, distan_all_array_scattering, frame_index,pixel_radial_velocity, target_move_scattering,...
            slow_time_delta_t,frame_step_for_velo,remo_index_dis,t_slow,temp_s_scattering,f0,c,Np,BW,Tr,t_fast);

        if settings.show_figure_60
            figure(60)
            subplot(4,4,11)
            plot(range_sequency,10*log10(abs(H_scattering(1,:))))
            title('CIR of Tx0 to Rx0')
            xlabel("Range [m]");
            ylabel("Received power in dBm");grid on;
            drawnow
            xlim([1,7])
        end

        %% estimation
        nfft = 144;

        % Dop
        [fx_Dop_scattering,fx_Dop_scattering_win] = fun_hybrid_Doppler_estimation(H_scattering,Np,nfft);
        [fx_Dop_max_scattering, fx_Dop_Ave_scattering] = fun_hybrid_generate_concatenated_Dop(fx_Dop_scattering,settings.range_bin_length);
        [fx_Dop_max_scattering, fx_Dop_Ave_scattering] = fun_hybrid_generate_concatenated_Dop(fx_Dop_scattering_win,settings.range_bin_length);


        concatenated_fx_Dop_mean_scattering = [concatenated_fx_Dop_mean_scattering Dop_mean_scattering];
        concatenated_fx_Dop_spread_scattering = [concatenated_fx_Dop_spread_scattering Dop_spread_scattering];

        concatenated_fx_Dop_max_scattering = [concatenated_fx_Dop_max_scattering fx_Dop_max_scattering];
        concatenated_fx_Dop_Ave_scattering =[concatenated_fx_Dop_Ave_scattering fx_Dop_Ave_scattering];

        % Ang
        [fx_ang_scattering,capRangeAzimuth_scattering,Ncap] = fun_hybrid_Ang_estimation(H_scattering,Np,Ns,nfft,num_tx,num_rx,angle_view_cali);

        concatenated_fx_Ang_scattering = [];
        concatenated_fx_Range_scattering = [];


        if settings.show_figure_50
            figure(50)
            subplot(2,2,1)
            imagesc(range_sequency,[-PRF/2,PRF/2]/f0*c, 10*log10(fx_Dop_scattering/max(max(fx_Dop_scattering))))
            xlim([1,7])
            ylim([-3.5,3.5])
            %zlim([0,max(max(abs(fx)))])
            set(gca,'YDir','normal')
            xlabel('range [m]',FontSize=10)
            ylabel('Doppler velocity [m/s]',FontSize=10)
            title("Scattering Doppler image" + " frame index "+num2str(frame_index))
            shading flat
            colorbar
            caxis([  - 30,-0  ]);

            concatenated_fx_Dop_Ave_scattering_2 = concatenated_fx_Dop_Ave_scattering;
%             concatenated_fx_Dop_Ave_scattering_2 = concatenated_fx_Dop_max_scattering;
            for ii = 1:size(concatenated_fx_Dop_Ave_scattering,2)
                concatenated_fx_Dop_Ave_scattering_2(:,ii) = concatenated_fx_Dop_Ave_scattering(:,ii)/max(concatenated_fx_Dop_Ave_scattering(:,ii));
            end
            figure(50)
            subplot(2,2,2)
            imagesc([1:(frame_index - frame_start+1)]*slow_time_delta_t,[-PRF/2,PRF/2]/f0*c, 10*log10([ concatenated_fx_Dop_Ave_scattering_2 ]))
            set(gca,'YDir','normal')
            xlabel('Time [s]','FontSize', 15)
            ylabel('Doppler velocity [m/s]','FontSize', 15)
            shading flat
            ylim([-4.5,4.5])
            caxis([  -35, -0 ]);

            figure(50)
            subplot(2,2,3)
            plot([1:(frame_index - frame_start+1)]*slow_time_delta_t,concatenated_fx_Dop_mean_scattering);
%             ylim([-3.5,3.5])
            xlabel('Time [s]','FontSize', 15)
            ylabel('Normalized Dop velocity PSD [m/s]','FontSize', 15)
            legend('Dop mean')

            figure(50)
            subplot(2,2,4)
            plot([1:(frame_index - frame_start+1)]*slow_time_delta_t,concatenated_fx_Dop_spread_scattering);
%             ylim([-3.5,3.5])
             xlabel('Time [s]','FontSize', 15)
            ylabel('RMS Doppler spread [m/s]','FontSize', 15)
            legend('Dop spread')
        end 

        if settings.show_figure_60
            % plot
            figure(60)
            subplot(4,4,12)
            imagesc(range_sequency,[-PRF/2,PRF/2]/f0*c, 10*log10(fx_Dop_scattering/max(max(fx_Dop_scattering))))
            xlim([1,7])
            ylim([-3,3])
            %zlim([0,max(max(abs(fx)))])
            set(gca,'YDir','normal')
            xlabel('range [m]',FontSize=20)
            ylabel('Doppler velocity [m/s]',FontSize=20)
            %     title('Doppler velocity  estimation')
            shading flat
            colorbar
            caxis([  - 30,-0  ]);
            % caxis([max(max(fx))-5,max(max(fx))]);

            figure(60)
            subplot(4,4,13)
            %pcolor(range_sequency,linspace(-90,90,nfft),fx)
            imagesc(range_sequency,linspace(angle_view_cali(1),angle_view_cali(2),nfft),10*log10(fx_ang_scattering/max(max(fx_ang_scattering))))
            %bimagesc(range_sequency,linspace(-90,90,nfft), 10*log10(fx_ang))
            xlim([1, 7])
            % ylim([-75,75])
            %zlim([0,max(max(abs(fx)))])
            set(gca,'YDir','normal')
            xlabel('range [m]')
            ylabel('angle [deg.]')
            %     title('Angle estimation')
            shading flat
            colorbar
            %     caxis([max(max(10*log10(fx_ang)))-50,max(max(10*log10(fx_ang)))])
            caxis([-30,-0]);
            %     concatenated_fx_Ang = [concatenated_fx_Ang max(fx_ang_reflection,[],2)];
            %         keyboard

            figure(60)
            subplot(4,4,14)
            imagesc(range_sequency,linspace(angle_view_cali(1),angle_view_cali(2),Ncap), 10*log10(abs(capRangeAzimuth_scattering.')))
            % ylim([-45,45])
            %zlim([0,max(max(abs(fx)))])
            set(gca,'YDir','normal')
            xlabel('Range [m]','FontSize', 20)
            ylabel('Azimuth angle [deg]','FontSize', 20)
            shading flat
            colorbar
            caxis([-40,-0])
            xlim([1,7])
        end
    else
    end

    %% if considering both reflection and scattering, mergy them
    if settings.simu_reflecting_paths && settings.simu_scattering_paths
        if settings.show_figure_60
            figure(60)
            subplot(4,4,15)
            scatter3(Antenna.loc_tx(:,2),Antenna.loc_tx(:,1),Antenna.loc_tx(:,3),30,"*",'red')
            hold on
            scatter3(Antenna.loc_rx(:,2),Antenna.loc_rx(:,1),Antenna.loc_rx(:,3),30,"*",'yellow')
            hold on
            scatter3(target_locations_reflection(:,1),target_locations_reflection(:,2),target_locations_reflection(:,3),5,10*log10(temp_s_reflection.' ) )
            hold on
            scatter3(target_locations_scattering(:,1),target_locations_scattering(:,2),target_locations_scattering(:,3),5,10*log10(temp_s_scattering.' ) )
            %         colorbar
            hold off
            % legend('Location of Txs','Locations of Rxs','Constructed target points')
            title("Point-cloud of both reflection and scattering" )
            xlabel('X-axis [m]')
            ylabel('Y-axis [m]')
            zlabel('Z-axis [m]')
            xlim([-3,3])
            ylim([-1,6])
            zlim([0,3])
            colorbar
            caxis([-90, -80])

            figure(60)
            subplot(4,4,16)
            plot(range_sequency,10*log10(abs(H_reflection(1,:))))
            hold on
            plot(range_sequency,10*log10(abs(H_scattering(1,:))))
            hold on
            plot(range_sequency,10*log10(abs(H_reflection(1,:) + H_scattering(1,:))))
            xlabel("Range [m]");
            ylabel("Received power in dBm");grid on;
            xlim([1,7])
            legend('Reflection paths','Scattering paths','Overal channel');
        end

    else
    end


%     keyboard
end

keyboard

concatenated_fx_Dop_Ave_reflection_2 = concatenated_fx_Dop_Ave_reflection;
for ii = 1:size(concatenated_fx_Dop_Ave_reflection,2)
    concatenated_fx_Dop_Ave_reflection_2(:,ii) = concatenated_fx_Dop_Ave_reflection(:,ii)/max(concatenated_fx_Dop_Ave_reflection(:,ii));
end
figure(25)
imagesc([1:frame_end]*slow_time_delta_t,[-PRF/2,PRF/2]/f0*c, 10*log10([ concatenated_fx_Dop_Ave_reflection_2 ]))
set(gca,'YDir','normal')
xlabel('Time [s]','FontSize', 20)
ylabel('Doppler velocity [m/s]','FontSize', 20)
shading flat
ylim([-2,2])
caxis([  -30, -0 ]);





