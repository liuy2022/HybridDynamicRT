% clc;
% clear;

filename = "radar_parameters_hybrid.json";

% Read Tx and Rx antenna positions [x,y,z] from json file to a struct
params = jsondecode(fileread(filename));


% Read tx positions
offsets = params.antenna_offsets_tx;
values = fieldnames(offsets);
for i = 1:numel(values)
    pos = sprintf('%.5f. ', offsets.(values{i}));
    fprintf("%s offset: %s\n", values{i}, pos)
end

% Read rx positions
offsets = params.antenna_offsets_rx;
values = fieldnames(offsets);
for i = 1:numel(values)
    pos = sprintf('%.5f. ', offsets.(values{i}));
    fprintf("%s offset: %s\n", values{i}, pos)
end

%% Change position
% params.antenna_offsets_tx.Tx0 = [0, 0, 0];

%% Add position
% params.antenna_offsets_tx.Tx1 = [0, 0, 0];

%% Write struct to json file
% fileID = fopen(filename,'w');
% fprintf(fileID,'%s', jsonencode(params, PrettyPrint=true));
% fclose(fileID);
