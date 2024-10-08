function distan_all_array_reflection = fun_hybrid_generate_virtual_MIMO(distan_all_array_reflection, lambda, pixel_angle, num_tx, num_rx, frame_index)
%UNTITLED5 Summary of this function goes here
for i = 1:num_tx
    for j = 1:num_rx
        distan_all_array_reflection{i,j}(:,frame_index) = squeeze(distan_all_array_reflection{1,1}(:,frame_index)) + (lambda/2*(j-1)+2*lambda*(i-1))*(sin(pixel_angle(:,2)).*cos(pixel_angle(:,1)));
    end
end

end