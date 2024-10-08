function target_locations_reflection = fun_hybrid_generate_point_cloud(temp_range_reflection,tem_ang_reflection,camera_loc)
%UNTITLED4 Summary of this function goes here
target_locations_reflection = ones(length(temp_range_reflection),3);
target_locations_reflection(:,1) = temp_range_reflection.*[sin(tem_ang_reflection(:,2)).*cos(tem_ang_reflection(:,1))  ] + camera_loc(2);
target_locations_reflection(:,2) = temp_range_reflection.*[sin(tem_ang_reflection(:,2)).*sin(tem_ang_reflection(:,1))  ]+ camera_loc(1);
target_locations_reflection(:,3) = temp_range_reflection.*[cos(tem_ang_reflection(:,2))]+ camera_loc(3);
end