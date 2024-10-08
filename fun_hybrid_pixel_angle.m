function [pixel_angle,angle_vec,angle_view_cali] = fun_hybrid_pixel_angle(camera_ang,camera_ang_rotate,pixel_width,pixel_height)

angle_view_cali = [(camera_ang/2 + camera_ang_rotate(1)  - camera_ang)  abs((camera_ang/2 - camera_ang_rotate(1) - camera_ang))];
angle_vec = angle_view_cali(1):(camera_ang/(pixel_width - 1)):angle_view_cali(2);
angle_vec_rad = angle_vec*pi/180;
pixel_angle_az = zeros(pixel_width,pixel_height);
pixel_angle_ele = zeros(pixel_width,pixel_height);
for index_pixel = 1: pixel_width
    % azimuth
    pixel_angle_az(:,index_pixel) = pi/2 - angle_vec_rad(index_pixel);
    %ele
    pixel_angle_ele(index_pixel,:)=  pi/2 - flip(angle_vec_rad(index_pixel));
end
pixel_angle= [reshape(pixel_angle_az,[],1) reshape(pixel_angle_ele,[],1)];

end