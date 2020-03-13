close all
cfg.fx = 1456.80744057419;
cfg.fy = 1470.31905093091;
cfg.cx = 625.949323219046;
cfg.cy = 303.522715545930;
cfg.r = 0;
% rad
cfg.theta_z = -1.459852625;
cfg.theta_y = 0.008844569881;
cfg.theta_x = -1.568385001;

cfg.theta_x = -1.457;
% meter
cfg.t_z = 1.712;
cfg.t_y = -0.04;
cfg.t_x = 1.118;
cfg.mounting_height = 1.21;
%
cfg.k1 = -0.249040866495024;
cfg.k2 = 0.0840174379167989;
cfg.k3 = 0.0;
cfg.p1 = 0.0;
cfg.p2 = 0.0;
cfg.width = 1280;
cfg.height = 720;
cfg.fov_horizontal = 52;
cfg.fov_vertical = 32;
cfg.camera_intrinsic = [cfg.fx, 0,      cfg.cx; 
                        0,      cfg.fy, cfg.cy; 
                        0,      0,      1];
%the coordinates have been rotated about -rotation_x
%or interpreted as the vector in coordinates have been rotated rotation_x
%here we chose the former interpretation 
Rotation_x = [1, 0,                 0; 
              0, cos(cfg.theta_x), -sin(cfg.theta_x); 
              0, sin(cfg.theta_x), cos(cfg.theta_x)];
          
Rotation_y = [cos(cfg.theta_y), 0,  sin(cfg.theta_y); 
              0,                1,  0; 
              -sin(cfg.theta_y), 0,  cos(cfg.theta_y)];
          
Rotation_z = [cos(cfg.theta_z), -sin(cfg.theta_z),  0; 
              sin(cfg.theta_z), cos(cfg.theta_z),   0; 
              0,                 0,                 1];
cfg.camera2VCS = zeros(4, 4);                    
cfg.camera2VCS(1:3, 1:3) = Rotation_z * Rotation_y * Rotation_x;
cfg.camera2VCS(:, 4) = [cfg.t_x; cfg.t_y; cfg.t_z; 1];

cfg.VCS2camera = inv(cfg.camera2VCS);
% vehicle coordinate system, meter
%              ^ x
%              1
%              1
%y<------------1
cfg.x_min = 5;
cfg.x_max = 100;
cfg.y_min = -20;
cfg.y_max = 20;
cfg.imp_limits = [cfg.x_min, cfg.x_max, cfg.y_min, cfg.y_max]; 
cfg.imp_img_width = 1280;
cfg.imp_img_height = 720;
cfg.display = false;

% path to data directory 
data_dir_src = fullfile('data','image-src');
data_dir_out = fullfile('data','image-output');

image_num = 1000;
image_names = cell(image_num, 1);
for image_cnt = 1 : image_num
    image_name = char(string(image_cnt));
    image_name(end + 1 : end + 4) = '.jpg';
    image_names{image_cnt} = image_name;
end

for image_cnt = 1 : image_num
    this_img = fullfile(data_dir_src, image_names{image_cnt});
    im = imread(this_img);

    if cfg.display
        figure(1);clf;
        imagesc(im);axis image off;
        title("Inpute image")
    end

    % construct xy_grid 
    xy_grid = zeros(cfg.imp_img_height, cfg.imp_img_width, 2); 
    step_col = (cfg.y_max - cfg.y_min) / cfg.imp_img_width;
    step_row = (cfg.x_max - cfg.x_min) / cfg.imp_img_height;
    for xy_grid_v = 1 : cfg.imp_img_height
        for xy_grid_u = 1 : cfg.imp_img_width
            xy_grid(xy_grid_v, xy_grid_u, 1) = cfg.x_max-xy_grid_v*step_row;
            xy_grid(xy_grid_v, xy_grid_u, 2) = cfg.y_max-xy_grid_u*step_col;
        end
    end

    % construct uv_grid
    uv_grid = zeros(cfg.imp_img_height, cfg.imp_img_width, 3); 
    for uv_grid_v = 1 : cfg.imp_img_height
        for uv_grid_u = 1 : cfg.imp_img_width
            % point in world coordinates
            point = [xy_grid(uv_grid_v, uv_grid_u, 1); xy_grid(uv_grid_v, uv_grid_u, 2); 0; 1];
            point_camera = cfg.VCS2camera * point;
            point_img = cfg.camera_intrinsic * point_camera(1:3);
            point_img = point_img/point_img(3);
            if point_img(1) < cfg.width && point_img(1) > 1 && point_img(2) < cfg.height && point_img(2) > 1
                point_img = int64(point_img);
                uv_grid(uv_grid_v, uv_grid_u, :) = im(point_img(2), point_img(1), :);
                %im(point_img(2), point_img(1), :) = [255, 0, 0];
            %uv = point;
            %uv_grid(uv_grid_v, uv_grid_u, 1) = [cfg.x_max-xy_grid_v*step_row];
            %uv_grid(uv_grid_v, uv_grid_u, 2) = [cfg.y_max-xy_grid_u*step_col];
            end
        end
    end
    
    out_img_name = fullfile(data_dir_out, image_names{image_cnt});
    imwrite(uint8(uv_grid), out_img_name);
    if cfg.display
        figure(2);clf;
        imagesc(uint8(uv_grid));
        title("IPM image");
        pause(1);
    end
end
