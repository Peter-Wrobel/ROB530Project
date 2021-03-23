%%

classdef dataparse
   methods
      function res = func1(obj,a)
         res = a * 5; 
      end
      
      function res = parse_robot(obj,dataset_num, robot_num)
        path_str = strcat("data/MRCLAM_Dataset", int2str(dataset_num), "/");

        robot_str = strcat("Robot", int2str(robot_num), "_");


        rb_gt_file = strcat(path_str,robot_str, "Groundtruth.dat");
        rb_od_file = strcat(path_str,robot_str, "Odometry.dat");
        rb_ms_file = strcat(path_str,robot_str, "Measurement.dat");


        RB_GT = readtable(rb_gt_file);
        RB_OD = readtable(rb_od_file);
        RB_MS = readtable(rb_ms_file);



        res = table2array(RB_GT);


        end

   end
end





