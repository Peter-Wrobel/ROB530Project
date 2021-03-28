%%

classdef dataparse
   methods
       
      function table_ans = parse_landmark(obj,dataset_num)
        path_str = strcat("data/MRCLAM_Dataset", int2str(dataset_num), "/");
        

        lmfile = strcat(path_str,robot_str, "Landmark_Groundtruth.dat");
        
        LM_GT= readtable(lmfile);
        table_ans = table2array(LM_GT);
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
        
      %returns polar coordinates of robot2 relative to robot1
      function reso = rel_measure(obj ,RB_GT1, RB_GT2, var)
          
          x1 = RB_GT1(:,2);
          y1 = RB_GT1(:,3);
          x2 = RB_GT2(:,2);
          y2=  RB_GT2(:,3);

          range = sqrt((x2-x1).^2 +(y2-y1).^2);
          range = range + normrnd(0,var,size(x2)); 
          theta = atan2((y2-y1),(x2-x1));
          
          reso = [theta, range];
          
          
      end
      

   end
end





