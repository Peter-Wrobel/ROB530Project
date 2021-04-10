%%

classdef dataparse
   properties 
       dataset_num;                 % number of MRCLAM trial
       path_str;                    % directory of MCRLAM dataset
       SYNC_TIME_BEF;               % No time readings before this
       SYNC_TIME_AFT;               % No time readings after  this
       TOTAL_TIME;
       Tdel;
  
   end
   
   methods
       
      function obj = dataparse(val, ts, num_sec)
          obj.dataset_num = val;
          obj.path_str = strcat("data/MRCLAM", int2str(obj.dataset_num), "/MRCLAM_Dataset",int2str(obj.dataset_num), "/");
          
          if val == 2
            obj.SYNC_TIME_BEF = 1248275433;
          elseif val == 3
            obj.SYNC_TIME_BEF = 1248294066;
          else
            obj.SYNC_TIME_BEF = 1248272280;
          end
          
          obj.SYNC_TIME_AFT = obj.SYNC_TIME_BEF + num_sec;
          obj.TOTAL_TIME    = obj.SYNC_TIME_AFT - obj.SYNC_TIME_BEF;
          obj.Tdel = ts;


      end
      
      function times = time_length(obj)
          times =  obj.TOTAL_TIME/obj.Tdel  +1;
      end
      
      function barcode = parse_barcode(obj)
          bc_file = strcat(obj.path_str, "Barcodes.dat");
          BC_TABLE = readtable(bc_file);
          
          barcode = table2array(BC_TABLE);
      end
      
      

       
      function table_ans = parse_landmark(obj)

        lmfile = strcat(obj.path_str, "Landmark_Groundtruth.dat");
        
        LM_GT= readtable(lmfile);
        table_ans = table2array(LM_GT);
      end
       
      function [GT, OD, MS] = parse_robot(obj,robot_num, landmark_codes)

        robot_str = strcat("Robot", int2str(robot_num), "_");

        rb_gt_file = strcat(obj.path_str,robot_str, "Groundtruth.dat");
        rb_od_file = strcat(obj.path_str,robot_str, "Odometry.dat");
        rb_ms_file = strcat(obj.path_str,robot_str, "Measurement.dat");

        RB_GT = readtable(rb_gt_file);
        RB_OD = readtable(rb_od_file);
        RB_MS = readtable(rb_ms_file);
        
        GT = table2array(RB_GT);
        OD = table2array(RB_OD);
        MS = table2array(RB_MS);
        
        %GT adjustment
        GT(GT(:,1)<obj.SYNC_TIME_BEF, :) = [];
        GT(GT(:,1)>obj.SYNC_TIME_AFT, :) = [];
        GT(:,1) = GT(:,1)- obj.SYNC_TIME_BEF;
        
        index = 1;
        for ts = 0:obj.Tdel:obj.TOTAL_TIME
            have_mes = false;
            while index<= size(GT,1) &&  GT(index, 1) < ts+0.5

                if(have_mes)
                    GT(index, :) = [];
                    
                else
                    have_mes = true;
                    index = index+1;
                    GT(index,1) = ts;

                end
                
            end
            
            if have_mes == false
                add = GT(index-1,:);
                add(1,1) = ts;
                GT = [GT(1:index-1,:); add; GT(index:end,:)];
                index = index+1;
            end
            
        end
        
        %OD adjustment
        OD(OD(:,1)<obj.SYNC_TIME_BEF, :) = [];
        OD(OD(:,1)>obj.SYNC_TIME_AFT, :) = [];
        OD(:,1) = OD(:,1)- obj.SYNC_TIME_BEF;
        
        index = 1;
        
        for ts = 0:obj.Tdel:obj.TOTAL_TIME
            have_mes = false;
            while index<= size(OD,1) &&  OD(index, 1) < ts+0.5

                if(have_mes)
                    OD(index, :) = [];
                    
                else
                    have_mes = true;
                    OD(index,1) = ts;
                    index = index+1;

                end
                
            end
            if have_mes == false
                add = OD(index-1,:);
                add(1,1) = ts;
                OD = [OD(1:index-1,:); add; OD(index:end,:)];
                index = index+1;
            end
            
        end
        
        %MS adjustment
        MS(MS(:,1)<obj.SYNC_TIME_BEF, :) = [];
        MS(MS(:,1)>obj.SYNC_TIME_AFT, :) = [];
        MS(:,1) = MS(:,1)- obj.SYNC_TIME_BEF;
        
        index = 1;
        for ts = 0:obj.Tdel:obj.TOTAL_TIME
            have_mes = false;
            while index<= size(MS,1) &&  MS(index, 1) < ts+0.5
                if(have_mes)
                    MS(index, :) = [];
                     
                elseif(landmark_codes.isKey(MS(index,2)))
                    
                    have_mes = true;
                    MS(index, 1) = ts;
                    MS(index,2) = landmark_codes(MS(index,2))-5;
                    index = index+1;
                    
                else
                    MS(index, :) = [];
                end
                
            end
            if have_mes == false
                MS = [MS(1:index-1,:); [ts,-1,-1, -1]; MS(index:end,:)];
                index = index+1;
            end
        end
      end
        
      %returns polar coordinates of robot2 relative to robot1
      function reso = rel_measure(obj ,RB_GT1, RB_GT2, var)
          
          theta1 = RB_GT1(:,1);
          x1 = RB_GT1(:,2);
          y1 = RB_GT1(:,3);
          
          theta2 = RB_GT2(:,1);
          x2 = RB_GT2(:,2);
          y2=  RB_GT2(:,3);
          
          
          range = sqrt((x2-x1).^2 +(y2-y1).^2);
          range = range + normrnd(0,var,size(x2)); 
          theta = wrapToPi(atan2((y2-y1),(x2-x1)));
          
          numerator = -sin(theta1).*(x2-x1) + cos(theta1).*(y2-y1);
          denominator = cos(theta1).*(x2-x1) + sin(theta1).*(y2-y1);
          theta1 = wrapToPi(atan2(numerator,denominator));
          
          reso = [theta1, range];
          
      end
   end
end





