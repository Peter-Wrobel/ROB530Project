parseObj = dataparse(1, 1);

%Raw groundtruth data from dataset. Arranged [time, x pos, y pos, theta(unneccessary)]
M = containers.Map()
RB_1 = parseObj.parse_robot(3,M);
RB_2 = parseObj.parse_robot(2,M);


% Takes 100 points from robot 1 and robot 2
rb_1 = RB_1(1:100,:);
rb_2 = RB_2(1:100,:);



hold on;

% I plot both robot 1 ground truth and robot 2 ground trush
plot(rb_1(:,2), rb_1(:,3));
plot(rb_2(:,2), rb_2(:,3));




% reso = [ relative theta of robot2 to robot 1, relative distance of robot2
% to robot1
reso = parseObj.rel_measure(rb_1, rb_2,0.1);

% Here, I convert the relative [theta, distance] to relative [x,y]
[x,y] = pol2cart(reso(:,1), reso(:,2));




% I plot my guess for robot2 by adding relative measurement of robot2 to
% actual position of robot1
plot(rb_1(:,2)+x, rb_1(:,3) +y);