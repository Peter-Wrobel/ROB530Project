parseObj = dataparse;
RB_1 = parseObj.parse_robot(1,1);
RB_2 = parseObj.parse_robot(1,2);

rb_1 = RB_1(1:100:10000,:);
rb_2 = RB_2(1:100:10000,:);



hold on;

plot(rb_1(:,2), rb_1(:,3));
plot(rb_2(:,2), rb_2(:,3));




reso = parseObj.rel_measure(rb_1, rb_2,0.1);
[x,y] = pol2cart(reso(:,1), reso(:,2));

plot(rb_1(:,2)+x, rb_1(:,3) +y);