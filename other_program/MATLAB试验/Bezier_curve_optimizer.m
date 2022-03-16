p0_x = 1;
p0_y = 1;
p1_x = 3;
p1_y = 3;
p2_x = 9;
p2_y = 0;
b_x=[];
b_y=[];
i = 1;

for t = 0 : 0.02: 1
    b_x(i) = (1-t) * (1-t) * p0_x + 2 * t * (1-t) * p1_x + t * t * p2_x;
    b_y(i) = (1-t) * (1-t) * p0_y + 2 * t * (1-t) * p1_y + t * t * p2_y;
    i = i + 1;
    %hold on;
    plot(b_x,b_y);

end
% plot(p0_x,p0_y,'.');

    