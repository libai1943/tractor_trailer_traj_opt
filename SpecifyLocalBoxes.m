function SpecifyLocalBoxes(PATH)
global params
x1 = PATH.path1.x;
y1 = PATH.path1.y;
x2 = PATH.path2.x;
y2 = PATH.path2.y;
x3 = PATH.path3.x;
y3 = PATH.path3.y;
x4 = PATH.path4.x;
y4 = PATH.path4.y;

AABB_1 = ExploreAABB(x1, y1);
AABB_2 = ExploreAABB(x2, y2);
AABB_3 = ExploreAABB(x3, y3);
AABB_4 = ExploreAABB(x4, y4);

if isfile('AABB'), delete('AABB'); end
fid = fopen('AABB', 'w');
for ii = 1 : params.nfe
    fprintf(fid, '%g 1 %.17g \r\n', ii, AABB_1(ii,1));
    fprintf(fid, '%g 2 %.17g \r\n', ii, AABB_1(ii,2));
    fprintf(fid, '%g 3 %.17g \r\n', ii, AABB_1(ii,3));
    fprintf(fid, '%g 4 %.17g \r\n', ii, AABB_1(ii,4));
    
    fprintf(fid, '%g 5 %.17g \r\n', ii, AABB_2(ii,1));
    fprintf(fid, '%g 6 %.17g \r\n', ii, AABB_2(ii,2));
    fprintf(fid, '%g 7 %.17g \r\n', ii, AABB_2(ii,3));
    fprintf(fid, '%g 8 %.17g \r\n', ii, AABB_2(ii,4));
    
    fprintf(fid, '%g 9 %.17g \r\n', ii, AABB_3(ii,1));
    fprintf(fid, '%g 10 %.17g \r\n', ii, AABB_3(ii,2));
    fprintf(fid, '%g 11 %.17g \r\n', ii, AABB_3(ii,3));
    fprintf(fid, '%g 12 %.17g \r\n', ii, AABB_3(ii,4));
    
    fprintf(fid, '%g 13 %.17g \r\n', ii, AABB_4(ii,1));
    fprintf(fid, '%g 14 %.17g \r\n', ii, AABB_4(ii,2));
    fprintf(fid, '%g 15 %.17g \r\n', ii, AABB_4(ii,3));
    fprintf(fid, '%g 16 %.17g \r\n', ii, AABB_4(ii,4));
end
fclose(fid);
end

function AABB = ExploreAABB(x, y)

for ii = 1 : length(x)
    xc = x(ii);
    yc = y(ii);
    lb = GetBoxVertexes(xc, yc);
    if (~any(lb))
        counter = 0;
        is_lb_nonzero = 0;
        while (~is_lb_nonzero)
            counter = counter + 1;
            assert(counter<=1000,'Cannot construct a local corridor.');
            for jj = 1 : 4
                switch jj
                    case 1
                        x_nudge = xc + counter * 0.01;
                        y_nudge = yc;
                    case 2
                        x_nudge = xc - counter * 0.01;
                        y_nudge = yc;
                    case 3
                        x_nudge = xc;
                        y_nudge = yc + counter * 0.01;
                    case 4
                        x_nudge = xc;
                        y_nudge = yc - counter * 0.01;
                end
                lb = GetBoxVertexes(x_nudge, y_nudge);
                if (any(lb))
                    is_lb_nonzero = 1;
                    xc = x_nudge;
                    yc = y_nudge;
                    break;
                end
            end
        end
    end
    AABB(ii,:) = [xc - lb(2), xc + lb(4), yc - lb(3), yc + lb(1)];
end
end

function lb = GetBoxVertexes(x, y)
global params
% up left down right
lb = zeros(1,4);
is_completed = zeros(1,4);
while (sum(is_completed) < 4)
    for ind = 1 : 4
        if (is_completed(ind))
            continue;
        end
        test = lb;
        if (test(ind) + params.unit_step > params.max_step)
            is_completed(ind) = 1;
            continue;
        end
        test(ind) = test(ind) + params.unit_step;
        if (IsCurrentEnlargementValid(x, y, test, lb, ind))
            lb = test;
        else
            is_completed(ind) = 1;
        end
    end
end
end

function is_valid = IsCurrentEnlargementValid(x, y, test, lb, ind)
switch ind
    case 1
        A = [x - lb(2), y + lb(1)];
        B = [x + lb(4), y + lb(1)];
        EA = [x - test(2), y + test(1)];
        EB = [x + test(4), y + test(1)];
        V_check = [A; B; EB; EA];
    case 2
        A = [x - lb(2), y + lb(1)];
        D = [x - lb(2), y - lb(3)];
        EA = [x - test(2), y + test(1)];
        ED = [x - test(2), y - test(3)];
        V_check = [A; D; ED; EA];
    case 3
        C = [x + lb(4), y - lb(3)];
        D = [x - lb(2), y - lb(3)];
        EC = [x + test(4), y - test(3)];
        ED = [x - test(2), y - test(3)];
        V_check = [C; D; ED; EC];
    case 4
        B = [x + lb(4), y + lb(1)];
        C = [x + lb(4), y - lb(3)];
        EB = [x + test(4), y + test(1)];
        EC = [x + test(4), y - test(3)];
        V_check = [C; B; EB; EC];
    otherwise
        is_valid = 0;
        return;
end
x_min = min(V_check(:,1));
x_max = max(V_check(:,1));
y_min = min(V_check(:,2));
y_max = max(V_check(:,2));

global params
if ((x_min < params.xmin + params.radius)||(x_max > params.xmax - params.radius)||(y_min < params.ymin + params.radius)||(y_max > params.ymax - params.radius))
    is_valid = 0;
    return;
end

ind_x_min = ceil((x_min - params.xmin) / params.dx) + 1;
ind_y_min = ceil((y_min - params.ymin) / params.dy) + 1;
ind_x_max = ceil((x_max - params.xmin) / params.dx) + 1;
ind_y_max = ceil((y_max - params.ymin) / params.dy) + 1;
ind_x = ind_x_min : ind_x_max;
ind_y = ind_y_min : ind_y_max;

if (any(any(params.dialated_map(ind_x, ind_y))))
    is_valid = 0;
    return;
end

is_valid = 1;
end