
function [x1, y1, x2, y2] = twoClothoids(gamm, s_end, alpha)

    // preprocessing
    step = 0.05*s_end;

    // first clothoid calculation
    i = 1;
    for s = 0:step:s_end
        x1(i, :) = xCoord(gamm, alpha, s);
        y1(i, :) = yCoord(gamm, alpha, s);
        i = i + 1;
    end

    // HT matrices calculation
    theta = alpha * s_end^2 / 2;
    T1 = [cos(theta), -sin(theta), x1($); sin(theta), cos(theta), y1($); 0, 0, 1];
    T2 = [[cos(-theta), -sin(-theta), -x1($); sin(-theta), cos(-theta), y1($); 0, 0, 1]];

    // second clothoid calcuation
    i = 1;
    alpha = -alpha;
    for s = 0:step:s_end
        x2_help(i,:) = xCoord(-gamm, alpha, (s_end - s));
        y2_help(i,:) = yCoord(-gamm, alpha, (s_end - s));
        new_cords = T1 * inv(T2) * [x2_help(i); y2_help(i); 1];
        x2(i,:) = new_cords(1);
        y2(i,:) = new_cords(2);
        i = i + 1;
    end

endfunction


function poses = calcClothoidPoints(gamm, alpha, s_end, typ, step)

    // preprocessing
    step = 0.05*s_end;

    if typ == "in" then
        i = 1;
        for s = 0:step:s_end
            poses(i, :) = [xCoord(gamm, alpha, s), yCoord(gamm, alpha, s), 0.5 * alpha * s^2];
            i = i + 1;
        end
    else
        origin_pose = [xCoord(-gamm, alpha, s_end), yCoord(-gamm, alpha, s_end), 0.5 * alpha * s_end^2];
        i = 1;
        for s = 0:step:s_end
            aux_pose = [xCoord(-gamm, alpha, s_end - s), yCoord(-gamm, alpha, s_end - s), 0.5 * alpha * (s_end - s)^2];
            poses(i, :) = transformCoords(origin_pose, aux_pose, 'b');
            i = i + 1;
        end
    end
endfunction


function poses = calcArcPoints(delta, k, step)

    i = 1;
    for d = 0:step:delta
        poses(i, :) = [sin(d) / k, (1 - cos(d)) / k, d]
        i = i + 1;
    end

endfunction


function poses = calcStraightLinePoints(pose_0, pose_1, s_end, step)

    alpha = atan(pose_1(2) - pose_0(2), pose_1(1) - pose_0(1));
    i = 1;
    for s = 0:step:s_end
        poses(i, :) = [s*cos(alpha),  s*sin(alpha), pose_0(3)]
        i = i + 1;
    end

endfunction


function new_poses = transformCoords(origin_pose, old_poses, direction)

    T = [cos(origin_pose(3)), -sin(origin_pose(3)), origin_pose(1);
        sin(origin_pose(3)),  cos(origin_pose(3)), origin_pose(2);
        0, 0, 1];

    // 'b' means 'backward'
    if argn(2) == 3 & direction == "b" then
        T = inv(T);
        new_poses(:,3) = old_poses(:,3) - origin_pose(3);
    else
        new_poses(:,3) = old_poses(:,3) + origin_pose(3);
    end
    aux_poses = T * [old_poses(:, 1:2)'; ones(old_poses(:,1)')];
    new_poses(:, [1,2]) = aux_poses([1,2],:)';

endfunction


function poses = calcPathElementPoints(element, start_pose, step)

    if element(1) == "ClothoidLine" then
        aux_poses = calcClothoidPoints(element(2), element(3), element(4), element(5), step);
    elseif element(1) == "CircleLine" then
        aux_poses = calcArcPoints(element(2), element(3), element(4), element(5), step); //needs to be fixed
    elseif element(1) == "StraightLine" then
        aux_poses = calcStraightLinePoints(element(2), element(3), element(4), step);
    end
    poses = transformCoords(start_pose, aux_poses);
endfunction
