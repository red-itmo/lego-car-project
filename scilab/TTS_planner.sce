// specification
k_max = 1.5;
k_max = 0.9 * k_max;
L = 0.165;
v_min = 0.05;
Omega_max = 10;
sigma_max = Omega_max / L;
alpha_max = sigma_max / v_min;


function [descr, _length, robot_poses] = plannerTTS(start_pose, goal_pose, v_des, _mode)

    // swap start and goal poses if necessary
    if argn(2) == 4 & _mode == 'r' then
        init_pose = goal_pose;
        goal_pose = start_pose;
    else
        init_pose = start_pose;
    end

    // finding of init coordinates in the frame where goal point has (0,0,0) coordinates
    pose(1,:) = transformCoords(goal_pose, init_pose, 'b');

    // check for boundaries
    if pose(1,3) > %pi then
        pose(1,3) = pose(1,3) - 2*%pi;
    elseif pose(1,3) < -%pi then
        pose(1,3) = pose(1,3) + 2*%pi;
    end

    // handy aliases
    x1 = pose(1,1);
    y1 = pose(1,2);
    theta_1 = pose(1,3);

    // chosing of delta and k
    something_goes_wrong = %t;
    while something_goes_wrong

        // choosing of delta_1
        if 0 <= theta_1 & theta_1 < %pi then
            flag = %f;
            while flag <> %t
                flag = %t;
                delta(1,:) = grand(1, 1, "unf", -%pi, %pi - theta_1);
                if delta(1) == 0 then
                    flag = %f;
                end
            end
        elseif -%pi <= theta_1 & theta_1 < 0 then
            flag = %f;
            while flag <> %t
                flag = %t;
                delta(1,:) = grand(1, 1, "unf", -%pi - theta_1, %pi);
                if delta(1) == 0  then
                    flag = %f;
                end
            end
        end

        // choosing of k_1
        flag = %f;
        while flag <> %t
            flag = %t;
            k(1,:) = grand(1, 1, "unf", -k_max, k_max);
            if k(1) == 0 then
                flag = %f; 
            end
        end

        // choosing of R1 and R2
        R(1,:) = grand(1, 1, "unf", 0, 1);
        R(2,:) = grand(1, 1, "unf", 0, 1);

        // calculating of other deltas
        delta(2,:) = -delta(1) - theta_1;
        delta_CC = ((1 - R) .* delta) / 2;
        delta_C = R .* delta;

        // calculating of k_2
        k(2,:) = B(-2*delta_CC(2), -delta_C(2)) / (D([2 * delta_CC(1), delta_C(1)], theta_1) / k(1) + y1);
        if abs(k(2)) > k_max then
            continue
        end
        
        // calculating of gamma_3*s_3
        pose(3, 1) = -(A(-2*delta_CC(2), -delta_C(2)) / k(2) - C([2 * delta_CC(1), delta_C(1)], theta_1) / k(1) - x1);
        gamm(3,:) = -sign(pose(3, 1));
        s_end(3,:) = abs(pose(3, 1));
        pose(3, 2) = 0;
        
        // calculating of gamma, s and alpha
        for i = 1:2
            gamm(i,:) = sign(delta_CC(i)) * sign(k(i));
            s_end(i,:) = abs(2 * delta_CC(i) / k(i));
            alpha(i,:) = gamm(i) * sign(k(i)) * abs(k(i) / s_end(i));
            if sigma_max / abs(alpha(i)) < v_des then
                v(i,:) = sigma_max / abs(alpha(i));
            else
                v(i,:) = v_des;
            end
        end
    
        // final checking
        something_goes_wrong  = %f;
        for i = 1:2
            if alpha(i) > alpha_max then
                something_goes_wrong  = %t;
            end
        end
    
    end

    // outputs calculation

    // path's description in list form
    if argn(2) == 4 & _mode == 'r' then
        descr = list();
        descr($+1) = list("StraightLine", [0, 0, 0], [pose(3, 1:2), 0], s_end(3), v_des);
        for i = 2:-1:1
            descr($+1) = list("ClothoidLine", [0, 0, 0], -gamm(i), -alpha(i), s_end(i), 'in', v(i));
            if -delta_C(i) > 0 then
                direction = 'ccw';
            else
                direction = 'cw';
            end
            descr($+1) = list("CircleLine", [0, 1/k(i), 0], [0, 0, 0], [sin(-delta_C(i)) / k(i), (1 - cos(-delta_C(i))) / k(i), -delta_C(i)], direction, v(i));
            descr($+1) = list("ClothoidLine", [0, 0, 0], -gamm(i), alpha(i), s_end(i), 'out', v(i));
        end
    else
        descr = list();
        for i = 1:2
            descr($+1) = list("ClothoidLine", [0, 0, 0], gamm(i),  alpha(i), s_end(i), 'in', v(i));
            if delta_C(i) > 0 then
                direction = 'ccw';
            else
                direction = 'cw';
            end
            descr($+1) = list("CircleLine", [0, 1/k(i), 0], [0, 0, 0], [sin(delta_C(i)) / k(i), (1 - cos(delta_C(i))) / k(i), delta_C(i)], direction, v(i));
            descr($+1) = list("ClothoidLine", [0, 0, 0], gamm(i), -alpha(i), s_end(i), 'out', v(i));
        end
        descr($+1) = list("StraightLine", [pose(3, 1:2), 0], [0, 0, 0], s_end(3), v_des);
    end

    // choosing of appropriate start point and some plotting
    if argn(2) == 4 & _mode == 'r' then
        aux_pose = [0,0,0];
        //path's length
//        _length = sum(s_end) + sum(s_end([2, 4, 5, 7]));
    else
        aux_pose = pose(1,:);
        //path's length
        _length = sum(s_end) - s_end(3);
    end

    // calculation arrays of data
    robot_poses = [];
    for i = 1:7
        poses = calcPathElementPoints(descr(i), aux_pose, 0.05);

        if descr(i)(1) == "ClothoidLine" then
            descr(i)(2) = transformCoords(goal_pose, transformCoords(aux_pose, descr(i)(2)));
        elseif descr(i)(1) == "CircleLine" then
            descr(i)(2) = transformCoords(goal_pose, transformCoords(aux_pose, descr(i)(2)));
            descr(i)(3) = transformCoords(goal_pose, transformCoords(aux_pose, descr(i)(3)));
            descr(i)(4) = transformCoords(goal_pose, transformCoords(aux_pose, descr(i)(4)));
        elseif descr(i)(1) == "StraightLine" then
            descr(i)(2) = transformCoords(goal_pose, transformCoords(aux_pose, descr(i)(2)));
            descr(i)(3) = transformCoords(goal_pose, transformCoords(aux_pose, descr(i)(3)));
        end

        aux_pose = poses($, :);
        add_poses = transformCoords(goal_pose, poses);
        robot_poses = [robot_poses; add_poses];
        plot2d(add_poses(:,1), add_poses(:,2), i);
    end

    // choosing of appropriate legend
    if argn(2) == 4 & _mode == 'r' then
        legend("Прямая", "Клотоида", "Дуга", "Клотоида", "Клотоида", "Дуга", "Клотоида", -4);
    else
        legend("Клотоида", "Дуга", "Клотоида", "Клотоида", "Дуга", "Клотоида", "Прямая", -4);
    end

endfunction
