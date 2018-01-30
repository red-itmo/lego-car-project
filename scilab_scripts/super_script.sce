
exec("functions_for_FI.sci");
exec("for_plotting.sce");
exec("EES_planner.sce");

args = sciargs();


start_pose = [strtod(args(5:7))];
goal_pose = [strtod(args(5:7))];
v_des = strtod(args(8));
disp(start_pose - 1);
[descr, _length, robot_poses] = plannerEES(start_pose, goal_pose, v_des);
disp(args(6))

fprintfMat("XYAngle.txt", real(robot_poses));
fprintfMat("length.txt", _length);

exit()
