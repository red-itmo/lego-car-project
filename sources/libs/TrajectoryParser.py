class TrajectoryParser:
    self.init_Pose = NULL

    def __init__(self, initP):
        self.init_Pose = Pose(initP[0], initP[1], initP[2])

    def setInitPose(self, initP):
        self.init_Pose = Pose(initP[0], initP[1], initP[2])

    def transformPoint(self, point):
        theta = self.init_Pose.angle
        x_init = self.init_Pose.point.x
        y_init = self.init_Pose.point.y
        frame_x = point.x
        frame_y = point.y

        x = -sin(theta) * frame_y + cos(theta) * frame_x + sin(theta) * y_init - cos(theta) * x_init
        y = -cos(theta) * frame_y - sin(theta) * frame_x + cos(theta) * y_init + sin(theta) * x_init
        return Point(x, y)

    def transformPose(self, pose):
        theta = self.init_Pose.angle
        angle = -pose.angle
        x_init = self.init_Pose.point.x
        y_init = self.init_Pose.point.y
        frame_x = pose.point.x
        frame_y = pose.point.y

        x = -sin(theta) * frame_y + cos(theta) * frame_x + sin(theta) * y_init - cos(theta) * x_init
        y = -cos(theta) * frame_y - sin(theta) * frame_x + cos(theta) * y_init + sin(theta) * x_init
        return Pose(x, y, angle)

    def decode_trajectory(self, path):
        full_trajectory = []
        for traj in path:
            print(traj)
            if traj[0] == 'ClothoidLine':
                f_pose = Pose(traj[1][0, 0], traj[1][0, 1], traj[1][0, 2])
                pose = self.transformPose(f_pose)
                t = ClothoidLine(pose, traj[2][0], -traj[3][0], traj[4][0], traj[6][0], type=traj[5])
                full_trajectory.append(t)
            elif traj[0] == 'CircleLine':
                f_centerPoint = Point(traj[1][0, 0], traj[1][0, 1])
                f_startPoint = Point(traj[2][0, 0], traj[2][0, 1])
                f_endPoint = Point(traj[3][0, 0], traj[3][0, 1])
                vel = asscalar(traj[5])
                dir = traj[4]

                centerPoint = self.transformPoint(f_centerPoint)
                startPoint = self.transformPoint(f_startPoint)
                endPoint = self.transformPoint(f_endPoint)

                if dir == 'cw':
                    t = CircleLine(centerPoint, startPoint, endPoint, 'ccw', v=vel)
                else:
                    t = CircleLine(centerPoint, startPoint, endPoint, 'cw', v=vel)
                full_trajectory.append(t)

            elif traj[0] == 'StraightLine':
                f_startPoint = Point(traj[1][0, 0], traj[1][0, 1])
                f_endPoint = Point(traj[2][0, 0], traj[2][0, 1])
                vel = traj[4]

                startPoint = self.transformPoint(f_startPoint)
                endPoint = self.transformPoint(f_endPoint)

                t = StraightLine(startPoint, endPoint, v=vel)
                full_trajectory.append(t)
        return full_trajectory
