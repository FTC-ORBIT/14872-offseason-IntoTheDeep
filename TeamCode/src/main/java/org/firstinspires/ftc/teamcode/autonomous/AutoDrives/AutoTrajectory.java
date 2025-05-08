    package org.firstinspires.ftc.teamcode.autonomous.AutoDrives;

    import static org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.AutoDrive.OrbitAutoDrive.moveRobot;

    import org.firstinspires.ftc.teamcode.OrbitUtils.Angle;
    import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
    import org.firstinspires.ftc.teamcode.robotSubSystems.SubSystemManager;
    import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.AutoDrivesAndAssist.AutoDrive.AutoDrive;

    public class AutoTrajectory {

        public enum TrajectoryType {
            NORMAL(),
            TIMED(),
            SPLINE(),
            TIMED_SPLINE();
        }


        private AutoTrajectoryPoint[] autoTrajectoryPoints;
        private final AutoDrive autoDrive = new AutoDrive(1, Angle.degToRad(3));
        private Pose2D[] targetPoses;
        private int autoTrajectoryPointIndex = 0;
        private final TrajectoryType type;

        public AutoTrajectory(final TrajectoryType type, final AutoTrajectoryPoint... autoTrajectoryPoints) {
            AutoTrajectoryPoint.adjustPointByAllianceColor(autoTrajectoryPoints);
            autoDrive.init();
            this.autoTrajectoryPoints = autoTrajectoryPoints;
            this.type = type;
            autoTrajectoryPointIndex = 0;

            targetPoses = new Pose2D[autoTrajectoryPoints.length];
            for (int i = 0; i < autoTrajectoryPoints.length; i++) {
                targetPoses[i] = autoTrajectoryPoints[i].targetPose;
            }
        }


        public void runTrajectory() {
            switch (type) {
                case NORMAL:
                    autoDrive.trajectory(targetPoses);
                    break;
                case TIMED:
                    autoDrive.timedTrajectory(targetPoses);
                    break;
                case SPLINE:
                    autoDrive.splineTrajectory(targetPoses);
                    break;
                case TIMED_SPLINE:
                    autoDrive.timedSplineTrajectory(targetPoses);
                    break;
            }

            autoTrajectoryPointIndex = autoDrive.getPoseIndex();
            SubSystemManager.setSubsystemToState(autoTrajectoryPoints[autoTrajectoryPointIndex].wantedState, false);

        }

        public boolean arrived() {
            return autoDrive.arrivedToPose(targetPoses[autoTrajectoryPointIndex]); // add yourSystemsHere
        }

        public AutoTrajectoryPoint getCurrentTrajectoryPoint() {
            return autoTrajectoryPoints[autoTrajectoryPointIndex];
        }

        public int getAutoTrajectoryPointIndex() {
            return autoTrajectoryPointIndex;
        }

    }
