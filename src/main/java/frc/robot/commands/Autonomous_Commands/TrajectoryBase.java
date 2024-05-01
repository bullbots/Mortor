package frc.robot.commands.Autonomous_Commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainFalcon;
import frc.robot.util.TrajectoryManager;

public class TrajectoryBase extends Command {

    private Timer timer = new Timer();

    private RamseteController ramsete = new RamseteController();

    private int loopIdx = 0;

    private Trajectory trajectory;

    private boolean isInitialized = false;

    private DrivetrainFalcon drivetrain;

    private final String trajectoryName;

    private boolean resetGyro = true;

    public TrajectoryBase(DrivetrainFalcon drivetrain, String trajectoryName, boolean resetGyro) {
        this.drivetrain = drivetrain;
        this.trajectoryName = trajectoryName;
        this.resetGyro = resetGyro;
        addRequirements(drivetrain);
    }

    private void getTrajectory() {
        if (trajectory == null && TrajectoryManager.getTrajectories() != null) {
            System.out.println("INFO: getting trajectory: $trajectoryName");
            trajectory = TrajectoryManager.getTrajectories().get(trajectoryName);
        }
    }

    private void initializeTrajectory() {
        if (!isInitialized) {
            System.out.println("INFO: initializing trajectory");
            getTrajectory();
            if (trajectory != null) {
                drivetrain.resetOdometry(trajectory.getInitialPose());
                isInitialized = true;
                timer.reset();
                timer.start();
            }
        }
    }

    @Override
    public void initialize() {
        if (resetGyro) {
            drivetrain.resetGyro();
        }

        ramsete.setEnabled(true);
//        drivetrain.setOdometryDirection(isBackwards)
    }
    
    @Override
    public void execute() {
        var elapsed = timer.get();

        initializeTrajectory();

        if (!isInitialized) {
            return;
        }

        var reference = trajectory.sample(elapsed);

        var speeds = ramsete.calculate(drivetrain.getPose(), reference);

        var ramseteSpeed = speeds.vxMetersPerSecond;
        var ramseteRot = speeds.omegaRadiansPerSecond;

        var normalizedRamseteSpeed = ramseteSpeed / Constants.FTPERSEC_TOPSPEED;
        var normalizedRamseteRot = -ramseteRot / Constants.FTPERSEC_TOPSPEED * Constants.WHEEL_RADIUS_FT;

//        var direction = if (isBackwards) -1.0 else 1.0
        drivetrain.arcadeDrive(normalizedRamseteSpeed, normalizedRamseteRot, false);

        var tPose = reference.poseMeters;
        var tX = tPose.getX();
        var tY = tPose.getY();
        var tRotation = tPose.getRotation().getDegrees();

        var aPose = drivetrain.getPose();
        var aX = aPose.getX();
        var aY = aPose.getY();
        var aRotation = aPose.getRotation().getDegrees();

//        loopIdx++
//        if (loopIdx == 10) {
//            loopIdx = 0
//
//            SmartDashboard.putNumber("Ramsete Speed - Normalized", normalizedRamseteSpeed)
//            SmartDashboard.putNumber("Ramsete Rot - Normalized", normalizedRamseteRot)
//
//            SmartDashboard.putNumber("Pose X - Trajectory", tX)
//            SmartDashboard.putNumber("Pose Y - Trajectory", tY)
//            SmartDashboard.putNumber("Pose R - Trajectory", tRotation)
//
//            SmartDashboard.putNumber("Pose X - Actual", aX)
//            SmartDashboard.putNumber("Pose Y - Actual", aY)
//            SmartDashboard.putNumber("Pose R - Actual", aRotation)
//        }

        DrivetrainFalcon.m_fieldSim.setRobotPose(reference.poseMeters);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("INFO: trajectory end: $trajectoryName");
        drivetrain.setOdometryDirection(false);
        isInitialized = false;
    }

    @Override
    public boolean isFinished() {
        if (trajectory != null) {
            System.out.println("INFO: checking isFinished: $trajectoryName");
            return timer.get() > trajectory.getTotalTimeSeconds();
        }
        return true;
    }
}