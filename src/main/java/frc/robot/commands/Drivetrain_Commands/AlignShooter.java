package frc.robot.commands.Drivetrain_Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainFalcon;
import java.util.function.DoubleSupplier;

public class AlignShooter extends Command {

    private double loopIdx = 0;
    private double delta = 0.0;

    private DoubleSupplier measurementSource;
    private DoubleSupplier setpointSource;
    private DrivetrainFalcon drivetrain;

    public AlignShooter(DoubleSupplier measurementSource,
                        DoubleSupplier setpointSource,
                        DrivetrainFalcon drivetrain) {
        this.measurementSource = measurementSource;
        this.setpointSource = setpointSource;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("INFO: AlignShooter.initialize");
        delta = MathUtil.inputModulus(setpointSource.getAsDouble() - measurementSource.getAsDouble(), -180.0, 180.0);
    }

    @Override
    public void execute() {

        delta = MathUtil.inputModulus(setpointSource.getAsDouble() - measurementSource.getAsDouble(), -180.0, 180.0);

        double output = 0.0;

        if (Math.abs(delta) > 45) {
            output = Math.signum(delta) * 0.575;
        } else if(Math.abs(delta) > 30) {
            output = Math.signum(delta) * 0.375;
        } else if(Math.abs(delta) > 1) {
            output = Math.signum(delta) * 0.26;
        }
        drivetrain.drive(0.0, -output);

//        // Debugging values
//        loopIdx++
//        if (loopIdx == 10) {
//            loopIdx = 0
//            println("INFO: Yaw: ${m_measurement.asDouble}, Heading: ${m_setpoint.asDouble}, PidOut: $pidOut, FF: $ff, TotalOut: ${pidOut+ff}")
//            println("INFO: Yaw: ${m_measurement.asDouble}, Heading: ${m_setpoint.asDouble}, Delta: $delta")
//            println("INFO: Yaw: ${m_measurement.asDouble}, Heading: ${m_setpoint.asDouble}, Output value: ${pidOut + ff}")
//
//        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(delta) < 1 && Math.abs(drivetrain.getVelocities()[0]) < 0.015;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("INFO: AlignShooter.end");
        drivetrain.stop();
    }
}