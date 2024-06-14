package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDControllerDebug;

import java.util.function.DoubleSupplier;

public class PIDDebugSubsystem extends SubsystemBase {

    public PIDDebugSubsystem(PIDControllerDebug pidController,
                             DoubleSupplier measurementSource,
                             DoubleSupplier setpointSource) {

    }

    @Override
    public void periodic() {
//  pidController.calculateDebug(measurementSource.asDouble, setpointSource.asDouble, true)
    }
}