package frc.robot.subsystems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDControllerDebug
import java.util.function.DoubleSupplier

class PIDDebugSubsystem(private val pidController: PIDControllerDebug, private val measurementSource: DoubleSupplier,
                        private val setpointSource: DoubleSupplier) : SubsystemBase() {

 override fun periodic() {
//  pidController.calculateDebug(measurementSource.asDouble, setpointSource.asDouble, true)
 }

}