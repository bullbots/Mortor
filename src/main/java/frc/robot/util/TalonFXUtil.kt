package frc.robot.util

import edu.wpi.first.math.util.Units
import frc.robot.Constants
import java.text.DecimalFormat

object TalonFXUtil {

    //These numbers are an example AndyMark Drivetrain with some additional weight.  This is a fairly light robot.
    //Note you can utilize results from robot characterization instead of theoretical numbers.
    //https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html#introduction-to-robot-characterization

    const val kCountsPerRev = 2048 //Encoder counts per revolution of the motor shaft.
//    val kWheelRadiusInches = 4.0
    val k100msPerSecond = 10

    val df = DecimalFormat("#.##")

    // Helper methods to convert between meters and native units
    fun distanceToNativeUnits(positionMeters: Double): Int {
        val wheelRotations = positionMeters / Constants.PI_WHEEL_DIAMETER_METERS
        val motorRotations = wheelRotations * Constants.DRIVETRAIN_GEAR_RATIO
        return (motorRotations * kCountsPerRev).toInt()
    }

    fun velocityToNativeUnits(velocityMetersPerSecond: Double): Int {
        val wheelRotationsPerSecond = velocityMetersPerSecond / Constants.PI_WHEEL_DIAMETER_METERS
        val motorRotationsPerSecond = wheelRotationsPerSecond * Constants.DRIVETRAIN_GEAR_RATIO
        val motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond
        return (motorRotationsPer100ms * kCountsPerRev).toInt()
    }

    fun nativeUnitsToDistanceMeters(sensorCounts: Double): Double {
        val motorRotations = sensorCounts / kCountsPerRev
        val wheelRotations = motorRotations / Constants.DRIVETRAIN_GEAR_RATIO
        return wheelRotations *  Constants.PI_WHEEL_DIAMETER_METERS
    }

    fun nativeUnitsToDistanceFeet(sensorCounts: Double): Double {
        val motorRotations = sensorCounts / kCountsPerRev
        val wheelRotations = motorRotations / Constants.DRIVETRAIN_GEAR_RATIO
        return wheelRotations *  Constants.PI_WHEEL_DIAMETER_FT
    }
}