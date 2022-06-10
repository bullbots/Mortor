package frc.robot.util

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.ctre.phoenix.sensors.WPI_Pigeon2
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection
import com.ctre.phoenix.sensors.BasePigeonSimCollection
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

class DrivebaseSimFX(
    private val _leftMaster: WPI_TalonFX,
    private val _rightMaster: WPI_TalonFX,
    private val _pidgey: WPI_Pigeon2
) {
    private val _leftMasterSim: TalonFXSimCollection
    private val _rightMasterSim: TalonFXSimCollection
    private val _pidgeySim: BasePigeonSimCollection

    /**
     * Returns a 2D representation of the game field for dashboards.
     */
    private val field = Field2d()
    private val _odometry: DifferentialDriveOdometry

    //These numbers are an example AndyMark Drivetrain with some additional weight.  This is a fairly light robot.
    //Note you can utilize results from robot characterization instead of theoretical numbers.
    //https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html#introduction-to-robot-characterization
    private val kCountsPerRev = 2048 //Encoder counts per revolution of the motor shaft.
    private val kSensorGearRatio =
        1.0 //Gear ratio is the ratio between the *encoder* and the wheels.  On the AndyMark drivetrain, encoders mount 1:1 with the gearbox shaft.
    private val kGearRatio =
        10.71 //Switch kSensorGearRatio to this gear ratio if encoder is on the motor instead of on the gearbox.
    private val kWheelRadiusInches = 3.0
    private val k100msPerSecond = 10

    //Simulation model of the drivetrain
    private val _driveSim = DifferentialDrivetrainSim(
        DCMotor.getFalcon500(2),  //2 Falcon 500s on each side of the drivetrain.
        kGearRatio,  //Standard AndyMark Gearing reduction.
        2.1,  //MOI of 2.1 kg m^2 (from CAD model).
        26.5,  //Mass of the robot is 26.5 kg.
        Units.inchesToMeters(kWheelRadiusInches),  //Robot uses 3" radius (6" diameter) wheels.
        0.546,  //Distance between wheels is _ meters.
        // The standard deviations for measurement noise:
        // x and y:          0.001 m
        // heading:          0.001 rad
        // l and r velocity: 0.1   m/s
        // l and r position: 0.005 m
        null //VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //Uncomment this line to add measurement noise.
    )

    /**
     * Creates a new drivebase simualtor using Falcon 500 motors.
     *
     * @param leftMaster the left master Falcon
     * @param rightMaster the right master Falcon
     * @param pidgey the Pigeon IMU
     */
    init {
        SmartDashboard.putData("Sim Field", field)
        _leftMasterSim = _leftMaster.simCollection
        _rightMasterSim = _rightMaster.simCollection
        _pidgeySim = _pidgey.simCollection

        // Creating odometry object. Here,
        // our starting pose is 5 meters along the long end of the field and in the
        // center of the field along the short end, facing forward.
        _odometry = DifferentialDriveOdometry(_pidgey.rotation2d)
    }

    /**
     * Runs the drivebase simulator.
     */
    fun run() {
        // Set the inputs to the system. Note that we need to use
        // the output voltage, NOT the percent output.
        _driveSim.setInputs(
            _leftMasterSim.motorOutputLeadVoltage,
            -_rightMasterSim.motorOutputLeadVoltage
        ) //Right side is inverted, so forward is negative voltage

        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        _driveSim.update(0.02)

        // Update all of our sensors.
        _leftMasterSim.setIntegratedSensorRawPosition(
            distanceToNativeUnits(
                _driveSim.leftPositionMeters
            )
        )
        _leftMasterSim.setIntegratedSensorVelocity(
            velocityToNativeUnits(
                _driveSim.leftVelocityMetersPerSecond
            )
        )
        _rightMasterSim.setIntegratedSensorRawPosition(
            distanceToNativeUnits(
                -_driveSim.rightPositionMeters
            )
        )
        _rightMasterSim.setIntegratedSensorVelocity(
            velocityToNativeUnits(
                -_driveSim.rightVelocityMetersPerSecond
            )
        )
        _pidgeySim.setRawHeading(_driveSim.heading.degrees)

        //Update other inputs to Talons
        _leftMasterSim.setBusVoltage(RobotController.getBatteryVoltage())
        _rightMasterSim.setBusVoltage(RobotController.getBatteryVoltage())

        // This will get the simulated sensor readings that we set
        // in the previous article while in simulation, but will use
        // real values on the robot itself.

        val leftNativeUnits = nativeUnitsToDistanceMeters(_leftMaster.selectedSensorPosition)
        val rightNativeUnits = nativeUnitsToDistanceMeters(_rightMaster.selectedSensorPosition)

        _odometry.update(
            _pidgey.rotation2d, leftNativeUnits, rightNativeUnits
        )

        println("INFO: Pidgey: ${_pidgey.rotation2d.degrees} Left master: ${leftNativeUnits}, Right master: ${rightNativeUnits}")

        field.robotPose = _odometry.poseMeters
    }

    // Helper methods to convert between meters and native units
    private fun distanceToNativeUnits(positionMeters: Double): Int {
        val wheelRotations =
            positionMeters / (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches))
        val motorRotations = wheelRotations * kSensorGearRatio
        return (motorRotations * kCountsPerRev).toInt()
    }

    private fun velocityToNativeUnits(velocityMetersPerSecond: Double): Int {
        val wheelRotationsPerSecond =
            velocityMetersPerSecond / (2 * Math.PI * Units.inchesToMeters(
                kWheelRadiusInches
            ))
        val motorRotationsPerSecond = wheelRotationsPerSecond * kSensorGearRatio
        val motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond
        return (motorRotationsPer100ms * kCountsPerRev).toInt()
    }

    private fun nativeUnitsToDistanceMeters(sensorCounts: Double): Double {
        val motorRotations = sensorCounts / kCountsPerRev
        val wheelRotations = motorRotations / kSensorGearRatio
        return wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches))
    }

}