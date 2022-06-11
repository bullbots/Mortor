package frc.robot.util

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.ctre.phoenix.sensors.WPI_Pigeon2
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection
import com.ctre.phoenix.sensors.BasePigeonSimCollection
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants
import java.text.DecimalFormat

class DrivebaseSimFX(
    private val _leftMaster: WPI_TalonFX,
    private val _rightMaster: WPI_TalonFX,
    private val _pidgey: WPI_Pigeon2,
    private val _odometry: DifferentialDriveOdometry
) {
    private val _leftMasterSim: TalonFXSimCollection
    private val _rightMasterSim: TalonFXSimCollection
    private val _pidgeySim: BasePigeonSimCollection

    /**
     * Returns a 2D representation of the game field for dashboards.
     */
    private val field = Field2d()

    // For debug only
    val dff: (Double) -> String = TalonFXUtil.df::format

    //Simulation model of the drivetrain
    private val _driveSim = DifferentialDrivetrainSim(
        DCMotor.getFalcon500(2),  //2 Falcon 500s on each side of the drivetrain.
        Constants.DRIVETRAIN_GEAR_RATIO,  //Standard AndyMark Gearing reduction.
        2.1,  //MOI of 2.1 kg m^2 (from CAD model).
        26.5,  //Mass of the robot is 26.5 kg.
        Units.inchesToMeters(Constants.WHEEL_RADIUS_FT * 12.0),  //Robot uses 3" radius (6" diameter) wheels.
        Constants.TRACK_WIDTH_METERS,  //Distance between wheels is _ meters.
        // The standard deviations for measurement noise:
        // x and y:          0.001 m
        // heading:          0.001 rad
        // l and r velocity: 0.1   m/s
        // l and r position: 0.005 m
        null //VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //Uncomment this line to add measurement noise.
    )

    init {
        SmartDashboard.putData("Sim Field", field)
        _leftMasterSim = _leftMaster.simCollection
        _rightMasterSim = _rightMaster.simCollection
        _pidgeySim = _pidgey.simCollection
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

        // Advance the model by 20 ms.
        _driveSim.update(0.02)

        _leftMasterSim.setIntegratedSensorRawPosition(
            TalonFXUtil.distanceToNativeUnits(_driveSim.leftPositionMeters)
        )
        _leftMasterSim.setIntegratedSensorVelocity(
            TalonFXUtil.velocityToNativeUnits(_driveSim.leftVelocityMetersPerSecond)
        )
        _rightMasterSim.setIntegratedSensorRawPosition(
            TalonFXUtil.distanceToNativeUnits(_driveSim.rightPositionMeters)
        )
        _rightMasterSim.setIntegratedSensorVelocity(
            TalonFXUtil.velocityToNativeUnits(_driveSim.rightVelocityMetersPerSecond)
        )
        _pidgeySim.setRawHeading(_driveSim.heading.degrees)

        //Update other inputs to Talons
        _leftMasterSim.setBusVoltage(RobotController.getBatteryVoltage())
        _rightMasterSim.setBusVoltage(RobotController.getBatteryVoltage())

        // This will get the simulated sensor readings that we set
        // in the previous article while in simulation, but will use
        // real values on the robot itself.

        var leftDist = TalonFXUtil.nativeUnitsToDistanceMeters(_leftMaster.selectedSensorPosition)
        var rightDist =TalonFXUtil. nativeUnitsToDistanceMeters(_rightMaster.selectedSensorPosition)
//
//        _odometry.update(
//            _pidgey.rotation2d, leftDist, rightDist
//        )
//
//        leftDist = TalonFXUtil.nativeUnitsToDistanceFeet(_leftMaster.selectedSensorPosition)
//        rightDist = TalonFXUtil.nativeUnitsToDistanceFeet(_rightMaster.selectedSensorPosition)
//
        println("INFO: Pidgey: ${dff(_pidgey.rotation2d.degrees)}, Left master: ${dff(leftDist)}, Right master: ${dff(rightDist)}")
//
        field.robotPose = _odometry.poseMeters
    }
}
