package frc.robot

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrajectoryConfig
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import edu.wpi.first.networktables.EntryListenerFlags
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.Autonomous_Commands.TrajectoryBase
import frc.robot.commands.Climber_Commands.AutoClimber
import frc.robot.commands.Climber_Commands.ManualClimber
import frc.robot.commands.Drivetrain_Commands.AlignShooter
import frc.robot.commands.Drivetrain_Commands.DriveForDistanceCommand
import frc.robot.commands.Drivetrain_Commands.DriveForTimeCommand
import frc.robot.commands.Drivetrain_Commands.JoystickDrive
import frc.robot.commands.Intake_Commands.*
import frc.robot.commands.Shooter_Commands.ShooterCargos
import frc.robot.commands.Shooter_Commands.ShooterGroup
import frc.robot.subsystems.*
import frc.robot.util.NavX
import frc.robot.util.PIDControllerDebug
import java.util.concurrent.atomic.AtomicReference

class RobotContainer {

    // Any static variables need to go in the companion object
    companion object {
        private val stick = Joystick(0)
        private val button1 = JoystickButton(stick, 1)
        private val button2 = JoystickButton(stick, 2)
        private val button3 = JoystickButton(stick, 3)
        private val button4 = JoystickButton(stick, 4)
        private val button5 = JoystickButton(stick, 5)
        private val button6 = JoystickButton(stick, 6)
        private val button7 = JoystickButton(stick, 7)
        private val button8 = JoystickButton(stick, 8)
        private val button9 = JoystickButton(stick, 9)
        private val button10 = JoystickButton(stick, 10)
        private val button11 = JoystickButton(stick, 11)

        private val coStick = Joystick(1)
        private val coButton1 = JoystickButton(coStick, 1)
        private val coButton2 = JoystickButton(coStick, 2)
        private val coButton3 = JoystickButton(coStick, 3)
        private val coButton4 = JoystickButton(coStick, 4)
        private val coButton5 = JoystickButton(coStick, 5)
        private val coButton6 = JoystickButton(coStick, 6)
        private val coButton7 = JoystickButton(coStick, 7)
        private val coButton8 = JoystickButton(coStick, 8)
        private val coButton9 = JoystickButton(coStick, 9)
        private val coButton10 = JoystickButton(coStick, 10)
        private val coButton11 = JoystickButton(coStick, 11)

        private val pathColor = AtomicReference<Color>(Color.UNLOADED)
        var pathLetter = AtomicReference<Letter>(Letter.UNLOADED)

    }

    // Subsystems
    private val drivetrain = DrivetrainFalcon()
    private val intake = Intake()
    private val shooter = Shooter()
    private val climber = Climber()

    // Util
    private val pidController = PIDControllerDebug(0.008, 0.0008, 0.0)
    private val pidDebugger = PIDDebugSubsystem(pidController, { -imu.angle }, drivetrain::calcHeading)
    private val imu = NavX()

    private var loopIdx = 0
    private var trajectory: Trajectory

    var m_chooser = SendableChooser<Command>()

    enum class Color(value: Int) {
        UNLOADED(0),
        RED(1),
        BLUE(2);

        private var value: Int? = null
        companion object {
            private var map = HashMap<Int, Color>()

            // In replace of static {}
            fun addColor() {
                for (color in Color.values()) {
                    map[color.value!!.toInt()] = color
                }
            }

            fun valueOf(color: Int): Color { return map[color]!! }
        }

        fun getValue(): Int { return value!! }
    }

    enum class Letter(value: Int) {
        UNLOADED(0),
        A(1),
        B(2);

        private var value: Int? = null

        companion object {

            private val map = HashMap<Int, Letter>()

            fun addLetter() {
                for (letter in Letter.values()) {
                    map[letter.value!!] = letter
                }
            }

            fun valueOf(letter: Int): Letter { return map[letter]!! }



        }

        fun getValue(): Int { return value!! }

    }

    /**
     * The container for the robot. Contains subsystems, IO Devices, and commands.
     */
    init {
        DriverStation.silenceJoystickConnectionWarning(true)
        // Configure the button bindings
        configureButtonBindings()

        // initializeTrajectory must come before configureButtonBindings

        drivetrain.defaultCommand = JoystickDrive(
            drivetrain,
            { -stick.y * if (button3.get()) -1.0 else 1.0 },  // Because Negative Y is forward on the joysticks
            { stick.x }
        ) { (stick.z - 1) / -2.0 }

//        intake.defaultCommand = HoldArmCommand(intake)

        initializeAutonomousOptions()

        pidController.setTolerance(0.5)
        pidController.setIntegratorRange(-0.2, 0.2)
        pidController.enableContinuousInput(-180.0, 180.0)

        trajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(2.0, 2.0, Rotation2d()),
            listOf<Translation2d>(),
            Pose2d(6.0, 4.0, Rotation2d()),
            TrajectoryConfig(2.0, 2.0)
        )
        initializeStaticShooterVel()
        initializePIDControllerValue()

        drivetrain.resetEncoders()
        imu.reset()
    }

    /**
     * Adds the staticShooter velocity values to the SmartDashboard.
     * You are able to adjust the value inside the SmartDashboard to change velocity
     */
    private fun initializeStaticShooterVel() { SmartDashboard.putNumber("StaticShooter", 0.46) }

    private fun initializePIDControllerValue() {
        SmartDashboard.putNumber("PID P Value", 0.01)
        SmartDashboard.putNumber("PID I Value", 0.005)
        SmartDashboard.putNumber("PID D Value", 0.0)
    }


    private fun initializeAutonomousOptions()
    {
        // Add commands to the autonomous command chooser
        m_chooser.setDefaultOption("Leave Tarmac, Intake, and Shoot",
            SequentialCommandGroup(
                IntakeCargos(intake, 0.2, 0.6, shooter).withTimeout(0.5),
                ParallelDeadlineGroup(
                    DriveForDistanceCommand(drivetrain, 0.15, 9.0), //Distance is 9
                    IntakeGroup(intake, 0.6, 0.6, shooter)
                ),
                ShooterGroup(intake, shooter, true) {
                    SmartDashboard.getNumber("StaticShooter",0.0)
                }.withTimeout(5.0)
            )
        )
        m_chooser.addOption("Short Tarmac, Intake, and Shoot",
            SequentialCommandGroup(
                IntakeCargos(intake, 0.2, 0.2, shooter).withTimeout(0.5),
                ParallelDeadlineGroup(
                    DriveForDistanceCommand(drivetrain, 0.15, 7.5), //Distance is 7.5
                    IntakeGroup(intake, 0.6, 0.6, shooter)
                ),
                ShooterGroup(intake, shooter, true) {0.4}.withTimeout(5.0)
            )
        )
        m_chooser.addOption("Leave Tarmac Timed", SequentialCommandGroup(
            IntakeCargos(intake, 0.2, 0.2, shooter).withTimeout(0.5),
            DriveForTimeCommand(drivetrain, 2.0)))
        m_chooser.addOption("Leave Tarmac Distance", SequentialCommandGroup(
            IntakeCargos(intake, 0.2, 0.2, shooter).withTimeout(0.5),
            DriveForDistanceCommand(drivetrain, 0.25, 9.0)))
        m_chooser.addOption("Leave Tarmac and Shoot",
            SequentialCommandGroup(
                DriveForDistanceCommand(drivetrain, 0.15, 9.0),
                ShooterGroup(intake, shooter, true) {
                    SmartDashboard.getNumber(
                        "StaticShooter",
                        0.0
                    )
                }.withTimeout(5.0)
            )
        )

        // Add commands to the autonomous command chooser
        m_chooser.addOption("PathWeaver Not So Straight", SequentialCommandGroup(
            TrajectoryBase(drivetrain, "PATH-NOT-SO-STRAIGHT", resetGyro=true),
        ))

        m_chooser.addOption("PathWeaver Straight Backward", SequentialCommandGroup(
            TrajectoryBase(drivetrain, "PATH-STRAIGHT-BACKWARD", resetGyro=true),
        ))

        m_chooser.addOption("PathWeaver Curve Backward", SequentialCommandGroup(
            TrajectoryBase(drivetrain, "PATH-CURVE-BACKWARD", resetGyro=true),
        ))

        m_chooser.addOption("PathWeaver Curve", SequentialCommandGroup(
            TrajectoryBase(drivetrain, "PATH-CURVE", resetGyro=true),
        ))

        // TODO: Tune the Shooter Vel Values!
        m_chooser.addOption("PathWeaver Low Goal Start", SequentialCommandGroup(
            // Low Goal Shot
            ShooterGroup(intake, shooter, true) { 0.26 }.withTimeout(1.1),
            // First/Second Path "Grabbing Second/Third Ball"
            ParallelDeadlineGroup(
                TrajectoryBase(drivetrain, "PATH-1", resetGyro=true),
                IntakeGroup(intake, 0.3, 0.6, shooter)
            ),
            IntakeGroup(intake, 0.3, 0.6, shooter).withTimeout(0.04),
            AlignShooter({ -imu.angle }, { -95.0 }, drivetrain).withTimeout(1.0),
            ParallelDeadlineGroup(
                TrajectoryBase(drivetrain, "PATH-2", resetGyro=false),
                IntakeGroup(intake, 0.3, 0.6, shooter)
            ),
            IntakeGroup(intake, 0.3, 0.6, shooter).withTimeout(0.04),
            // Shooting Second and Third ball
            AlignShooter({ -imu.angle }, { -28.0 }, drivetrain).withTimeout(1.0),
            ShooterGroup(intake, shooter, true) { 0.2 }.withTimeout(1.5),
            // Third/Fourth Path "Grabbing Fourth/Fifth Ball"
            AlignShooter({ -imu.angle }, { -60.0 }, drivetrain).withTimeout(1.5),
            ParallelDeadlineGroup(
                TrajectoryBase(drivetrain, "PATH-3", resetGyro=false),
                IntakeGroup(intake, 0.3, 0.6, shooter)
            ),
            IntakeGroup(intake, 0.3, 0.6, shooter).withTimeout(0.5),
            ParallelCommandGroup(
                AlignShooter({ -imu.angle }, {-50.0 }, drivetrain),
                ShooterGroup(intake, shooter, true) { 0.2 }
            )


//            TrajectoryBase(drivetrain, "PATH-4", resetGyro=false),
//            // Shooting Fourth/Fifth ball
////            AlignShooter({ -imu.angle }, { -70.0 }, drivetrain),
//            ShooterGroup(intake, shooter, true) { 0.2 }.withTimeout(2.0)
        ))

        m_chooser.addOption("PathWeaver High Goal Start", SequentialCommandGroup(
            // High Goal Shot
            ShooterGroup(intake, shooter, true) { 0.4 }.withTimeout(2.0),
            AlignShooter({ -imu.angle }, { 60.0 }, drivetrain),
            // First/Second Path "Grabbing Second/Third Ball"
            ParallelDeadlineGroup(
                TrajectoryBase(drivetrain, "PATH-1B", resetGyro=true),
                IntakeGroup(intake, 0.3, 0.6, shooter)
            ),
            AlignShooter({ -imu.angle }, { 95.0 }, drivetrain),
            ParallelDeadlineGroup(
                TrajectoryBase(drivetrain, "PATH-2", resetGyro=false),
                IntakeGroup(intake, 0.3, 0.6, shooter)
            ),
            // Shooting Second and Third ball
            AlignShooter({ -imu.angle }, { 166.0 }, drivetrain),
            ShooterGroup(intake, shooter, true) { 0.2 }.withTimeout(2.0),
            // Third/Fourth Path "Grabbing Fourth/Fifth Ball"
            ParallelDeadlineGroup(
                TrajectoryBase(drivetrain, "PATH-3", resetGyro=false),
                IntakeGroup(intake, 0.3, 0.6, shooter)
            ),
            IntakeGroup(intake, 0.3, 0.6, shooter).withTimeout(2.0),
            TrajectoryBase(drivetrain, "PATH-4", resetGyro=false),
            // Shooting Fourth/Fifth ball
            AlignShooter({ -imu.angle }, { 110.0 }, drivetrain),
            ShooterGroup(intake, shooter, true) { 0.2 }.withTimeout(2.0)
        ))

        SmartDashboard.putData(m_chooser)

        val inst: NetworkTableInstance = NetworkTableInstance.getDefault()

        val table: NetworkTable = inst.getTable("SmartDashboard")

        table.addEntryListener(
            "isRed",
            { local_table, key, entry, value, flags -> pathColor.set(Color.valueOf(value.value as Int)) },
            EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
        println("INFO: Initialize Autonomous Options")
        SmartDashboard.putData(m_chooser)

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a GenericHID or one of its subclasses
     * edu.wpi.first.wpilibj.Joystick or XboxController, and then passing it to a
     * edu.wpi.first.wpilibj2.command.button.JoystickButton
     */
    private fun configureButtonBindings() {
        // Drivers Button Binding
        button1.whileHeld(IntakeGroup(intake, 0.3, 0.6, shooter)).whenReleased(ShooterCargos(shooter, true) { -0.7 }
            .withTimeout(0.3))

        button2.whileHeld(StartEndCommand(
            {drivetrain.isFullSpeed = 0.5},
            {drivetrain.isFullSpeed = 1.0}
        ))

        button4.whileHeld(ShooterGroup(intake, shooter, false, drivetrain::calcDist))

        button5.whileHeld(IntakeCargos(intake, -0.3, -0.3, shooter)) // TODO: REMOVE ARM MOVEMENT

        button6.whileHeld(ShooterGroup(intake, shooter, true) { SmartDashboard.getNumber("StaticShooter", 0.0) })

//        button7.whenPressed(AlignShooter({ -imu.angle }, drivetrain::calcHeading, drivetrain).withTimeout(2.0))
        button7.whenPressed(AlignShooter({ -imu.angle }, { -95.0 }, drivetrain).withTimeout(2.0))

        button8.whenPressed(InstantCommand(
            {
                drivetrain.resetEncoders()
                imu.reset()
                drivetrain.resetOdometry(Pose2d(3.1, 0.0, Rotation2d.fromDegrees(0.0)))
                println("INFO: THE POSE IS RESET!!!")
            }
        ))

        button9.whenPressed(InstantCommand(climber::resetEncoders, climber))

        button10.whileHeld(ManualClimber(climber, -0.3))

        button11.whileHeld(ManualClimber(climber, 0.3))

        // CO-Drivers Button Binding

        coButton1.whenPressed(AutoClimber(climber, isGrenade=false, isDown=true))

        coButton2.whenPressed(AutoClimber(climber, isGrenade=false, isDown=false))

        // Intake Arm Up
//        coButton3.whileHeld(AutoArm(intake, isDown=false))
        coButton3.whileHeld(ManualArm(intake, 0.3))

        coButton4.whileHeld(ShooterGroup(intake, shooter, true) { 0.26 })

        // Intake Arm Down
//        coButton5.whileHeld(AutoArm(intake, isDown=true))
        coButton5.whileHeld(ManualArm(intake, -0.3))

        coButton6.whenPressed(ParallelCommandGroup(
            AutoClimber(climber, isGrenade=true, isDown=true),
            IntakeArmUp(intake)
        ))

        SmartDashboard.putData(object : InstantCommand(
            drivetrain::resetEncoders,
            drivetrain
        ) {
            override fun initialize() {
                super.initialize()
                name = "Reset Encoders"
            }

            override fun runsWhenDisabled(): Boolean {
                return true
            }
        })

        // PIDController pidcontroller = new PIDControllerDebug(0.0006, 0.0005, 0.0);
        val pidcontroller: PIDController = PIDControllerDebug(0.002, 0.001, 0.0)
        pidcontroller.setIntegratorRange(-0.15, 0.15)
        if (RobotBase.isSimulation()) {
            SmartDashboard.putNumber("TargetX", 0.0)
        }

    }

    fun getAutonomousCommand(): Command { return m_chooser.selected }

    fun stopAllSubsystems() { drivetrain.stop() }

    fun periodic() {
        loopIdx++
        if (loopIdx == 10) {
            loopIdx = 0

            SmartDashboard.putNumber("Shooter Dist", drivetrain.calcDist())
            SmartDashboard.putNumber("Yaw", -imu.angle)
            SmartDashboard.putNumber("Heading", drivetrain.calcHeading())
//            SmartDashboard.putNumber("Delta Error", drivetrain.calcHeading()-(-imu.angle))
//            SmartDashboard.putNumber("Proportional Value", pidController.p * pidController.positionError)
//            SmartDashboard.putNumber("Integral Value", pidController.i * pidController.totalError())

        }
    }

    fun simulationPeriodic() { drivetrain.simulationPeriodic() }

    fun reset() {
        imu.reset()
        drivetrain.resetEncoders()
    }
}