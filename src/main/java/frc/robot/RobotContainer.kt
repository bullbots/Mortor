package frc.robot

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrajectoryConfig
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.StartEndCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.Climber_Commands.AutoClimber
import frc.robot.commands.Climber_Commands.ClimberGroup
import frc.robot.commands.Drivetrain_Commands.AlignShooter
import frc.robot.commands.Drivetrain_Commands.Drive
import frc.robot.commands.Drivetrain_Commands.JoystickDrive
import frc.robot.commands.Intake_Commands.IntakeGroup
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

        private val shooterMode = SendableChooser<ShooterMode>()
    }

    // Subsystems
    private val drivetrain = DrivetrainFalcon()
    private val intake = Intake()
    private val shooter = Shooter()
    private val climber = Climber()
//    private val lidar = LiDAR()

    // Util
    private val pidController = PIDControllerDebug(0.003, 0.0, 0.0)
    private val imu = NavX()

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

    private enum class Letter(value: Int) {
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
    var staticChooser = SendableChooser<Double>()

    private enum class ShooterMode {
        STATIC,
        DYANMIC
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

        initializeAutonomousOptions()

        pidController.setTolerance(1.0)
        pidController.enableContinuousInput(-180.0, 180.0)

        trajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(2.0, 2.0, Rotation2d()),
            listOf<Translation2d>(),
            Pose2d(6.0, 4.0, Rotation2d()),
            TrajectoryConfig(2.0, 2.0)
        )
         initializeStaticShooterVel()

//        shooterMode.setDefaultOption("Competition Shooting", ShooterMode.COMPETITION)
//        shooterMode.addOption("Demo Shooting", ShooterMode.DEMO)
//        SmartDashboard.putData(shooterMode)
    }

    /**
     * Adds the staticShooter velocity values to the SmartDashboard.
     * You are able to adjust the value inside the SmartDashboard to change velocity
     */
    private fun initializeStaticShooterVel() { SmartDashboard.putNumber("staticChooser", 0.3) }

    private fun initializeAutonomousOptions() {
        // Add commands to the autonomous command chooser
        m_chooser.setDefaultOption("Taxi Out", InstantCommand(
            { drivetrain.curvatureDrive(0.3, 0.0, false)}
        ).withTimeout(3.0))

        m_chooser.addOption("Taxi Out & Shoot", SequentialCommandGroup(
            Drive(drivetrain, 0.3).withTimeout(3.0),
            ShooterGroup(intake, -0.1, shooter, true) { 0.3 }.withTimeout(1.0)
        ))
//        m_chooser.setDefaultOption("Bounce Piece", new SequentialCommandGroup(
//                new TrajectoryBase(drivetrain, "/BOUNCE-1", false, true), // ... boolean isBackwards, boolean resetGyro
//                new TrajectoryBase(drivetrain, "/BOUNCE-2", true, false),
//                new TrajectoryBase(drivetrain, "/BOUNCE-3", false, false),
//                new TrajectoryBase(drivetrain, "/BOUNCE-4", true, false)
//        ));
//        m_chooser.addOption("Bounce Path", new SequentialCommandGroup(
//                new TrajectoryBase(drivetrain, "/BOUNCE-1", false, true), // ... boolean isBackwards, boolean resetGyro
//                new TrajectoryBase(drivetrain, "/BOUNCE-2", true, false),
//                new TrajectoryBase(drivetrain, "/BOUNCE-3", false, false),
//                new TrajectoryBase(drivetrain, "/BOUNCE-4", true, false)
//        ));
//        m_chooser.addOption("Slalom Path",
//                new TrajectoryBase(drivetrain, "/SLALOM")
//        );
//
//        System.out.println("Path Color: " + pathColor.get());
//        m_chooser.addOption("Galactic Search Challenge",
//                new ParallelCommandGroup(
//                        new AutonomousGSC(
//                                drivetrain,
//                                harm,
//                                () -> ((int) SmartDashboard.getNumber("isRed", 0) != 0), //&& pathLetter.get() != Letter.UNLOADED),
//                                () -> ((int) SmartDashboard.getNumber("isRed", 0) == 1),
//                                () -> (pathLetter.get() == Letter.A)
//                        )
//                ));
//
//        m_chooser.addOption("Galactic Red",
//                new ParallelCommandGroup(
//                        new TrajectoryBase(drivetrain, "/RED-COMBINED", true, false).deadlineWith(
//                                new IntakeGroup(harm))
//                )
//        );
//
//        // m_chooser.addOption("Galactic Search Challenge B", new AutonomousGSC_B(
//        //   drivetrain,
//        //   harm,
//        //   () -> (pathColor.get() != Color.UNLOADED),
//        //   () -> (pathColor.get() == Color.RED)
//        // ));
//
//        m_chooser.addOption("Forward Then Backward Path", new SequentialCommandGroup(
//                new TrajectoryBase(drivetrain, "/FORWARD-DISTANCE", false, true), // ... boolean isBackwards, boolean resetGyro
//                new TrajectoryBase(drivetrain, "/BACKWARD-DISTANCE", true, false)
//        ));
//
//        SmartDashboard.putData(m_chooser);
//
//        // NetworkTableInstance inst = NetworkTableInstance.getDefault();
//
//        // NetworkTable table = inst.getTable("SmartDashboard");
//
//        // table.addEntryListener("isRed",
//        //   (local_table, key, entry, value, flags) -> {
//        //     pathColor.set(Color.valueOf((int) value.getValue()));
//        //   },
//        //   EntryListenerFlags.kNew | EntryListenerFlags.kUpdate
//        // );
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a GenericHID or one of its subclasses
     * edu.wpi.first.wpilibj.Joystick or XboxController, and then passing it to a
     * edu.wpi.first.wpilibj2.command.button.JoystickButton
     */
    private fun configureButtonBindings() {


        // Drivers Button Binding
        button1.whileHeld(IntakeGroup(intake, 0.6, shooter) { -0.15 })

        button2.whileHeld(StartEndCommand(
            {drivetrain.isFullSpeed = 0.5},
            {drivetrain.isFullSpeed = 1.0}
        ))

//        button4.whileHeld(ShooterGroup(intake, -0.1, shooter, true) { 0.6 })
        button4.whileHeld(ShooterGroup(intake, -0.1, shooter, false, drivetrain::calcDist))

        button5.whileHeld(IntakeGroup(intake, -0.3, shooter) { -0.1 })

        button6.whileHeld(ShooterGroup(intake, -0.1, shooter, true) { SmartDashboard.getNumber("staticChooser", 0.0) })

        button7.whenPressed(AlignShooter(pidController, { -imu.angle }, drivetrain::calcHeading,
            { output: Double -> drivetrain.drive(0.0, -output) }, drivetrain).withTimeout(1.0))

        button8.whenPressed(InstantCommand(
            {
                drivetrain.resetEncoders()
                imu.reset()
                drivetrain.resetOdometry(Pose2d(3.1, 0.0, Rotation2d.fromDegrees(0.0)))
                println("INFO: THE POSE IS RESET!!!")
            }
        ))

        button10.whileHeld(ClimberGroup(climber, -0.5))

        button11.whileHeld(ClimberGroup(climber, 0.5))

        // CO-Drivers Button Binding

        coButton1.whenPressed(AutoClimber(climber, isGrenade = false, isDown = true))

        coButton2.whenPressed(AutoClimber(climber, isGrenade = false, isDown = false))

        coButton4.whileHeld(IntakeGroup(intake, 0.3, shooter) { 0.2 })

        coButton6.whenPressed(AutoClimber(climber, isGrenade = true, isDown = true))
//        coButton9.whenPressed(AutoClimber(isDown = true, isGrenade = true, climber = climber))

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
//        SmartDashboard.putNumber("Yaw", -imu.angle)
        SmartDashboard.putNumber("Shooter Dist", drivetrain.calcDist())
//        println("RobotContainer Periodic is being called")
    }

    fun simulationPeriodic() { drivetrain.simulationPeriodic() }
}