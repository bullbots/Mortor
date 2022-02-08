package frc.robot

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.Drivetrain_Commands.JoystickDrive
import frc.robot.commands.Intake_Commands.IntakeGroup
import frc.robot.commands.Shooter_Commands.ShooterGroup
import frc.robot.subsystems.DrivetrainFalcon
import frc.robot.subsystems.Intake
import frc.robot.subsystems.StaticShooter
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
        private val button10 = JoystickButton(stick, 10)

        private val pathColor = AtomicReference<Color>(Color.UNLOADED)

        private val shooterMode = SendableChooser<ShooterMode>()
    }

    private val drivetrain = DrivetrainFalcon()
    private val intake = Intake()
    private val staticShooter = StaticShooter()

    var m_chooser = SendableChooser<Command>()
    var static_chooser = SendableChooser<Command>()

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
        println("****************************CONFIG****************************************")
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
        initializeStaticShooterVel()

//        shooterMode.setDefaultOption("Competition Shooting", ShooterMode.COMPETITION)
//        shooterMode.addOption("Demo Shooting", ShooterMode.DEMO)
//        SmartDashboard.putData(shooterMode)
    }

    private fun initializeStaticShooterVel() {
        static_chooser.setDefaultOption("0.1 Vel", SequentialCommandGroup(
            ShooterGroup(staticShooter, 0.1)
        ))
        static_chooser.addOption("0.2 Vel", SequentialCommandGroup(
            ShooterGroup(staticShooter, 0.2)
        ))
        static_chooser.addOption("0.3 Vel", SequentialCommandGroup(
            ShooterGroup(staticShooter, 0.3)
        ))
        static_chooser.addOption("0.4 Vel", SequentialCommandGroup(
            ShooterGroup(staticShooter, 0.4)
        ))

        SmartDashboard.putData(static_chooser)
    }

    private fun initializeAutonomousOptions() {
        // Add commands to the autonomous command chooser
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

        button1.whileHeld(IntakeGroup(intake))

        button2.whileHeld(ShooterGroup(staticShooter))

        button5.whileHeld(IntakeGroup(intake, -0.3))

        SmartDashboard.putData(object : InstantCommand(
            { drivetrain.resetEncoders() },
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

    fun getAutonomousCommand(): Command {
        return m_chooser.selected
    }



    fun stopAllSubsystems() {
        drivetrain.stop()
    }

    fun periodic() {
        drivetrain.periodic()
    }







}