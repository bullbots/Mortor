package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.Climber_Commands.AutoClimber
import frc.robot.subsystems.Climber
import java.util.concurrent.atomic.AtomicReference

class RobotContainer {

    // Any static variables need to go in the companion object
    companion object {
        private val stick = Joystick(0)
        private val button1 = JoystickButton(stick, 1)
        private val button2 = JoystickButton(stick, 2)
        private val button3 = JoystickButton(stick, 3)



        private val pathColor = AtomicReference<Color>(Color.UNLOADED)

        private val shooterMode = SendableChooser<ShooterMode>()
    }

    // Subsystems
    private val climber = Climber()
//    private val lidar = LiDAR()

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



        initializeAutonomousOptions()
        // initializeStaticShooterVel()

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

        button1.whileHeld(AutoClimber(climber, isGrenade = true, isDown = true))
        button2.whileHeld(AutoClimber(climber, isGrenade = false, isDown = true))
        button3.whileHeld(AutoClimber(climber, isGrenade = false, isDown = false))






        // PIDController pidcontroller = new PIDControllerDebug(0.0006, 0.0005, 0.0);


    }

    fun getAutonomousCommand(): Command { return m_chooser.selected }

    fun stopAllSubsystems() { }

    private fun periodic() {

    }







}