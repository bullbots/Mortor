package frc.robot

class Constants {

    companion object {
        // Drivetrain Ports
        const val RIGHT_MASTER_PORT = 1
        const val RIGHT_SLAVE_PORT = 2
        const val LEFT_MASTER_PORT = 3
        const val LEFT_SLAVE_PORT = 4

        // Shooter Ports
        const val SHOOTER_PORT = 5

        // Climber Ports
        const val CLIMBER_PORT = 7

        // Unassigned Falcons
        const val unassigned1 = 6

        // Intake Ports
        const val INTAKE_SPINNER_PORT = 8

        // Unassigned NEOs
        const val unassigned2 = 9

        /**
         * ALL DRIVETRAIN RELATED CONSTANTS GO HERE
         */

        // TODO: These values need to be tuned
        const val LEFT_VELOCITY_P = 0.05
        const val LEFT_VELOCITY_I = 0.0
        const val LEFT_VELOCITY_D = 0.0
        const val LEFT_VELOCITY_FF = 2048.0 / 21000.0

        const val LEFT_MASTER_ACCELERATION = 0.0
        const val LEFT_MASTER_VELOCITY = 0.0

        const val RIGHT_VELOCITY_P = 0.05
        const val RIGHT_VELOCITY_I = 0.0
        const val RIGHT_VELOCITY_D = 0.0
        const val RIGHT_VELOCITY_FF = 2048.0 / 21000.0


        const val RIGHT_MASTER_ACCELERATION = 3300.0
        const val RIGHT_MASTER_VELOCITY = 3300.0

        /**
         * ALL INTAKE RELATED CONSTANTS GO HERE
         */

        /**
         *  ALL SHOOTER RELATED CONSTANTS GO HERE
         */

        // TODO: These values need to be tuned
        const val  SHOOTER_FF = 0.0473 // 0
        const val SHOOTER_P = 0.15 // 0
        const val SHOOTER_I = 0.0 // 1e-6
        const val SHOOTER_D = 0.0 // 0

        const val NEO_MAX_RPM = 5676.0 // http://www.revrobotics.com/rev-21-1650/ "Empirical Free Speed"

        /**
         * ALL CLIMB RELATED CONSTANTS GO HERE
         */
        const val CLIMBER_DOWN = -40000.0
        const val CLIMBER_UP = 405000.0
        const val CLIMBER_LIMIT_THRESHOLD = 1000
        const val kPIDLoopIdx = 0

        const val climberkP = 0.05
        const val climberkI = 0.0
        const val climberkD = 0.005 // kP / 10
        const val climberkF = 2048.0 / 21000.0
        const val climberkIzone = 0.0
        const val climberkPeakOutput = 1.0

        const val kSlotIdx = 0

        /**
         * MISCELLANEOUS CONSTANTS
         */
        const val kTIMEOUT_MS = 30

    }
}