package frc.robot

class Constants {


    companion object {
        // Drivetrain Ports
        const val RIGHT_MASTER_PORT = 1;
        const val RIGHT_SLAVE_PORT = 2;
        const val LEFT_MASTER_PORT = 3;
        const val LEFT_SLAVE_PORT = 4;

        // Shooter Ports
        const val SHOOTER_PORT = 5;

        // Unassigned Falcons
        const val unassigned = 6;
        const val unassigned1 = 7;

        // Intake Ports
        const val INTAKE_SPINNER = 8;

        /**
         * ALL DRIVETRAIN RELATED CONSTANTS GO HERE
         */

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

        const val  SHOOTER_FF = 0.0473 // 0
        const val SHOOTER_P = 0.15 // 0
        const val SHOOTER_I = 0.0 // 1e-6
        const val SHOOTER_D = 0.0 // 0

        const val NEO_MAX_RPM = 5676.0 // http://www.revrobotics.com/rev-21-1650/ "Empirical Free Speed"

        /**
         * ALL CLIMB RELATED CONSTANTS GO HERE
         */

        /**
         * MISCELLANEOUS CONSTANTS
         */
        const val kTIMEOUT_MS = 0
        const val VISION_OUTER_ALIGN_THRESHOLD = 100
        const val VISION_INNER_ALIGN_THRESHOLD = 80
    }
}