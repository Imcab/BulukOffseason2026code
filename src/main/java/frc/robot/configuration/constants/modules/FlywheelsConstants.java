package frc.robot.configuration.constants.modules;

import com.ctre.phoenix6.signals.InvertedValue;

public class FlywheelsConstants {

    public class IntakeWheelsConstants {
            
        public static final int IntakeWheels_ID = 14;

        public static InvertedValue invertedValue = InvertedValue.CounterClockwise_Positive;
        public static double StatorCurrentLimit = 40;
        public static double SupplyCurrentLimit = 60;
        public static final boolean SupplyCurrentLimitEnable = true;
        public static final boolean StatorCurrentLimitEnable = true;

    }
    
    public class shooterWheelsConstants{

        public static final int shooterLeaderID = 19;
        public static final int shooterFollowerID = 20;

        public static final double SupplyCurrentLimit = 70;
        public static final boolean SupplyCurrentLimitEnable = false;

        public static final double StatorCurrentLimit = 120;
        public static final boolean StatorCurrentLimitEnable = false;

        public static final double kRPMTolerance = 400;

        public static final double kGearRatio = 3.0;

        public static final double kS = 8;
        public static final double kV = 0.8;

        public static final double kP = 15;
        public static final double kI = 0;
        public static final double kD = 0.01;

        public static double idleVoltage = 0;

        // Valores iniciales de prueba. Se pueden cambiar en vivo desde Shooter/Tuning/TestVolts y TestRPM
        // (en MARS el shooter giraba con RPM negativas, ej. -3580)
        public static final double kTestVolts = 12.0;
        public static final double kTestRPM = 2000;

        /** RPM del shooter para disparar (D-pad arriba del operador) */
        public static final double kShootRPM = 4350;

        public static final double RPM_0_1 = 0;
        public static final double RPM_1_2 = 0;
        public static final double RPM_2_3 = 0;
        public static final double RPM_3_4 = 0;
        public static final double RPM_4_5 = 0;

        //sim
        public static final double kGearing = 4;
        public static final double kMOI = 0.002;
        //sim

    }

}