package frc.robot.configuration;

import com.stzteam.mars.blackboard.BlackboardKey;

public class KeyManager {

    private KeyManager() {}

    public static final String myKey = "TestKey";
    public static final String SWERVE_KEY = "Holonomic";
    public static final String LIMELIGHT_KEY = "Limelight";
    public static final BlackboardKey<String> myBlackboardKey = new BlackboardKey<>(myKey, String.class);
    public static final BlackboardKey<Double> myBlackBoardKeyDouble = new BlackboardKey<>(myKey, Double.class);
    public static final BlackboardKey<Boolean> myBlackBoardKeyBoolean = new BlackboardKey<>(myKey, Boolean.class);
    public static final String INTAKE_KEY = "Intake";
    public static final String SHOOTER_KEY = "Shooter";
    public static final String INDEXER_KEY = "Indexer";
    
}
