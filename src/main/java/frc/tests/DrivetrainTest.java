package frc.tests;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stzteam.mars.test.MARSTest;
import com.stzteam.mars.test.TestRoutine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.modules.swerve.CommandSwerveDrivetrain;
import frc.robot.modules.swerve.SwerveRequestFactory;
import java.util.function.DoubleSupplier;

/**
 * Drives the robot forward, sideways and rotating, checking that the measured speed reaches the
 * commanded one. Needs about 2 m of free space around the robot.
 */
@MARSTest(name = "Drivetrain")
public class DrivetrainTest extends TestRoutine {

  private static final double TEST_SPEED_MPS = 0.5;
  private static final double TEST_ROTATION_RADPS = 1.0;

  private static final double SPEED_TOLERANCE_MPS = 0.15;
  private static final double ROTATION_TOLERANCE_RADPS = 0.2;

  /* Max time to reach the target before the check runs anyway */
  private static final double SETTLE_TIMEOUT_SECONDS = 2.0;

  private final CommandSwerveDrivetrain drivetrain;

  private final SwerveRequest.RobotCentric driveRequest = SwerveRequestFactory.driveRobotCentric();
  private final SwerveRequest.SwerveDriveBrake brakeRequest = SwerveRequestFactory.brake();

  public DrivetrainTest(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public Command getRoutineCommand() {
    Command routine =
        Commands.sequence(
            checkStep(
                "Forward speed",
                TEST_SPEED_MPS, 0, 0,
                () -> drivetrain.getChassisSpeeds().vxMetersPerSecond,
                TEST_SPEED_MPS, SPEED_TOLERANCE_MPS),
            checkStep(
                "Sideways speed",
                0, TEST_SPEED_MPS, 0,
                () -> drivetrain.getChassisSpeeds().vyMetersPerSecond,
                TEST_SPEED_MPS, SPEED_TOLERANCE_MPS),
            checkStep(
                "Rotation speed",
                0, 0, TEST_ROTATION_RADPS,
                () -> drivetrain.getChassisSpeeds().omegaRadiansPerSecond,
                TEST_ROTATION_RADPS, ROTATION_TOLERANCE_RADPS),
            checkBrake());

    /* Hold the drivetrain for the whole test so the default command can't take over */
    routine.addRequirements(drivetrain);

    return routine.finallyDo(() -> drivetrain.setControl(brakeRequest));
  }

  /** Commands a robot-centric speed, waits for it to settle and checks the measured value. */
  private Command checkStep(
      String name,
      double vx,
      double vy,
      double omega,
      DoubleSupplier measured,
      double target,
      double tolerance) {
    DoubleSupplier error = () -> Math.abs(measured.getAsDouble() - target);

    return Commands.sequence(
        run(
            () ->
                drivetrain.setControl(
                    driveRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega))),
        waitFor(() -> error.getAsDouble() < tolerance, SETTLE_TIMEOUT_SECONDS),
        assertLessThan(error, tolerance, name));
  }

  /** Applies the brake and checks the robot actually stops. */
  private Command checkBrake() {
    DoubleSupplier linearSpeed =
        () -> {
          var speeds = drivetrain.getChassisSpeeds();
          return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        };

    return Commands.sequence(
        run(() -> drivetrain.setControl(brakeRequest)),
        waitFor(() -> linearSpeed.getAsDouble() < SPEED_TOLERANCE_MPS, SETTLE_TIMEOUT_SECONDS),
        assertLessThan(linearSpeed, SPEED_TOLERANCE_MPS, "Brake"));
  }
}
