// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.modules.swerve;

import static frc.robot.configuration.constants.swerve.HubAlignConstants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.configuration.constants.swerve.SwerveConstants;

/**
 * Ajuste fino a una pose fija: P sobre x, y, heading (los mismos kP de PathPlanner, validados en
 * tools/pid_sweep con la prueba "mantener_pose"), con velocidades acotadas. Termina cuando lleva
 * SETTLE_TIME_S dentro de tolerancia; el timeout lo pone quien lo usa.
 *
 * <p>Las velocidades van en coordenadas de campo con origen azul (ForwardPerspective BlueAlliance),
 * no en la perspectiva del operador: en rojo la perspectiva del operador esta girada 180 grados y
 * mandaria el robot al reves.
 */
public class DriveToPose extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final Pose2d target;
  private final SwerveRequest.FieldCentric request =
      new SwerveRequest.FieldCentric()
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.Idle stop = new SwerveRequest.Idle();
  private final Timer inTolerance = new Timer();
  private boolean wasInTolerance;

  public DriveToPose(CommandSwerveDrivetrain drivetrain, Pose2d target) {
    this.drivetrain = drivetrain;
    this.target = target;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    wasInTolerance = false;
    inTolerance.stop();
    inTolerance.reset();
  }

  @Override
  public void execute() {
    Pose2d pose = drivetrain.getState().Pose;
    double ex = target.getX() - pose.getX();
    double ey = target.getY() - pose.getY();
    double eth = target.getRotation().minus(pose.getRotation()).getRadians(); // [-pi, pi]

    double kpT = SwerveConstants.PathTranslationPID.kP;
    double kpR = SwerveConstants.PathRotationPID.kP;
    double vx = kpT * ex;
    double vy = kpT * ey;
    double speed = Math.hypot(vx, vy);
    if (speed > FINE_MAX_SPEED_MPS) { // acota la magnitud sin cambiar la direccion
      vx *= FINE_MAX_SPEED_MPS / speed;
      vy *= FINE_MAX_SPEED_MPS / speed;
    }
    double omega = MathUtil.clamp(kpR * eth, -FINE_MAX_OMEGA_RADS, FINE_MAX_OMEGA_RADS);

    drivetrain.setControl(request.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));

    boolean ok = isWithinTolerance(pose, target);
    if (ok && !wasInTolerance) {
      inTolerance.restart();
    } else if (!ok) {
      inTolerance.stop();
      inTolerance.reset();
    }
    wasInTolerance = ok;
  }

  @Override
  public boolean isFinished() {
    return wasInTolerance && inTolerance.hasElapsed(SETTLE_TIME_S);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(stop);
  }

  public static boolean isWithinTolerance(Pose2d pose, Pose2d target) {
    return pose.getTranslation().getDistance(target.getTranslation()) <= POSITION_TOLERANCE_M
        && Math.abs(target.getRotation().minus(pose.getRotation()).getDegrees())
            <= ANGLE_TOLERANCE_DEG;
  }
}
