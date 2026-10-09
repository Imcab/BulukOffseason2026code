// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.configuration.bindings;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stzteam.mars.models.containers.Binding;
import com.stzteam.mars.operator.ControllerOI;

import frc.robot.configuration.constants.swerve.SwerveConstants;
import frc.robot.modules.swerve.CommandSwerveDrivetrain;
import frc.robot.modules.swerve.SwerveRequestFactory;

public class DriverBindings implements Binding {

  /* Speed multipliers while holding the bumpers (both held = 0.5 * 0.25) */
  private static final double SLOW_MODE_FACTOR = 0.5;
  private static final double PRECISION_MODE_FACTOR = 0.25;

  private final CommandSwerveDrivetrain drivetrain;

  private final ControllerOI driver;

  /* Requests are created once and reused every loop instead of allocating new ones */
  private final SwerveRequest.FieldCentric driveRequest = SwerveRequestFactory.driveFieldCentric();
  private final SwerveRequest.SwerveDriveBrake brakeRequest = SwerveRequestFactory.brake();

  private DriverBindings(CommandSwerveDrivetrain drivetrain, ControllerOI driver) {
    this.drivetrain = drivetrain;
    this.driver = driver;
  }

  public static DriverBindings create(CommandSwerveDrivetrain drivetrain, ControllerOI driver) {
    return new DriverBindings(drivetrain, driver);
  }

  @Override
  public void bind() {
    var driverLeftStick = driver.getLeftStick();
    var driverRightStick = driver.getRightStick();
    var driverBumpers = driver.getBumpers();
    var driverButtons = driver.getActionButtons();

    /*
     * Field centric drive. The operator perspective (set in the drivetrain per alliance)
     * makes "forward" always point away from the driver, so no per-alliance sign flips here.
     */
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> {
              double speedFactor =
                  SwerveConstants.MaxSpeed
                      * (driverBumpers.right().getAsBoolean() ? SLOW_MODE_FACTOR : 1.0)
                      * (driverBumpers.left().getAsBoolean() ? PRECISION_MODE_FACTOR : 1.0);

              return driveRequest
                  .withVelocityX(-driverLeftStick.y().getAsDouble() * speedFactor)
                  .withVelocityY(-driverLeftStick.x().getAsDouble() * speedFactor)
                  .withRotationalRate(
                      -driverRightStick.x().getAsDouble() * SwerveConstants.MaxAngularRate);
            }));

    driverButtons.top().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    driverButtons.bottom().whileTrue(drivetrain.applyRequest(() -> brakeRequest));
  }
}
