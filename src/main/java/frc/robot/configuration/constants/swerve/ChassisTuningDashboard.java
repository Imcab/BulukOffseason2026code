package frc.robot.configuration.constants.swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.modules.swerve.CommandSwerveDrivetrain;
import frc.robot.modules.swerve.DriveCharacterizationCommand;

/**
 * Botones de SmartDashboard/Elastic para caracterizar el chasis sin ocupar botones del control.
 * Requieren el robot habilitado (teleop) y mueven el robot solo.
 *
 * <p>DriveChar/*: kS, kV y radio de llanta del drive girando en el lugar (circulo de ~1.5 m),
 * sin SignalLogger. El resultado aparece en DriveChar/kS, DriveChar/kV, DriveChar/radioRueda_m.
 */
public final class ChassisTuningDashboard {

  private ChassisTuningDashboard() {}

  public static void publish(CommandSwerveDrivetrain drivetrain) {
    SmartDashboard.putData(
        "DriveChar/Caracterizar drive (gira en el lugar)",
        new DriveCharacterizationCommand(drivetrain).withName("Caracterizar drive"));
  }
}
