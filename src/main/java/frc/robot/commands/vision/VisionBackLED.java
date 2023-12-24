package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.LEDMode;
import frc.robot.subsystems.Vision.VisionPipeline;

public class VisionBackLED extends InstantCommand {
  private final Vision subsystem = RobotContainer.BACK_VISION;
  private final LEDMode mode;

  public VisionBackLED(LEDMode mode) {
    addRequirements(subsystem);
    this.mode = mode;
  }

  @Override
  public void initialize() {
    subsystem.setPipeline(VisionPipeline.APRILTAG);
  }
}
