package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TestingDashboard;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.Auto;

public class DriveSquareAuto extends SequentialCommandGroup {
    public DriveSquareAuto() {
        double distance = 12; // inches
        double speed = .5; // % power
        double angle = 90.0; // degrees
    
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(new DriveDistance(distance, speed, true));
    
      }
    
    public static void registerWithTestingDashboard() {
    Auto auto = Auto.getInstance();
    DriveSquareAuto cmd = new DriveSquareAuto();
    TestingDashboard.getInstance().registerCommand(auto, "Auto", cmd);
  }
}
