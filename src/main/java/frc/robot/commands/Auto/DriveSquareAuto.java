package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TestingDashboard;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.GyroTurn;
import frc.robot.subsystems.Auto;

public class DriveSquareAuto extends SequentialCommandGroup {
    public DriveSquareAuto() {
        double distance = 30; // inches
        double driveSpeed = .24; // % power
        double angle = 86.0; // degrees
        double turnSpeed = 0.16;
    
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
          new DriveDistance(distance, driveSpeed, true),
          new GyroTurn(angle, turnSpeed, true),
          new DriveDistance(distance, driveSpeed, true),
          new GyroTurn(angle, turnSpeed, true),
          new DriveDistance(distance, driveSpeed, true),
          new GyroTurn(angle, turnSpeed, true),
          new DriveDistance(distance, driveSpeed, true),
          new GyroTurn(angle, turnSpeed, true)
        );

        /**addCommands(
          new DriveDistanceCheck(distance, driveSpeed, true)
        );*/
    
      }
    
    public static void registerWithTestingDashboard() {
    Auto auto = Auto.getInstance();
    DriveSquareAuto cmd = new DriveSquareAuto();
    TestingDashboard.getInstance().registerCommand(auto, "Auto", cmd);

    
  }
}
