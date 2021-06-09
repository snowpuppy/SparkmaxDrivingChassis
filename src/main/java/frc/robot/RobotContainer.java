// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.Auto.DriveSquareAuto;
//import frc.robot.auto.AutoDriveTest;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.Drive;
//import frc.robot.auto.AutoDriveTest;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 
   //Subsystems
   private final Drive drive;
   /** The container for the robot. Contains subsystems, OI devices, and commands. */
   //Commands
   private final TankDrive tankDrive;
   private final DriveSquareAuto driveSquareAuto;
   
   //AutoDriveTest autoDriveTest;
   
   //OI
   private static RobotContainer robotContainer;


  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // Subsystem instantiation
    drive = Drive.getInstance();

   //Default command instantiation
   tankDrive = new TankDrive(drive);
   driveSquareAuto = new DriveSquareAuto();
   drive.setDefaultCommand(tankDrive);

    //OI Device instantiation
    OI.getInstance();

   DriveDistance.registerWithTestingDashboard();
   DriveSquareAuto.registerWithTestingDashboard(); 

  //autoDriveTest = new AutoDriveTest(drive);
  TestingDashboard.getInstance().createTestingDashboard();
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   
 public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
   return autoDriveTest;
  }
  */

public static RobotContainer getInstance() {
  if (robotContainer == null) {
    robotContainer = new RobotContainer();
  } 
  return robotContainer;
}

public Command getAutonomousCommand() {
  // TODO: This needs to be changed to collect the autonomous command
  // from a chooser on ShuffleBoard
  // Create a voltage constraint to ensure we don't accelerate too fast
  return driveSquareAuto;
  // Run path following command, then stop at the end.
  //return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
}

}
