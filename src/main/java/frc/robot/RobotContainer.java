// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeInCommand;
import frc.robot.commands.AlgaeOutCommand;
import frc.robot.commands.ArmDownCommand;
import frc.robot.commands.ArmUpCommand;
import frc.robot.commands.AutoAlgaeInCommand;
import frc.robot.commands.AutoAlgaeOutCommand;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.CoralOutCommand;
import frc.robot.commands.CoralStackCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandPS5Controller driverController = new CommandPS5Controller(0);
  private final CommandPS5Controller m_operatorController = 
new CommandPS5Controller(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  public final RollerSubsystem m_roller = new RollerSubsystem();
  public final ArmSubsystem m_arm = new ArmSubsystem();
  public final ClimberSubsystem m_climber = new ClimberSubsystem();

  

  

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/maxSwerve"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> -driverController.getLeftY() * 1,
                                                                () -> -driverController.getLeftX() * 1)
                                                            .withControllerRotationAxis(()->-driverController.getRightX())
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);


  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverController.getLeftY(),
                                                                        () -> -driverController.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverController.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    CameraServer.startAutomaticCapture();
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("CoralOutCommand", new CoralOutCommand(m_roller));
    NamedCommands.registerCommand("CoralStackCommand", new CoralStackCommand(m_roller));
    NamedCommands.registerCommand("ArmUpCommand", new ArmUpCommand(m_arm));
    NamedCommands.registerCommand("ArmDownCommand", new ArmDownCommand(m_arm));
    NamedCommands.registerCommand("AutoAlgaeInCommand", new AutoAlgaeInCommand(m_roller));
    NamedCommands.registerCommand("AutoAlgaeOutCommand", new AutoAlgaeOutCommand(m_roller));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandPS5Controller Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    /**
     * Here we declare all of our operator commands, these commands could have been
     * written more compact but are left verbose so the intent is clear.
     */
   
    m_operatorController.R2().whileTrue(new AlgaeInCommand(m_roller));
    
    // Here we use a trigger as a button when it is pushed past a certain threshold
    m_operatorController.R1().whileTrue(new AlgaeOutCommand(m_roller));

    /**
     * The arm will be passively held up or down after this is used,
     * make sure not to run the arm too long or it may get upset!
     */
    m_operatorController.L2().whileTrue(new ArmUpCommand(m_arm));
    m_operatorController.L1().whileTrue(new ArmDownCommand(m_arm));

    /**
     * Used to score coral, the stack command is for when there is already coral
     * in L1 where you are trying to score. The numbers may need to be tuned, 
     * make sure the rollers do not wear on the plastic basket.
     */
    m_operatorController.square().whileTrue(new CoralOutCommand(m_roller));
    m_operatorController.triangle().whileTrue(new CoralStackCommand(m_roller));
    m_operatorController.cross().whileTrue(new ClimberUpCommand(m_climber));
    m_operatorController.circle().whileTrue(new ClimberDownCommand(m_climber));

    /**
     * POV is a direction on the D-Pad or directional arrow pad of the controller,
     * the direction of this will be different depending on how your winch is wound
     */
    //m_operatorController.pov(90).whileTrue(new ClimberUpCommand(m_climber));
    //m_operatorController.pov(270).whileTrue(new ClimberDownCommand(m_climber));

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {                                                                                                                                 
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driverController.create().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverController.square().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverController.triangle().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverController.create().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverController.options().whileTrue(drivebase.centerModulesCommand());
      driverController.L1().onTrue(Commands.none());
      driverController.R1().onTrue(Commands.none());
    } else
    {
      driverController.cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverController.square().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverController.circle().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driverController.create().whileTrue(Commands.none());
      driverController.options().whileTrue(Commands.none());
      driverController.L1().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverController.R1().onTrue(Commands.none());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("MiddleAuto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
