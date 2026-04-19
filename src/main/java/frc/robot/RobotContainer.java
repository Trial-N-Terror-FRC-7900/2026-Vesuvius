// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.TriggerEvent;
import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.fieldConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Hood;

import java.io.File;
import java.lang.reflect.Field;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController operatorXbox = new CommandXboxController(2);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/maxSwerve"));
                                                              
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();
  private final Spindexer spindexer = new Spindexer();
  //private final Kicker kicker = new Kicker();
  private final Turret turret = new Turret(drivebase, spindexer);
  //private final Hood hood = new Hood();

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleRotation(1)
                                                            .scaleTranslation(1)
                                                            .allianceRelativeControl(true);

  SwerveInputStream driveAngularVelocitySlowed = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -.2,
                                                                () -> driverXbox.getLeftX() * -.2)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleRotation(.2)
                                                            .scaleTranslation(.8)
                                                            .allianceRelativeControl(true);

  
  SwerveInputStream driveAngularVelocityShowcase = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -.5,
                                                                () -> driverXbox.getLeftX() * -.5)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleRotation(.375)
                                                            .scaleTranslation(.8)
                                                            .allianceRelativeControl(true);
                                                            

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  SwerveInputStream driveLawnmower = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getLeftX,
                                                                                           driverXbox::getLeftY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocityShowcase.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  Trigger turretSetpoint = new Trigger(turret.isTurretatSetpoint());
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //Create the NamedCommands that will be used in PathPlanner

    /*
     * INTAKE
     */
    NamedCommands.registerCommand("intake_angleUp", intake.angleUpCheck());
    NamedCommands.registerCommand("intake_agitateToggle", intake.agitateToggleMode(true));
    NamedCommands.registerCommand("intake_stopAgitateToggle", intake.agitateToggleMode(false));
    NamedCommands.registerCommand("intake_angleDown", intake.angleDownCheck());
    NamedCommands.registerCommand("intake_runWheels", intake.runWheels());
    NamedCommands.registerCommand("intake_reverseWheels", intake.reverseWheels());
    NamedCommands.registerCommand("intake_stopWheels", intake.stopWheels());
    /*
     * TURRET
     */
    NamedCommands.registerCommand("turret_feed", spindexer.spindexerFeed().alongWith(turret.kickerFeed()));
    NamedCommands.registerCommand("turret_unjam", spindexer.spindexerUnjam().alongWith(turret.kickerUnjam()));
    NamedCommands.registerCommand("turret_stop", spindexer.spindexerStop().alongWith(turret.kickerStop()));

    NamedCommands.registerCommand("turret_shoot", turret.toggleAutoTargeting(true).andThen(turret.flywheelFeed())/* INTEGRATED FEEDING */.alongWith(spindexer.spindexerFeed()/*.onlyIf(turret.isTurretatSetpoint())*/).alongWith(turret.kickerFeed()/*.onlyIf(turret.isTurretatSetpoint())*/));
    //turretSetpoint.onTrue(turret.kickerFeedCheck().andThen(spindexer.spindexerFeed())).onFalse(spindexer.spindexerStop().alongWith(turret.kickerStop()));
    NamedCommands.registerCommand("turret_stopShoot", turret.toggleAutoTargeting(false).andThen(turret.hoodDownCheck()).andThen(turret.flywheelStop()).andThen(turret.rotationHomeCheck().withTimeout(.666))/* INTEGRATED FEEDING STOP */.alongWith(spindexer.spindexerStop()).andThen(turret.kickerStop()));
    NamedCommands.registerCommand("turret_rotationHome", turret.rotationHomeCheck());
    NamedCommands.registerCommand("turret_hoodDown", turret.hoodDownCheck());
    /*
     * CLIMBER
     */
    NamedCommands.registerCommand("climber_climberUp", climber.climberUpCheck());
    NamedCommands.registerCommand("climber_climberHome", climber.climberHomeCheck());
    NamedCommands.registerCommand("climber_climbedDown", climber.climbedDownCheck());

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
    //CanandEventLoop.getInstance(); // Only needed to configure Gyro
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveDirectLawnmower      = drivebase.driveFieldOriented(driveLawnmower);
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAnglularVelocitySlowed = drivebase.driveFieldOriented(driveAngularVelocitySlowed);
    Command driveShowcase = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    /*
     * 
     * SETS THE DEFAULT SWERVE DRIVE TYPE
     * 
     */
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      //driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      //driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
      //                                               () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
      

//      driverXbox.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    if (DriverStation.isTest())
    {
      /*driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
      */
    } else
    {
      /*driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
      driverXbox.y().onTrue(drivebase.driveToDistanceCommandDefer(drivebase::getPose, 2, 14));
      driverXbox.y().whileTrue(drivebase.driveForward());
      */
    }

    // DRIVER CONTROLS
    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.rightTrigger().onTrue(intake.runWheels().andThen(intake.angleDown()));
    driverXbox.rightBumper().onTrue(intake.stopWheels());
    driverXbox.povLeft().onTrue(intake.manualDown()).onFalse(intake.manualAngleStop());
    driverXbox.povRight().onTrue(intake.manualUp()).onFalse(intake.manualAngleStop());
    driverXbox.b().onTrue(intake.reverseWheels());
    driverXbox.leftTrigger().onTrue(intake.angleDown());
    driverXbox.leftBumper().onTrue(intake.angleUp());
    driverXbox.y().onTrue(climber.climberUp());
    driverXbox.a().onTrue(climber.climbedDown());
    driverXbox.povDown().onTrue(climber.manualDown()).onFalse(climber.manualClimberStop());
    driverXbox.povUp().onTrue(climber.manualUp()).onFalse(climber.manualClimberStop());
    driverXbox.x().onTrue(climber.climberHome());
    //Toggle To Slow Mode
    driverXbox.leftStick().onTrue(drivebase.setDriveMode(driveFieldOrientedAnglularVelocitySlowed)).onFalse(drivebase.setDriveMode(driveFieldOrientedAnglularVelocity));

    // OPERATOR CONTROLS
    //operatorXbox.leftTrigger().onTrue(turret.hoodUp());
    //operatorXbox.leftBumper().onTrue(turret.hoodDown());
    //operatorXbox.rightBumper().onTrue(turret.kickerFeedCheck().andThen(spindexer.spindexerFeed())).onFalse(spindexer.spindexerStop().alongWith(turret.kickerStop()));
    operatorXbox.rightBumper().onTrue(turret.kickerFeedCheck().andThen(spindexer.spindexerFeed())).onFalse(spindexer.spindexerStop().alongWith(turret.kickerStop()));
    turretSetpoint.onTrue(turret.kickerFeedCheck().onlyIf(operatorXbox.rightBumper()).andThen(spindexer.spindexerFeed().onlyIf(operatorXbox.rightBumper()))).onFalse(spindexer.spindexerStop().alongWith(turret.kickerStop()));

    operatorXbox.rightTrigger().onTrue(drivebase.setDriveMode(driveFieldOrientedAnglularVelocitySlowed).alongWith(turret.toggleAutoTargeting(true)).andThen(turret.flywheelFeed())).onFalse(turret.hoodDownCheck().andThen(drivebase.setDriveMode(driveFieldOrientedAnglularVelocity)).alongWith(turret.toggleAutoTargeting(false)).andThen(turret.flywheelStop()).andThen(turret.rotationHomeCheck()));
    operatorXbox.b().onTrue(spindexer.spindexerUnjam().alongWith(turret.kickerUnjam())).onFalse(spindexer.spindexerStop().alongWith(turret.kickerStop()));
    operatorXbox.rightStick().onTrue(intake.runWheels().andThen(intake.angleAgitate())).onFalse(intake.angleDown());
    operatorXbox.leftTrigger().onTrue(turret.flywheelFeed()).onFalse(turret.flywheelStop());
    operatorXbox.povUp().onTrue(turret.hoodManualUp()).onFalse(turret.hoodManualStop());
    operatorXbox.povDown().onTrue(turret.hoodManualDown()).onFalse(turret.hoodManualStop());
    operatorXbox.povLeft().onTrue(turret.rotationLeft()).onFalse(turret.rotationStop());
    operatorXbox.povRight().onTrue(turret.rotationRight()).onFalse(turret.rotationStop());
    //operatorXbox.rightStick().onTrue(turret.toggleAutoTargeting(true)).onFalse(turret.toggleAutoTargeting(false));
    //operatorXbox.leftStick().onTrue(intake.agitateToggleMode(true)).onFalse(intake.agitateToggleMode(false).andThen(intake.angleDown()));
    operatorXbox.a().onTrue(turret.toggleAutoTargeting(false).andThen(turret.hoodDownCheck()).andThen(turret.flywheelStop()).andThen(spindexer.spindexerStop()).andThen(turret.kickerStop()).andThen(turret.rotationHomeCheck()));
    //X MODE
    operatorXbox.leftStick().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
