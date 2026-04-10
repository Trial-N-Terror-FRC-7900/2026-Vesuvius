// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);

  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class IntakeConstants
  {
    public static final int IntakeWheelsMotorCANID = 18;
    public static final int IntakeAngleMotorCANID = 17;
    public static final double IntakeWheelSpeed = -.95;
    public static final double IntakeAngleManualSpeed = .2;
    public static final double IntakeAngleUp = .63;
    public static final double IntakeAngleAgitate = .86;
    public static final double IntakeAngleAgitateAuto = .86;
    public static final double IntakeAngleDown = .98;
    public static final double tolerance = .015;
  }

  public static class ClimberConstants
  {
    public static final int ClimberMotorCANID = 19;
    public static final double ClimberUpPos = 100;
    public static final double ClimbedDownPos = 32.5;
    public static final double ClimberHomePos = 0;
    public static final double tolerance = 2;
  }

  public static class SpindexerConstants
  {
    public static final int SpindexerMotorCANID = 1;
    public static final double SpindexSpeed = .375;
  }

  public static class KickerConstants
  {
    public static final int KickerMotorCANID = 15;
    public static final double KickerSpeed = -1;
    public static final int maximumVelocity = 6784;
    public static final int tolerance = 33;
  }

  public static class TurretConstants
  {
    public static final int LeftFlywheelMotorCANID = 12;
    public static final int RightFlywheelMotorCANID = 11;
    public static final int RotationMotorCANID = 16;
    public static final int maximumFlywheelVelocity = 5676;
    public static final double FlywheelSpeed = -.6;
    public static final double FlywheelAdjust = -.125;
    public static final double RotationSpeed = -.25;
    public static final int HoodMotorCanID = 10;
    public static final double HoodSpeed = .1;
    public static final double HoodUpPos = 13;
    public static final double flywheelTolerance = 50;
    public static final double hoodTolerance = 1.00;
    public static final double rotationTolerance = .005;
    public static final double rotationLimitReverse = -.32;
    public static final double rotationLimitForward = .6754;

    public static final double hoodLimitFoward = 29.5;
    public static final double hoodLimitReverse = 0.0;
  }

  public static class fieldConstants
  {
    // Field dimensions
    public static final AprilTagFieldLayout FieldAprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static final double fieldLength = FieldAprilTags.getFieldLength();
    public static final double fieldWidth = FieldAprilTags.getFieldWidth();

    public static final Rectangle2d BlueAllianceZone = new Rectangle2d(new Pose2d(2.015, fieldWidth/2, new Rotation2d(0)), 4.03, fieldWidth);
    public static final Rectangle2d RedAllianceZone = new Rectangle2d(new Pose2d(fieldLength-2.015, fieldWidth/2, new Rotation2d(0)), 4.03, fieldWidth);
    public static final Rectangle2d CenterZone = new Rectangle2d(new Pose2d(fieldLength/2, fieldWidth/2, new Rotation2d(0)), fieldLength-8.06, fieldWidth);
    public static final Rectangle2d CenterZoneUno = new Rectangle2d(new Pose2d(fieldLength/2, fieldWidth*.75, new Rotation2d(0)), fieldLength-8.06, fieldWidth/2);
    public static final Rectangle2d CenterZoneDos = new Rectangle2d(new Pose2d(fieldLength/2, fieldWidth/4, new Rotation2d(0)), fieldLength-8.06, fieldWidth/2);

    public static final Rectangle2d RedAllianceZoneUno = new Rectangle2d(new Pose2d(fieldLength-2.015, fieldWidth*.75, new Rotation2d(0)), 4.03, fieldWidth/2);
    public static final Rectangle2d RedAllianceZoneDos = new Rectangle2d(new Pose2d(fieldLength-2.015, fieldWidth/4, new Rotation2d(0)), 4.03, fieldWidth/2);

    public static final Rectangle2d BlueAllianceZoneUno = new Rectangle2d(new Pose2d(2.015, fieldWidth*.75, new Rotation2d(0)), 4.03, fieldWidth/2);
    public static final Rectangle2d BlueAllianceZoneDos = new Rectangle2d(new Pose2d(2.015, fieldWidth/4, new Rotation2d(0)), 4.03, fieldWidth/2);

    //Hubs
    public static final Translation2d blueHubPos = new Translation2d(Inches.of(181.56), Meters.of(fieldWidth/2));
    public static final Translation2d redHubPos = new Translation2d(Inches.of(468.56), Meters.of(fieldWidth/2));
    //Pass Zone Unos
    public static final Translation2d blue_zoneUnoPassPos = new Translation2d(Inches.of(90.78), Meters.of(fieldWidth*.625));
    public static final Translation2d red_zoneUnoPassPos = new Translation2d(Inches.of(559.34), Meters.of(fieldWidth*.625));
    //Pass Zone Dos
    public static final Translation2d blue_zoneDosPassPos = new Translation2d(Inches.of(90.78), (Meters.of(fieldWidth*.375)));
    public static final Translation2d red_zoneDosPassPos = new Translation2d(Inches.of(559.34), (Meters.of(fieldWidth*.375)));
  }
}
