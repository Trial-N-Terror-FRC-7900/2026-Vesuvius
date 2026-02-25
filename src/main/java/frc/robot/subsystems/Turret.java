package frc.robot.subsystems;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import java.awt.Desktop;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;

import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Kicker;

import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;
import yams.units.*;


public class Turret
{
  private SparkFlexConfig turretMotorConfig;
  private SparkFlex m_turret;

  private AbsoluteEncoder directEncoder;
  private final Kicker kicker = new Kicker();

  public Turret()
  {
    turretMotorConfig = new SparkFlexConfig();

    m_turret = new SparkFlex(TurretConstants.turretMotorCANID, MotorType.kBrushless);
    directEncoder = m_turret.getAbsoluteEncoder();
  }

  public Supplier<Angle> getAbsoluteEncoderAngleSupplier(){

    Angle encoderPosition = Rotations.of(directEncoder.getPosition());

    return () -> encoderPosition;

  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to the pose for the robot to position
   *                    itself correctly.
   * @return The target pose of the AprilTag.
   */
  public EasyCRT CRT(Supplier<Angle> encoder1Supplier, Supplier<Angle> encoder2Supplier)
  {
    // Suppose: mechanism : drive gear = 12:1, drive gear = 50T, encoders use 19T and 23T pinions.
    var easyCrt = new EasyCRTConfig(encoder1Supplier, encoder2Supplier)
            .withCommonDriveGear(
                /* commonRatio (mech:drive) */ 9.52380952,
                /* driveGearTeeth */ 21,
                /* encoder1Pinion */ 19,
                /* encoder2Pinion */ 21)
            .withAbsoluteEncoderOffsets(Rotations.of(0.0), Rotations.of(0.0)) // set after mechanical zero
            .withMechanismRange(Rotations.of(-1.0), Rotations.of(1.0)) // -360 deg to +720 deg
            .withMatchTolerance(Rotations.of(0.06)) // ~1.08 deg at encoder2 for the example ratio
            .withAbsoluteEncoderInversions(false, false)
            .withCrtGearRecommendationConstraints(
                /* coverageMargin */ 1.2,
                /* minTeeth */ 15,
                /* maxTeeth */ 45,
                /* maxIterations */ 30);

    // you can inspect:
    easyCrt.getUniqueCoverage();          // Optional<Angle> coverage from prime counts and common scale
    easyCrt.coverageSatisfiesRange();     // Does coverage exceed maxMechanismAngle?
    easyCrt.getRecommendedCrtGearPair();  // Suggested pair within constraints

    // Create the solver:
    var easyCrtSolver = new EasyCRT(easyCrt);

    return easyCrtSolver;
  }
}