package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

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
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;
import yams.units.*;


public class Kicker
{
  private SparkFlexConfig kickerMotorConfig;
  private SparkFlex m_kicker;

  private AbsoluteEncoder kickerConnectedEncoder;

  public Kicker()
  {
    kickerMotorConfig = new SparkFlexConfig();

    m_kicker = new SparkFlex(TurretConstants.turretMotorCANID, MotorType.kBrushless);
    kickerConnectedEncoder = m_kicker.getAbsoluteEncoder();


  }

  /**
   * Gets the Position of the Encoder connected to the Kicker Motor
   * 
   * @return The Position of the Connected Encoder
   */
  public double getTurretCRTKickerEncoderPosition()
  {
    return kickerConnectedEncoder.getPosition();
  }
}