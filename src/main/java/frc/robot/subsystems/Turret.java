package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.KickerConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.fieldConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class Turret extends SubsystemBase{
    private SparkMaxConfig turretMotorConfig;
    private SparkMaxConfig followerMotorConfig;
    private SparkFlexConfig rotateMotorConfig;
    private SparkMax m_leftTurret;
    private RelativeEncoder m_leftTurretEncoder;
    private SparkClosedLoopController m_leftTurretPID;
    private SparkMax m_rightTurret;
    private SparkFlex m_rotationTurret;
    private SparkClosedLoopController m_rotationTurretPID;

    double FlywheelAdjust = 0.0;

    Kicker kicker = new Kicker();
    Hood hood = new Hood();
    MatchTelemetry matchTelem = new MatchTelemetry();

    Angle setpoint = Radians.of(0);

    boolean inAllianceZone = false;
    boolean inCenterZone = false;
    boolean inFarZone = false;

    boolean autoTargeting = false;

    SwerveSubsystem drivebase;

    public Turret(SwerveSubsystem drivebase)
    {
        this.drivebase = drivebase; 
        turretMotorConfig = new SparkMaxConfig();
        followerMotorConfig = new SparkMaxConfig();
        rotateMotorConfig = new SparkFlexConfig();
        m_leftTurret = new SparkMax(TurretConstants.LeftFlywheelMotorCANID, MotorType.kBrushless);
        m_leftTurretEncoder = m_leftTurret.getEncoder();
        m_leftTurretPID = m_leftTurret.getClosedLoopController();
        m_rightTurret = new SparkMax(TurretConstants.RightFlywheelMotorCANID, MotorType.kBrushless);
        m_rotationTurret = new SparkFlex(TurretConstants.RotationMotorCANID, MotorType.kBrushless);
        m_rotationTurretPID = m_rotationTurret.getClosedLoopController();

        followerMotorConfig.smartCurrentLimit(60);

        turretMotorConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        turretMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed
            // loop slot, as it will default to slot 0.
            .p(.0005)
            .i(0)
            .d(0)
            .outputRange(-1, 0);
            // Set PID values for velocity control in slot 1
            //.velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
            //.outputRange(-1, 0, ClosedLoopSlot.kSlot1);
        turretMotorConfig.idleMode(IdleMode.kCoast);
        followerMotorConfig.follow(TurretConstants.LeftFlywheelMotorCANID, true);
        followerMotorConfig.idleMode(IdleMode.kCoast); // Dont care 
        followerMotorConfig.smartCurrentLimit(60);

        rotateMotorConfig.smartCurrentLimit(20);
        rotateMotorConfig.encoder
            .positionConversionFactor(.021)
            .velocityConversionFactor(1);

        rotateMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed
            // loop slot, as it will default to slot 0.
            .p(3)
            .i(0)
            .d(0)
            .outputRange(-0.15, 0.15)
            // Set PID values for velocity control in slot 1
            .p(0.0001, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        rotateMotorConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(TurretConstants.rotationLimitForward)
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(TurretConstants.rotationLimitReverse);

        rotateMotorConfig.idleMode(IdleMode.kCoast);

        m_leftTurret.configure(turretMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_rightTurret.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_rotationTurret.configure(rotateMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        setupTurret();
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Flywheel Speed:", TurretConstants.FlywheelSpeed + FlywheelAdjust);
        if(turretSolver(getAbsoluteEncoderAngleSupplier(), kicker.getAbsoluteEncoderAngleSupplier()).getAngleOptional().isPresent()){
            turretSolver(getAbsoluteEncoderAngleSupplier(), kicker.getAbsoluteEncoderAngleSupplier()).getAngleOptional().ifPresent(turretAngle -> { SmartDashboard.putNumber("Turret Angle Pos", turretAngle.in(Rotations));});
        }
        SmartDashboard.putNumber("Kicker Encoder", kicker.getAbsoluteEncoderAngleSupplier().get().in(Rotations));
        SmartDashboard.putNumber("Turret Motor Connected Encoder", m_rotationTurret.getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("Turret Relative Encoder", m_rotationTurret.getEncoder().getPosition());

        SmartDashboard.putNumber("Setpoint for Turret", setpoint.in(Rotations));

        SmartDashboard.putBoolean("In Alliance Zone:", inAllianceZone);
        SmartDashboard.putBoolean("In Center Zone:", inCenterZone);
        SmartDashboard.putBoolean("In Far Zone:", inFarZone);

        if(autoTargeting == true){
            activePeriodicTurretAndHood();
        }
    }

    public Supplier<Angle> getAbsoluteEncoderAngleSupplier(){

        Angle encoderPosition = Rotations.of(m_rotationTurret.getAbsoluteEncoder().getPosition());

    return () -> encoderPosition;

  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param encoder1Supplier    The First Encoder
   * @param encoder2Supplier    The Second Encoder
   * 
   * @return The Easy CRT solver.
   */
  public EasyCRT turretSolver(Supplier<Angle> encoder1Supplier, Supplier<Angle> encoder2Supplier)
  {
    // Suppose: mechanism : drive gear = 12:1, drive gear = 50T, encoders use 19T and 23T pinions.
    var easyCrt = new EasyCRTConfig(encoder1Supplier, encoder2Supplier)
            .withCommonDriveGear(
                /* commonRatio (mech:drive) */ 1,
                /* driveGearTeeth */ 200,
                /* encoder1Pinion */ 21,
                /* encoder2Pinion */ 19)
            .withAbsoluteEncoderOffsets(Rotations.of(0.0), Rotations.of(0.0)) // set after mechanical zero
            .withMechanismRange(Rotations.of(-1.0), Rotations.of(1.0)) // -360 deg to +720 deg
            .withMatchTolerance(Rotations.of(0.06)) // ~1.08 deg at encoder2 for the example ratio
            .withAbsoluteEncoderInversions(true, false)
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
    
    public Command adjustFlywheelSpeed(double isInverted){
    return this.runOnce(() -> FlywheelAdjust += (isInverted * TurretConstants.FlywheelAdjust));
    }

    public Command activeTargeting(){
        return this.runOnce(() -> {
            //if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
                
                Angle angleToHub = Degrees.of(Radians.of(Math.atan2((drivebase.getPose().getX()) - 0.23 - fieldConstants.blueHubPos.getX(), drivebase.getPose().getY() + 0.23 - fieldConstants.blueHubPos.getY())).in(Degrees));
               
                //SmartDashboard.putNumber("Drivebase Heading", drivebase.getHeading().getDegrees());
                
                setpoint = angleToHub.plus(Degrees.of(90)).plus(drivebase.getHeading().getMeasure());/*.plus(drivebase.getHeading().getMeasure()).minus(Rotations.of(0.109794))*/;
                SmartDashboard.putNumber("setpoint", setpoint.in(Rotations));
                //SmartDashboard.putNumber("turretHubDiff", angleToHub.in(Rotations)-setpoint.in(Rotations));
                m_rotationTurretPID.setSetpoint(setpoint.in(Rotations), ControlType.kPosition, ClosedLoopSlot.kSlot0);
                

            //}
            //else {

            //}
        });
    }

    public Command hoodAdjust(Translation2d target){
        return this.runOnce(() -> {
            double distanceToHub = Math.sqrt(Math.pow((target.getX() - drivebase.getPose().getX()),2) + Math.pow((target.getY() - drivebase.getPose().getY()),2));

            SmartDashboard.putNumber("Distance to Hub", distanceToHub);

            inAllianceZone = matchTelem.inAllianceZone(drivebase);
            inCenterZone = matchTelem.inCenterZone(drivebase);
            inFarZone = matchTelem.inFarZone(drivebase);

            hood.setHoodfromDistance(distanceToHub);
        });
    }

    public void activePeriodicTurretAndHood(){
        double distanceToTarget = 0;
        Angle angleToTarget = Degrees.of(-90); //This should have the default position keep the turret pointed forward
        /*
         * This properly offsets the position of the shooter vs the center of the robot. 
         * 
         * Firstly it maps the turret to a circle on the robot of radius R=0.325269 and Angle -45 degrees or 0.785398 radians
         *  r*cos(Heading of Chassis - 45deg) = x
         *  r*sin(Heading of Chassis - 45deg) = y
         */
        double shooterPoseX = drivebase.getPose().getX() - (0.325269 * Math.cos(drivebase.getHeading().getRadians() - 0.785398));
        double shooterPoseY = drivebase.getPose().getY() + (0.325269 * Math.sin(drivebase.getHeading().getRadians() - 0.785398));
        // Need to confirm CW or CCW, but it checks out outside of that
        
        double targetX = 0;
        double targetY = 0;

        //Needs to be updated for more than hub targets
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            if (fieldConstants.BlueAllianceZone.contains(drivebase.getPose().getTranslation())){
                targetX = fieldConstants.blueHubPos.getX();
                targetY = fieldConstants.blueHubPos.getY();
            }
            else if (fieldConstants.CenterZoneUno.contains(drivebase.getPose().getTranslation())){
                targetX = fieldConstants.blue_zoneUnoPassPos.getX();
                targetY = fieldConstants.blue_zoneUnoPassPos.getY();
            }
            else if (fieldConstants.CenterZoneDos.contains(drivebase.getPose().getTranslation())){
                targetX = fieldConstants.blue_zoneDosPassPos.getX();
                targetY = fieldConstants.blue_zoneDosPassPos.getY();
            }
            /*else if (fieldConstants.RedAllianceZone.contains(drivebase.getPose().getTranslation())){
                targetX = fieldConstants.bob.getX();
                targetY = fieldConstants.bob.getY();
            }*/
        }
        else{
            if (fieldConstants.RedAllianceZone.contains(drivebase.getPose().getTranslation())){
                targetX = fieldConstants.redHubPos.getX();
                targetY = fieldConstants.redHubPos.getY();
            }
            else if (fieldConstants.CenterZoneUno.contains(drivebase.getPose().getTranslation())){
                targetX = fieldConstants.red_zoneUnoPassPos.getX();
                targetY = fieldConstants.red_zoneUnoPassPos.getY();
            }
            else if (fieldConstants.CenterZoneDos.contains(drivebase.getPose().getTranslation())){
                targetX = fieldConstants.red_zoneDosPassPos.getX();
                targetY = fieldConstants.red_zoneDosPassPos.getY();
            }
            /*else if (fieldConstants.BlueAllianceZone.contains(drivebase.getPose().getTranslation())){
                targetX = fieldConstants.bob.getX();
                targetY = fieldConstants.bob.getY();
            }*/
        }      

        distanceToTarget = Math.sqrt(Math.pow((targetX - shooterPoseX),2) + Math.pow((targetY - shooterPoseY),2));
        angleToTarget = Degrees.of(Radians.of(Math.atan2(shooterPoseX - targetX, shooterPoseY - targetY)).in(Degrees));
        
        //The angle calculated doesnt take into account the robot facing different directions.
        // Plus 90 makes it so that the -90 right infront of the robot is mapped to 0 for the robot turret
        // Plus robot heading is to account for the robots rotation
        setpoint = angleToTarget.plus(Degrees.of(90)).plus(drivebase.getHeading().getMeasure());

        //The Encoder for the Turret is mapped to Rotations
        SmartDashboard.putNumber("Turret Setpoint", setpoint.in(Rotations));
        m_rotationTurretPID.setSetpoint(setpoint.in(Rotations), ControlType.kPosition, ClosedLoopSlot.kSlot0);

        SmartDashboard.putNumber("Distance to Target", distanceToTarget);
        hood.setHoodfromDistance(distanceToTarget);
    }

    public Command flywheelFeed(){
        return this.run(() -> {
            m_leftTurretPID.setSetpoint(
            TurretConstants.maximumFlywheelVelocity * TurretConstants.FlywheelSpeed, 
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0);
        });
    }

    public BooleanSupplier isFlywheelVelo(double velocitySetpoint){
        return () -> Math.abs(m_leftTurretEncoder.getVelocity() - velocitySetpoint) <= TurretConstants.flywheelTolerance;
    }

    public Command flywheelStop(){
        return this.run(() -> {
            m_leftTurret.stopMotor();
        });
    }
    
    public Command rotationLeft(){
        return this.run(() -> {
            m_rotationTurret.set(-TurretConstants.RotationSpeed);
            /*m_rotationTurretPID.setSetpoint(
                .2, 
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0
            );*/
        });
    }

    public Command rotationRight(){
        return this.run(() -> {
            m_rotationTurret.set(TurretConstants.RotationSpeed);
            /*m_rotationTurretPID.setSetpoint(
                -.2, 
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0
            );*/
        });
    }

    public Command rotationHome(){
        return this.run(() -> {
             m_rotationTurretPID.setSetpoint(
                0, 
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0
             );
        });
    }

    public void setupTurret(){
        if(turretSolver(getAbsoluteEncoderAngleSupplier(), kicker.getAbsoluteEncoderAngleSupplier()).getAngleOptional().isPresent()){
            turretSolver(getAbsoluteEncoderAngleSupplier(), kicker.getAbsoluteEncoderAngleSupplier()).getAngleOptional().ifPresent(turretAngle -> { m_rotationTurret.getEncoder().setPosition(-turretAngle.in(Rotations));});
        }
    }

    public Command rotationStop(){
        return this.run(() -> {
            m_rotationTurret.stopMotor();
        });
    }

    public Command kickerFeed(){
        return kicker.kickerFeed();
    }

    public Command kickerUnjam(){
        return kicker.kickerUnjam();
    }

    public Command kickerStop(){
        return kicker.kickerStop();
    }

    public Command hoodManualUp(){
        return hood.manualUp();
    }

    public Command hoodManualDown(){
        return hood.manualDown();
    }

    public Command hoodManualStop(){
        return hood.stop();
    }

    public Command hoodUp(){
        return hood.hoodUp();
    }

    public Command hoodDown(){
        return hood.hoodDown();
    }

    public Command toggleAutoTargeting(Boolean Input){
        return this.runOnce(() -> {
            autoTargeting = Input;
            //autoTargeting = !autoTargeting;
        });
    }

    public Command shootProcess(){
        return this.run(() -> {
            autoTargeting = true;
            flywheelFeed();
        });
    }

    public Command stopShootProcess(){
        return this.run(() -> {
            autoTargeting = false;
            flywheelStop();
            hoodDown();
            rotationHome();
        });
    }

    //Limit -0.6754 and 0.2273
}
