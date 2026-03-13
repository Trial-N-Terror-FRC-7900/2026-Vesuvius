package frc.robot.subsystems;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.Constants.fieldConstants;

public class MatchTelemetry extends SubsystemBase{

    public MatchTelemetry(){
        
    }

    public boolean inAllianceZone(SwerveSubsystem drivebase){
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            // Blue Alliance Zone
            return fieldConstants.BlueAllianceZone.contains(drivebase.getPose().getTranslation());
        }
        else{
            //Red Alliance Zone
            return fieldConstants.RedAllianceZone.contains(drivebase.getPose().getTranslation());
        }
    }

    public boolean inCenterZone(SwerveSubsystem drivebase){
        return fieldConstants.CenterZone.contains(drivebase.getPose().getTranslation());
    }

    public boolean inFarZone(SwerveSubsystem drivebase){
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            //Red Alliance Zone when on Blue Alliance
            return fieldConstants.RedAllianceZone.contains(drivebase.getPose().getTranslation());
        }
        else{
            // Blue Alliance Zone when on Red Alliance
            return fieldConstants.BlueAllianceZone.contains(drivebase.getPose().getTranslation());
        }
    }

    //Could add Prior to changing timer triggers
    // Trigger CRT Sync
    // Shoot before actually changed over


    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
            // If we have invalid game data, assume hub is active.
            return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }
}