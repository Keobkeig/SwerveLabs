package com.stuypulse.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractModule extends SubsystemBase {    

    public abstract Rotation2d getAngle();
    public abstract String getID();
    public abstract Translation2d getOffset();
    public abstract double getVelocity();
    public abstract SwerveModuleState getState();
    public abstract SwerveModulePosition getModulePosition();
    
    public abstract void setTargetState(SwerveModuleState state);
    
}



