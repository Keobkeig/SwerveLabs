package com.stuypulse.robot.subsystems.swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;

public class SwerveModule extends AbstractModule {

    private String id;

    private Rotation2d angleOffset;
    private Translation2d distanceOffset;

    private SwerveModuleState targetState;
    
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    private SparkAbsoluteEncoder turnEncoder;
    private RelativeEncoder driveEncoder;

    private PIDController driveController;
    private AnglePIDController turnController;

    public SwerveModule(String id, Translation2d distanceOffset, Rotation2d angleOffset) {
        this.id = id;
        this.distanceOffset = distanceOffset;
        this.angleOffset = angleOffset;

        this.driveMotor = new CANSparkMax(0, MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(1, MotorType.kBrushless);

        this.driveEncoder = driveMotor.getEncoder();
        this.turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

        this.driveController = new PIDController(0, 0, 0);
        this.turnController = new AnglePIDController(0, 0, 0);
    }

    @Override
    public String getID() {
        return this.id;
    }
    
    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnEncoder.getPosition()).minus(this.angleOffset);
    }
    
    @Override
    public Translation2d getOffset() {
       return this.distanceOffset;
    }

    @Override
    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition();
    }

    @Override
    public void setState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getAngle());
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Swerve/" + id + "/Raw Angle (deg)", Units.rotationsToDegrees(turnEncoder.getPosition()));
        SmartDashboard.putNumber("Swerve/" + id + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle Error", turnController.getError().toDegrees());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle Voltage", turnController.getOutput());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle Current", turnMotor.getOutputCurrent());
        SmartDashboard.putNumber("Swerve/" + id + "/Target Velocity", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity", getVelocity());
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity Error", driveController.getError());
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity Voltage", driveController.getOutput());
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity Current", driveMotor.getOutputCurrent());
    }
}