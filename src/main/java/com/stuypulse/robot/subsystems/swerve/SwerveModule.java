package com.stuypulse.robot.subsystems.swerve;

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

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.SwerveDrive.Drive;
import com.stuypulse.robot.constants.Settings.SwerveDrive.Encoder;
import com.stuypulse.robot.constants.Settings.SwerveDrive.Turn;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.ARateLimit;

public class SwerveModule extends AbstractModule {

    private String id;

    private Rotation2d angleOffset;
    private Translation2d distanceOffset;

    private SwerveModuleState targetState;
    
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    private SparkAbsoluteEncoder turnEncoder;
    private RelativeEncoder driveEncoder;

    private Controller driveController;
    private AngleController turnController;

    public SwerveModule(String id, Translation2d distanceOffset, int driveID, Rotation2d angleOffset, int turnID) {
        this.id = id;
        this.distanceOffset = distanceOffset;
        this.angleOffset = angleOffset;

        this.driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

        this.driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);

        this.turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
        turnEncoder.setPositionConversionFactor(Encoder.Turn.POSITION_CONVERSION);
        turnEncoder.setVelocityConversionFactor(Encoder.Turn.VELOCITY_CONVERSION);

        this.targetState = new SwerveModuleState();

        this.driveController = new PIDController(Drive.kP, Drive.kI, Drive.kD)
            .add(new MotorFeedforward(Drive.kS, Drive.kV, Drive.kA).velocity());
      
        this.turnController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
            .setSetpointFilter(new ARateLimit(Settings.SwerveDrive.MAX_MODULE_TURN));

        Motors.disableStatusFrames(driveMotor, 3, 4, 5, 6);
        Motors.Swerve.DRIVE_CONFIG.configure(driveMotor);
        Motors.Swerve.TURN_CONFIG.configure(turnMotor);

        //XXX: check this
        driveEncoder.setPosition(0);
        turnEncoder.setZeroOffset(angleOffset.getDegrees());
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
    public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getAngle());
    }
    
    @Override
    public void periodic() {
        turnMotor.setVoltage(
            turnController.update(Angle.fromRotation2d(targetState.angle), Angle.fromRotation2d(getAngle()))
        );

        driveMotor.setVoltage(
            driveController.update(targetState.speedMetersPerSecond, getVelocity())
        );
        
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