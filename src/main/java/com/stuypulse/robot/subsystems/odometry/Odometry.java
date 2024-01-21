package com.stuypulse.robot.subsystems.odometry;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Odometry extends AbstractOdometry {
    private final SwerveDriveOdometry odometry;

    private final Field2d field;
    private final FieldObject2d odometryPose2D;

    protected Odometry() {
        SwerveDrive swerve = SwerveDrive.getInstance();

        this.odometry =
            new SwerveDriveOdometry(
                swerve.getKinematics(),
                swerve.getGyroAngle(),
                swerve.getModulePositions(),
                new Pose2d());

        this.field = new Field2d();
        this.odometryPose2D = field.getObject("Odometry Pose");

        swerve.initFieldObjects(field);
        SmartDashboard.putData("Field", field);
    }

    @Override
    public Field2d getField() {
        return field;
    }

    @Override
    public Pose2d getPose() {
        return new Pose2d(
            odometry.getPoseMeters().getTranslation(),
            odometry.getPoseMeters().getRotation());
    }

    @Override
    public void reset(Pose2d pose2d) {
        SwerveDrive swerve = SwerveDrive.getInstance();

        odometry.resetPosition(swerve.getGyroAngle(), swerve.getModulePositions(), pose2d);
    }
    
    @Override
    public void periodic() {
        SwerveDrive swerve = SwerveDrive.getInstance();

        odometry.update(swerve.getGyroAngle(), swerve.getModulePositions());
        odometryPose2D.setPose(odometry.getPoseMeters());
    }
}