package com.stuypulse.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

public class SwerveDrive extends SubsystemBase {
    
    private static SwerveDrive instance;

    static {
        if (RobotBase.isReal()) {
            instance = new SwerveDrive(
                new AbstractModule[] {
                    new SwerveModule("Front Left", new Translation2d(0, 0), new Rotation2d(0)),
                    new SwerveModule("Front Right", new Translation2d(0, 0), new Rotation2d(0)),
                    new SwerveModule("Back Left", new Translation2d(0, 0), new Rotation2d(0)),
                    new SwerveModule("Back Right", new Translation2d(0, 0), new Rotation2d(0))
                }
            );
        }
        
    }
    public static SwerveDrive getInstance(){
        return instance;
    }

    private final AbstractModule[] modules;  
    private final SwerveDriveKinematics kinematics;
    private final AHRS gyro;
    private final FieldObject2d[] module2ds;

    public SwerveDrive(AbstractModule[] modules) {
        this.modules = modules;
        kinematics = new SwerveDriveKinematics(getModulesOffset());
        gyro = new AHRS(SPI.Port.kMXP);
        module2ds = new FieldObject2d[modules.length];
    }

    public void initFieldObjects(Field2d field) {
        for (int i = 0; i < modules.length; i++) {
            module2ds[i] = field.getObject(modules[i].getID() + "-2d");
        }
    }

    public Translation2d[] getModulesOffset() {
        Translation2d[] offsets = new Translation2d[modules.length];
        for (int i = 0; i < modules.length; i++) {
            offsets[i] = modules[i].getOffset();
        }
        return offsets;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePosition() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }
}
