package com.northeasternrobotics.wrappers.encoders;

import com.northeasternrobotics.wrappers.encoders.CANCoder.RealCANCoder;
import com.northeasternrobotics.wrappers.encoders.SRXEncoder.RealSRXEncoder;
import com.northeasternrobotics.wrappers.encoders.Sim.SimSwerveAzmthEncoder;
import com.northeasternrobotics.wrappers.encoders.ThriftyEncoder.RealThriftyEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class WrapperedSwerveAzimuthEncoder {

    AbstractSwerveAzmthEncoder enc;
    double curAngleRad;
    // TODO: Bring in oxconfig maybe?
    double mountingOffset;

    public WrapperedSwerveAzimuthEncoder(SwerveAzmthEncType type, String prefix, int id, double dfltMountingOffset_rad) {
        if (RobotBase.isReal()) {
            switch (type) {
                case SRXEncoder:
                    //ID = digital input
                    enc = new RealSRXEncoder(id);
                    break;
                case CANCoder:
                    //ID = CAN ID
                    enc = new RealCANCoder(id);
                    break;
                case Thrifty:
                    //ID = Analog Input
                    enc = new RealThriftyEncoder(id);
                    break;
            }
        } else {
            enc = new SimSwerveAzmthEncoder(id);
        }
        this.mountingOffset = dfltMountingOffset_rad;
    }

    public void update() {
        curAngleRad = Units.degreesToRadians(wrapAngleDeg(Units.radiansToDegrees(enc.getRawAngle_rad() - mountingOffset)));
    }

    public double getAngle_rad() {
        return curAngleRad;
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(this.getAngle_rad());
    }

    public enum SwerveAzmthEncType {
        SRXEncoder,
        CANCoder,
        Thrifty
    }
    private double wrapAngleDeg(double angle) {
        angle %= 360;
        angle = angle > 180 ? angle - 360 : angle;
        angle = angle < -180 ? angle + 360 : angle;
        return angle;
    }


}
