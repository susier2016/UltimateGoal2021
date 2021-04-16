//package org.firstinspires.ftc.teamcode.Autonomous;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.arcrobotics.ftclib.geometry.Rotation2d;
//import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
//
//public class Subsystem extends SubsystemBase {
//    /**
//     * Creates a new Subsystem.
//     */
//    public Subsystem() {
//
//    }
//
//    @Override
//    public void periodic() {
//        // Get my wheel speeds; assume .getRate() has been
//        // set up to return velocity of the encoder
//        // in meters per second.
//        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds
//                (
//                        m_frontLeftEncoder.getRate(), m_frontRightEncoder.getRate(),
//                        m_backLeftEncoder.getRate(), m_backRightEncoder.getRate()
//                );
//
//        // Get my gyro angle.
//        Rotation2d gyroAngle = Rotation2d.fromDegrees(m_gyro.getAngle());
//
//        // Update the pose
//        m_pose = m_odometry.update(gyroAngle, wheelSpeeds);
//    }
//}