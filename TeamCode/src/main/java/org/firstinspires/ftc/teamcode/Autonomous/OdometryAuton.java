package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous(name = "Odometry Autonomous", group = "Linear OpMode")
public class OdometryAuton extends LinearOpMode {

    //Create a robot---responsible for connecting hardware of Hardware class to methods
    Hardware robot;

    //Define the Motors and Servos here to not rely on referencing the robot variable to access the motors and servos
    DcMotor leftFront, rightFront, leftBack, rightBack, intakeMotor, leftLauncher, rightLauncher, wobbleGoalArm;
    Servo pushServo;
    CRServo intakeServo;
    RevIMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        //Map hardware
        robot = new Hardware(hardwareMap);

        //Assign the motors and servos to the ones on the robot to not require calling robot everytime a method or servo needs to be called.
        //Motors for drive train
        leftFront = robot.leftFront;
        rightFront = robot.rightFront;
        rightBack = robot.rightBack;
        leftBack = robot.leftBack;


        //Motors for intake
        intakeMotor = robot.intakeMotor;
        rightLauncher = robot.rightLauncher;
        leftLauncher = robot.leftLauncher;
        intakeServo = robot.intakeServo;

        //Gyroscope
        imu = new RevIMU(hardwareMap);

        //Servo for launcher
        pushServo = robot.pushServo;
        pushServo.setPosition(Servo.MAX_POSITION);

        // Locations of the wheels relative to the robot center. (UPDATE WITH ACTUAL VALUES)
        Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
        Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
        Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
        Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

        // Creating my kinematics object using the wheel locations.
        MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

        // Creating my odometry object from the kinematics object. Here, our starting pose is 5 meters along the long end of the field and in the center of the field along the short end, facing forward.
        MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(m_kinematics, imu.getRotation2d(), new Pose2d(5.0, 13.5, new Rotation2d()));
    }

//    public void autonomousPeriodic()
//    {
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
}
