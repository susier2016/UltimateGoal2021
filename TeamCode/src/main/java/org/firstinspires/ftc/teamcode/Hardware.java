package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * This class defines all the hardware components of the robot. It has eight DcMotors, five Servos, and one WebcamName. It can reset drive encoders and stop all motors
 */
public class Hardware {

    //Define all the motors, servos, and cameras of the robot

    //Drive Train
    public DcMotor leftFront, rightFront, rightBack, leftBack;

    //Intake
    public DcMotor intakeMotor, leftLauncher, rightLauncher, wobbleGoalArm;

    /**
     * Creates a new Hardware with all parts connected to a name
     *
     * @param hwmp map of robot parts on the control hub
     */
    public Hardware(HardwareMap hwmp) {

        //Drive Train
        leftFront = hwmp.dcMotor.get("Left Front");
        rightFront = hwmp.dcMotor.get("Right Front");
        rightBack = hwmp.dcMotor.get("Right Back");
        leftBack = hwmp.dcMotor.get("Left Back");

        //Intake
        intakeMotor = hwmp.dcMotor.get("Intake");
        leftLauncher = hwmp.dcMotor.get("Left Launcher");
        rightLauncher = hwmp.dcMotor.get("Right Launcher");

        //Wobble Goal Grasp
        wobbleGoalArm = hwmp.dcMotor.get("Wobble Goal Arm");

        /*greenWheelLeft = hwmp.dcMotor.get("Green Wheel Left");
        greenWheelRight = hwmp.dcMotor.get("Green Wheel Right");

        //Lift
        //horizontalLift = hwmp.dcMotor.get("Horizontal Lift");
        verticalLift = hwmp.dcMotor.get("Vertical Lift");
        pusher = hwmp.servo.get("Stone Pusher");

        //Arm to grip the stone
        stoneGripper = hwmp.servo.get("Stone Gripper");

        //Suction Platform
        suctionPlatformL = hwmp.servo.get("Suction Platform Left");
        suctionPlatformR = hwmp.servo.get("Suction Platform Right");

        //Arm that holds the brick and rotates.
        armRotate = hwmp.servo.get("Arm Rotate");

        //Platform
        platform = hwmp.servo.get("Platform");
        //platformR = hwmp.servo.get("Platform Right");

        //SkyBlock Holders
        //constrictL = hwmp.servo.get("Constriction Left");
        //constrictR = hwmp.servo.get("Constriction Right");

        //Camera
        cameraName = hwmp.get(WebcamName.class, "Webcam");

        gate = hwmp.servo.get("Gate");


        /*
        Extra motors and servos in case we add them later on
            horizontalIntake = hwmp.dcMotor.get("Horizontal Intake");
            verticalIntake = hwmp.dcMotor.get("Vertical Intake");
            liftServo = hwmp.servo.get("Lift Servo");
            gate = hwmp.servo.get("Gate");
            extrusion = hwmp.servo.get("Extrusion Servo");
            extrusionL = hwmp.servo.get("Extrusion Left");
            extrusionR = hwmp.servo.get("Extrusion Right");
         */

        //Flips motors because they are placed in the opposite direction on the robot---allows for all motors to move in the same direction for one value
        //rightBack.setDirection(DcMotor.Direction.REVERSE);
        //rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        /*//Set all servo directions
        armRotate.setDirection(Servo.Direction.FORWARD);
        //arm.setDirection(Servo.Direction.FORWARD);
        constrictL.setDirection(Servo.Direction.FORWARD);
        //constrictR.setDirection(Servo.Direction.FORWARD);
        platform.setDirection(Servo.Direction.FORWARD);
        stoneGripper.setDirection(Servo.Direction.FORWARD);
        suctionPlatformR.setDirection(Servo.Direction.FORWARD);
        suctionPlatformR.setDirection(Servo.Direction.FORWARD);
        //platformR.setDirection(Servo.Direction.FORWARD);
        pusher.setDirection(Servo.Direction.FORWARD);*/

    }

    /**
     * Resets drive encoders so that they are starting from 0 at every time
     * Encoders are used to control how much a motor moves---used for travelling by distance and setting levels for lifting
     */
    public void resetDriveEncoders() {

        //Stop and Reset Encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //horizontalLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //verticalLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //greenWheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //greenWheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Start motors using resetted encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //horizontalLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //verticalLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //greenWheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //greenWheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /**
     * Stop all the motors and servos
     * Essentially stop the robot
     * Note: Motor power ranges go from -1 to 1 as a double explicit parameter
     * Note: Servo positions go from 0 to 1 as a double explicit parameter. The actual position of 0 and 1 are based on servo limits programmed by the servo programmer
     */
    public void stopAllMotors() {

        //Motors
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        //verticalLift.setPower(0);
        //horizontalLift.setPower(0);

        //Servos
        //arm.setPosition(0);
        //constrictR.setPosition(0);
        //constrictL.setPosition(0);
        //platform.setPosition(0);
        //stoneGripper.setPosition(0);
        //pusher.setPosition(0);
        //armRotate.setPosition(0);
    }

}