package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous(name = "Main Autonomous", group = "Linear OpMode")
public class MainAuton extends LinearOpMode {

    //Create a robot---responsible for connecting hardware of Hardware class to methods
    Hardware robot;

    //Define the Motors and Servos here to not rely on referencing the robot variable to access the motors and servos
    DcMotor leftFront, rightFront, leftBack, rightBack, intakeMotor, leftLauncher, rightLauncher, wobbleGoalArm;
    Servo pushServo;
    CRServo intakeServo;

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
        pushServo = robot.pushServo;
        pushServo.setPosition(Servo.MAX_POSITION);
        intakeServo = robot.intakeServo;

        double movement = 1;
        double magnitude = 1;
        double direction = Math.atan2(0, 1);

        double lf = magnitude * Math.sin(direction + Math.PI / 4);
        double lb = magnitude * Math.cos(direction + Math.PI / 4);
        double rf = magnitude * Math.cos(direction + Math.PI / 4);
        double rb = magnitude * Math.sin(direction + Math.PI / 4);

        double hypot = Math.hypot(movement, 0);
        double ratio;

        ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf)));

        //Motors and servos for wobble goal
        wobbleGoalArm = robot.wobbleGoalArm;

        waitForStart();

        //Picks up wobble goal
        h

        //Stafes left for half a second
        leftFront.setPower(-ratio * lf);
        leftBack.setPower(ratio * lb);
        rightFront.setPower(ratio * rf);
        rightBack.setPower(-ratio * rb);
        sleep(500);

        //Moves forward for one and a half seconds
        leftFront.setPower(ratio * lf);
        leftBack.setPower(ratio * lb);
        rightFront.setPower(ratio * rf);
        rightBack.setPower(ratio * rb);
        sleep(1500);

//        //Jerks to stop robot from turning
//        leftFront.setPower(-ratio * lf);
//        leftBack.setPower(-ratio * lb);
//        rightFront.setPower(-ratio * rf);
//        rightBack.setPower(-ratio * rb);
//        sleep(50);

        //Stafes right for half a second
        leftFront.setPower(ratio * lf);
        leftBack.setPower(-ratio * lb);
        rightFront.setPower(-ratio * rf);
        rightBack.setPower(ratio * rb);
        sleep(500);

        //Stops all motors
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        rightLauncher.setPower(1);
        leftLauncher.setPower(-1);
        sleep(500);
        //Launches 3 disks
        for (int i = 0; i < 3; i++) {
            pushServo.setPosition(Servo.MIN_POSITION);
            sleep(250);
            pushServo.setPosition(Servo.MAX_POSITION);
            sleep(250);
        }

        rightLauncher.setPower(0);
        leftLauncher.setPower(0);

        

        if (EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.ONE()) {
            ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf))) / 2;

            //Robot moves back to intake 1 ring
            leftFront.setPower(-ratio * lf);
            leftBack.setPower(-ratio * lb);
            rightFront.setPower(-ratio * rf);
            rightBack.setPower(-ratio * rb);
            intakeMotor.setPower(1);
            intakeServo.setPower(-1);
            sleep(150);

            //move forward and shoot the ring
            leftFront.setPower(ratio * lf);
            leftBack.setPower(ratio * lb);
            rightFront.setPower(ratio * rf);
            rightBack.setPower(ratio * rb);
            rightLauncher.setPower(1);
            leftLauncher.setPower(-1);
            sleep(75);
            rightLauncher.setPower(1);
            leftLauncher.setPower(-1);
            for (int i = 0; i < 1; i++) {
                pushServo.setPosition(Servo.MIN_POSITION);
                sleep(250);
                pushServo.setPosition(Servo.MAX_POSITION);
                sleep(250);
            }

            //take wobble goal to designated box(first one to the right)
            leftFront.setPower(ratio * lf);
            leftBack.setPower(ratio * lb);
            rightFront.setPower(ratio * rf);
            rightBack.setPower(ratio * rb);
            sleep(1000);
            leftFront.setPower(ratio * lf);
            leftBack.setPower(-ratio * lb);
            rightFront.setPower(-ratio * rf);
            rightBack.setPower(ratio * rb);
            sleep(500);
            //INPUT CODE THAT MAKES THE ROBOT DROP THE WOBBLE GOAL DOWN

            //park
            leftFront.setPower(-ratio * lf);
            leftBack.setPower(-ratio * lb);
            rightFront.setPower(-ratio * rf);
            rightBack.setPower(-ratio * rb);
            sleep(75);
        }
        else if EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR()) {
            ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf))) / 5;
            //Robot moves back to intake 3 out of the 4 rings
            leftFront.setPower(-(ratio * lf));
            leftBack.setPower(-(ratio * lb));
            rightFront.setPower(-(ratio * rf));
            rightBack.setPower(-(ratio * rb));
            intakeMotor.setPower(1);
            intakeServo.setPower(-1);
            sleep(150);

            //move forward and shoot the rings
            leftFront.setPower(ratio * lf);
            leftBack.setPower(ratio * lb);
            rightFront.setPower(ratio * rf);
            rightBack.setPower(ratio * rb);
            sleep(75);
            rightLauncher.setPower(1);
            leftLauncher.setPower(-1);
            for (int i = 0; i < 3; i++) {
                pushServo.setPosition(Servo.MIN_POSITION);
                sleep(250);
                pushServo.setPosition(Servo.MAX_POSITION);
                sleep(250);

            //take wobble goal to designated box(first middle one)
            leftFront.setPower(ratio * lf);
            leftBack.setPower(ratio * lb);
            rightFront.setPower(ratio * rf);
            rightBack.setPower(ratio * rb);
            sleep(1000);
            //INPUT CODE THAT MAKES THE ROBOT DROP THE WOBBLE GOAL DOWN

            //park
            leftFront.setPower(-ratio * lf);
            leftBack.setPower(-ratio * lb);
            rightFront.setPower(-ratio * rf);
            rightBack.setPower(-ratio * rb);
            sleep(75);
        }
        else if (EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.NONE) {
                //take wobble goal to designated box(first last one)
                leftFront.setPower(ratio * lf);
                leftBack.setPower(ratio * lb);
                rightFront.setPower(ratio * rf);
                rightBack.setPower(ratio * rb);
                sleep(1000);
                leftFront.setPower(ratio * lf);
                leftBack.setPower(-ratio * lb);
                rightFront.setPower(-ratio * rf);
                rightBack.setPower(ratio * rb);
                sleep(500);
                //INPUT CODE THAT MAKES THE ROBOT DROP THE WOBBLE GOAL DOWN

                //park
                leftFront.setPower(-ratio * lf);
                leftBack.setPower(-ratio * lb);
                rightFront.setPower(-ratio * rf);
                rightBack.setPower(-ratio * rb);
                sleep(75);
        }
    }
}
