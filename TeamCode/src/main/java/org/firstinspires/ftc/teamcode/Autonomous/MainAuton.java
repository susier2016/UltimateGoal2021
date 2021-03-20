package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous(name = "Main Autonomous", group = "Linear OpMode")
public class MainAuton extends LinearOpMode {

    //Create a robot---responsible for connecting hardware of Hardware class to methods
    Hardware robot;

    //Define the Motors and Servos here to not rely on referencing the robot variable to access the motors and servos
    DcMotor leftFront, rightFront, leftBack, rightBack, intakeMotor, leftLauncher, rightLauncher, wobbleGoalArm;
    Servo pushServo;

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

        //Motors and servos for wobble goal
        //wobbleGoalArm = robot.wobbleGoalArm;

        //Robot moves up and shoots 3 preloaded rings into goals
        leftFront.setPower(-1);
        leftBack.setPower(-1);
        rightFront.setPower(1);
        rightBack.setPower(1);

        sleep(2500);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        rightLauncher.setPower(1);
        leftLauncher.setPower(1);

        //Launches 3 disks
        for (int i = 0; i < 3; i++) {
            pushServo.setPosition(Servo.MAX_POSITION);
            sleep(500);
            pushServo.setPosition(Servo.MIN_POSITION);
        }

        sleep(10000);

        //Robot moves back and lands on parking line
        leftFront.setPower(1);
        leftBack.setPower(1);
        rightFront.setPower(-1);
        rightBack.setPower(-1);

        sleep(2500);

        //ADD MOVEMENT TO RIGHT OR LEFT IF NEEDED
    }
}
