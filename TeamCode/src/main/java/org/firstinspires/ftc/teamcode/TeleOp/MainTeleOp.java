package org.firstinspires.ftc.teamcode.TeleOp;

//import android.os.Handler;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;

/*
  Notes:
    Encoders are doubles
    .getCurrentPosition() retrieves encoder values
    left and right triggers on controllers are scaled 0-1
    .getMode() exists
 */
@TeleOp(name = "Main TeleOp", group = "Linear OpMode")

/**
  MainTeleOp is the class responsible for all of the TeleOp methods. It has a robot, movement, rotation, strafe, eight motors, and five servos
 */
public class MainTeleOp extends OpMode {

    //Create a robot---responsible for connecting hardware of Hardware class to methods
    Hardware robot;

    //Directions
    double movement;
    double rotation;
    double strafe;

    //Define the Motors and Servos here to not rely on referencing the robot variable to access the motors and servos
    DcMotor leftFront, rightFront, leftBack, rightBack, intakeMotor, leftLauncher, rightLauncher, wobbleGoalArm;
    Servo wobbleGoalGrasp, pushServo;

    @Override
    /**
     * Initializes the robot by mapping the hardware, resetting encoders, and setting servos to the correct starting positions
     */
    public void init() {
        //Map hardware
        robot = new Hardware(hardwareMap);

        //Assign the motors and servos to the ones on the robot to not require calling robot everytime a method or servo needs to be called.
        //Motors for drive train
        leftFront = robot.leftFront;
        rightFront = robot.rightFront;
        rightBack = robot.rightBack;
        leftBack = robot.leftBack;

        //Motors and servos for intake
        intakeMotor = robot.intakeMotor;
        rightLauncher = robot.rightLauncher;
        leftLauncher = robot.leftLauncher;
        pushServo = robot.pushServo;
        pushServo.setPosition(Servo.MAX_POSITION);

        //Motors and servos for wobble goal
        //wobbleGoalArm = robot.wobbleGoalArm;
        //wobbleGoalGrasp = robot.wobbleGoalGrasp;
    }

    @Override
    /**
     * Runs the main methods of TeleOp and telemetry.
     * Loop repeats so that it is checking controllers and telemetry values at all times for continuous running
     */
    public void loop() {
        //Methods responsible for control of different parts of the the robot

        Intake();
        //WobbleGoal();
        DriveControl();

        //Show Telemetry on Driver Station Phone
        telemetry.update();
    }

    public void Intake() {
        //Turns on Intake and shooting
        if(gamepad1.right_trigger > 0)
        {
            intakeMotor.setPower(1);
        }
        else if(gamepad1.left_trigger > 0)
        {
            intakeMotor.setPower(-1);
        }
        else
        {
            intakeMotor.setPower(0);
        }

        if(gamepad1.left_bumper)
        {
            rightLauncher.setPower(1);
            leftLauncher.setPower(-1);
        }
        else
        {
            rightLauncher.setPower(0);
            leftLauncher.setPower(0);
        }

        //Turns on servo to push disks into launcher
        if(gamepad1.a)
        {
            pushServo.setPosition(Servo.MIN_POSITION);
        }
        if(gamepad1.b)
        {
            pushServo.setPosition(Servo.MAX_POSITION);
        }
    }

    /*
    public void WobbleGoal() {
        if(gamepad1.dpad_right)
        {
            wobbleGoalGrasp.setPosition(Servo.MAX_POSITION);
        }
        if(gamepad1.dpad_left)
        {
            wobbleGoalGrasp.setPosition(Servo.MIN_POSITION);
        }
        if(gamepad1.dpad_up) //need to get values for min and max positions
        {
            telemetry.addData("Wobble Goal Arm Position: ", wobbleGoalArm.getCurrentPosition());
        }
        if(gamepad1.dpad_down)
        {
            telemetry.addData("Wobble Goal Arm Position: ", wobbleGoalArm.getCurrentPosition());
        }
    }
     */

    public void DriveControl() {
        movement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double direction = Math.atan2(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double rotation = gamepad1.right_stick_x;
        boolean precision = gamepad1.right_bumper;

        //INFO Increasing speed to a maximum of 1
        /*double lf = magnitude * Math.sin(direction + Math.PI / 4) - rotation;
        double lb = magnitude * Math.cos(direction + Math.PI / 4) - rotation;
        double rf = magnitude * Math.cos(direction + Math.PI / 4) + rotation;
        double rb = magnitude * Math.sin(direction + Math.PI / 4) + rotation;*/

        //I'm just desperate at this point
        double lf = magnitude * Math.sin(direction + 1.07) - rotation;
        double lb = magnitude * Math.cos(direction + 0.485) - rotation;
        double rf = magnitude * Math.cos(direction + 0.485) + rotation;
        double rb = magnitude * Math.sin(direction + 1.07) + rotation;

        double hypot = Math.hypot(movement, strafe);
        double ratio;
        if (movement == 0 && strafe == 0)
            ratio = 1;
        else if(precision)
            ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf))) / 2;
        else
            ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf)));

        leftFront.setPower(ratio * lf);
        leftBack.setPower(ratio * lb);
        rightFront.setPower(ratio * rf);
        rightBack.setPower(ratio * rb);
    }
}