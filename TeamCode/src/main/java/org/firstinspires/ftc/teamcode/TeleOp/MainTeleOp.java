package org.firstinspires.ftc.teamcode.TeleOp;

//import android.os.Handler;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
@TeleOp(name = "Testing TeleOp", group = "Linear Opmode")

/**
 * MainTeleOp is the class responsible for all of the TeleOp methods. It has a robot, movement, rotation, strafe, eight motors, and five servos
 */
public class MainTeleOp extends OpMode {

    //Create a robot---responsible for connecting hardware of Hardware class to methods
    Hardware robot;

    //private ElaspedTime runTime; for if you need to drive by time

    //Directions
    double movement;
    double rotation;
    double strafe;

    //Define the Motors and Servos here to not rely on referencing the robot variable to access the motors and servos
    DcMotor leftFront, rightFront, leftBack, rightBack, intakeMotor, liftMotor, launcherMotor, greenWheelLeft, greenWheelRight, horizontalLift, verticalLift;
    Servo arm, platform, armRotate, stoneGripper,suctionPlatformR, suctionPlatformL, constrictL, gate, pusher; //extrusionL, extrusionR;

    @Override
    /**
     * Initializes the robot by mapping the hardware, resetting encoders, and setting servos to the correct starting positions
     */
    public void init() {
        //Map hardware
        robot = new Hardware(hardwareMap);
        //Assign the motors and servos to the ones on the robot to not require calling robot everytime a method or servo needs to be called.
        leftFront = robot.leftFront;
        rightFront = robot.rightFront;
        rightBack = robot.rightBack;
        leftBack = robot.leftBack;

        intakeMotor = robot.intakeMotor;
        liftMotor = robot.liftMotor;
        launcherMotor = robot.launcherMotor;

        /*pusher = robot.pusher;
        greenWheelLeft = robot.greenWheelLeft;
        greenWheelRight = robot.greenWheelRight;
        horizontalLift = robot.horizontalLift;
        verticalLift = robot.verticalLift;
        platform = robot.platform;
        armRotate = robot.armRotate;
        stoneGripper = robot.stoneGripper;
        suctionPlatformL = robot.suctionPlatformL;
        suctionPlatformR = robot.suctionPlatformR;
        constrictL = robot.constrictL;
        //constrictR = robot.constrictR;
        gate = robot.gate;
        //extrusionL = robot.extrusionL;
        //extrusionR = robot.extrusionR;

        suctionPlatformR.setPosition(Servo.MAX_POSITION);
        suctionPlatformL.setPosition(Servo.MAX_POSITION);
        //Sets encoders back to 0 so that they are not messed up.
        robot.resetDriveEncoders();

        //Set starting position for arm servo
        arm.setPosition(Servo.MAX_POSITION);
        platform.setPosition(Servo.MIN_POSITION);

        //constrictL.setPosition(Servo.MIN_POSITION);
        pusher.setPosition(Servo.MAX_POSITION);*/


        //Variable to track time for running robot on time if needed
        //ElapsedTime runtime = new ElapsedTime();
    }

    @Override
    /**
     * Runs the main methods of TeleOp and telemetry.
     * Loop repeats so that it is checking controllers and telemetry values at all times for continuous running
     */
    public void loop() {
        //Methods responsible for control of different parts of the the robot

        //Test();
        DriveControl();
        //ArmControl();
        //LiftControl();
        //PlatformControl();
        Intake();
        //automatedMotions();

        //Intake wheels should be running at all times so that the builders

        //horizontalLift.setPower(0.2);
        //TODO ask about horizontal lift constant motion && the specificity of the compliance wheels

        /*
        telemetry.addData("Rotation Times", verticalLift.getCurrentPosition());

        telemetry.addData("Lift Encoder", verticalLift.getCurrentPosition());
        telemetry.addData("Horizontal Encoder", horizontalLift.getCurrentPosition());
        telemetry.addData("Horizontal Power", horizontalLift.getPower());

        telemetry.addData("Right Wheel Motor Power", greenWheelRight.getPower());
        telemetry.addData("Left Wheel Motor Power", greenWheelLeft.getPower());
        //telemetry.addData("Right Wheel Motor Encoder", greenWheelRight.getCurrentPosition());
        //telemetry.addData("Left Wheel Motor Encoder", greenWheelLeft.getCurrentPosition());

        telemetry.addData("Servo :((", constrictL.getPosition());


        //Possible Telemetrys
        /* telemetry.addData("Arm Position", robot.arm.getPosition());
           telemetry.addData("Left Drive Position", robot.leftBack.getCurrentPosition());
           telemetry.addData("Right Drive Position", robot.rightBack.getCurrentPosition());
           telemetry.addData("Left JoyStick Y", gamepad1.left_stick_y);
           telemetry.addData("Right JoyStick X", gamepad1.right_stick_x);
           telemetry.addData("Left Back Power", robot.leftBack.getPower());
           telemetry.addData("Left Front Power", robot.leftFront.getPower());
           telemetry.addData("Right Back Power", robot.rightBack.getPower());
           telemetry.addData("Right Front Power", robot.rightFront.getPower());*/

        //Show Telemetry on Driver Station Phone
        telemetry.update();
    }

    //Driving Control function
    public void Test(){
        rightBack.setPower(.5);
        rightFront.setPower(.5);
    }

    public void Intake(){
        /*boolean forward = true;
        boolean beginning = true;
        double speed = 0.7;
        double spit = 1;
        if(gamepad2.b)
        {
            beginning=false;
        }
        if(beginning)
        {
            //greenWheelRight.setPower(-1*spit);
            //greenWheelLeft.setPower(spit);
            greenWheelRight.setPower(spit);
            greenWheelLeft.setPower(-1*spit);
        }
        if(gamepad2.a)
            forward = false;
        else
            forward = true;
        if(!beginning && forward)
        {
            //greenWheelRight.setPower(speed);
            //greenWheelLeft.setPower(-1*speed);
            greenWheelRight.setPower(-1*speed);
            greenWheelLeft.setPower(speed);
        }
        else if(!beginning && !forward)
        {
            //greenWheelRight.setPower(-1*spit);
            //greenWheelLeft.setPower(spit);
            greenWheelRight.setPower(spit);
            greenWheelLeft.setPower(-1*spit);
        }*/

        if(gamepad1.left_trigger > 0) //push out
        {
            intakeMotor.setPower(-1);
            liftMotor.setPower(1);
            launcherMotor.setPower(1);
        }
        if(gamepad1.left_bumper) //suck in
        {
            intakeMotor.setPower(1);
            liftMotor.setPower(-1);
            launcherMotor.setPower(-1);
        }
        if(gamepad1.x) //stop
        {
            intakeMotor.setPower(0);
            liftMotor.setPower(0);
            launcherMotor.setPower(0);
        }
    }

    public void automatedMotions(){
        //TODO Move verticalLifts and armRotate
        if(gamepad2.x) {
            stoneGripper.setPosition(Servo.MAX_POSITION);
            pusher.setPosition(Servo.MIN_POSITION);
        }if (gamepad2.y){
            stoneGripper.setPosition(Servo.MIN_POSITION);
        }
    }

    public void DriveControl() {
        movement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double direction = Math.atan2(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double rotation = gamepad1.right_stick_x;
        boolean precision = gamepad1.right_bumper;

        //trig implementation
        //double power = Math.hypot(x1, y1);
        //double angle = Math.atan2(y1, x1) - Math.PI/4;

        //INFO Increasing speed to maximum of 1
        double lf = magnitude * Math.sin(direction + Math.PI / 4) - rotation;
        double lb = magnitude * Math.cos(direction + Math.PI / 4) - rotation;
        double rf = magnitude * Math.cos(direction + Math.PI / 4) + rotation;
        double rb = magnitude * Math.sin(direction + Math.PI / 4) + rotation;
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

    //Function for handling horizontal lift
    public void LiftControl() {

        double vertical = gamepad2.left_stick_y;
        double horizontal = gamepad2.right_stick_x;

        boolean leftBumper = gamepad2.left_bumper;
        boolean rightBumper = gamepad2.right_bumper;


        if(leftBumper)
            stoneGripper.setPosition(Servo.MAX_POSITION);
        if(rightBumper)
            stoneGripper.setPosition(Servo.MIN_POSITION);

        //README intakes
        // robot.horizontalLift.setPower(horizontal);
        //robot.verticalLift.setPower((verticalLift.getCurrentPosition() < -300
        //|| verticalLift.getCurrentPosition() > 2260) ? 0 : vertical);


        //avcrobot.verticalLift.setPower(vertical);

        //TODO Move verticalLift by increments when dpad_up and dpad_down pressed.

        //README Suction wheels
        /*robot.greenWheelLeft.setPower(0.1);
        robot.greenWheelRight.setPower(0.1);
        robot.greenWheelLeft.setPower(0.1);
        robot.greenWheelRight.setPower(0.1);*/
        /*robot.greenWheelLeft.setPower((gamepad2.y) ? -1 : 0);
        robot.greenWheelRight.setPower((gamepad2.y) ? -1 : 0);*/
//aa
        //README Gate open/close using triggers and constriction.


        //INFO Do constriction and close gate.=
        /*if (gamepad2.dpad_left) {
            pusher.setPosition(Servo.MAX_POSITION);
        }
        if(gamepad2.dpad_right)
            pusher.setPosition(Servo.MIN_POSITION);*/
            /*robot.gate.setPosition(1);
            //Delay 0.8 second
            Handler h = new Handler();
            Runnable r = new Runnable() {
                @Override
                public void run() {
                    robot.constrictL.setPosition(1);
                }
            };
            h.postDelayed(r, 800);*/
        //INFO Undo constriction and open gate.

//else if (gamepad2.x) {
            /*robot.gate.setPosition(0);
            //Delay 0.8 second
            Handler h = new Handler();
            Runnable r = new Runnable() {
                @Override
                public void run() {
                    robot.constrictL.setPosition(0);
                }
            };
            h.postDelayed(r, 800);*/


        /*if(gamepad2.dpad_up)
            constrictL.setPosition(Servo.MIN_POSITION);
        if(gamepad2.dpad_down)
            constrictL.setPosition(Servo.MAX_POSITION);*/


//        if (gamepad2.dpad_left)
//            robot.horizontalLift.setPower(-.7);
//        else if (gamepad2.dpad_right)
//            robot.horizontalLift.setPower(.7);
        //else
        //robot.horizontalLift.setPower(0);
    }

    //Function for handling vertical lift
    /*public void VerticalLiftControl(){
        if (gamepad1.a)
            robot.verticalLift.setPower(-.7);
        else if (gamepad1.b)
            robot.verticalLift.setPower(.7);
        else
            robot.verticalLift.setPower(0);
    }*/

    boolean up = true;

    public void ArmControl() {
        if(gamepad1.a)
            armRotate.setPosition(Servo.MAX_POSITION);
        if(gamepad1.b)
            armRotate.setPosition(Servo.MIN_POSITION);

        /*if (gamepad1.x) {
            robot.arm.setPosition(Servo.MAX_POSITION);
            robot.arm.setPosition(Servo.MAX_POSITION);
        }
        if (gamepad1.y) {
            robot.arm.setPosition(Servo.MIN_POSITION);
            robot.arm.setPosition(Servo.MIN_POSITION);
        }*/
    }

    public void PlatformControl()
    {
        if(gamepad1.x) {
            platform.setPosition(Servo.MAX_POSITION);
            //platformR.setPosition(Servo.MAX_POSITION);
            //platformL.setPosition(Servo.MIN_POSITION);
        }
        if(gamepad1.y) {
            platform.setPosition(Servo.MIN_POSITION);
//platformR.setPosition(Servo.MIN_POSITION);
            //platformL.setPosition(Servo.MAX_POSITION);
        }
    }
}