package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import android.text.method.Touch;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import java.io.DataInput;

/**
 * FTC WIRES TeleOp Example
 *
 */

@TeleOp(name = "FTC Wires TeleOp", group = "00-Teleop")
public class FTCWiresTeleOpMode extends LinearOpMode {
    private DcMotor ArmExtend_MOTOR;
    private DcMotor ArmRotate_MOTOR;
    private Servo ArmFlip_SERVO;
    private Servo PlaneServo;
    private Servo PixelZeroServo;
    private Servo PixelOneServo;
    boolean planeShotInterlock = false;
    private TouchSensor ArmPullLimit;

    private TouchSensor ArmUpLimit;
    private TouchSensor ArmdownLimit;


    public static String TEAM_NAME = "NB GEARS"; //
    public static int TEAM_NUMBER = 22999; //





    @Override
    public void runOpMode() throws InterruptedException {
        double SLOW_DOWN_FACTOR = 1;
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        boolean pixelZeroLock = false;
        boolean pixelOneLock = false; // Logic for PixelLock1
        boolean triggerLock = false;
        int ExtendLimit = 5200;
        int PullLimit = -150;
        int RotateLimit = 0;
        int LastRotatePosition = 0;
        double LastFlipPosition = 1;
        double ArmRotateMUT = 1;
        double ServoZeroPosition = 1;
        double ServoOnePosition = 0.75;
        double ServoFlipPosition = 1;




        ArmExtend_MOTOR = hardwareMap.get(DcMotor.class,"ArmExtend_MOTOR");
        ArmExtend_MOTOR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);;

        ArmRotate_MOTOR = hardwareMap.get(DcMotor.class,"ArmRotate_MOTOR");
        ArmRotate_MOTOR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);;
        ArmRotate_MOTOR.setDirection(DcMotorSimple.Direction.REVERSE);





        PlaneServo = hardwareMap.get(Servo.class, "Plane_SERVO");
        PixelZeroServo = hardwareMap.get(Servo.class, "PixelZero_SERVO");  //TODO map to controller let controllelr know
        PixelOneServo = hardwareMap.get(Servo.class, "PixelOne_SERVO");
        ArmFlip_SERVO = hardwareMap.get(Servo.class,"ArmFlip_SERVO");
        ArmFlip_SERVO.setDirection(REVERSE);

        ArmPullLimit = hardwareMap.get(TouchSensor.class,"ArmPullLimit");
        ArmUpLimit = hardwareMap.get(TouchSensor.class,"ArmUpLimit");
        ArmdownLimit = hardwareMap.get(TouchSensor.class,"ArmDownLimit");




        telemetry.addData("Initializing FTC Wires (ftcwires.org) TeleOp adopted for Team:",TEAM_NUMBER);
        telemetry.update();

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                telemetry.addData("Running FTC Wires (ftcwires.org) TeleOp Mode adopted for Team:",TEAM_NUMBER);
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);
                // Store the gamepad values from this loop iteration in
                // currentGamepad1/2 to be used for the entirety of this loop iteration.
                // This prevents the gamepad values from changing between being
                // used and stored in previousGamepad1/2.
                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);

                if(gamepad2.right_trigger > 0.2){
                    triggerLock = true;
                }else{
                    triggerLock = false;
                }

                if((ArmExtend_MOTOR.getCurrentPosition() > 1500)){ // SLOW DOWN WHILE EXTEND
                    SLOW_DOWN_FACTOR = (0.44/(6000/ArmExtend_MOTOR.getCurrentPosition()));

                }
                else{
                    SLOW_DOWN_FACTOR = 1;
                }
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                (-gamepad1.left_stick_y * SLOW_DOWN_FACTOR + -gamepad2.left_stick_y * SLOW_DOWN_FACTOR),
                                (-gamepad1.left_stick_x * SLOW_DOWN_FACTOR +  -gamepad2.left_stick_x * SLOW_DOWN_FACTOR)
                        ),
                        (-gamepad1.right_stick_x * SLOW_DOWN_FACTOR + -gamepad2.right_stick_x * SLOW_DOWN_FACTOR)*0.75
                ));

                drive.updatePoseEstimate();

                //telemetry.addData("LF Encoder", drive.leftFront.getCurrentPosition());
                //telemetry.addData("LB Encoder", drive.leftBack.getCurrentPosition());
                //telemetry.addData("RF Encoder", drive.rightFront.getCurrentPosition());
                //telemetry.addData("RB Encoder", drive.rightBack.getCurrentPosition());

                 PixelZeroServo.setPosition(ServoZeroPosition); // TODO
                 PixelOneServo.setPosition(ServoOnePosition); //TODO while both this loop it always send command ensure servo at defined postion
                 ArmFlip_SERVO.setPosition(ServoFlipPosition); // TODO


                if(ArmExtend_MOTOR.getCurrentPosition() < 1750)
                {
                    ArmRotateMUT = 1;
                }
                else
                {

                    ArmRotateMUT = 0.3;

                }

                if (gamepad2.right_trigger > 0.1 && (ArmExtend_MOTOR.getCurrentPosition() < ExtendLimit)) { // If left trigger, extend arm propotional to amount with speed
                    ArmExtend_MOTOR.setTargetPosition((int) ((ArmExtend_MOTOR.getCurrentPosition())+(1000 * gamepad2.right_trigger)));
                    ArmExtend_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(ArmExtend_MOTOR.getCurrentPosition() < 2500)
                    { ArmExtend_MOTOR.setPower(1 * gamepad2.right_trigger);

                    }
                    else
                    {
                        ArmExtend_MOTOR.setPower(0.5 * gamepad2.right_trigger);


                    }
                }
                // If right trigger, extend arm proportional to amount with speed
                else if (gamepad2.left_trigger > 0.1 /*&& (ArmExtend_MOTOR.getCurrentPosition() > PullLimit)*/) {
                    ArmExtend_MOTOR.setTargetPosition((int) ((ArmExtend_MOTOR.getCurrentPosition())-(1000 * gamepad2.left_trigger)));
                    ArmExtend_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(ArmExtend_MOTOR.getCurrentPosition() > 2500){
                        ArmExtend_MOTOR.setPower(-1 * gamepad2.left_trigger);}
                    else {
                        ArmExtend_MOTOR.setPower(-0.45 * gamepad2.left_trigger);
                    }

                }

                 else {
                    ArmExtend_MOTOR.setTargetPosition((int) ((ArmExtend_MOTOR.getCurrentPosition())));
                    ArmExtend_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmExtend_MOTOR.setPower(0);

                }
                if(ArmPullLimit.isPressed() && !triggerLock) {
                    ArmExtend_MOTOR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                ///////////////
                if (gamepad2.dpad_up  &&  (ArmUpLimit.isPressed())) { // If left trigger, extend arm propotional to amount with speed
                    LastRotatePosition = ArmRotate_MOTOR.getCurrentPosition();
                    ArmRotate_MOTOR.setTargetPosition((int) ((ArmRotate_MOTOR.getCurrentPosition())+(1000)));
                    ArmRotate_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(ArmRotate_MOTOR.getCurrentPosition() > 1250 || (ArmRotate_MOTOR.getCurrentPosition() < 300)){
                        ArmRotate_MOTOR.setPower(0.15 * ArmRotateMUT);
                        /*    ServoFlipPosition = 90;  //TODO /  map angle
                         */
                    }
                    else {
                        ArmRotate_MOTOR.setPower(0.4 * ArmRotateMUT);
                    }

                }
                // If right trigger, extend arm proportional to amount with speed
                else if (gamepad2.dpad_down  &&  !ArmdownLimit.isPressed()) {
                    LastRotatePosition = ArmRotate_MOTOR.getCurrentPosition();
                    ArmRotate_MOTOR.setTargetPosition((int) ((ArmRotate_MOTOR.getCurrentPosition())-(750)));
                    ArmRotate_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(ArmRotate_MOTOR.getCurrentPosition() > 800 || (ArmRotate_MOTOR.getCurrentPosition() < 300)){
                        ArmRotate_MOTOR.setPower(-0.1 * ArmRotateMUT);
                        /*    ServoFlipPosition = 90;  //TODO /  map angle
                         */
                    }
                    else {
                        ArmRotate_MOTOR.setPower(-0.4 * ArmRotateMUT);
                    }






                }
                else if (gamepad2.dpad_down  &&  ArmdownLimit.isPressed() || ArmdownLimit.isPressed()) {
                    LastRotatePosition = 0;
                    ArmRotate_MOTOR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    ArmRotate_MOTOR.setPower(0);



                }else {
                    ArmRotate_MOTOR.setTargetPosition(LastRotatePosition);
                    ArmRotate_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmRotate_MOTOR.setPower(1);


                }
                if((ArmRotate_MOTOR.getCurrentPosition() > 1000) && gamepad2.dpad_up && ServoFlipPosition != 0 ){
                    ServoFlipPosition = 0;
                    /*    ServoFlipPosition = 90;  //TODO /  map angle
                     */
                } else if (gamepad2.dpad_down && (ArmRotate_MOTOR.getCurrentPosition() < 1000 && ArmRotate_MOTOR.getCurrentPosition() > 500)) {
                    ServoFlipPosition = LastFlipPosition;
                }

                /////////////
                if (gamepad1.x && gamepad2.x ){
                    PlaneShot();
                }
                if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper){ // left pixel logic
                    pixelZeroLock = !pixelZeroLock;
                    if(pixelZeroLock){
                        ServoOnePosition = 1; //

                    }
                    else if(!pixelZeroLock){
                        ServoOnePosition = 0.75; //
                    }
                }
                if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper){// right pixel logic
                    pixelOneLock = !pixelOneLock;
                    if(pixelOneLock){
                        ServoZeroPosition =  0.75; // TODO map
                    }
                    else if(!pixelOneLock){
                        ServoZeroPosition = 1; // TODO Map
                    }
                }
                if((gamepad2.a && ArmRotate_MOTOR.getCurrentPosition() < 1000|| gamepad2.y && ArmRotate_MOTOR.getCurrentPosition() > 1000) && ServoFlipPosition != 1 ){
                    ServoFlipPosition = ServoFlipPosition + 0.01;
                    LastFlipPosition = ServoFlipPosition;
                } else if ((gamepad2.y  && ArmRotate_MOTOR.getCurrentPosition() < 1000 || gamepad2.a && ArmRotate_MOTOR.getCurrentPosition() > 1000) && ServoFlipPosition != 0){
                    ServoFlipPosition = ServoFlipPosition - 0.01;
                    LastFlipPosition = ServoFlipPosition;
                }




                telemetry.addLine("Current Pose");
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", Math.toDegrees(drive.pose.heading.log()));
                telemetry.addData("TooHightDetect",!ArmUpLimit.isPressed());
                telemetry.addData("ToolowDetect",ArmdownLimit.isPressed());
                telemetry.addData("Motor Rotate Encoder",ArmRotate_MOTOR.getCurrentPosition()); /// 1000 at 90
                telemetry.addData("Is rotate Motor Allow?", (ArmExtend_MOTOR.getCurrentPosition() < 500));
                telemetry.addData("Extend Encoder Position (M)", ArmExtend_MOTOR.getCurrentPosition());
                telemetry.addData("Plane Shoted", String.valueOf(planeShotInterlock));
                telemetry.addData("PixelZeroStatus", ServoOnePosition);
                telemetry.addData("PixelOneStatus", ServoZeroPosition);
                telemetry.update();
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) { // DONT CARE
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                                0.0
                        ),
                        -gamepad1.right_stick_x * SLOW_DOWN_FACTOR
                ));

                drive.updatePoseEstimate();






                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", drive.pose.heading);
                telemetry.update();
            }
        } else {
            throw new AssertionError();
        }
    }
    public void PlaneShot(){

        if (planeShotInterlock == false){
            PlaneServo.setPosition(45.0);
            planeShotInterlock = true;
        }
    }
}