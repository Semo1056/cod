package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lift{
    public DcMotorEx liftLeft;
    public DcMotorEx liftRight;
    public Servo outTakeRight;
    public Servo outTakeLeft;
    public Servo outTakeClaw;
    public Servo AlignerServo;
    public double outTakeUp = 0.6;
    public double outTakeDown = 0.035;
    public double closedClaw = 0.6;
    public double openedClaw = 0.95;
    public int medium = 210;
    public int high = 480;
    public void initLiftAuto(HardwareMap hardwareMap){

        AlignerServo = hardwareMap.get(Servo.class,"AlignerServo" );
        outTakeRight = hardwareMap.get(Servo.class, " outTakeRight");
        outTakeLeft = hardwareMap.get(Servo.class, " outTakeLeft");
        outTakeClaw = hardwareMap.get(Servo.class, " outTakeClaw");
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outTakeClaw.setPosition(closedClaw);
        outTakeLeft.setPosition(outTakeDown);
        outTakeRight.setPosition(outTakeDown);
        outTakeRight.setPosition(outTakeDown);
    }

    public void initLiftTeleOp(HardwareMap hardwareMap){

        AlignerServo = hardwareMap.get(Servo.class,"AlignerServo" );
        outTakeRight = hardwareMap.get(Servo.class, " outTakeRight");
        outTakeLeft = hardwareMap.get(Servo.class, " outTakeLeft");
        outTakeClaw = hardwareMap.get(Servo.class, " outTakeClaw");
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        outTakeClaw.setPosition(0.);
        outTakeLeft.setPosition(outTakeDown);
        outTakeRight.setPosition(outTakeDown);
    }

    public void Low(){
        outTakeLeft.setPosition(0.8);
        outTakeRight.setPosition(0.8);
    }

    public void OutTakeUpAuto(){
        outTakeLeft.setPosition(0.65);
        outTakeRight.setPosition(0.65);
    }
    public void OutTakeUp(){
        outTakeLeft.setPosition(0.68);
        outTakeRight.setPosition(0.68);
    }

    public void midPreload(){
        outTakeClaw.setPosition(closedClaw);
        outTakeLeft.setPosition(0.85);
        outTakeRight.setPosition(0.85);

        liftLeft.setTargetPosition(375);
        liftRight.setTargetPosition(375);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(1);
        liftRight.setPower(1);
    }

    public void OutTakeDown(){
        outTakeClaw.setPosition(0.8);
        outTakeLeft.setPosition(0.03);
        outTakeRight.setPosition(0.03);
    }

    public void OutTakeDownTELEOP(){
        outTakeClaw.setPosition(0.84);
        outTakeLeft.setPosition(0.017);
        outTakeRight.setPosition(0.017);
    }
    public void downLift() {

        AlignerServo.setPosition(0.8);
        outTakeClaw.setPosition(0.86);
        outTakeLeft.setPosition(0);
        outTakeRight.setPosition(0);

        liftLeft.setTargetPosition(0);
        liftRight.setTargetPosition(0);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(0.9);
        liftRight.setPower(0.9);
    }

    public void Medium() {

        outTakeClaw.setPosition(closedClaw);
        OutTakeUp();

        liftLeft.setTargetPosition(medium);
        liftRight.setTargetPosition(medium);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(1);
        liftRight.setPower(1);

    }
    public void High() {

        outTakeClaw.setPosition(0.7);

        liftLeft.setTargetPosition(high);
        liftRight.setTargetPosition(high);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(1);
        liftRight.setPower(1);

    }

    public void HighAuto() {

        outTakeClaw.setPosition(0.7);
        AlignerServo.setPosition(1);
        liftLeft.setTargetPosition(469);
        liftRight.setTargetPosition(469);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(1);
        liftRight.setPower(1);

    }

}