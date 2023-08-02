package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Horizontal {
    public Servo intakeLeft;
    public Servo intakeRight;
    public double extended = 0.7;
    public double retracted =0.2;
    public Servo ArmLeft;
    public Servo ArmRight;
    public Servo UpAndDown;
    public double ArmUp = 0.53;
    public double ArmDown = 0.1;
    public double down = 0.1;
    public Servo intakeClaw;

    public double closed = 0.3;
    public double opened = 0;


    public void initIntake(HardwareMap hardwareMap){
        intakeLeft = hardwareMap.get(Servo.class, " intakeLeft");
        intakeRight = hardwareMap.get(Servo.class, " intakeRight");
        ArmRight = hardwareMap.get(Servo.class, " ArmRight");
        ArmLeft = hardwareMap.get(Servo.class, " ArmLeft");
        UpAndDown = hardwareMap.get(Servo.class, " UpAndDown");
        intakeClaw = hardwareMap.get(Servo.class, " intakeClaw");
        intakeClaw.setPosition(0);
        intakeinit();
    }
    public void ArmDownAll(){

        ArmLeft.setPosition(ArmDown);
        ArmRight.setPosition(ArmDown);
        UpAndDown.setPosition(0.107);
    }
    public void ArmUpAll(){

        ArmLeft.setPosition(ArmUp);
        ArmRight.setPosition(ArmUp);
        UpAndDown.setPosition(0.1);

    }
    public void intakeExtend(){

        intakeRight.setPosition(extended);
        intakeLeft.setPosition(extended);

        ArmRight.setPosition(ArmDown);
        ArmLeft.setPosition(ArmDown);

        UpAndDown.setPosition(0.1);
        intakeClaw.setPosition(0);

    }


    public void intakeRetractTeleOP(){

        intakeRight.setPosition(retracted);
        intakeLeft.setPosition(retracted);

        ArmLeft.setPosition(0.585);
        ArmRight.setPosition(0.585);

        UpAndDown.setPosition(0.01);
    }

    public void intakeinit(){
        intakeRight.setPosition(retracted);
        intakeLeft.setPosition(retracted);

        ArmLeft.setPosition(0.57);
        ArmRight.setPosition(0.57);

        UpAndDown.setPosition(0.05);
    }
}