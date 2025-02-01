package org.firstinspires.ftc.teamcode.Auto;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PID.Arm_PID_Class;

public class Arm {

    public DcMotorEx Arm;
    public int setPosition;

    Servo link;
    Servo inY;
    Servo inX;
    Servo inClaw;
    Servo inmainPiv;
    Servo outY;
    Servo outRot;
    Servo outClaw;

    public Arm(HardwareMap hardwareMap) {


        Arm = hardwareMap.get(DcMotorEx.class, "Arm");
        Arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        link = hardwareMap.servo.get("link");
        inY = hardwareMap.servo.get("inY");
        inX = hardwareMap.servo.get("inX");
        inClaw = hardwareMap.servo.get("inClaw");
        inmainPiv = hardwareMap.servo.get("inmainPiv");
        outY = hardwareMap.servo.get("outY");
        outRot = hardwareMap.servo.get("outRot");
        outClaw = hardwareMap.servo.get("outClaw");

        link.setDirection(Servo.Direction.REVERSE);

    }

    public class updatePID implements Action {
        public updatePID(){

        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Arm.setPower(Arm_PID_Class.returnArmPID(setPosition,Arm.getCurrentPosition()));
            return true;
        }
    }
    public Action UpdatePID(){return new updatePID();}

    public class setPosition implements Action {
        int set;
        public setPosition(int position){set = position;}
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setPosition = set;
            return false;
        }
    }
    public Action SetPosition(int pos){return new setPosition(pos);}
////////////////////////////////////////////////////////////////////////////////////////////////
    public class armDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            outY.setPosition(0.4);
            outRot.setPosition(0.27);

            return false;
        }
    }
    public Action armDown() {
        return new Arm.armDown();
    }
///////////////////////////////////////////////////////////////////////////////////////////////
    public class armUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            outY.setPosition(0.55);
            outRot.setPosition(0.95);
            outClaw.setPosition(0.72);


            return false;
        }
    }
    public Action armUp() {
        return new Arm.armUp();
    }
//////////////////////////////////////////////////////////////////////////////////////
    public class openClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            outClaw.setPosition(0.5);



            return false;
        }
    }
    public Action openClaw() {
        return new Arm.openClaw();
    }

    //////////////////////////////////////////////////////////////////////////////////////
    public class lockIntake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            link.setPosition(0.62);



            return false;
        }
    }
    public Action lockIntake() {
        return new Arm.lockIntake();
    }


    public class closeClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            outClaw.setPosition(0.72);



            return false;
        }
    }
    public Action closeClaw() {
        return new Arm.closeClaw();
    }

}