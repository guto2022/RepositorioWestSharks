package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class TesteDEMotor extends OpMode {
    DcMotor motorD;
    DcMotor motorE;

    public void init(){
        motorD = hardwareMap.get(DcMotor.class, "Front_right");
        motorE = hardwareMap.get(DcMotor.class, "Front_left");
        motorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorD.setDirection(DcMotorSimple.Direction.REVERSE);
        motorE.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.addData("Hardware: ", "Initialized");
    }

    public void loop(){
        telemetry.addData("Hardware: ", "Running");

        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double rightSidePower, leftSidePower;

        leftSidePower = Range.clip(drive - turn, -1, 1);
        rightSidePower = Range.clip(drive + turn, -1, 1);

        motorD.setPower(rightSidePower);
        motorE.setPower(leftSidePower);




    }
}
