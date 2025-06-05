package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class XDrive extends OpMode {
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor coreHexL;
    DcMotor coreHexR;
    Servo garra;
    Servo intake;

    @Override
    public void init(){
        motorFL = hardwareMap.get(DcMotor.class, "FL");
        motorFR = hardwareMap.get(DcMotor.class, "FR");
        motorBL = hardwareMap.get(DcMotor.class, "BL");
        motorBR = hardwareMap.get(DcMotor.class, "BR");
        coreHexL = hardwareMap.get(DcMotor.class, "CHexL");
        coreHexR = hardwareMap.get(DcMotor.class, "CHexR");
        garra = hardwareMap.get(Servo.class, "garra");
        intake = hardwareMap.get(Servo.class, "intake");

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        coreHexL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        coreHexR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        coreHexL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        coreHexR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Hardware: ", "Initialized");
    }

    // public void init_loop() {}
    // public void start(){}

    @Override
    public void loop(){
        telemetry.addData("Hardware: ", "Running");

        double drive = gamepad1.left_stick_y;  // frente e atrás
        double turn = gamepad1.right_stick_x;  // gira
        double strafe = gamepad1.left_stick_x; // direita e esquerda
        double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;
        double power = 1 - (0.5 * gamepad1.right_trigger);

        leftFrontPower = Range.clip(drive - strafe - turn, -power, power);
        rightFrontPower = Range.clip(drive - strafe + turn, -power, power);
        leftBackPower = Range.clip(drive + strafe - turn, -power, power);
        rightBackPower = Range.clip(drive + strafe + turn, -power, power);

        motorFL.setPower(leftFrontPower);
        motorFR.setPower(rightFrontPower);
        motorBL.setPower(leftBackPower);
        motorBR.setPower(rightBackPower);
    }
}
