package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp(name = "rkslegop1 (Blocks to Java)")

public class AAArkslegop extends LinearOpMode {

  private DcMotor lift;
  private DcMotor back_left;
  private DcMotor back_right;
  private DcMotor front_left;
  private DcMotor front_right;
  private Servo claw;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double requestedRadialTranslation;
    double linearTranslationCoeff;
    double radialTranslationCoeff;
    float requestedLinearXTranslation;
    float requestedLinearYTranslation;
    ElapsedTime Delay;

    lift = hardwareMap.get(DcMotor.class, "lift");
    back_left = hardwareMap.get(DcMotor.class, "back_left");
    back_right = hardwareMap.get(DcMotor.class, "back_right");
    front_left = hardwareMap.get(DcMotor.class, "front_left");
    front_right = hardwareMap.get(DcMotor.class, "front_right");
    claw = hardwareMap.get(Servo.class, "claw");

    // Put initialization blocks here.
    lift.setDirection(DcMotorSimple.Direction.REVERSE);
    lift.setPower(-0.1);
    lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Delay = new ElapsedTime();
    Delay.reset();
    telemetry.addData("DelayKey", Delay.seconds());
    telemetry.update();
    while (Delay.seconds() < 2) {
      telemetry.addData("DelayKeyLoop", Delay.seconds());
      telemetry.update();
    }
    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ((DcMotorEx) lift).setVelocityPIDFCoefficients(0, 0, 0, 55) ;
    lift.setPower(1);
    lift.setTargetPosition((int) ((384.5 / 4.4095) * 38));
    linearTranslationCoeff = 0.210526;
    radialTranslationCoeff = 7.91578;
    back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    front_left.setDirection(DcMotorSimple.Direction.FORWARD);
    front_right.setDirection(DcMotorSimple.Direction.REVERSE);
    back_left.setDirection(DcMotorSimple.Direction.FORWARD);
    back_right.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Shrek
      while (opModeIsActive()) {
        // Put loop blocks here.
        if (gamepad1.a) {
          claw.setPosition(3 / 190);
        } else {
          claw.setPosition(10 / 190);
        }
        if (gamepad1.b) {
          requestedLinearXTranslation = gamepad1.left_stick_x * 2;
          requestedLinearYTranslation = gamepad1.left_stick_y * 2;
          requestedRadialTranslation = gamepad1.right_stick_x * 0.1;
        } else {
          requestedLinearXTranslation = gamepad1.left_stick_x * 5;
          requestedLinearYTranslation = gamepad1.left_stick_y * 5;
          requestedRadialTranslation = gamepad1.right_stick_x * 0.2;
        }
        front_right.setPower(requestedLinearXTranslation * linearTranslationCoeff + requestedLinearYTranslation * linearTranslationCoeff + requestedRadialTranslation * radialTranslationCoeff);
        front_left.setPower(requestedLinearXTranslation * -1 * linearTranslationCoeff + requestedLinearYTranslation * linearTranslationCoeff + requestedRadialTranslation * -1 * radialTranslationCoeff);
        back_right.setPower(requestedLinearXTranslation * -1 * linearTranslationCoeff + requestedLinearYTranslation * linearTranslationCoeff + requestedRadialTranslation * radialTranslationCoeff);
        back_left.setPower(requestedLinearXTranslation * linearTranslationCoeff + requestedLinearYTranslation * linearTranslationCoeff + requestedRadialTranslation * -1 * radialTranslationCoeff);
        telemetry.addData("position of claw", Double.parseDouble(JavaUtil.formatNumber(claw.getPosition(), 2)));
        telemetry.addData("0 ω", Double.parseDouble(JavaUtil.formatNumber(requestedLinearXTranslation * linearTranslationCoeff + requestedLinearYTranslation * linearTranslationCoeff + requestedRadialTranslation * radialTranslationCoeff, 2)));
        telemetry.addData("0 ω - true (Shrek)", Double.parseDouble(JavaUtil.formatNumber(front_right.getPower(), 2)));
        telemetry.addData("0 position", Double.parseDouble(JavaUtil.formatNumber(front_right.getCurrentPosition(), 2)));
        telemetry.addData("1 ω", Double.parseDouble(JavaUtil.formatNumber(requestedLinearXTranslation * linearTranslationCoeff + requestedLinearYTranslation * linearTranslationCoeff + requestedRadialTranslation * radialTranslationCoeff, 2)));
        telemetry.addData("1 ω - true", Double.parseDouble(JavaUtil.formatNumber(front_left.getPower(), 2)));
        telemetry.addData("1 position", Double.parseDouble(JavaUtil.formatNumber(front_left.getCurrentPosition(), 2)));
        telemetry.addData("2 ω", Double.parseDouble(JavaUtil.formatNumber(requestedLinearXTranslation * linearTranslationCoeff + requestedLinearYTranslation * linearTranslationCoeff + requestedRadialTranslation * radialTranslationCoeff, 2)));
        telemetry.addData("2 ω - true", Double.parseDouble(JavaUtil.formatNumber(back_right.getPower(), 2)));
        telemetry.addData("2 position", Double.parseDouble(JavaUtil.formatNumber(back_right.getCurrentPosition(), 2)));
        telemetry.addData("3 ω", Double.parseDouble(JavaUtil.formatNumber(requestedLinearXTranslation * linearTranslationCoeff + requestedLinearYTranslation * linearTranslationCoeff + requestedRadialTranslation * radialTranslationCoeff, 2)));
        telemetry.addData("3 ω - true", Double.parseDouble(JavaUtil.formatNumber(back_left.getPower(), 2)));
        telemetry.addData("3 position", Double.parseDouble(JavaUtil.formatNumber(back_left.getCurrentPosition(), 2)));
        telemetry.addData("Joystick Y", Double.parseDouble(JavaUtil.formatNumber(requestedLinearYTranslation, 2)));
        telemetry.addData("Joystick X", Double.parseDouble(JavaUtil.formatNumber(requestedLinearXTranslation, 2)));
        telemetry.update();
      }
    }
  }
}
