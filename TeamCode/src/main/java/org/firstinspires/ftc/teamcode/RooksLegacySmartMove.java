package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.intel.realsense.librealsense.Config;
import com.intel.realsense.librealsense.StreamType;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import ma.phoenix.ftc.realsensecamera.ConfigurableRealSenseCamera;
import ma.phoenix.ftc.realsensecamera.FrameData;
import ma.phoenix.ftc.realsensecamera.exceptions.CameraStartException;
import ma.phoenix.ftc.realsensecamera.exceptions.CameraStopException;
import ma.phoenix.ftc.realsensecamera.exceptions.DisconnectedCameraException;
import ma.phoenix.ftc.realsensecamera.exceptions.FrameQueueCloseException;
import ma.phoenix.ftc.realsensecamera.exceptions.NoFrameSetYetAcquiredException;
import ma.phoenix.ftc.realsensecamera.exceptions.StreamTypeNotEnabledException;
import ma.phoenix.ftc.realsensecamera.exceptions.UnsupportedStreamTypeException;

@TeleOp(name = "Rooks Legacy Smart Move")
public class RooksLegacySmartMove extends LinearOpMode {

  private DcMotor lift;
  private DcMotor leftRear;
  private DcMotor rightRear;
  private DcMotor leftFront;
  private DcMotor rightFront;
  private Servo claw;
  private SampleMecanumDrive drive;

  double Encoder_ticks_per_rotation;
  double Circumference_in_Inches;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    try (ConfigurableRealSenseCamera camera = new ConfigurableRealSenseCamera(hardwareMap, () -> isStopRequested())) {

      ElapsedTime delay;
      int max;
      double linearTranslationCoeff;
      double radialTranslationCoeff;
      int liftTest;
      float requestedLinearXTranslation;
      float requestedLinearYTranslation;
      double requestedRadialTranslation;
      drive = new SampleMecanumDrive(hardwareMap);

      lift = hardwareMap.get(DcMotor.class, "lift");
      leftRear = hardwareMap.get(DcMotor.class, "leftRear");
      rightRear = hardwareMap.get(DcMotor.class, "rightRear");
      leftFront = hardwareMap.get(DcMotor.class, "leftFront");
      rightFront = hardwareMap.get(DcMotor.class, "rightFront");
      claw = hardwareMap.get(Servo.class, "claw");

      ((DcMotorEx) lift).setVelocityPIDFCoefficients(0, 0, 0, 12.411);
      ((DcMotorEx) lift).setPositionPIDFCoefficients(15);
      // Put initialization blocks here.
      Circumference_in_Inches = 4.409;
      Encoder_ticks_per_rotation = 384.5;
      lift.setDirection(DcMotorSimple.Direction.REVERSE);
      lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      lift.setPower(-0.3);
      delay = new ElapsedTime();
      while (delay.seconds() < 2) {
        telemetry.addData("key", delay);
        telemetry.update();
      }
      Config findCone = new Config();
      findCone.enableStream(StreamType.COLOR);
      findCone.enableStream(StreamType.DEPTH);
      findCone.enableStream(StreamType.INFRARED);

      camera.switchConfig(findCone);

      max = 0;
      lift.setPower(1);
      lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      lift.setPower(1);
      linearTranslationCoeff = 0.210526;
      radialTranslationCoeff = 7.91578;
      liftTest = 0;
      waitForStart();
      if (opModeIsActive()) {
        while (opModeIsActive()) {
          // Put loop blocks here.
          if (gamepad1.right_bumper) {
            liftTest += 1;
          } else {
            if (gamepad1.left_bumper) {
              liftTest += -1;
            }
          }
          if (gamepad1.dpad_up) {
            liftTest = 4;
          }
          if (gamepad1.dpad_right) {
            liftTest = 3;
          }
          if (gamepad1.dpad_down) {
            liftTest = 2;
          }
          if (gamepad1.dpad_left) {
            liftTest = 1;
          }
          if (gamepad1.y) {
            liftTest = 0;
          }
          if (liftTest < 0) {
            liftTest = 0;
          }
          if (liftTest > 4) {
            liftTest = 4;
          }
          if (liftTest == 0) {
            moveLift(0);
          } else if (liftTest == 1) {
            moveLift(2.75);
          } else {
            if (liftTest == 2) {
              moveLift(14.5);
            } else if (liftTest == 3) {
              moveLift(24.5);
            } else {
              moveLift(34.5);
            }
          }
          if ((gamepad1.right_trigger > 0.5) || gamepad1.a) {
            claw.setPosition(20.0 / 190); // In pressed pos
          } else {
            claw.setPosition(12.0 / 190); // In release pos
          }
          if (gamepad1.left_trigger > 0.5 || gamepad1.b) {
            requestedLinearXTranslation = gamepad1.left_stick_x * 1;
            requestedLinearYTranslation = gamepad1.left_stick_y * 1;
            requestedRadialTranslation = gamepad1.right_stick_x * 0.05;
          } else {
            if (liftTest == 3 || liftTest == 4) {
              requestedLinearXTranslation = gamepad1.left_stick_x * 2;
              requestedLinearYTranslation = gamepad1.left_stick_y * 2;
              requestedRadialTranslation = gamepad1.right_stick_x * 0.08;
            } else {
              requestedLinearXTranslation = gamepad1.left_stick_x * 4;
              requestedLinearYTranslation = gamepad1.left_stick_y * 4;
              requestedRadialTranslation = gamepad1.right_stick_x * 0.1;
            }
          }
          drive.updatePoseEstimate();
          Pose2d poseEstimate = drive.getPoseEstimate();
          Vector2d input = new Vector2d(
                  -gamepad1.left_stick_y,
                  -gamepad1.left_stick_x
          ).rotated(-poseEstimate.getHeading());

          if (gamepad1.x) {
            int i;
            if(!camera.updateFrameSet()) continue;
            FrameData data = camera.getImageFrame(StreamType.DEPTH);
            FrameData colourData = camera.getImageFrame(StreamType.COLOR);
            int middleCone = -1;
            float depth = 100000000;
            int x = -1;
            int scanlineY = (int) (data.getHeight() * 0.54);
            for (i = (int) (data.getWidth() * 0.2); i < data.getWidth() * 0.8; i++) {
              int rgb = camera.getARGB(i, scanlineY);
              int red = Color.red(rgb);
              int green = Color.green(rgb);
              int blue = Color.green(rgb);
              double satfact = 0.6;
              if (camera.getDistance(i, scanlineY) != 0 && camera.getDistance(i, scanlineY) < depth) {
                if((red * satfact > green && red * satfact > blue) || (blue * satfact > green && blue * satfact > red)) {
                  x = i;
                  depth = camera.getDistance(i, scanlineY);
                }
              }
            }
            int leftSideCone = -1;
            int rightSideCone = -1;
            int notConeCountdown = 3;
            for(i = x; i >= 0; i--){
              int rgb = camera.getARGB(i, scanlineY);
              int red = Color.red(rgb);
              int green = Color.green(rgb);
              int blue = Color.green(rgb);
              double satfact = 0.6;
              if((red * satfact > green && red * satfact > blue) || (blue * satfact > green && blue * satfact > red)) {
                leftSideCone = i;
                notConeCountdown = 3;
              }else{
                if(notConeCountdown == 0){
                  break;
                }
                assert(notConeCountdown > 0);
                notConeCountdown--;
              }
            }
            notConeCountdown = 3;
            if(leftSideCone == -1) continue;
            for(i = x; i <= data.getWidth(); i++){
              int rgb = camera.getARGB(i, scanlineY);
              int red = Color.red(rgb);
              int green = Color.green(rgb);
              int blue = Color.green(rgb);
              double satfact = 0.6;
              if((red * satfact > green && red * satfact > blue) || (blue * satfact > green && blue * satfact > red)) {
                notConeCountdown = 3;
                rightSideCone = i;
              }else{
                if(notConeCountdown == 0){
                  break;
                }
                assert(notConeCountdown > 0);
                notConeCountdown--;
              }
            }
            if(rightSideCone == -1) continue;
            middleCone = (leftSideCone + rightSideCone) / 2;
            int coneWidth = rightSideCone - leftSideCone;
            System.out.println("CONEWIDTH: " + coneWidth);
            int distanceFromMiddle = data.getWidth() / 2 - middleCone;
            double degreesPerPixel = 90.0 / data.getWidth();

            camera.drawHorizontalLine(scanlineY);
            camera.drawVerticalLine(middleCone);
            camera.drawVerticalLine(leftSideCone);
            camera.drawVerticalLine(rightSideCone);
            camera.transmitMonochromeImage();

            drive.turn(Math.toRadians(degreesPerPixel * distanceFromMiddle));
            System.out.println(-Math.toRadians(degreesPerPixel * distanceFromMiddle));
          } else {
            drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x));
          }
          //DISABLED: rightFront.setPower(requestedLinearXTranslation * +1 * linearTranslationCoeff + requestedLinearYTranslation * linearTranslationCoeff + requestedRadialTranslation * radialTranslationCoeff);
          // -        leftFront. setPower(requestedLinearXTranslation * -1 * linearTranslationCoeff + requestedLinearYTranslation * linearTranslationCoeff + requestedRadialTranslation * -1 * radialTranslationCoeff);
          // -        rightRear. setPower(requestedLinearXTranslation * -1 * linearTranslationCoeff + requestedLinearYTranslation * linearTranslationCoeff + requestedRadialTranslation * radialTranslationCoeff);
          // -        leftRear.  setPower(requestedLinearXTranslation * +1 * linearTranslationCoeff + requestedLinearYTranslation * linearTranslationCoeff + requestedRadialTranslation * -1 * radialTranslationCoeff);
          telemetry.addData("position of claw", Double.parseDouble(JavaUtil.formatNumber(claw.getPosition(), 2)));
          telemetry.addData("0 ω", Double.parseDouble(JavaUtil.formatNumber(requestedLinearXTranslation * linearTranslationCoeff + requestedLinearYTranslation * linearTranslationCoeff + requestedRadialTranslation * radialTranslationCoeff, 2)));
          telemetry.addData("0 ω - true (Shrek)", Double.parseDouble(JavaUtil.formatNumber(rightFront.getPower(), 2)));
          telemetry.addData("0 position", Double.parseDouble(JavaUtil.formatNumber(rightFront.getCurrentPosition(), 2)));
          telemetry.addData("1 ω", Double.parseDouble(JavaUtil.formatNumber(requestedLinearXTranslation * linearTranslationCoeff + requestedLinearYTranslation * linearTranslationCoeff + requestedRadialTranslation * radialTranslationCoeff, 2)));
          telemetry.addData("1 ω - true", Double.parseDouble(JavaUtil.formatNumber(leftFront.getPower(), 2)));
          telemetry.addData("1 position", Double.parseDouble(JavaUtil.formatNumber(leftFront.getCurrentPosition(), 2)));
          telemetry.addData("2 ω", Double.parseDouble(JavaUtil.formatNumber(requestedLinearXTranslation * linearTranslationCoeff + requestedLinearYTranslation * linearTranslationCoeff + requestedRadialTranslation * radialTranslationCoeff, 2)));
          telemetry.addData("2 ω - true", Double.parseDouble(JavaUtil.formatNumber(rightRear.getPower(), 2)));
          telemetry.addData("2 position", Double.parseDouble(JavaUtil.formatNumber(rightRear.getCurrentPosition(), 2)));
          telemetry.addData("3 ω", Double.parseDouble(JavaUtil.formatNumber(requestedLinearXTranslation * linearTranslationCoeff + requestedLinearYTranslation * linearTranslationCoeff + requestedRadialTranslation * radialTranslationCoeff, 2)));
          telemetry.addData("3 ω - true", Double.parseDouble(JavaUtil.formatNumber(leftRear.getPower(), 2)));
          telemetry.addData("3 position", Double.parseDouble(JavaUtil.formatNumber(leftRear.getCurrentPosition(), 2)));
          telemetry.addData("Joystick Y", Double.parseDouble(JavaUtil.formatNumber(requestedLinearYTranslation, 2)));
          telemetry.addData("Joystick X", Double.parseDouble(JavaUtil.formatNumber(requestedLinearXTranslation, 2)));
          telemetry.update();
        }
      }
    } catch (NoFrameSetYetAcquiredException e) {
      e.printStackTrace();
    } catch (UnsupportedStreamTypeException e) {
      e.printStackTrace();
    } catch (StreamTypeNotEnabledException e) {
      e.printStackTrace();
    } catch (FrameQueueCloseException e) {
      e.printStackTrace();
    } catch (DisconnectedCameraException e) {
      e.printStackTrace();
    } catch (CameraStopException e) {
      e.printStackTrace();
    } catch (InterruptedException e) {
      e.printStackTrace();
    } catch (CameraStartException e) {
      e.printStackTrace();
    }
  }

  /**
   * Describe this function...
   */
  private void moveLift(double lift_distance) {
    lift.setTargetPosition((int) ((lift_distance / Circumference_in_Inches) * Encoder_ticks_per_rotation));
  }
}