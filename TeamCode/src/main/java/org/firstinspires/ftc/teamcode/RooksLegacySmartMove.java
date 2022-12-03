package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
  private Servo claw;
  private SampleMecanumDrive drive;

  float requestedLinearXTranslation;
  float requestedLinearYTranslation;
  double requestedRadialTranslation;

  double encoderTicksPerRotation;
  double circumferenceInInches;

  boolean rightTriggerAlreadyPressed = false;
  boolean leftTriggerAlreadyPressed  = false;
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {

    try (ConfigurableRealSenseCamera camera = new ConfigurableRealSenseCamera(hardwareMap, () -> isStopRequested())) {

      ElapsedTime delay;
      int liftPosition;
      drive = new SampleMecanumDrive(hardwareMap);

      lift = hardwareMap.get(DcMotor.class, "lift");
      claw = hardwareMap.get(Servo.class, "claw");

      ((DcMotorEx) lift).setVelocityPIDFCoefficients(0, 0, 0, 12.411);
      ((DcMotorEx) lift).setPositionPIDFCoefficients(15);
      // Put initialization blocks here.

      circumferenceInInches = 4.409;
      encoderTicksPerRotation = 384.5;

      lift.setDirection(DcMotorSimple.Direction.REVERSE);
      lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      lift.setPower(-0.3);

      delay = new ElapsedTime();

      while (delay.seconds() < 0.5) {
        telemetry.addData("key", delay);
        telemetry.update();
      }

      Config findCone = new Config();

      findCone.enableStream(StreamType.COLOR);
      findCone.enableStream(StreamType.DEPTH);
      findCone.enableStream(StreamType.INFRARED);

      camera.switchConfig(findCone);

      lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      lift.setPower(1);

      liftPosition = 0;
      waitForStart();

      if (opModeIsActive()) {
        while (opModeIsActive()) {
          drive.updatePoseEstimate();
          // Put loop blocks here.
          if (gamepad1.right_bumper) {
            if (!rightTriggerAlreadyPressed) {
              liftPosition += 1;
              rightTriggerAlreadyPressed = true;
            }
          }
          else rightTriggerAlreadyPressed = false;

          if (gamepad1.left_bumper)  {
            if(!leftTriggerAlreadyPressed){
              liftPosition -= 1;
              leftTriggerAlreadyPressed = true;
            }
          }
          else leftTriggerAlreadyPressed  = false;

          if (gamepad1.dpad_up)    liftPosition = 4;
          if (gamepad1.dpad_right) liftPosition = 3;
          if (gamepad1.dpad_down)  liftPosition = 2;
          if (gamepad1.dpad_left)  liftPosition = 1;
          if (gamepad1.y)          liftPosition = 0;

          if (liftPosition < 0) liftPosition = 0;
          if (liftPosition > 4) liftPosition = 4;

          if (liftPosition == 0) moveLift(0.00);
          if (liftPosition == 1) moveLift(2.75);
          if (liftPosition == 2) moveLift(14.5);
          if (liftPosition == 3) moveLift(24.5);
          if (liftPosition == 4) moveLift(34.5);

          if ((gamepad1.right_trigger > 0.5) || gamepad1.a) claw.setPosition(20.0 / 190); // In pressed pos
          else claw.setPosition(12.0 / 190); // In release pos

          if (gamepad1.left_trigger > 0.5 || gamepad1.b) {
            requestedLinearXTranslation = gamepad1.left_stick_x  * 1;
            requestedLinearYTranslation = gamepad1.left_stick_y  * 1;
            requestedRadialTranslation  = gamepad1.right_stick_x * 0.05;
          } else if (liftPosition == 3 || liftPosition == 4) {
            requestedLinearXTranslation = gamepad1.left_stick_x  * 2;
            requestedLinearYTranslation = gamepad1.left_stick_y  * 2;
            requestedRadialTranslation  = gamepad1.right_stick_x * 0.08;
          } else {
            requestedLinearXTranslation = gamepad1.left_stick_x  * 4;
            requestedLinearYTranslation = gamepad1.left_stick_y  * 4;
            requestedRadialTranslation  = gamepad1.right_stick_x * 0.1;
          }


          if (gamepad1.x) {
            int i;
            if(!camera.updateFrameSet()) continue;
            FrameData data = camera.getImageFrame(StreamType.DEPTH);
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

            if(middleCone < 241){
              System.out.println("Closer than should be");
            }else{
              System.out.println("Farther than should be");
            }

            camera.drawHorizontalLine(scanlineY);
            camera.drawVerticalLine(middleCone);
            camera.drawVerticalLine(leftSideCone);
            camera.drawVerticalLine(rightSideCone);
            camera.transmitMonochromeImage();

            drive.turn(Math.toRadians(degreesPerPixel * distanceFromMiddle));
            System.out.println(-Math.toRadians(degreesPerPixel * distanceFromMiddle));
          } else {
            Vector2d requestedLinearInput = requestedLinearDriveInput();
            drive.setWeightedDrivePower(new Pose2d(requestedLinearInput.getX(), requestedLinearInput.getY(), -requestedRadialTranslation));
          }
          telemetry.addData("position of claw", Double.parseDouble(JavaUtil.formatNumber(claw.getPosition(), 2)));
          telemetry.addData("Joystick Y", Double.parseDouble(JavaUtil.formatNumber(requestedLinearYTranslation, 2)));
          telemetry.addData("Joystick X", Double.parseDouble(JavaUtil.formatNumber(requestedLinearXTranslation, 2)));
          telemetry.update();
        }
      }
    } catch (NoFrameSetYetAcquiredException |
            UnsupportedStreamTypeException  |
            StreamTypeNotEnabledException   |
            FrameQueueCloseException        |
            DisconnectedCameraException     |
            CameraStopException             |
            InterruptedException            |
            CameraStartException e) {
      e.printStackTrace();
    }
  }

  /**
   * Describe this function...
   */
  private void moveLift(double lift_distance) {
    lift.setTargetPosition((int) ((lift_distance / circumferenceInInches) * encoderTicksPerRotation));
  }

  private Vector2d requestedLinearDriveInput(){
    return new Vector2d(
            -requestedLinearYTranslation,
            -requestedLinearXTranslation
    ).rotated(-drive.getPoseEstimate().getHeading());
  }
}