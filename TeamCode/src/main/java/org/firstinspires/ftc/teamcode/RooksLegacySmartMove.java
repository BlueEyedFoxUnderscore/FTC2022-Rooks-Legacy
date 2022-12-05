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
import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import kotlin.Triple;
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

  private ConfigurableRealSenseCamera camera = null;

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

    try {
      camera = new ConfigurableRealSenseCamera(hardwareMap, () -> isStopRequested());
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
      lift.setPower(-0.5);

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
      lift.setPower(0.3);

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

          if (liftPosition < 2){
            lift.setPower(0.5);
          } else {
            lift.setPower(1);
          }

          if (liftPosition == 0) moveLift(0.00);
          if (liftPosition == 1) moveLift(2.75);
          if (liftPosition == 2) moveLift(14.5);
          if (liftPosition == 3) moveLift(24.5);
          if (liftPosition == 4) moveLift(34.5);

          if ((gamepad1.right_trigger > 0.5) || gamepad1.a) claw.setPosition(30.0 / 190); // In pressed pos
          else claw.setPosition(12.0 / 190); // In release pos

          if (gamepad1.left_trigger > 0.5 || gamepad1.b) {
            requestedLinearXTranslation = gamepad1.left_stick_x  * 0.25f;
            requestedLinearYTranslation = gamepad1.left_stick_y  * 0.25f;
            requestedRadialTranslation  = gamepad1.right_stick_x * 0.25f;
          } else if (liftPosition == 3 || liftPosition == 4) {
            requestedLinearXTranslation = gamepad1.left_stick_x  * 0.5f;
            requestedLinearYTranslation = gamepad1.left_stick_y  * 0.5f;
            requestedRadialTranslation  = gamepad1.right_stick_x * 0.5f;
          } else {
            requestedLinearXTranslation = gamepad1.left_stick_x  * 1f;
            requestedLinearYTranslation = gamepad1.left_stick_y  * 1f;
            requestedRadialTranslation  = gamepad1.right_stick_x * 1f;
          }

          //EXPERIMENTAL
          if (gamepad1.x) {
            if (!camera.updateFrameSet()) continue;
            FrameData data = camera.getImageFrame(StreamType.DEPTH);
            int scanlineY = (int) (data.getHeight() * 0.75);

            int i;
            int nearestPointGuess = -1;


            float[] hsv = new float[3];

            Pair<Float, Float>[] coneColourRanges = new Pair[]{
                    new Pair<>(220f, 260f),
                    new Pair<>(320f, 10f ) };

            Triple<Integer, Integer, Float> positionAndHueRangeIndex = getNearestObjectWithHueRange(scanlineY, coneColourRanges);

            int middleCone = -1;
            int leftSideCone = -1;
            int rightSideCone = -1;
            int notConeTolerance = 5;
            int notConeCountdown = notConeTolerance;
            float depth;

            boolean redCone = (positionAndHueRangeIndex.getSecond() == 1);// INFO: If the hue range is 1, then that indicates the first range, which is the hue range for RED.
            nearestPointGuess = positionAndHueRangeIndex.getFirst();
            depth = positionAndHueRangeIndex.getThird();
            //DISABLED:
            // - float depth = 100000000;
            // - for (i = (int) (data.getWidth() * 0.2); i < data.getWidth() * 0.8; i++) {
            // -   Color.colorToHSV(camera.getARGB(i, scanlineY), hsv);
            // -   double hue = hsv[0];
            // -   double sat = hsv[1];
            // -   double val = hsv[2];
            // -   if (camera.getDistance(i, scanlineY) != 0 && camera.getDistance(i, scanlineY) < depth) {
            // -     if ((hueRange(hue, 220, 260) || hueRange(hue, 320, 10)) && sat > 0.3) {
            // -       nearestPointGuess = i;
            // -       depth = camera.getDistance(i, scanlineY);
            // -     }
            // -   }
            // - }

            int coneWidth;



            ObjectExtentParameters extentParameters = findObjectExtentUsingHueRange(nearestPointGuess, scanlineY, redCone? coneColourRanges[1] : coneColourRanges[0], false);
            rightSideCone = extentParameters.getRightSide();
            leftSideCone  = extentParameters.getLeftSide() ;
            middleCone    = extentParameters.getMiddle()   ;
            coneWidth     = extentParameters.getWidth()    ;

            extentParameters = findObjectExtentUsingHueRange(middleCone, (int)(data.getHeight() * 0.05), redCone? coneColourRanges[1] : coneColourRanges[0], false);
            camera.drawHorizontalLine((int)(data.getHeight()*.05));

            int duoConeWidth = extentParameters.getWidth();

            boolean onlyOneCone = duoConeWidth < 45;

            if(!onlyOneCone){
              int coneRimWidth = 0;

              for(float f = 0.75f; f <= 1f; f += 0.02f){
                extentParameters = findObjectExtentUsingHueRange(middleCone, ((int) (data.getHeight() * f)), redCone? coneColourRanges[1] : coneColourRanges[0], false);
                camera.drawHorizontalLine((int)(data.getHeight() * f));
                rightSideCone = extentParameters.getRightSide();
                leftSideCone  = extentParameters.getLeftSide() ;
                middleCone    = extentParameters.getMiddle()   ;
                coneWidth     = extentParameters.getWidth()    ;
                coneRimWidth  = coneRimWidth < coneWidth ? coneWidth : coneRimWidth;
              }
              System.out.println("CONEWIDTH RIM: " + coneRimWidth);
            }

            System.out.println("doublecone width: "+ duoConeWidth);
            if(rightSideCone == -1) continue;

            System.out.println("CONEWIDTH: " + coneWidth);
            int distanceFromMiddle = data.getWidth() / 2 - middleCone - 40;
            double degreesPerPixel = 90.0 / data.getWidth();

            // INFO: Best fit equation goes crazy after this point, plus we can't see a cone out that far.
            if(coneWidth < 90) continue;

            // INFO: Best fit equation
            double pseudoDistance = 0.571 * Math.tan(Math.toRadians(-0.046 * (coneWidth - 53) + 90.0)) - 2.85; // Forward = less negative

            drive.updatePoseEstimate();
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(pseudoDistance).build());

            camera.drawHorizontalLine(scanlineY);
            camera.drawHorizontalLine((int)(data.getHeight()*.05));

            camera.drawVerticalLine(middleCone);
            camera.drawVerticalLine(leftSideCone);
            camera.drawVerticalLine(rightSideCone);
            camera.drawVerticalLine(nearestPointGuess);
            System.out.println("distance to detected cone point: "+depth);

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
            DisconnectedCameraException     |
            CameraStopException             |
            InterruptedException            |
            CameraStartException e) {
      e.printStackTrace();
    } finally {
      try {
        camera.close();
      } catch (FrameQueueCloseException e) {
        e.printStackTrace();
      } catch (CameraStopException e) {
        e.printStackTrace();
      }
    }
  }

  /**
   * Describe this function....
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

  private boolean range(double x, double min, double max){
    return (min < x && x < max);
  }
  private boolean hueRange(double x, double from, double to){
    if(from < to) return range(x, from, to);
    return (range(x, from, 360) || range(x, 0, to));
  }

  private Triple<Integer, Integer, Float> getNearestObjectWithHueRange(int scanlineY, Pair<Float, Float>[] hueRanges) throws NoFrameSetYetAcquiredException, UnsupportedStreamTypeException, StreamTypeNotEnabledException {
    int i;
    FrameData data = camera.getImageFrame(StreamType.DEPTH);
    int middleCone = -1;
    float depth = 100000000;
    int x = -1;

    float[] hsv = new float[3];

    int associatedColourRange = 0;

    int avgCount=0;
    for (i = (int) (data.getWidth() * 0.2); i < data.getWidth() * 0.8; i++) {
      Color.colorToHSV(camera.getARGB(i, scanlineY), hsv);

      double hue = hsv[0];
      double sat = hsv[1];
      double val = hsv[2];

      float newDepth=camera.getDistance(i, scanlineY);
      if (newDepth != 0 && newDepth <= depth) {
        int m = 0;
        for (Pair<Float, Float> hueRange : hueRanges) {
          if (hueRange(hue, hueRange.fst, hueRange.snd) && sat > 0.3) {
            x = i;
            depth = newDepth;
            associatedColourRange = m;
          }
          m++;
        }
      }
    }
    return new Triple<>(x, associatedColourRange, depth);
  }

  private ObjectExtentParameters findObjectExtentUsingHueRange(int startX, int scanlineY, Pair<Float, Float> hueRange, boolean debug) throws NoFrameSetYetAcquiredException, UnsupportedStreamTypeException, StreamTypeNotEnabledException {
    FrameData data = camera.getImageFrame(StreamType.COLOR);

    int i;
    float[] hsv = new float[3];

    int notConeTolerance = 5;
    int notConeCountdown = notConeTolerance;

    StringBuilder hueString = null;
    StringBuilder satString = null;
    StringBuilder valString = null;

    int leftSideCone  = -1;
    int rightSideCone = -1;
    int coneWidth     = -1;
    int middleCone    = -1;

    if(debug){
      hueString = new StringBuilder("HUE: ");
      satString = new StringBuilder("SAT: ");
      valString = new StringBuilder("VAL: ");
    }

    for(i = startX; i >= 0; i-=2){
      Color.colorToHSV(camera.getARGB(i, scanlineY), hsv);
      double hue = hsv[0];
      double sat = hsv[1];
      double val = hsv[2];
      if(hueRange(hue, hueRange.fst, hueRange.snd) && sat > 0.3) {
        if(notConeCountdown == notConeTolerance){
          if(debug) {
            hueString.insert(0, String.format("%03.0f ", hue));
            satString.insert(0, String.format("%.2f ", sat).substring(1));
            valString.insert(0, String.format("%.2f ", val).substring(1));
          }

          leftSideCone = i;
        } else notConeCountdown++;
      }else{
        if(notConeCountdown == 0){
          break;
        }
        assert(notConeCountdown > 0);
        notConeCountdown--;
      }
    }
    if(debug){
      System.out.println(hueString + "\n" + satString + "\n" + valString);
    }

    notConeCountdown = notConeTolerance;

    if(debug){
      hueString = new StringBuilder("HUE: ");
      satString = new StringBuilder("SAT: ");
      valString = new StringBuilder("VAL: ");
    }

    for(i = startX; i <= data.getWidth(); i+=2){
      Color.colorToHSV(camera.getARGB(i, scanlineY), hsv);
      double hue = hsv[0];
      double sat = hsv[1];
      double val = hsv[2];
      if(hueRange(hue, hueRange.fst, hueRange.snd) && sat > 0.3) {
        if(notConeCountdown == notConeTolerance){
          if(debug) {
            hueString.append(String.format("%03.0f ", hue));
            satString.append(String.format("%.2f ", sat).substring(1));
            valString.append(String.format("%.2f ", val).substring(1));
          }
          rightSideCone = i;
        } else notConeCountdown++;
      }else{
        if(notConeCountdown == 0){
          break;
        }
        assert(notConeCountdown > 0);
        notConeCountdown--;
      }
    }

    if(debug){
      System.out.println(hueString + "\n" + satString + "\n" + valString);
    }

    middleCone = (leftSideCone + rightSideCone) / 2;
    coneWidth  =  rightSideCone - leftSideCone;

    return new ObjectExtentParameters(leftSideCone, rightSideCone, middleCone, coneWidth);
  }

  class ObjectExtentParameters{

    private int leftSide;
    private int rightSide;
    private int middle;
    private int width;

    public ObjectExtentParameters(int leftSide, int rightSide, int middle, int width){
      this.leftSide  =  leftSide;
      this.rightSide = rightSide;
      this.middle    =    middle;
      this.width     =     width;
    }

    public  int getLeftSide(){
      return leftSide;
    }

    public int getRightSide() {
      return rightSide;
    }

    public int getMiddle() {
      return middle;
    }

    public int getWidth() {
      return width;
    }
  }
}