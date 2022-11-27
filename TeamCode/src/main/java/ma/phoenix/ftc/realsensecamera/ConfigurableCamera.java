package ma.phoenix.ftc.realsensecamera;

import android.graphics.Color;

import com.intel.realsense.librealsense.Align;
import com.intel.realsense.librealsense.Config;
import com.intel.realsense.librealsense.DepthFrame;
import com.intel.realsense.librealsense.Device;
import com.intel.realsense.librealsense.Extension;
import com.intel.realsense.librealsense.Frame;
import com.intel.realsense.librealsense.FrameQueue;
import com.intel.realsense.librealsense.FrameSet;
import com.intel.realsense.librealsense.Option;
import com.intel.realsense.librealsense.Pipeline;
import com.intel.realsense.librealsense.PipelineProfile;
import com.intel.realsense.librealsense.RsContext;
import com.intel.realsense.librealsense.Sensor;
import com.intel.realsense.librealsense.StreamFormat;
import com.intel.realsense.librealsense.StreamProfile;
import com.intel.realsense.librealsense.StreamType;
import com.intel.realsense.librealsense.VideoFrame;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

import ma.phoenix.ftc.realsensecamera.exceptions.NoFrameSetYetAcquired;
import ma.phoenix.ftc.realsensecamera.exceptions.CameraStartException;
import ma.phoenix.ftc.realsensecamera.exceptions.CameraStopException;
import ma.phoenix.ftc.realsensecamera.exceptions.DisconnectedCameraException;
import ma.phoenix.ftc.realsensecamera.exceptions.FrameQueueCloseException;
import ma.phoenix.ftc.realsensecamera.exceptions.StreamTypeNotEnabledException;
import ma.phoenix.ftc.realsensecamera.exceptions.UnsupportedStreamTypeException;

public class ConfigurableCamera implements AutoCloseable{
    private Pipeline pipeline = new Pipeline();
    private boolean pipelineStopped;

    private Device mDevice;
    private Sensor mSensor;

    private Config config;

    private FrameQueue frameQueue = new FrameQueue(1);
    private FrameSet depthAndInfraredFrameSet =null;
    private FrameSet processedColorFrameSet =null;
    private DepthFrame depthFrame = null;
    private int infraredWidth, infraredHeight;
    private int colorWidth, colorHeight;
    private int depthWidth, depthHeight;
    private StreamFormat colorStreamFormat;

    private Align align = new Align(StreamType.DEPTH);

    public byte[] infraredFrameBuffer = new byte[1];
    public byte[] colourFrameBuffer = new byte[1];
    public byte[] depthFrameBuffer = new byte[1];
    boolean colorFrameCached =false;
    boolean infraredFrameCached =false;
    boolean depthFrameCached =false;

    private int gain = -1;
    private int exp = -1;

    public ConfigurableCamera(HardwareMap hardwareMap) throws DisconnectedCameraException {
        System.out.println("constructor called");
        pipelineStopped = true;
        System.out.println("initializing context");
        RsContext.init(hardwareMap.appContext);
        try {
            System.out.println("sleeping for one second");
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        System.out.println("getting context");
        RsContext context = new RsContext();
        System.out.println("checking that we don't have zero devices");
        if(context.queryDevices().getDeviceCount() == 0) throw new DisconnectedCameraException();
    }

    public void enableAutoExposure(Boolean enabled){
        System.out.println("enableAutoExposure() called");
        System.out.println("Setting auto exposure");
        mSensor.setValue(Option.ENABLE_AUTO_EXPOSURE, enabled? 1 : 0);
        if(enabled) return;
        System.out.println("Setting gain");
        mSensor.setValue(Option.GAIN, gain);
        System.out.println("Setting exposure");
        mSensor.setValue(Option.EXPOSURE, exp);
    }

    private void start() throws CameraStartException, CameraStopException {
        System.out.println("start() called");
        System.out.println("checking if pipeline running");
        if(!pipelineStopped) stop();
        PipelineProfile pipelineProfile;
        try{
            System.out.println("starting pipeline");
            pipelineProfile = pipeline.start(config, frameQueue::Enqueue);
        }catch (Exception e) {
            e.printStackTrace();
            throw new CameraStartException();
        }
        System.out.println("getting device");
        mDevice = pipelineProfile.getDevice();
        System.out.println("setting advanced mode");
        if(!mDevice.isInAdvancedMode()) { // If advanced mode is already enabled, setting advanced mode will crash the library.
            mDevice.toggleAdvancedMode(true);
        }
        System.out.println("getting sensor");
        mSensor = mDevice.querySensors().get(0);
        mSensor.setValue(Option.FRAMES_QUEUE_SIZE, 1);
        List<StreamProfile> allActive= mSensor.getActiveStreams();
        for(StreamProfile streamProfile : allActive) {
            System.out.println(
                    "Active profile: unique id:"+streamProfile.getUniqueId() +
                    " Handle:" + streamProfile.getHandle() +
                    " Index:" + streamProfile.getIndex() +
                    " Type:" + streamProfile.getType() +
                    " Format:" + streamProfile.getFormat()+
                    " Frame rate:" + streamProfile.getFrameRate());
        }
        pipelineStopped = false;
    }

    public void stop() throws CameraStopException {
        System.out.println("stop() called");
        if(pipelineStopped) return;
        try{
            System.out.println("Stopping pipeline");
            pipeline.stop();
            System.out.println("Completed stop");
        } catch (Exception e){
            throw new CameraStopException();
        }
        pipelineStopped = true;
    }

    private Device getDevice(){
        return mDevice;
    }

    private Sensor getSensor(){
        return mSensor;
    }

    public void switchConfig(Config toSwitch) throws CameraStartException, CameraStopException {
        System.out.println("switchConfig called()");
        System.out.println("calling stop()");
        stop();
        System.out.println("getting reference to config");
        config = toSwitch;
        System.out.println("calling start()");
        start();
    }

    public void setGain(int gain){
        System.out.println("setGain()");
        this.gain = gain;
        System.out.println("calling enableAutoExposure(false)");
        this.enableAutoExposure(false);
    }

    public void setExp(int exp){
        System.out.println("setExp()");
        this.exp = exp;
        System.out.println("calling enableAutoExposure(false)");
        this.enableAutoExposure(false);
    }

    public boolean updateFrameSet(){
        FrameSet newColorAndInfraredFrameSet = frameQueue.pollForFrames();
        if(newColorAndInfraredFrameSet == null) {
            return false;
        }
        //System.out.println("New frameset");
        //FrameSet newColorProcessedFrameSet = newColorAndInfraredFrameSet.applyFilter(align);
        FrameSet newColorProcessedFrameSet = align(newColorAndInfraredFrameSet);
        //System.out.println("closing old depthFrame");
        Frame freeFrame;
        if(depthAndInfraredFrameSet !=null) {
            while((freeFrame= depthAndInfraredFrameSet.first(StreamType.ANY))!=null)
            {
                depthAndInfraredFrameSet.close();
                System.out.println("Freed a color and infra frame");
            }
            depthAndInfraredFrameSet.close();
        }
        if(processedColorFrameSet !=null) {
            while((freeFrame= processedColorFrameSet.first(StreamType.ANY))!=null)
            {
                processedColorFrameSet.close();
                System.out.println("Freed a processedDepthFrameSet frame");
            }
            processedColorFrameSet.close();
        }
        if (depthFrame != null) depthFrame.close();
        depthFrame=null;
        colorFrameCached =false;
        infraredFrameCached =false;
        depthFrameCached =false;
        //System.out.println("saving new frameSet");
        depthAndInfraredFrameSet = newColorAndInfraredFrameSet;
        processedColorFrameSet = newColorProcessedFrameSet;
        return true;
    }


    public float getDistance(int x, int y) throws StreamTypeNotEnabledException, NoFrameSetYetAcquired {
        //System.out.println("getDistance("+x+", "+y+")"); //848x480
        //System.out.println("checking to see if depth frame needs updating");
        if (depthFrame == null) {
            //System.out.println("in Get Distance, checking for frameset");
            if(depthAndInfraredFrameSet == null){
                throw new NoFrameSetYetAcquired();
            }
            //System.out.println("checking for depth frame");
            depthFrame = depthAndInfraredFrameSet.first(StreamType.DEPTH).as(Extension.DEPTH_FRAME);
            if(depthFrame == null){
                throw new StreamTypeNotEnabledException();
            }
        }
        //if(x==600) System.out.println("getDistance("+x+", "+y+") width: " +depthFrame.getWidth() +" height: "+depthFrame.getHeight() + "=" + depthFrame.getDistance(x, y));
        return depthFrame.getDistance(x, y);
    }

    public FrameData getImageFrame(StreamType type) throws UnsupportedStreamTypeException, StreamTypeNotEnabledException, NoFrameSetYetAcquired {
        //System.out.println("getImageFrame()");
        if(depthAndInfraredFrameSet == null){
            throw new NoFrameSetYetAcquired();
        }
        //System.out.println("checking For Frame");
        switch (type) {
            case DEPTH:
                if (depthFrame == null){
                    Frame frame = depthAndInfraredFrameSet.first(type);
                    //System.out.println("testing if we support it");
                    //System.out.println("testing if frame exists in frameSet");
                    if (frame == null) {
                        throw new StreamTypeNotEnabledException();
                    }
                    depthFrame = frame.as(Extension.DEPTH_FRAME);
                    frame.close();
                }
                //System.out.println("is a depth frame");
                if (!depthFrameCached) {
                    //System.out.println("testing frame buffer length");
                    if (depthFrameBuffer.length < depthFrame.getDataSize()) {
                        System.out.println("creating frame buffer");
                        depthFrameBuffer = new byte[depthFrame.getDataSize()];
                    }
                    //System.out.println("getting frame buffer data");
                    depthFrame.getData(depthFrameBuffer);
                    depthWidth = depthFrame.getWidth();
                    depthHeight = depthFrame.getHeight();
                    depthFrameCached = true;
                }
                return new FrameData(depthFrameBuffer,
                        depthWidth,
                        depthHeight);
            case COLOR:
                if (!colorFrameCached) {
                    try(Frame frame = processedColorFrameSet.first(type)) {
                        //System.out.println("testing if we support it");
                        //System.out.println("testing if frame exists in frameSet");
                        if (frame == null) {
                            throw new StreamTypeNotEnabledException();
                        }
                        //System.out.println("color frame found");
                        //System.out.println("is a color frame");
                        colorStreamFormat = frame.getProfile().getFormat();
                        if (colourFrameBuffer.length < frame.getDataSize()) {
                            System.out.println("creating frame buffer");
                            colourFrameBuffer = new byte[frame.getDataSize()];
                        }
                        //System.out.println("getting frame buffer data");
                        frame.getData(colourFrameBuffer);
                        colorFrameCached = true;
                        VideoFrame videoFrame = frame.as(Extension.VIDEO_FRAME);
                        colorWidth = videoFrame.getWidth();
                        colorHeight = videoFrame.getHeight();
                        videoFrame.close();
                    }
                }
                return new FrameData(colourFrameBuffer,
                        colorWidth,
                        colorHeight);
            case INFRARED:
                if(!infraredFrameCached) {
                    try(Frame frame = depthAndInfraredFrameSet.first(type)) {
                        //System.out.println("testing if we support it");
                        //System.out.println("testing if frame exists in frameSet");
                        if (frame == null) {
                            throw new StreamTypeNotEnabledException();
                        }
                        //System.out.println("is a depth frame");
                        System.out.println("is a infrared frame");
                        if (infraredFrameBuffer.length < frame.getDataSize()) {
                            System.out.println("creating frame buffer");
                            infraredFrameBuffer = new byte[frame.getDataSize()];
                        }
                        //System.out.println("getting frame buffer data");
                        frame.getData(infraredFrameBuffer);
                        infraredFrameCached =true;
                        VideoFrame videoFrame = frame.as(Extension.VIDEO_FRAME);
                        infraredWidth = videoFrame.getWidth();
                        infraredHeight = videoFrame.getHeight();
                        videoFrame.close();
                    }
                }
                return new FrameData(infraredFrameBuffer,
                        infraredWidth,
                        infraredHeight);
            default:
                throw new UnsupportedStreamTypeException();
        }
    }

    public int getARGB(int x, int y) throws UnsupportedStreamTypeException, StreamTypeNotEnabledException, NoFrameSetYetAcquired {
        //System.out.println("GetARGB("+x+","+y+")");
        if(!colorFrameCached) {
            //System.out.println("Getting color frame");
            getImageFrame(StreamType.COLOR);
        }

        // see https://www.wikiwand.com/en/YUV#Y%E2%80%B2UV444_to_RGB888_conversion
        int c, d, e;
        int index;
        switch (colorStreamFormat)
        {
            case UYVY:
                //System.out.println("Getting UYVY");
                index = ((y*colorWidth)+x>>2)<<3; // Get the lowest multiple of 4 address then multiply by 2
                c = byteToInt(colourFrameBuffer[(((y*colorWidth)+x>>1)<<1)+1])-16;
                d = byteToInt(colourFrameBuffer[index])-128;
                e = byteToInt(colourFrameBuffer[index+2])-128;
                return Color.argb(0,
                        Math.min(Math.max((298*c+409*e+128)>>8,0),255),
                        Math.min(Math.max((298*c-100*d-208*e+128)>>8,0),255),
                        Math.min(Math.max((298*c+516*d+128)>>8,0),255));
            case YUYV:
                //System.out.println("Getting YUYV");
                index = ((y*colorWidth)+x>>2)<<3; // Get the lowest multiple of 4 address then multiply by 2
                c = byteToInt(colourFrameBuffer[(((y*colorWidth)+x>>1)<<2)])-16;
                d = byteToInt(colourFrameBuffer[index+1])-128;
                e = byteToInt(colourFrameBuffer[index+3])-128;
                //System.out.println(
                //        " c:"+c+
                //        " d:"+d+
                //        " e:"+e+
                //        " Y:"+colourFrameBuffer[index+0] +
                //        " U:"+colourFrameBuffer[index+1]+
                //        " Y:"+colourFrameBuffer[index+2]+
                //        " V:"+colourFrameBuffer[index+3]);
        return Color.argb(0,
                        Math.min(Math.max((298*c+409*e+128)>>8,0),255),
                        Math.min(Math.max((298*c-100*d-208*e+128)>>8,0),255),
                        Math.min(Math.max((298*c+516*d+128)>>8,0),255));
            case RGB8:
                System.out.println("Getting RGB");
                index = ((y*colorWidth)+x)*3; // Get pixel index
                return Color.argb(0,
                        byteToInt(colourFrameBuffer[index]),
                        byteToInt(colourFrameBuffer[index+1]),
                        byteToInt(colourFrameBuffer[index+2]));
            default:
                throw new UnsupportedStreamTypeException();
        }
    }

    public void close() throws FrameQueueCloseException, CameraStopException {
        System.out.println("close() called");
        System.out.println("closing pipeline");
        if(pipeline != null) pipeline.close();
        //System.out.println("calling stop()");
        //stop();
        System.out.println("closing frameSet");
        if(depthAndInfraredFrameSet != null) depthAndInfraredFrameSet.close();
        System.out.println("closing depthFrame");
        if(depthFrame!=null) depthFrame.close();
        System.out.println("closing queue");
        try {
            frameQueue.close();
        } catch (Exception e){
            throw new FrameQueueCloseException();
        }
    }
    private int byteToInt(byte x) {return x & 0xff;}
}
