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

import ma.phoenix.ftc.realsensecamera.exceptions.NoFramesetAvailable;
import ma.phoenix.ftc.realsensecamera.exceptions.CameraStartException;
import ma.phoenix.ftc.realsensecamera.exceptions.CameraStopException;
import ma.phoenix.ftc.realsensecamera.exceptions.DisconnectedCameraException;
import ma.phoenix.ftc.realsensecamera.exceptions.FrameQueueCloseException;
import ma.phoenix.ftc.realsensecamera.exceptions.StreamTypeNotEnabledException;
import ma.phoenix.ftc.realsensecamera.exceptions.UnsupportedStreamTypeException;

// The current compilable version of the library will crash an FTC robot when the pipeline is closed.
// 2.5.0 seems to work fine.

// To compile the library, gradele must be installed.  Download gradle from https://gradle.org/
// Go to the librealsense\wrappers\android folder and execute gradleW assembleRelease.

// To use the version you built, remove the references to the online version from build.common.gradle
// and the teamcode build.gradlem then add
// librealsense\wrappers\android\librealsense\build\outputs\aar\librealsense-release.aar
// as a dependencey to the Teamcode module in file->Project Structure, Dependencies
// select the Teamcode, hit add, select the jar/aar option, add the file as an "implementation".
// This will add the line
// implementation files('C:\\ [other dirs here] \\librealsense\\wrappers\\android\\librealsense\\build\\outputs\\aar\\librealsense-release.aar')
// to the teamcode build.gradle file.  (You can always add this manually if you wish.)


// Do not upgrade the gradle version, or the library will not compile.

public class ConfigurableCamera implements AutoCloseable{
    StringBuilder updateFrameSetDebugString = new StringBuilder();
    StringBuilder getDepthFrameIfNecessaryDebugString = new StringBuilder();
    StringBuilder getDepthDebugString = new StringBuilder();
    StringBuilder getImageFrameIfNecessaryDebugString = new StringBuilder();
    StringBuilder getImageFrameDebugString = new StringBuilder();
    private Pipeline pipeline = new Pipeline();
    private boolean pipelineStopped;

    private Device mDevice;
    private Sensor mSensor;

    private Config config;

    private FrameQueue frameQueue = new FrameQueue(1);
    private FrameSet cachedAlignedFrameSet =null;
    private boolean haveCachedFrameSet =false;

    private Frame cachedDepthFrameRealFrame = null;
    private DepthFrame cachedDepthFrame = null;
    private boolean haveCachedDepthFrame = false;

    public byte[] colourFrameBuffer = new byte[1];
    private int colorWidth, colorHeight, colorStride;
    private StreamFormat colorStreamFormat;
    private boolean colorFrameCached =false;

    public byte[] infraredFrameBuffer = new byte[1];
    private int infraredWidth, infraredHeight, infraredStride;
    private boolean infraredFrameCached =false;

    public byte[] depthFrameBuffer = new byte[1];
    private int depthWidth, depthHeight, depthStride;
    private boolean depthFrameBufferCached =false;

    private Align alignFilter = new Align(StreamType.DEPTH); // Align other streams to the depth stream

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
        System.out.println("Current frames queue size: "+mSensor.getValue(Option.FRAMES_QUEUE_SIZE));

        // All Frames and FrameSets must be released via close().  If they are not, the library's internal
        // cache of frames will be exhausted and the library will stop transmitting images (until additional
        // frames are released.)  When the pipeline is closed, you will be told how many frames you are
        // "holding on to".  If the frames are large, the device will run out of memory.  In order to
        // prevent this possibility, it is necessary to reduce the maxim number of "publishable" frames
        // "Published" means that the library user has control of the frame.  When the frame is close()ed,
        // it is unpublished and made available for reuse to the library.
        // FRAMES_QUEUE_SIZE controls this.
        //mSensor.setValue(Option.FRAMES_QUEUE_SIZE, 12);

        System.out.println("Current frames queue size: "+mSensor.getValue(Option.FRAMES_QUEUE_SIZE));
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
        updateFrameSetDebugString.append("|| updateFrameSet() ");
        FrameSet unalignedFrameSet = frameQueue.pollForFrames();
        if(unalignedFrameSet == null) {
            return false;
        }
        updateFrameSetDebugString.append("New has "+unalignedFrameSet.getSize()+" frames, ");
        //FrameSet alignedFrameSet = unalignedFrameSet.applyFilter(alignFilter);
        //FrameSet alignedFrameSet = alignFilter.process(unalignedFrameSet);
        updateFrameSetDebugString.append("after align "+unalignedFrameSet.getSize()+" frames ");
        updateFrameSetDebugString.append("proccessed has "+alignedFrameSet.getSize()+" frames ");
        //unalignedFrameSet.close(); // The new frameset has all the frames, but me must close the old one, and it's fine to do it now.
        //updateFrameSetDebugString.append("after close 'new' processed has "+alignedFrameSet.getSize()+"frames. ");
        if(haveCachedFrameSet) {
            updateFrameSetDebugString.append("closed old fs. ");
            cachedAlignedFrameSet.close();
            // No need to set this to false because we're going to immediately
            // repopulate the reference and set haveCachedFrameSet to true.
        }
        cachedAlignedFrameSet = unalignedFrameSet;
        if (haveCachedDepthFrame) {
            updateFrameSetDebugString.append("closed old depthframe. ");
            //cachedDepthFrame.close();
            // Not necessary because this frame does not own the resource.
            cachedDepthFrameRealFrame.close();
            haveCachedDepthFrame =false;
        }
        colorFrameCached =false;
        infraredFrameCached =false;
        depthFrameBufferCached =false;
        haveCachedFrameSet =true;
        //System.out.println(updateFrameSetDebugString);
        //System.out.println(getDepthFrameIfNecessaryDebugString);
        //System.out.println(getDepthDebugString);
        //System.out.println(getImageFrameIfNecessaryDebugString);
        //System.out.println(getImageFrameDebugString);
        updateFrameSetDebugString=new StringBuilder();
        getDepthFrameIfNecessaryDebugString=new StringBuilder();
        getImageFrameIfNecessaryDebugString=new StringBuilder();
        getDepthDebugString=new StringBuilder();
        getImageFrameDebugString = new StringBuilder();
        return true;
    }


    public void getDepthFrameIfNecessary () throws NoFramesetAvailable, StreamTypeNotEnabledException {
        getDepthFrameIfNecessaryDebugString.append("||getDepthFrameIfNecessary() ");
        if (haveCachedDepthFrame) return;
        if(!haveCachedFrameSet){
            getDepthFrameIfNecessaryDebugString.append("No frameset. ");
            throw new NoFramesetAvailable();
        }
        cachedDepthFrameRealFrame = cachedAlignedFrameSet.first(StreamType.DEPTH);
        if(cachedDepthFrameRealFrame == null){
            getDepthFrameIfNecessaryDebugString.append("Frameset but no depth frame. ");
            throw new StreamTypeNotEnabledException();
        }
        cachedDepthFrame = cachedDepthFrameRealFrame.as(Extension.DEPTH_FRAME);
        getDepthFrameIfNecessaryDebugString.append("Found new depthframe. ");
        haveCachedDepthFrame = true;
    }

    public float getDistance(int x, int y) throws StreamTypeNotEnabledException, NoFramesetAvailable {
        getDepthFrameIfNecessaryDebugString.append("||Get distance() called getDepthFrameIfNecessary() ");
        getDepthFrameIfNecessary();
        if(x==600) getDepthFrameIfNecessaryDebugString.append("returning ("+x+", "+y+")="+ cachedDepthFrame.getDistance(x, y));
        return cachedDepthFrame.getDistance(x, y);
    }

    public FrameData getImageFrame(StreamType type) throws UnsupportedStreamTypeException, StreamTypeNotEnabledException, NoFramesetAvailable {
        getImageFrameDebugString.append("||getImageFrame() ");
        if(!haveCachedFrameSet){
            getImageFrameDebugString.append("No frameset. ");
            throw new NoFramesetAvailable();
        }
        //System.out.println("checking For Frame");
        switch (type) {
            case DEPTH:
                if (!depthFrameBufferCached) {
                    getDepthFrameIfNecessary();
                    getImageFrameDebugString.append("First depth req., called getDepthFrameIfNecessary(), ");
                    if (depthFrameBuffer.length < cachedDepthFrameRealFrame.getDataSize()) {
                        getImageFrameDebugString.append("creating depthFrameBuffer, ");
                        depthFrameBuffer = new byte[cachedDepthFrameRealFrame.getDataSize()];
                    }
                    cachedDepthFrameRealFrame.getData(depthFrameBuffer);
                    getImageFrameDebugString.append("got depthFrameBuffer");
                    depthWidth = cachedDepthFrame.getWidth();
                    depthHeight = cachedDepthFrame.getHeight();
                    depthStride = cachedDepthFrame.getStride();
                    depthFrameBufferCached = true;
                }
                return new FrameData(depthFrameBuffer,
                        depthWidth,
                        depthHeight);
            case COLOR:
                if (!colorFrameCached) {
                    getImageFrameDebugString.append("First color req., ");
                    try(Frame frame = cachedAlignedFrameSet.first(type)) {
                        if (frame == null) {
                            getImageFrameDebugString.append("Frame set didn't have a color frame, ");
                            throw new StreamTypeNotEnabledException();
                        }
                        if (colourFrameBuffer.length < frame.getDataSize()) {
                            getImageFrameDebugString.append("creating colourFrameBuffer frame buffer, ");
                            colourFrameBuffer = new byte[frame.getDataSize()];
                        }
                        VideoFrame videoFrame = frame.as(Extension.VIDEO_FRAME);
                        videoFrame.getData(colourFrameBuffer);
                        getImageFrameDebugString.append("got colourFrameBuffer ");
                        colorStreamFormat = videoFrame.getProfile().getFormat();
                        getImageFrameDebugString.append("format is "+colorStreamFormat);
                        colorWidth = videoFrame.getWidth();
                        colorHeight = videoFrame.getHeight();
                        colorStride = videoFrame.getStride();
                        //videoFrame.close(); //not needed because this frame doesn't own the resource
                        colorFrameCached = true;
                    }
                }
                return new FrameData(colourFrameBuffer,
                        colorWidth,
                        colorHeight);
            case INFRARED:
                if(!infraredFrameCached) {
                    getImageFrameDebugString.append("First infrared req., ");
                    try(Frame frame = cachedAlignedFrameSet.first(type)) {
                        if (frame == null) {
                            getImageFrameDebugString.append("Frame set didn't have a infrared frame, ");
                            throw new StreamTypeNotEnabledException();
                        }
                        if (infraredFrameBuffer.length < frame.getDataSize()) {
                            getImageFrameDebugString.append("creating infraredFrameBuffer, ");
                            infraredFrameBuffer = new byte[frame.getDataSize()];
                        }
                        VideoFrame videoFrame = frame.as(Extension.VIDEO_FRAME);
                        videoFrame.getData(infraredFrameBuffer);
                        getImageFrameDebugString.append("got infraredFrameBuffer");
                        infraredWidth = videoFrame.getWidth();
                        infraredHeight = videoFrame.getHeight();
                        infraredStride = videoFrame.getStride();
                        //videoFrame.close(); //not needed because this frame doesn't own the resource
                        infraredFrameCached =true;
                    }
                }
                return new FrameData(infraredFrameBuffer,
                        infraredWidth,
                        infraredHeight);
            default:
                throw new UnsupportedStreamTypeException();
        }
    }

    public int getARGB(int x, int y) throws UnsupportedStreamTypeException, StreamTypeNotEnabledException, NoFramesetAvailable {
        if(!colorFrameCached) {
            getImageFrame(StreamType.COLOR);
        }
        if(!colorFrameCached) {
            if(haveCachedFrameSet) throw new StreamTypeNotEnabledException();
            throw new NoFramesetAvailable();
        }

        // see https://www.wikiwand.com/en/YUV#Y%E2%80%B2UV444_to_RGB888_conversion
        int c, d, e;
        int index;
        switch (colorStreamFormat)
        {
            case UYVY:
                //System.out.println("Getting UYVY, stride = "+colorStride);
                index = y*colorStride+((x>>2)<<3); // Get the lowest multiple of 4 address then multiply by 2
                c = byteToInt(colourFrameBuffer[y*colorStride+((x>>1)<<2)+1])-16;
                d = byteToInt(colourFrameBuffer[index])-128;
                e = byteToInt(colourFrameBuffer[index+2])-128;
                return Color.argb(0,
                        Math.min(Math.max((298*c+409*e+128)>>8,0),255),
                        Math.min(Math.max((298*c-100*d-208*e+128)>>8,0),255),
                        Math.min(Math.max((298*c+516*d+128)>>8,0),255));
            case YUYV:
                //System.out.println("Getting YUYV, stride = "+colorStride);
                index = y*colorStride+((x>>2)<<3); // Get the lowest multiple of 4 address then multiply by 2
                c = byteToInt(colourFrameBuffer[y*colorStride+((x>>1)<<2)])-16;
                d = byteToInt(colourFrameBuffer[index+1])-128;
                e = byteToInt(colourFrameBuffer[index+3])-128;
                System.out.println(
                        " stride: "+colorStride+
                        " index: "+index+
                        " indexofY: "+(y*colorStride+((x>>1)<<2))+
                        " x:"+x+
                        " y:"+y+
                        " c:"+c+
                        " d:"+d+
                        " e:"+e+
                        " Y:"+byteToInt(colourFrameBuffer[index+0])+
                        " U:"+byteToInt(colourFrameBuffer[index+1])+
                        " Y:"+byteToInt(colourFrameBuffer[index+2])+
                        " V:"+byteToInt(colourFrameBuffer[index+3]));
                return Color.argb(0,
                        Math.min(Math.max((298*c+409*e+128)>>8,0),255),
                        Math.min(Math.max((298*c-100*d-208*e+128)>>8,0),255),
                        Math.min(Math.max((298*c+516*d+128)>>8,0),255));
            case RGB8:
                System.out.println("Getting RGB");
                index = y*colorStride+x*3; // Get pixel index
                return Color.argb(0,
                        byteToInt(colourFrameBuffer[index]),
                        byteToInt(colourFrameBuffer[index+1]),
                        byteToInt(colourFrameBuffer[index+2]));
            default:
                throw new UnsupportedStreamTypeException();
        }
    }

    public void close() throws FrameQueueCloseException, CameraStopException {
        if(haveCachedDepthFrame) cachedDepthFrameRealFrame.close();
        if(haveCachedFrameSet) cachedAlignedFrameSet.close();
        pipeline.stop();
        pipeline.close();
        try {
            frameQueue.close();
        } catch (Exception e){
            throw new FrameQueueCloseException();
        }
    }

    private int byteToInt(byte x) {return x & 0xff;}
}
