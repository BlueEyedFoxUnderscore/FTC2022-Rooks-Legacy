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
import com.intel.realsense.librealsense.StreamType;
import com.intel.realsense.librealsense.VideoFrame;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ma.phoenix.ftc.realsensecamera.exceptions.NoFrameSetYetAcquiredException;
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
    private Pipeline mPipeline = new Pipeline();
    private boolean mPipelineStopped = true;

    private Device mDevice;
    private Sensor mSensor;

    private Config mConfig;
    private boolean mHaveConfig = false;

    private FrameQueue mFrameQueue = new FrameQueue(1);
    private FrameSet mCachedFrameSet;
    private boolean mHaveCachedFrameSet = false;

    private Frame mUncastedDepthFrame;
    private DepthFrame mCachedDepthFrame;
    private boolean mHaveCachedDepthFrame = false;


    private StreamFormat mColourStreamFormat;

    private Align alignFilter = new Align(StreamType.DEPTH); // Align other streams to the depth stream

    public byte[] infraredFrameBuffer = new byte[1];
    private int mInfraredWidth, mInfraredHeight, mInfraredStride;
    private boolean mHaveCachedInfraredFrameBuffer =false;
    //
    public byte[] colourFrameBuffer = new byte[1];
    private int mColourWidth, mColourHeight, mColourStride;
    private boolean mHaveCachedColourFrameBuffer = false;
    //
    public byte[] depthFrameBuffer = new byte[1];
    private int mDepthWidth, mDepthHeight, mDepthStride;
    private boolean mHaveCachedDepthFrameBuffer = false;

    private int gain = -1;
    private int exp = -1;

    public ConfigurableCamera(HardwareMap hardwareMap) throws DisconnectedCameraException, InterruptedException {
        System.out.println("constructor called");
        System.out.println("initializing context");
        RsContext.init(hardwareMap.appContext);
        System.out.println("sleeping for one second");
        Thread.sleep(2000);
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
        if(!mPipelineStopped) stop();
        PipelineProfile pipelineProfile;
        assert mHaveConfig: "A config must be set before calling start";
        try{
            System.out.println("starting pipeline");
            pipelineProfile = mPipeline.start(mConfig, mFrameQueue::Enqueue);
        }catch (Exception e) {
            e.printStackTrace();
            throw new CameraStartException();
        }
        //DEBUG: System.out.println("getting device");
        mDevice = pipelineProfile.getDevice();
        //DEBUG: System.out.println("setting advanced mode");
        if(!mDevice.isInAdvancedMode())  mDevice.toggleAdvancedMode(true); // If advanced mode is already enabled, setting advanced mode will crash the library.

        //DEBUG: System.out.println("getting sensor");
        mSensor = mDevice.querySensors().get(0);
        System.out.println("Current frames queue size: "+mSensor.getValue(Option.FRAMES_QUEUE_SIZE));
        mSensor.setValue(Option.FRAMES_QUEUE_SIZE, 12);
        System.out.println("Current frames queue size: "+mSensor.getValue(Option.FRAMES_QUEUE_SIZE));
        //DEBUG: List<StreamProfile> allActive= mSensor.getActiveStreams();
        // -     for(StreamProfile streamProfile : allActive) {
        // -         System.out.println(
        // -                "Active profile: unique id:"+streamProfile.getUniqueId() +
        // -                " Handle:" + streamProfile.getHandle() +
        // -                " Index:" + streamProfile.getIndex() +
        // -                " Type:" + streamProfile.getType() +
        // -                " Format:" + streamProfile.getFormat()+
        // -                " Frame rate:" + streamProfile.getFrameRate());
        // -      }

        // All Frames and FrameSets must be released via close().  If they are not, the library's internal
        // cache of frames will be exhausted and the library will stop transmitting images (until additional
        // frames are released.)  When the pipeline is closed, you will be told how many frames you are
        // "holding on to".  If the frames are large, the device will run out of memory.  In order to
        // prevent this possibility, it is necessary to reduce the maxim number of "publishable" frames
        // "Published" means that the library user has control of the frame.  When the frame is close()ed,
        // it is unpublished and made available for reuse to the library.
        // FRAMES_QUEUE_SIZE controls this.
        mPipelineStopped = false;
    }

    public void stop() throws CameraStopException {
        //DEBUG: System.out.println("stop() called");
        if(mPipelineStopped) return;
        try{
            //DEBUG: System.out.println("Stopping pipeline");
            mPipeline.stop();
            //DEBUG: System.out.println("Completed stop");
        } catch (Exception e){
            throw new CameraStopException();
        }
        mPipelineStopped = true;
    }

    private Device getDevice(){
        return mDevice;
    }

    private Sensor getSensor(){
        return mSensor;
    }

    public void switchConfig(Config toSwitch) throws CameraStartException, CameraStopException {
        //DEBUG: System.out.println("switchConfig called()");
        // -     System.out.println("calling stop()");
        if(!mPipelineStopped) stop();
        //DEBUG: System.out.println("getting reference to config");
        mConfig = toSwitch;
        //DEBUG: System.out.println("calling start()");
        mHaveConfig = true;
        if(mPipelineStopped) start();
    }

    public void setGain(int gain){
        //DEBUG: System.out.println("setGain()");
        this.gain = gain;
        //DEBUG: System.out.println("calling enableAutoExposure(false)");
        this.enableAutoExposure(false);
    }

    public void setExp(int exp){
        //DEBUG: System.out.println("setExp()");
        this.exp = exp;
        //DEBUG: System.out.println("calling enableAutoExposure(false)");
        this.enableAutoExposure(false);
    }

    public boolean updateFrameSet(){
        updateFrameSetDebugString.append("|| updateFrameSet() ");
        FrameSet newUnalignedFrameSet = mFrameQueue.pollForFrames();
        if(newUnalignedFrameSet == null) {
            return false;
        }
        updateFrameSetDebugString.append("New has "+newUnalignedFrameSet.getSize()+" frames, ");
        //FrameSet alignedFrameSet = newUnalignedFrameSet.applyFilter(alignFilter);
        //FrameSet alignedFrameSet = alignFilter.process(newUnalignedFrameSet);
        updateFrameSetDebugString.append("after align "+newUnalignedFrameSet.getSize()+" frames ");
        updateFrameSetDebugString.append("proccessed has "+alignedFrameSet.getSize()+" frames ");
        //newUnalignedFrameSet.close(); // The new frameset has all the frames, but me must close the old one, and it's fine to do it now.
        //updateFrameSetDebugString.append("after close 'new' processed has "+alignedFrameSet.getSize()+"frames. ");
        if(mHaveCachedFrameSet) {
            updateFrameSetDebugString.append("closed old fs. ");
            mCachedFrameSet.close();
            // No need to set this to false because we're going to immediately
            // repopulate the reference and set haveCachedFrameSet to true.
        }
        if (mHaveCachedDepthFrame) {
            updateFrameSetDebugString.append("closed old depthframe. ");
            //cachedDepthFrame.close(); Not necessary because this frame does not own the resource.
            mUncastedDepthFrame.close();
            mHaveCachedDepthFrame =false; /////////// *!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        }
        mCachedFrameSet = newUnalignedFrameSet;
        mHaveCachedFrameSet = true;
        mHaveCachedColourFrameBuffer = false;
        mHaveCachedInfraredFrameBuffer = false;
        mHaveCachedDepthFrameBuffer = false;
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

    public void cacheDepthFrameIfNecessary() throws NoFrameSetYetAcquiredException, StreamTypeNotEnabledException {
        getDepthFrameIfNecessaryDebugString.append("||getDepthFrameIfNecessary() ");
        if(mHaveCachedDepthFrame)return;
        if(!mHaveCachedFrameSet) throw new NoFrameSetYetAcquiredException();
        mUncastedDepthFrame = mCachedFrameSet.first(StreamType.DEPTH);
        if(mUncastedDepthFrame == null){
            throw new StreamTypeNotEnabledException();
            //getDepthFrameIfNecessaryDebugString.append("Frameset but no depth frame. ");
        }
        mCachedDepthFrame = mUncastedDepthFrame.as(Extension.DEPTH_FRAME);
        //getDepthFrameIfNecessaryDebugString.append("Found new depthframe. ");
        mHaveCachedDepthFrame = true;
    }

    public float getDistance(int x, int y) throws StreamTypeNotEnabledException, NoFrameSetYetAcquiredException {
        //DEBUG: System.out.println("getDistance("+x+", "+y+")"); //848x480
        getDepthFrameIfNecessaryDebugString.append("||Get distance() called getDepthFrameIfNecessary() ");
        cacheDepthFrameIfNecessary();
        if(x==600) getDepthFrameIfNecessaryDebugString.append("returning ("+x+", "+y+")="+ mCachedDepthFrame.getDistance(x, y));
        return mCachedDepthFrame.getDistance(x, y);
    }

    public FrameData getImageFrame(StreamType type) throws UnsupportedStreamTypeException, StreamTypeNotEnabledException, NoFrameSetYetAcquiredException {
        getImageFrameDebugString.append("||getImageFrame() ");
        if(!mHaveCachedFrameSet){
            getImageFrameDebugString.append("No frameset. ");
            throw new NoFrameSetYetAcquiredException();
        }
        //System.out.println("checking For Frame");
        switch (type) {
            case DEPTH:
                if(!mHaveCachedDepthFrameBuffer) {
                    cacheDepthFrameIfNecessary();
                    //getImageFrameDebugString.append("First depth req., called getDepthFrameIfNecessary(), ");
                    if (depthFrameBuffer.length < mUncastedDepthFrame.getDataSize()) {
                        getImageFrameDebugString.append("creating depthFrameBuffer, ");
                        depthFrameBuffer = new byte[mUncastedDepthFrame.getDataSize()];
                    }
                    mUncastedDepthFrame.getData(depthFrameBuffer);
                    getImageFrameDebugString.append("got depthFrameBuffer");
                    mDepthWidth = mCachedDepthFrame.getWidth();
                    mDepthHeight = mCachedDepthFrame.getHeight();
                    mDepthStride = mCachedDepthFrame.getStride();
                    mHaveCachedDepthFrameBuffer = true;
                }
                return new FrameData(depthFrameBuffer,
                        mDepthWidth,
                        mDepthHeight,
                        mDepthStride);
            case COLOR:
                if (!mHaveCachedColourFrameBuffer) {
                    //getImageFrameDebugString.append("First color req., ");
                    try(Frame frame = mCachedFrameSet.first(type)) {
                        //DEBUG: System.out.println("testing if we support it");
                        // -     System.out.println("testing if frame exists in frameSet");
                        if (frame == null) {
                            getImageFrameDebugString.append("Frame set didn't have a color frame, ");
                            throw new StreamTypeNotEnabledException();
                        }
                        //DEBUG: System.out.println("color frame found");
                        // -     System.out.println("is a color frame");
                        if (colourFrameBuffer.length < frame.getDataSize()) {
                            //getImageFrameDebugString.append("creating colourFrameBuffer frame buffer, ");
                            colourFrameBuffer = new byte[frame.getDataSize()];
                        }
                        //DEBUG: System.out.println("getting frame buffer data");
                        VideoFrame videoFrame = frame.as(Extension.VIDEO_FRAME);
                        videoFrame.getData(colourFrameBuffer);
                        //getImageFrameDebugString.append("got colourFrameBuffer ");
                        //getImageFrameDebugString.append("format is "+ mColourStreamFormat);
                        mColourWidth = videoFrame.getWidth();
                        mColourHeight = videoFrame.getHeight();
                        mColourStride = videoFrame.getStride();
                        mColourStreamFormat = videoFrame.getProfile().getFormat();
                        mHaveCachedColourFrameBuffer = true;
                        //UNNEEDED: videoFrame.close(); Not needed because it does not own frame
                    }
                }
                return new FrameData(colourFrameBuffer,
                        mColourWidth,
                        mColourHeight,
                        mColourStride);
            case INFRARED:
                if(!mHaveCachedInfraredFrameBuffer) {
                    getImageFrameDebugString.append("First infrared req., ");
                    try(Frame frame = mCachedFrameSet.first(type)) {
                        //DEBUG: System.out.println("testing if we support it");
                        //DEBUG: System.out.println("testing if frame exists in frameSet");
                        if (frame == null) {
                            getImageFrameDebugString.append("Frame set didn't have a infrared frame, ");
                            throw new StreamTypeNotEnabledException();
                        }
                        //DEBUG: System.out.println("is a depth frame");
                        //DEBUG: System.out.println("is a infrared frame");
                        if (infraredFrameBuffer.length < frame.getDataSize()) {
                            //getImageFrameDebugString.append("creating infraredFrameBuffer, ");
                            infraredFrameBuffer = new byte[frame.getDataSize()];
                        }
                        //DEBUG: System.out.println("getting frame buffer data");
                        getImageFrameDebugString.append("got infraredFrameBuffer");
                        VideoFrame videoFrame = frame.as(Extension.VIDEO_FRAME);
                        videoFrame.getData(infraredFrameBuffer);
                        mInfraredWidth = videoFrame.getWidth();
                        mInfraredHeight = videoFrame.getHeight();
                        mInfraredStride = videoFrame.getStride();
                        mHaveCachedInfraredFrameBuffer = true;
                        //UNNEEDED: videoFrame.close(); Not needed because it does not own frame
                    }
                }
                return new FrameData(infraredFrameBuffer,
                        mInfraredWidth,
                        mInfraredHeight,
                        mInfraredStride);
            default:
                throw new UnsupportedStreamTypeException();
        }
    }

    public int getARGB(int x, int y) throws UnsupportedStreamTypeException, StreamTypeNotEnabledException, NoFrameSetYetAcquiredException {
        //System.out.println("GetARGB("+x+","+y+")");
        if(!mHaveCachedColourFrameBuffer) {
            //System.out.println("Getting color frame");
            getImageFrame(StreamType.COLOR);
        }

        // see https://www.wikiwand.com/en/YUV#Y%E2%80%B2UV444_to_RGB888_conversion
        int c, d, e;
        int index;
        switch (mColourStreamFormat)
        {
            case UYVY:
                //System.out.println("Getting UYVY");
                // UYVY UYVY UYVY UYVYUYVYUYVY
                //
                // 1/2 -> 0 * 2 -> 0
                index = y*mColourStride+(x>>1)<<2; // Get the lowest multiple of 4 address then multiply by 2
                c = byteToInt(colourFrameBuffer[y*mColourStride+x<<1+1])-16;
                d = byteToInt(colourFrameBuffer[index])-128;
                e = byteToInt(colourFrameBuffer[index+2])-128;
                return Color.argb(0,
                        Math.min(Math.max((298*c+409*e+128)>>8,0),255),
                        Math.min(Math.max((298*c-100*d-208*e+128)>>8,0),255),
                        Math.min(Math.max((298*c+516*d+128)>>8,0),255));
            case YUYV:
                //System.out.println("Getting YUYV, stride = "+colorStride);
                index = y*mColourStride+(x>>1)<<2; // Get the lowest multiple of 4 address then multiply by 2
                c = byteToInt(colourFrameBuffer[y*mColourStride+x<<1])-16;
                d = byteToInt(colourFrameBuffer[index+1])-128;
                e = byteToInt(colourFrameBuffer[index+3])-128;
                System.out.println(
                        " stride: "+ mColourStride +
                        " index: "+index+
                        " indexofY: "+(y* mColourStride +((x>>1)<<2))+
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
                index = y*mColourStride + x*3; // Get pixel index
                return Color.argb(0,
                        byteToInt(colourFrameBuffer[index]),
                        byteToInt(colourFrameBuffer[index+1]),
                        byteToInt(colourFrameBuffer[index+2]));
            default:
                throw new UnsupportedStreamTypeException();
        }
    }

    public void close() throws FrameQueueCloseException, CameraStopException {
        //DEBUG: System.out.println("close() called");
        //DEBUG: System.out.println("calling stop()");
        stop();
        //DEBUG: System.out.println("closing pipeline");
        if(mHaveCachedFrameSet) mCachedFrameSet.close();
        //DEBUG: System.out.println("closing depthFrame");
        if(mHaveCachedDepthFrame) mUncastedDepthFrame.close();

        if(!mPipelineStopped) mPipeline.close();

        try {
            mFrameQueue.close();
        } catch (Exception e){
            throw new FrameQueueCloseException();
        }
    }

    private int byteToInt(byte x) {return x & 0xff;}
}
