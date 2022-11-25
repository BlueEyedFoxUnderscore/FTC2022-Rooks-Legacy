package ma.phoenix.ftc.cameradebugger;

import android.util.Log;

import com.arcrobotics.ftclib.command.LogCatCommand;
import com.qualcomm.robotcore.util.RobotLog;

public class ImageTransmitter {
    private final static char[] hexArray = "0123456789ABCDEF".toCharArray();

    private static String bytesToHex(byte[] bytes, int start, int length) {
        char[] hexChars = new char[length * 2];
        for ( int j = 0; j < length; j++ ) {
            int v = bytes[j + start] & 0xFF;
            hexChars[j * 2] = hexArray[v >> 4];
            hexChars[j * 2 + 1] = hexArray[v & 0x0F];
        }
        return new String(hexChars);
    }

    //static int skip = 0;
    public static void transmitImage(ImageType type, byte[] frameBuffer, int width, int height){
        final int CHUNK_SIZE = 2000;
        //if (skip>0) {skip--; return;}
        //skip=5;
        //System.out.println("frame number for sending: " + sendAttempt++);

        //bite a chunk of bytes off an array of bytes

        System.out.println("ImageTransmitter ImageType: " + type.name());
        {
            System.out.println("width: "+width);
            System.out.println("width: "+height);
            int i;
            int j=0;
            for (i = 0; i + CHUNK_SIZE <= frameBuffer.length; i += CHUNK_SIZE) {
                Log.d("x", "ImageTransmitter ImageChunk: " + bytesToHex(frameBuffer, i, CHUNK_SIZE));

                if(j++>10) {
                    j=0;
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
            if(frameBuffer.length != i) System.out.println("ImageTransmitter ImageChunk: " + bytesToHex(frameBuffer, i, frameBuffer.length - i));
        }

        System.out.println("ImageTransmitter ImageWidth: " + width);
        System.out.println("ImageTransmitter ImageHeight: "+ height);
    }
}
