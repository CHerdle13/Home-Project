/* Copyright 2011-2013 Google Inc.
 * Copyright 2013 mike wakerly <opensource@hoho.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 *
 * Project home page: https://github.com/mik3y/usb-serial-for-android
 */

package src.com.ch;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.util.Log;
import android.view.WindowManager;
import android.widget.ScrollView;
import android.widget.TextView;

//import com.hoho.android.usbserial.examples.R;
import com.ch.R;
import com.ch.driver.UsbSerialPort;
import com.ch.util.HexDump;
import com.ch.util.SerialInputOutputManager;

import java.io.IOException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;

import static android.graphics.Color.blue;
import static android.graphics.Color.colorToHSV;
import static android.graphics.Color.green;
import static android.graphics.Color.red;

/**
 * Monitors a single {@link UsbSerialPort} instance, showing all data
 * received.
 *
 * @author mike wakerly (opensource@hoho.com)
 */
public class SerialConsoleActivity extends Activity implements TextureView.SurfaceTextureListener {

    private Camera mCamera;
    private TextureView mTextureView;
    private SurfaceView mSurfaceView;
    private SurfaceHolder mSurfaceHolder;
    private Bitmap bmp = Bitmap.createBitmap(640,480,Bitmap.Config.ARGB_8888);
    private Canvas canvas = new Canvas(bmp);
    private Paint paint1 = new Paint();
    private TextView mTextView;

    static long prevtime = 0;

    private final String TAG = SerialConsoleActivity.class.getSimpleName();

    /**
     * Driver instance, passed in statically via
     * {@link #show(Context, UsbSerialPort)}.
     *
     * <p/>
     * This is a devious hack; it'd be cleaner to re-create the driver using
     * arguments passed in with the {@link #startActivity(Intent)} intent. We
     * can get away with it because both activities will run in the same
     * process, and this is a simple demo.
     */
    private static UsbSerialPort sPort = null;

    private TextView mTitleTextView;
    private TextView mDumpTextView;
    private ScrollView mScrollView;

    private final ExecutorService mExecutor = Executors.newSingleThreadExecutor();

    private SerialInputOutputManager mSerialIoManager;

    private final SerialInputOutputManager.Listener mListener =
            new SerialInputOutputManager.Listener() {

        @Override
        public void onRunError(Exception e) {
            Log.d(TAG, "Runner stopped.");
        }

        @Override
        public void onNewData(final byte[] data) {
            SerialConsoleActivity.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    SerialConsoleActivity.this.updateReceivedData(data);
                }
            });
        }
    };

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.serial_console);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        mTitleTextView = (TextView) findViewById(R.id.demoTitle);
        mDumpTextView = (TextView) findViewById(R.id.consoleText);
        mScrollView = (ScrollView) findViewById(R.id.demoScroller);

        mSurfaceView = (SurfaceView) findViewById(R.id.surfaceview);
        mSurfaceHolder = mSurfaceView.getHolder();

        mTextureView = (TextureView) findViewById(R.id.textureview);
        mTextureView.setSurfaceTextureListener(this);

        mTextView = (TextView) findViewById(R.id.cameraStatus);

        paint1.setColor(0xff0000ff); // blue?
        paint1.setTextSize(24);
    }

    @Override
    protected void onPause() {
        super.onPause();
        stopIoManager();
        if (sPort != null) {
            try {
                sPort.close();
            } catch (IOException e) {
                // Ignore.
            }
            sPort = null;
        }
        finish();
    }

    @Override
    protected void onResume() {
        super.onResume();
        Log.d(TAG, "Resumed, port=" + sPort);
        if (sPort == null) {
            mTitleTextView.setText("No serial device.");
        } else {
            final UsbManager usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);

            UsbDeviceConnection connection = usbManager.openDevice(sPort.getDriver().getDevice());
            if (connection == null) {
                mTitleTextView.setText("Opening device failed");
                return;
            }

            try {
                sPort.open(connection);
                sPort.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
            } catch (IOException e) {
                Log.e(TAG, "Error setting up device: " + e.getMessage(), e);
                mTitleTextView.setText("Error opening device: " + e.getMessage());
                try {
                    sPort.close();
                } catch (IOException e2) {
                    // Ignore.
                }
                sPort = null;
                return;
            }
            mTitleTextView.setText("Serial device: " + sPort.getClass().getSimpleName());
        }
        onDeviceStateChange();
    }

    private void stopIoManager() {
        if (mSerialIoManager != null) {
            Log.i(TAG, "Stopping io manager ..");
            mSerialIoManager.stop();
            mSerialIoManager = null;
        }
    }

    private void startIoManager() {
        if (sPort != null) {
            Log.i(TAG, "Starting io manager ..");
            mSerialIoManager = new SerialInputOutputManager(sPort, mListener);
            mExecutor.submit(mSerialIoManager);
        }
    }

    private void onDeviceStateChange() {
        stopIoManager();
        startIoManager();
    }

    private void updateReceivedData(byte[] data) {
        final String message = "Read " + data.length + " bytes: \n"
                + HexDump.dumpHexString(data) + "\n\n";
        mDumpTextView.append(message);
        mScrollView.smoothScrollTo(0, mDumpTextView.getBottom());
    }

    /**
     * Starts the activity, using the supplied driver instance.
     *
     * @param context
     * @param driver
     */
    static void show(Context context, UsbSerialPort port) {
        sPort = port;
        final Intent intent = new Intent(context, SerialConsoleActivity.class);
        intent.addFlags(Intent.FLAG_ACTIVITY_SINGLE_TOP | Intent.FLAG_ACTIVITY_NO_HISTORY);
        context.startActivity(intent);
    }

    public boolean isRed(int pixel) {
        float [] hsv = new float [3];
        colorToHSV(pixel, hsv);
        if (hsv[0] > 340 && hsv [1] > .65 && hsv[2] > .5){
            return true;
        }
        else if (hsv[0] < 10 && hsv [1] > .65 && hsv[2] > .5){
            return true;
        }
        else {
            return false;
        }
        /*if (red(pixel) > 125 && green(pixel) < 100 && blue(pixel) < 100)
        {
            return true;
        }
        else if (red(pixel) > 200 && green(pixel) < 150 && blue(pixel) < 150)
        {
            return true;
        }
        else {
            return false;
        }
        */
    }
    public boolean isBlue(int pixel) {
        float [] hsv = new float [3];
        colorToHSV(pixel,hsv);
        if (hsv[0] > 200 && hsv[0] < 250 && hsv [1] > .4 && hsv[2] > .3){
            return true;
        }
        else {
            return false;
        }
    }

    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        mCamera = Camera.open();
        Camera.Parameters parameters = mCamera.getParameters();
        parameters.setPreviewSize(640, 480);
        //parameters.setColorEffect(Camera.Parameters.EFFECT_MONO); // black and white
        parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_INFINITY); // no autofocusing
        mCamera.setParameters(parameters);
        mCamera.setDisplayOrientation(90); // rotate to portrait mode

        try {
            mCamera.setPreviewTexture(surface);
            mCamera.startPreview();
        } catch (IOException ioe) {
            // Something bad happened
        }
    }

    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        // Ignored, Camera does all the work for us
    }

    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        mCamera.stopPreview();
        mCamera.release();
        return true;
    }

    public int centerOfMass(int[] pixels){
        int[] pixelWeights = new int[pixels.length];
        int wbTotal = 0; // total mass
        int wbCOM = 0; // total (mass time position)
        for (int i = 0; i < pixels.length; i++) {
            // sum the red, green and blue, subtract from 255 to get the darkness of the pixel.
            // if it is greater than some value (600 here), consider it black
            // play with the 600 value if you are having issues reliably seeing the line

            if (isBlue(pixels[i])) {
                pixelWeights[i] = 255*3;
            }
            else {
                pixelWeights[i] = 0;
            }
            wbTotal = wbTotal + pixelWeights[i];
            wbCOM = wbCOM + pixelWeights[i]*i;
        }
        int COM;
        //watch out for divide by 0
        if (wbTotal<=0) {
            COM = bmp.getWidth()/2;
            moveState = 1;
        }
        else {
            COM = wbCOM/wbTotal;
        }
        return COM;
    }

   //public int startY = 320; // which row in the bitmap to analyse
   // public int startX = 240; // which column in the bitmap to analyse
    public int targetX = 320;
    public int targetY = 240;
    public int moveState = 0;
    public int upperBound, lowerBound;
    public boolean hasStarted = false;

    // the important function
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        // Invoked every time there's a new Camera preview frame
        mTextureView.getBitmap(bmp);
        final Canvas c = mSurfaceHolder.lockCanvas();
        if (c != null) {

            int[] xPixels = new int[bmp.getWidth()];
            int[] yPixels = new int[bmp.getHeight()];


            // only look at one row in the image
            bmp.getPixels(xPixels, 0, bmp.getWidth(), 0, targetY, bmp.getWidth(), 1); // (array name, offset inside array, stride (size of row), start x, start y, num pixels to read per row, num rows to read)
            bmp.getPixels(yPixels,0, 1, targetX, 0, 1, bmp.getHeight());

            if (!hasStarted) {
                canvas.drawText("Not Started", 20, 10, paint1);
                if (isBlue(xPixels[xPixels.length/2])) {
                    hasStarted = true;
                    int firstred = 1;
                    //int lastred = 0;
                    for (int y = 0; y < yPixels.length; y++){
                        if (isBlue(yPixels[y])){
                            if (firstred == 1) {
                                if (y >= 10) {
                                    lowerBound = y - 10;
                                } else {
                                    lowerBound = 0;
                                }
                                //lastred = 1;
                                firstred = 0;
                            }
                            if (y <= yPixels.length - 10) {
                                upperBound = y + 10;
                            } else {
                                upperBound = yPixels.length;
                            }


                            /*if (lastred == 1)
                            {
                                if (y <= 630) {
                                    upperBound = y + 10;
                                } else {
                                    upperBound = 640;
                                }

                            }*/
                       }
                    }
                    targetX = centerOfMass(xPixels);
                    targetY = centerOfMass(yPixels);
                }
            }
            else {
                moveState = 2;
                targetX = centerOfMass(xPixels);
                targetY = centerOfMass(yPixels);
            }



            // pixels[] is the RGBA data (in black and white).
            // instead of doing center of mass on it, decide if each pixel is dark enough to consider black or white
            // then do a center of mass on the thresholded array

            //int[] thresholdedPixels = new int[bmp.getWidth()];
            //int wbTotal = 0; // total mass
            //int wbCOM = 0; // total (mass time position)
            //for (int i = 0; i < bmp.getWidth(); i++) {
                // sum the red, green and blue, subtract from 255 to get the darkness of the pixel.
                // if it is greater than some value (600 here), consider it black
                // play with the 600 value if you are having issues reliably seeing the line

                //if (red(xPixels[i]) > 125 && green(xPixels[i]) < 100 && blue(xPixels[i]) < 100)
                //{

                  //  thresholdedPixels[i] = 255*3;
                //}
                //else if (red(xPixels[i]) > 200 && green(xPixels[i]) < 150 && blue(xPixels[i]) < 150)
                //{
                    //thresholdedPixels[i] = 255*3;
                //}


                /*if (255*3-(red(pixels[i])+green(pixels[i])+blue(pixels[i])) > 600) {
                    thresholdedPixels[i] = 255*3;
                }*/

                /*else {
                    thresholdedPixels[i] = 0;
                }
                wbTotal = wbTotal + thresholdedPixels[i];
                wbCOM = wbCOM + thresholdedPixels[i]*i;
            }
            int COM;
            //watch out for divide by 0
            if (wbTotal<=0) {
                COM = bmp.getWidth()/2;
            }
            else {
                COM = wbCOM/wbTotal;
            }*/

            // draw a circle where you think the COM is


            canvas.drawCircle(targetX, targetY, 5, paint1);

            // also write the value as text
            canvas.drawText("COM = " + targetX, 10, 200, paint1);
            c.drawBitmap(bmp, 0, 0, null);
            mSurfaceHolder.unlockCanvasAndPost(c);

            // calculate the FPS to see how fast the code is running
            long nowtime = System.currentTimeMillis();
            long diff = nowtime - prevtime;
            mTextView.setText("FPS " + 1000/diff);
            prevtime = nowtime;

            int i = targetX; int j = moveState; // the two numbers to send
            String sendString = String.valueOf(i) + " " + String.valueOf(j);
            try {
                sPort.write(sendString.getBytes(),10); // 10 is the timeout (error)
            }
            catch (IOException e) {}

            //byte[] sData = {'a',0}; try { sPort.write(sData, 10); } catch (IOException e) { }
        }
    }

}
