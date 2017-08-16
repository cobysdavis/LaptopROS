import java.net.*;
import java.io.*;

import java.awt.*;
import java.awt.image.*;

public class ReactiveRobot {
  private final int imageWidth = 160;   // width of image from onboard digitizer (max 640)
  private final int imageHeight = 120;  // height of image (max 480)
  private final int blockFactor = 4;    // number of rows per block 
  
  private Socket theSocket; 
  private DataInputStream in;
  private DataOutputStream out;
  
  private byte[] magic_number = {0x12, 0x34, 0x56, 0x78};

  ReactiveRobot(String host, int port) {
    try {
      theSocket = new Socket(host, port);
      in = new DataInputStream(theSocket.getInputStream());
      out = new DataOutputStream(theSocket.getOutputStream());
    } catch(Exception e){
      System.out.println("Unable to connect to robot " + host + " at port " + port);
      theSocket = null;
    }
  }
  
  
  public boolean startup() {
    byte[] startup = {0x00, 0x04};
    
    byte[] filler = new byte[50];
    byte[] resp = new byte[56];
    byte[] msg = new byte[56];
    
    try {
      for(int i= 0;i<4;i++)
        msg[i] = magic_number[i];
      for(int i = 0; i < 2; i++)
        msg[4+i] = startup[i];
      for(int i = 0; i< 50; i++)
        msg[6+i] = 0;
      out.write(msg, 0, msg.length);
      out.flush();
      if(in.read(resp) != resp.length)
        System.out.println("startup read failed " + resp);
    } catch(IOException e){
      System.out.println("Read failed " + e);
    }    
    return resp[7] == 1;
    
  }
  
   public void shutdown() {
    byte[] startup = {0x00, 0x05};
    
    byte[] filler = new byte[50];
    byte[] resp = new byte[56];
    byte[] msg = new byte[56];
    
    try {
      for(int i= 0;i<4;i++)
        msg[i] = magic_number[i];
      for(int i = 0; i < 2; i++)
        msg[4+i] = startup[i];
      for(int i = 0; i< 50; i++)
        msg[6+i] = 0;
      out.write(msg, 0, msg.length);
      out.flush();
      if(in.read(resp) != resp.length)
        System.out.println("Query pose failed " + resp);
    } catch(IOException e){
      System.out.println("Read failed " + e);
    }
    if(resp[7] != 1)
      System.out.println("shutdown did not ACK");
    this.close();
    this.theSocket = null;
  }
  
  public void setPose(RobotPose r) { // tell the robot where it is
    byte[] set_pose = {0x00, 0x06};
    
    byte[] v;
    byte[] resp = new byte[56];
    byte[] msg = new byte[56];
    
    try {
      for(int i= 0;i<4;i++)
        msg[i] = magic_number[i];
      for(int i = 0; i < 2; i++)
        msg[4+i] = set_pose[i];
      
      // put the values in 
      v = floatToBytes(r.x);
      for(int i=0;i<4;i++)
        msg[8+i] = v[i];
      v = floatToBytes(r.y);
      for(int i=0;i<4;i++)
        msg[12+i] = v[i];
      v = floatToBytes(r.t);
      for(int i=0;i<4;i++)
        msg[16+i] = v[i];
      
      out.write(msg, 0, msg.length);
      out.flush();
      if(in.read(resp) != resp.length)
        System.out.println("setPose failed " + resp);
    } catch(IOException e){
      System.out.println("Read failed " + e);
    }
    if(resp[7] != 1){
      System.out.println("setPose did not ACK");
    }
  }
  
  /**
   * set the Robot's goal location as a RobotPose. The robot will start to move
   * to the goal location as soon as this is set.
   */
  public void setGoal(RobotPose r) { // tell the robot where the Goal is
    byte[] set_pose = {0x00, 0x07};
    
    byte[] v;
    byte[] resp = new byte[56];
    byte[] msg = new byte[56];
    
    try {
      for(int i= 0;i<4;i++)
        msg[i] = magic_number[i];
      for(int i = 0; i < 2; i++)
        msg[4+i] = set_pose[i];
      
      // put the values in 
      v = floatToBytes(r.x);
      for(int i=0;i<4;i++)
        msg[20+i] = v[i];
      v = floatToBytes(r.y);
      for(int i=0;i<4;i++)
        msg[24+i] = v[i];
      v = floatToBytes(r.t);
      for(int i=0;i<4;i++)
        msg[28+i] = v[i];
      
      out.write(msg, 0, msg.length);
      out.flush();
      if(in.read(resp) != resp.length)
        System.out.println("setGoal failed " + resp);
    } catch(IOException e){
      System.out.println("Read failed " + e);
    }
    if(resp[7] != 1){
      System.out.println("setGoal did not ACK");
    }
  }  
  
  public RobotPose queryPose() {  // query where the robot thinks it is
    byte[] query_pose = {0x00, 0x0a};
    byte[] resp = new byte[56];
    byte[] msg = new byte[56];
    
    try {
      for(int i= 0;i<4;i++)
        msg[i] = magic_number[i];
      for(int i = 0; i < 2; i++)
        msg[4+i] = query_pose[i];
      for(int i = 0; i< 50; i++)
        msg[6+i] = 0;
      out.write(msg, 0, msg.length);
      out.flush();
      if(in.read(resp) != resp.length)
        System.out.println("Query pose failed " + resp);
    } catch(IOException e){
      System.out.println("Read failed " + e);
    }
    if(resp[7] != 1){
      System.out.println("queryPose did not ACK");
      return new RobotPose();
    }    
    return new RobotPose(bytesToFloat(resp, 8), bytesToFloat(resp, 12), bytesToFloat(resp, 16));
  }      
    
  /**
   * Query the robot and determine its state. Possible return values are
   * <ol>
   * <li> 0 (RCT_MOVING) - the robot is currently moving towards the local goal.
   * <li> 1 (RCT_GOAL) - the robot is currently at the goal.
   * <li> 2 (RCT_BLOCKED) - the robot is currently blocked.
   * </ol>
   */
  public int queryState() {  
    byte[] query_pose = {0x00, 0x0c};
    byte[] resp = new byte[56];
    byte[] msg = new byte[56];
    
    try {
      for(int i= 0;i<4;i++)
        msg[i] = magic_number[i];
      for(int i = 0; i < 2; i++)
        msg[4+i] = query_pose[i];
      for(int i = 0; i< 50; i++)
        msg[6+i] = 0;
      out.write(msg, 0, msg.length);
      out.flush();
      if(in.read(resp) != resp.length)
        System.out.println("Query state failed " + resp);
    } catch(IOException e){
      System.out.println("Read failed " + e);
    }
    if(resp[7] != 1){
      System.out.println("queryState did not ACK");
      return -1;
    }    
    return bytesToInt(resp, 52);
  }      
  
 /**
   * Take a picture with the onboard video system. The picture is stamped with
   * robot pose when it was taken. There is only a single image buffer onboard so
   * multiple snaps will overwrite each other.   
   */
  public RobotPose snapPicture() {  
    byte[] snap_picture = {0x00, 0x14};
    byte[] resp = new byte[56];
    byte[] msg = new byte[56];
    
    try {
      for(int i= 0;i<4;i++)
        msg[i] = magic_number[i];
      for(int i = 0; i < 2; i++)
        msg[4+i] = snap_picture[i];
      for(int i = 0; i< 50; i++)
        msg[6+i] = 0;
      out.write(msg, 0, msg.length);
      out.flush();
      if(in.read(resp) != resp.length)
        System.out.println("Short snap picture call " + resp);
    } catch(IOException e){
      System.out.println("Read failed " + e);
    }
    return new RobotPose(bytesToFloat(resp, 8), bytesToFloat(resp, 12), bytesToFloat(resp, 16));    
  }   
  
  /**
   * get a scanline of the image
   *
   */
  public byte[] getBlock(int blockid) {
    final int blocksize = blockFactor * imageWidth;
    byte[] get_scanline = {0x00, 0x13};
    byte[] resp = new byte[56];
    byte[] msg = new byte[56];
    byte[] row = new byte[2*blocksize]; // rows 16 bit pixels/row
    
    try {
      for(int i= 0;i<4;i++)
        msg[i] = magic_number[i];
      for(int i = 0; i < 2; i++)
        msg[4+i] = get_scanline[i];
      byte[] rowb = intToBytes(blockid);
      for(int i=0;i<4;i++)
        msg[8+i] = rowb[i];
      out.write(msg, 0, msg.length);
      out.flush();
      if(in.read(resp) != resp.length)
        System.out.println("Short snap picture call " + resp);
    } catch(IOException e){
      System.out.println("Read failed " + e);
    }
    if(resp[7] != 1){
      System.out.println("getRow did not ACK");
      return null;
    }    
    try {
      if(in.read(row) != row.length){
        System.out.println("Short read of row " + blockid);
        return null;
      }
    } catch(IOException e){
      System.out.println("Bad read of row " + blockid);
      return null;
    }
    return row;
  }   
  
  /**
   * gen an Image from the camera
   */
  public Image getImage() {
    int[] im = new int[imageWidth*imageHeight];
    byte[] rowdata;
    
    int o = 0;
    for(int i=0;i<imageHeight;i+=blockFactor){
      rowdata =  this.getBlock(i);
      for(int j=0;j<imageWidth*blockFactor;j++){
        int v =  (0xff00 & (rowdata[j*2+1] << 8))|(0xff & rowdata[j*2]);
        im[o++] = (0xff<<24) |(0xff0000 & (v<<9))|(0x00ff00 & (v<<6))
        |(0x0000ff & (v << 3));
      }
    }
    MemoryImageSource mis = new MemoryImageSource(imageWidth, imageHeight, im, 0, imageWidth);
    return Toolkit.getDefaultToolkit().createImage(mis); 
  } 
   
    
  /**
    * Close the connection to the robot. 
    */
  private void close() {
    if(this.isAlive()) {
      try {
        theSocket.close();
      } catch(IOException e) {
        System.out.println("Closing the robot failed!");
      }
      theSocket = null;
    }
  }
  
  
  private float bytesToFloat(byte[] b, int o) { // turn 4 bytes into a float
    int v = bytesToInt(b, o);
    return Float.intBitsToFloat(v);
  }
  
  private byte[] floatToBytes(float v) {
    return intToBytes(Float.floatToIntBits(v));
  }
  
  private byte[] intToBytes(int v) { // turn an int into 4 bytes
    byte[] res =  new byte[4];
    
    res[0] = (byte)(0xff & (v >> 24));
    res[1] = (byte)(0xff & (v >> 16));
    res[2] = (byte)(0xff & (v >> 8));
    res[3] = (byte)(0xff & v);
    
    return res;
  } 
  
  private int bytesToInt(byte[] b, int o){ // turn 4 bytes into an int
    return (0xff000000 & (b[o]<<24))|
           (0x00ff0000 & (b[o+1]<<16))|
           (0x0000ff00 & (b[o+2]<<8))|
           (0x000000ff & (b[o+3]));
  }
    
  public boolean isAlive() {
    return theSocket != null;
  }
  
  public String toString() {
    if(this.isAlive())
      return "Reactive robot at " + theSocket.getInetAddress() + " on port " +
        theSocket.getPort();
    else
      return "Reactive robot is not alive";
  }
}
  
  