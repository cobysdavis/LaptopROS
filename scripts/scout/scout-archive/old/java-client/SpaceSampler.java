import java.awt.*;
import java.awt.image.*;

public class SpaceSampler {
  private final int nPoints = 20;
  private ReactiveRobot r;
  
  private Image[] images = new Image[nPoints];;
  private RobotPose[] poses = new RobotPose[nPoints];;
  
  
  SpaceSampler(String host, int port) {
    r = new ReactiveRobot(host, port);
    r.startup();
  }
  
  private Image doSpot(RobotPose rp) {
    System.out.println("Moving to " + rp + " to collect image.");
    r.setGoal(rp);
    while(r.queryState() == 0)
      ;
    RobotPose where = r.snapPicture();
    Image im = r.getImage();
    rp.setLocation(where);
    System.out.println("Image collected at " + rp);
    return im;
  }
  
  public void doSpots(){
    r.setPose(new RobotPose(0.0f, 0.0f, 0.0f));
    for(int i=0;i<5;i++){
      poses[i] = new RobotPose(0.0f, 0.0f, 360.0f * i / 5.0f);
      images[i] = doSpot(poses[i]);
      ImageFrame im = new ImageFrame("View " + poses[i], images[i]);
      try {
        PPMImagery ppm = new PPMImagery(images[i]);
        ppm.writeImage("Frame." + i);
      } catch(java.io.IOException rx) {
        System.out.println("Write of image " + i + " failed");
      }
      im.setVisible(true);

    }
    for(int i=5;i<10;i++){
      poses[i] = new RobotPose(1.0f, 0.0f, 360.0f * i / 5.0f);
      images[i] = doSpot(poses[i]);
      ImageFrame im = new ImageFrame("View " + poses[i], images[i]);
      im.setVisible(true);
    }
    r.setGoal(new RobotPose(0.0f, 0.0f, 0.0f));
    while(r.queryState() == 0)
      ;
  }
  
  public void done() {
    r.shutdown();
  }
  
}
  
  
  