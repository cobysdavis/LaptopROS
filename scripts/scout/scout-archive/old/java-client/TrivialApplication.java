/*
	Trivial application that displays a string - 4/96 PNL
*/
import java.awt.*;
import java.awt.image.*;

public class TrivialApplication {
	ReactiveRobot r;
	
	public void go() {
	/*
	  EigenPlace ep = new EigenPlace(20);
	  ep.readPairs();
	  Image mean = ep.makeMeanImage();
	  ImageFrame imf = new ImageFrame("Mean View", mean);
	  imf.setVisible(true);
	  for(int i=0;i<ep.images.length;i++){
	      imf = new ImageFrame("Base " + i, ep.images[i]);
	     imf.setVisible(true);
	   }
*/	  
	
	  SpaceSampler sp = new SpaceSampler("envy1", 1024);
	  sp.doSpots();
	  sp.done();
	
	
	/*
				r = new ReactiveRobot("envy1", 1054);
				  r.startup();
				 System.out.println("Query Pose " +  r.queryPose());
				 r.setPose(new RobotPose(1.0f, 2.0f, 0.0f));
				 System.out.println("Picture snapped at " + r.snapPicture());
				 System.out.println("Query Pose " +  r.queryPose());
				 System.out.println("Picture snapped at " + r.snapPicture());
				 
				 System.out.println("Getting image.....");
				 Image view = r.getImage();
				 ImageFrame imf = new ImageFrame("Robot View", view);
				 imf.setVisible(true);
				 System.out.println("Image collected");
				 System.out.println("Query Pose " +  r.queryPose());
				 r.setGoal(new RobotPose(1.0f, 2.0f, 180.0f));
				 while(r.queryState() == 0)
				 	System.out.println("Robot pose is " + r.queryPose());
				   ;
				 	 System.out.println("Picture snapped at " + r.snapPicture());
		
				 Image view2 = r.getImage();
				 ImageFrame imf2 = new ImageFrame("Robot View 2", view2);
				 imf2.setVisible(true);
				 
				 System.out.println("Query Pose " +  r.queryPose());
				  r.shutdown();
		System.out.println(r);
		*/
	}

	public static void main(String args[]) {
		System.out.println( "Hello World!" );
		(new TrivialApplication()).go();


	}
}
