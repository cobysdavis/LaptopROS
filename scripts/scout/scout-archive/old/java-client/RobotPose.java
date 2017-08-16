public class RobotPose {
  float x, y, t;
  
  RobotPose() {
    this(0.0f, 0.0f, 0.0f);
  }
  
  RobotPose(float x, float y, float t){
    this.x = x;
    this.y = y;
    this.t = t;
  }
  
  public void setLocation(RobotPose r){
    this.x = r.x;
    this.y = r.y;
    this.t = r.t;
  }
  
  public String toString() {
    return "RobtPose x:" + x + " y:" + y + " theta: " + t;
  }
}