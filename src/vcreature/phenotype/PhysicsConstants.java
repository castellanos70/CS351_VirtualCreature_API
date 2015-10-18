package vcreature.phenotype;

import com.jme3.math.Vector3f;

public class PhysicsConstants
{
  public static final float BLOCK_BOUNCINESS  = 0.5f;
  public static final float GROUND_BOUNCINESS = 0.3f;
  public static final float SLIDING_FRICTION = 1.0f;
  public static final float GROUND_SLIDING_FRICTION = 1.0f;
  
  public static final float LINEAR_DAMPINING = 0.2f;
  public static final float ANGULAR_DAMPINING = 0.1f;
  
  public static final float GROUND_LINEAR_DAMPINING = 1.0f;
  public static final float GROUND_ANGULAR_DAMPINING = 0.2f;

  public static final Vector3f GRAVITY = new Vector3f(0, -9.81f, 0); //meters/sec^2
  public static final float PHYSICS_UPDATE_RATE = 1f/80f;
  
  //Maxinium angular speed of the hinge joint in Radians per second. 
  //This value is so high that almost always the limiting factor will be the 
  //supplied impulse with acting on the block mass.
  public static final float JOINT_MAX_ANGULAR_SPEED = 1000f; 
  
  public static final float JOINT_ANGLE_MIN = -(float)(Math.PI/2.0);
  public static final float JOINT_ANGLE_MAX =  (float)(Math.PI/2.0);
}
