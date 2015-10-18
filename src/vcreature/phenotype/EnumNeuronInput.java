package vcreature.phenotype;

/**
 * The Enum EnumNeuronInput.
 */
public enum EnumNeuronInput
{
  /** Height (distance above ground) of center of specified block. **/
  HEIGHT, 
  
  /** 0.0 iff nothing is touching specified block. **/
  TOUCH, 
  
  /** Angle of specified DOF of specified block. **/
  JOINT,  
  
  /** Floating point constant. **/
  CONSTANT, 
  
  /** Simulation seconds since start. **/
  TIME, 
}
