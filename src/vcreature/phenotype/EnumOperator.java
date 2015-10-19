package vcreature.phenotype;

/**
 * The Enum EnumOperator.
 */
public enum EnumOperator
{
  /**
   *
   */
  ADD      { public boolean isUnary() {return false;}},
  /**
   *
   */
  SUBTRACT { public boolean isUnary() {return false;}},
  /**
   *
   */
  MULTIPLY { public boolean isUnary() {return false;}},
  /**
   *
   */
  POWER    { public boolean isUnary() {return false;}},
  /**
   *
   */
  MAX      { public boolean isUnary() {return false;}},
  /**
   *
   */
  MIN      { public boolean isUnary() {return false;}},
  /**
   *
   */
  ARCTAN2  { public boolean isUnary() {return false;}},
  
  /** Unary operator. Returns absolute value of input. **/
  ABS      { public boolean isUnary() {return true;}}, 
  
  /** Unary operator. Returns input unchanged. **/
  IDENTITY { public boolean isUnary() {return true;}}, 
  
  /** Unary operator. Returns the trigonometric sine of the input. **/
  SIN      { public boolean isUnary() {return true;}},  
  
  /** Unary operator. Returns 0.0 if input is 0.0, 1.0 if input >0.0, -1.0 if input <0.0 **/
  SIGN     { public boolean isUnary() {return true;}},  
  
  /** Unary operator. Returns negative of input. **/
  NEGATIVE { public boolean isUnary() {return true;}}, 
  
  /** Unary operator. Returns natural logarithm of input. Returns 0 if input is <= 0 **/
  LOG      { public boolean isUnary() {return true;}},  
  
  /** Unary operator. Returns natural exponential of input */
  EXP      { public boolean isUnary() {return true;}}; 

  /**
   *
   * @return
   */
  public abstract boolean isUnary();
  
  /**
   *
   */
  public static final int SIZE = values().length;
}
