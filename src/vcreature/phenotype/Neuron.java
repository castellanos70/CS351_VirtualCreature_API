package vcreature.phenotype;

/**
 * 
 * @author Joel
 * The Neuron class is used to give the creature the intellegance to send an impulse 
 * of a particular value to a particular joint under a particular condition.<br<br>
 * 
 * <img src="./doc-files/Neuron.png">
 * Each effector (joint) has a sequence of 0 to n Neurons.<br>
 * Each Neuron can take up to 5 inputs. Any inputs that are left undefined default to
 * type=CONSTANT, value=0.
 * 
 * The default binary operator is +.
 * The default unary oporator is Identity.
 * 
 * Each rule takes five inputs (a, b, c, d, and e). 
 * Input can come from any one of five types of sources: <br>
 * 1)  The height of the center of gravity of the ith block in the creature's block array.<br>
 * 2)  The touch value of block i.<br>
 * 3)  The angle of the joint between block i and its parent.<br>
 * 4)  A constant float value.<br>
 * 5)  A global time equal to the number of simulation seconds since start.<br><br>
 * Each rule has four operators: two binary operators, and two unary operators. 
 * If the output of the first neuron, y, is greater 
 * than input c, then the second neuron in the rule is evaluated. 
 * The output of the second neuron (with inputs d and e) is the force
 * sent to the attached effector.<br><br>
 * 
 * Each sequence of Neurons form what is called a rule table that is evaluated 
 * from index 0 through n. The first Neuron in a rule table for which the output to the 
 * threshold trigger (if (y>c)) is true, will have its second half (with inputs d and e) evaluated 
 * and the second half output sent to the effector as an impulse to the joint.
 * Evaluation of the rule table stops when the first neuron threshold is triggered or when
 * all neurons in the table have failed to fire.<br>
 * When a neuron is activated, an output is sent to its attached effector as an inpulse (Newton seconds).<br>
 *
*/

public class Neuron
{

  /**
   * Constant used to specify input A.
   */
  public static final int A=0;
  /**
   * Constant used to specify input B.
   */
  public static final int B=1;
  /**
   * Constant used to specify input C.
   */
  public static final int C=2;
  /**
   * Constant used to specify input D.
   */
  public static final int D=3;
  /**
   * Constant used to specify input E.
   */
  public static final int E=4;
  
  /**
   *
   */
  public static final int FIRST_HALF = 0;
  /**
   *
   */
  public static final int SECOND_HALF = 2;

  /**
   * Total number of neuron inputs (a, b, c, d, and e).
   */
  public static final int TOTAL_INPUTS    = 5;
  
  /**
   * Total number of operations in each neuron. Each neuron has two binary and 
   * two unary operations.
   */
  public static final int TOTAL_OPERATIONS = 4;
  
  
  
  private EnumNeuronInput[] type = new EnumNeuronInput[TOTAL_INPUTS]; 
  private float[] constantValue = new float[TOTAL_INPUTS];  //used only for CONSTANT type.
  private int[]   blockIndex    = new   int[TOTAL_INPUTS];  //used for HEIGHT, TOUCH and JOINT types.
  
  private EnumOperator[] operator =  new EnumOperator[TOTAL_OPERATIONS];
  
  
  
  /**
   *Each Neuron can have up to 5 inputs (a, b, c, d and e). Each of these inputs
   * can be specified in the constructor.
   * Any inputs specified as null will be given the default input
   * (EnumNeuronInput.CONSTANT with value =0)
   * This constructor also sets default operations of the neuron to both 
   * binary operations being EnumOperator.ADD and both unary operations to 
   * EnumOperator.IDENTITY.
   * @param a 
   * @param b
   * @param c
   * @param d
   * @param e
   */
  public Neuron(EnumNeuronInput a, EnumNeuronInput b, EnumNeuronInput c,
                EnumNeuronInput d, EnumNeuronInput e) 
  {
    if (a!=null) type[A] = a; else type[A] = EnumNeuronInput.CONSTANT;
    if (b!=null) type[B] = b; else type[B] = EnumNeuronInput.CONSTANT;
    if (c!=null) type[C] = c; else type[C] = EnumNeuronInput.CONSTANT;
    if (d!=null) type[D] = d; else type[D] = EnumNeuronInput.CONSTANT;
    if (e!=null) type[E] = e; else type[E] = EnumNeuronInput.CONSTANT;
    
    operator[0] = EnumOperator.ADD;
    operator[2] = EnumOperator.ADD;
    
    operator[1] = EnumOperator.IDENTITY;
    operator[3] = EnumOperator.IDENTITY;
  }
  
  /**
   *
   * @param i
   * @return
   */
  public EnumOperator getOp(int i) {return operator[i];}
  /**
   *
   * @param i
   * @return
   */
  public EnumNeuronInput getInputType(int i) {return type[i];}
  /**
   *
   * @param i
   * @return
   */
  public float getInputValue(int i) {return constantValue[i];}
  /**
   *
   * @param i
   * @return
   */
  public int   getBlockIdx(int i) {return blockIndex[i];}
  
  
  /**
   *
   * @param operatortype
   * @param i
   */
  public void setOp(EnumOperator operatortype, int i) 
  { 
    if (i == 0 || i ==2)
    { if (operatortype.isUnary()) 
      { throw new IllegalArgumentException("Operator idx="+i+" must be binary");
      }
    }
    else 
    { if (!operatortype.isUnary()) 
      { throw new IllegalArgumentException("Operator idx="+i+" must be unary");
      }
    }
    operator[i]=operatortype;
  }
  /**
   * Sets the type of the specified input, i.
   * @param i Must be (A, B, C, D or E)
   * @param inputType
   */
  public void setInputType (int i, EnumNeuronInput inputType) {type[i]=inputType;}
  /**
   * Sets the value of the specified input, i. Note: the value field is only used for 
   * EnumNeuronInput.CONSTANT 
   * @param i Must be (A, B, C, D or E)
   * @param value 
   */
  public void setInputValue(int i, float value) 
  {
    if (i<A || i>E) throw new IllegalArgumentException("Neuron Input index must be 0 through 4");
    constantValue[i]=value;
  }
  /**
   * Sets the block index of the specified input, i. Note block index is only used for
   * /EnumNeuronInput.HEIGHT,  EnumNeuronInput.TOUCH, and EnumNeuronInput.JOINT.  
   * @param i Must be (A, B, C, D or E)
   * @param blockId
   */
  public void setBlockIdx  (int i, int blockId)   
  {
    if (i<A || i>E) throw new IllegalArgumentException("Neuron Input index must be 0 through 4");
    blockIndex[i]=blockId;
  }
  

  
  /**
   * This method us used to get the output of either the first half or second half of 
   * the neuron. First the binary operator is applied to the two inputs. Then, the
   * operator result is given as input the the unary operator.
   * 
   * @param a First input value to specified half of this neuron.
   * @param b Second input value to the specified half of this neuron.
   * @param half Must be either FIRST_HALF  |  SECOND_HALF
   * @return The ouput value of the specified half of this neuron.
   */
  public float getOutput(float a, float b, int half)
  {
    
    if (half != FIRST_HALF && half != SECOND_HALF) 
    {  throw new IllegalArgumentException("input half must be either FIRST_HALF | SECOND_HALF");
    }
    
    //Apply binary operator opIdx
    float x = 0;
    if (operator[half] == EnumOperator.ADD)           x = a+b;
    else if (operator[half] == EnumOperator.SUBTRACT) x = a-b;
    else if (operator[half] == EnumOperator.MULTIPLY) x = a*b;
    else if (operator[half] == EnumOperator.POWER)    x = (float)Math.pow(a,b);
    else if (operator[half] == EnumOperator.MIN)      x = Math.min(a,b);
    else if (operator[half] == EnumOperator.MAX)      x = Math.max(a,b);
    else if (operator[half] == EnumOperator.ARCTAN2)  x = (float)Math.atan2(a,b);
    
    //Apply unary operator opIdx+1
    if (operator[half+1] == EnumOperator.ABS)           return Math.abs(x);
    else if (operator[half+1] == EnumOperator.IDENTITY) return x;
    else if (operator[half+1] == EnumOperator.SIN)      return (float)Math.sin(x);
    else if (operator[half+1] == EnumOperator.SIGN)     return Math.signum(x);
    else if (operator[half+1] == EnumOperator.NEGATIVE) return -x;
    else if (operator[half+1] == EnumOperator.LOG)        
    { if (x<0) return 0.0f;
      return (float)Math.log(x);
    }
    else if (operator[half+1] == EnumOperator.EXP)      return (float)Math.exp(x);
    return 0f;
  }
  

  
  public String toString()
  {
    String out = "Neuron: if {" + type[0] + " " + operator[0] + " " + type[1] + 
        " --> " + operator[1] + "} > {" + type[2] + 
        " then {" + type[3] + ", " + operator[3] + " " + type[4] + " --> " + operator[4];
    return out;
  }
 
}
