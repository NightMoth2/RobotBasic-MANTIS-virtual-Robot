/*
NEURAL NETWORK FOR ARDUINO

Two classes: NN_Layer and NN (NN has array of NN_Layers as its member)
Features:
 - utilises backpropagation algorythm for learning
 - Six activation functions for hiddean layers and output layer can be set independently (Sigmoid, Identity, ReLU, Softplus, Leaky ReLU, ELU)
 - MSE function as error function
 - It is possible to estimate RAM usage before allocation (using estimateTotalLayersRAM())
 - it is possible to programm recurrent neural network behavior using copy_node_activation() method

Version 3.7 - 2026-09-18
- Gradient clipping option added
- Weights initialisation coeficient added (now it is [-1*NN_WEIGHTS_INIT_COEF, 1*NN_WEIGHTS_INIT_COEF])
- Bias weights initialisation range fixed (now it is the same as node weights: [-1*NN_WEIGHTS_INIT_COEF, 1*NN_WEIGHTS_INIT_COEF]) 


Version 3.6 - 2026-08-18
- Fixed math error in derivative of sigmoid function

Version 3.5 - 2026-08-14
 - Fixed error in delta calculations for hidden and output layers (now df(z) instead of df(a) used)
 - Fixed error in delta calculations for output now (now dJ(o,y) instead of J(o,y) used)
 - calcNeuroneNetInput() added

Version 3.4 - 2026-08-11
 - Normalisation of weighted sums for hidden layers added (function propagate_layer() updated, directive #define NN_HL_NORMALISATION added)

Version 3.3 - 2026-06-11
 - added function domains and corresponding getters

Version 3.2 - 2026-04-27
 - added list_train_static_data() to train on static 2D arrays (not pointers)

Version 3.1 - 2026-04-24
 - renaming some members and methods

Version 3.0 - 2026-04-13
 - Back to NEW/DELETE memory allocation methods (revised)
 - Object creation and initialisation revised
 - void setNumOfLayers() added
 - copy_node_activation() added

Version 2.1 - 2026-04-11
 - allocation of memory for NN_Layer objects using malloc: 
 - pointer arythmetic to address NN_Layer objects instead of array indexing
 - estimating RAM need for eack NN_Layer object using estimateTotalLayersRAM()
 - another methods for NN initialisation: setNumOfNodes() and initEmptyNN()

Version 2.0 - 2026-04-03
 - implemented as class
 - added inverse of activation functions
 - added methods to call current activation functions (for input/hidden and for output)
 - added general map function for double type
 - weights is zero centerd upun initialisation

Version 1.1 - 2026-03-29
 - Added Get and Set functions for weights
 - Added Set function to Out nodes
 - Added get and set function for each individual element (for each weight, each node etc)
 - Trying to use smaller datatypes (byte instead of int where possible)

Version 1.0 - 2026-03-23
 - Basic

Refs:
--
NN
--
[1] R. Yehoshua, “Backpropagation: Step-By-Step Derivation,” Towards Data Science. 
    Accessed: Mar. 24, 2026. [Online]. Available: https://towardsdatascience.com/backpropagation-step-by-step-derivation-99ac8fbdcc28/
[2] LDS team, “Backpropagation: The Engine of Deep Learning,” Let’s Data Science. 
    Accessed: Mar. 24, 2026. [Online]. Available: https://letsdatascience.com/blog/backpropagation-engine-of-deep-learning
[3] M. Nielsen, “How the backpropagation algorithm works (Chapter 2),” Neuralnetworksanddeeplearning. Accessed: Mar. 24, 2026. [Online]. 
    Available: http://neuralnetworksanddeeplearning.com/chap2.html
[7] D. E. Rumelhart, G. E. Hinton, and R. J. Williams, “Learning representations by back-propagating errors,” Nature, vol. 323, no. 6088, pp. 533–536, Oct. 1986, doi: 10.1038/323533a0.
    Accesed: April 03, 2026. [Online]. Available: https://web.stanford.edu/~jlmcc/papers/PDP/Volume%201/Chap8_PDP86.pdf
[4] GeeksForGeeks, “Activation functions in Neural Networks,” GeeksForGeeks. Accessed: Mar. 24, 2026. [Online]. 
    Available: https://www.geeksforgeeks.org/machine-learning/activation-functions-neural-networks/
[5] Ł. Gebel, “Why We Need Bias in Neural Networks,” Towards Data Science. Accessed: Mar. 24, 2026. [Online]. 
    Available: https://towardsdatascience.com/why-we-need-bias-in-neural-networks-db8f7e07cb98/   
---    
RNN
---
[6] J. L. Elman, “Finding Structure in Time,” Cognitive Science, vol. 14, no. 2, pp. 179–211, Mar. 1990, doi: 10.1207/s15516709cog1402_1.
    Accesed: April 03, 2026. [Online]. Available: https://onlinelibrary.wiley.com/doi/10.1207/s15516709cog1402_1
-------------------------     
Implementation on Arduino 
-------------------------
[7] Baxter, “Running a Neural Net on an Arduino UNO,” Baxter Builds. Accessed: Mar. 24, 2026. [Online]. 
    Available: https://baxterbuilds.com/running-a-neural-net-on-an-arduino-uno/  
-----------------------    
Memory usage estimation
-----------------------
[8] G. Shetty, “Determining the Size of a Class Object,” Cprogramming. Accessed: Apr. 20, 2026. [Online]. 
    Available: https://www.cprogramming.com/tutorial/size_of_class_object.html
----------    
C++ Coding   
----------
[9] A. Alam, “How to Pass 2D Array to a Function in C++,” Intellipaat Software Solutions. Accessed: Mar. 24, 2026. [Online]. 
    Available: https://intellipaat.com/blog/pass-2d-array-parameter-to-functions-in-c-plus-plus/
[10] J. Hu, “How to Pass 2D Array to a Function in C++,” DelftStack. Accessed: Mar. 24, 2026. [Online]. 
    Available: https://www.delftstack.com/howto/cpp/how-to-pass-2d-array-to-function-in-cpp/    
[11] GeeksForGeeks, “How to Create a Dynamic 2D Array Inside a Class in C++?,” GeeksForGeeks. Accessed: Apr. 20, 2026. 
    [Online]. Available: https://www.geeksforgeeks.org/cpp/create-dynamic-2d-array-inside-class-c/    
[12] GeeksForGeeks, “How to Deallocate a 2D Array in C++?,” GeeksForGeeks. Accessed: Apr. 20, 2026. [Online]. 
    Available: https://www.geeksforgeeks.org/cpp/how-to-deallocate-a-2d-array-in-cpp/
[13] GeeksForGeeks, “Destructors in C++,” GeeksForGeeks. Accessed: Apr. 20, 2026. [Online]. 
    Available: https://www.geeksforgeeks.org/cpp/destructors-c/
[14] GeeksForGeeks, “How to initialize Array of objects with parameterized constructors in C++,” GeeksForGeeks. 
    Accessed: Apr. 20, 2026. [Online]. Available: https://www.geeksforgeeks.org/cpp/how-to-initialize-array-of-objects-with-parameterized-constructors-in-c/

 */

#ifndef MANTIS_NN
#define MANTIS_NN

// ================================================================================================
// Include
#include <Arduino.h>

// ================================================================================================
// Settings section

// DISPLAY DATA
//#define NN_DEBUG_MODE   // All debug messages to be printed via Serial
#define NN_REPORT_MODE    // NN key parameters, weights, training process can be printed via serial

// HIDDEN LAYER NORMALISATION
//#define NN_HL_NORMALISATION

// GRADIENT CLIPPING
#define NN_USE_GRADIENT_CLIPPING
#define NN_GRADIENT_CLIPPING_COEF 0.9

// WEIGHTS INITIALIZATION
#define NN_WEIGHTS_INIT_COEF 1.0 // weights initialised randomly -1*NN_WEIGHTS_INIT_COEF...+1*NN_WEIGHTS_INIT_COEF

// Gradient update optimisation (NOT IMPLEMENTED)
#define NN_GRADIENT_UPD_USE_MOMENTUM  // (NOT IMPLEMENTED)
#define NN_MOMENTUM_COEF 0.1          // (NOT IMPLEMENTED)

// ACTIVATION FUNCTIONS FOR HIDDEN LAYER(S) [CHOOSE ONE]
#define NN_HL_AFUN_SIGMOID        // Sigmoid activation function
//#define NN_HL_AFUN_TANH         // TANH activation function
//#define NN_HL_AFUN_ID           // Identity activation function
//#define NN_HL_AFUN_RELU         // ReLU activation function
//#define NN_HL_AFUN_SOFTPLUS     // Softplus activation function
//#define NN_HL_AFUN_LEAKYRELU    // Leaky ReLU activation function
//#define NN_HL_AFUN_ELU          // ELU activation function

// ACTIVATION FUNCTIONS FOR OUTPUT LAYER  [CHOOSE ONE]
#define NN_OL_AFUN_SIGMOID        // Sigmoid activation function
//#define NN_OL_AFUN_TANH         // TANH activation function
//#define NN_OL_AFUN_ID           // Identity activation function
//#define NN_OL_AFUN_RELU         // ReLU activation function
//#define NN_OL_AFUN_SOFTPLUS     // Softplus activation function
//#define NN_OL_AFUN_LEAKYRELU    // Leaky ReLU activation function
//#define NN_OL_AFUN_ELU          // ELU activation function

// DOMAIN OF SELECTED ACTIVATION FUNCTIONS (input range)
#define NN_HL_DOM_MIN -2.5   // lower bound of NN's input/hidden layers activation function domain
#define NN_HL_DOM_MAX 2.5    // upper bound of NN's input/hidden layers activation function domain
#define NN_OL_DOM_MIN -2.5   // lower bound of NN's output layer activation function domain
#define NN_OL_DOM_MAX 2.5    // upper bound of NN's output layer activation function domain

// LOSS FUNCTIONS [CHOOSE ONE]
#define NN_LOSSFUN_MSE            // Mean Square Error loss function

// OTHER PARAMS
#define NN_BIAS_INIT_VAL 1.0         // Initial value for bias nodes
#define NN_TRAINING_SPEED 0.3       // Training speed   
#define NN_LEAKY_RELU_COEFF 0.01     // Coeff used in Leaky ReLU and ELU activation functions

// TRAINING
// (Used for list_train_dynamic_data() and list_train_static_data
#define NN_EPOCHS_CNT  200          // Number of loops throught data set to finish training cycle
#define NN_TRAINING_CYCLES_CNT 1    // Number of times to repeat tearning for each row
#define NN_MAX_LOSS 0.0025          // It is OK to stop learning when network error is below this value
#define NN_CHECK_AVG_D              // Directive to check average Delta value of whole network
#define NN_MIN_AVG_D 0.01           // It is OK to stop learning when network average Delta is below this value 

// RAM USAGE ESTIMATION (OBSOLETE, NOT USED)
#define LAYER_EXTRA_RAM 25        // extra RAM for safety, experementally tested value (Arduino Nano). For 10-node layer with
                                  // 10 input weights for each node, difference between actual and estimated value was 17 bytes (593 vs 610). 
                                  // Or, for example for 6-node layer with 10 input weights for each node
                                  // difference was 9 bytes (361 vs 370 bytes)
// ================================================================================================

// Layer
class NN_Layer
{
   
  private:

    bool is_input_layer;  // is it input layer? (input layer doesn't have input weights and biases)
    byte n_cnt;           // number of nodes
    byte w_cnt;           // number of inputs (input weights) for each node

    double* A;            // array of calculated activation values for each node
    double* B;            // array of bias values for each node
    double* D;            // array of calculated delta therm for each node
    double* Wb;           // array of weights of biases for each node
    double** W;           // 2D array of input weights values for each node (rows - nodes, columns - weights)

  public:
  
    // Constructors/Destructor 
    NN_Layer();                                       // Dummy constructor
    NN_Layer(bool is_input, byte n_cnt, byte w_cnt);  // Basic constructor
    
    // Init/Setup
    void init(bool is_input, byte n_cnt, byte w_cnt); // same as Basic constructor

    // Getters / Settters
    byte getCntN();                       // Get number of nodes
    byte getCntW();                       // Get number of input weights
    double getA(byte n);                  // Get n-th node activation value
    double getB(byte n);                  // Get n-th bias node
    double getD(byte n);                  // Get n-th node delta
    double getWb(byte n);                 // Get n-th node bias weight
    double getW(byte n, byte w);          // Get value of w-th input weight for n-th node
    void setA(double x, byte n);          // Set n-th node activation value = x
    void setB(double x, byte n);          // Set n-th bias node value = x
    void setD(double x, byte n);          // Set n-th node delta = x
    void setWb(double x, byte n);         // Set n-th node bias weight = x
    void setW(double x, byte n, byte w);  // Set n-th node w-th input weight = x

    // Destructor
    ~NN_Layer(); 
};

// Neural network
class NN
{
  private:

    byte l_cnt;           // number of layers in NN
    byte* LNodesCnt;      // array to store number of nodes in each layer  
    NN_Layer* L;          // array of NN_Layer objects (array of Layers)

    #ifdef NN_GRADIENT_UPD_USE_MOMENTUM

    #endif
        
  public:

    // Constructors/Destructor
    NN();                             // Default constructor
    NN(byte l_cnt);                   // Minimal Constructor  (Not used)
    NN(byte l_cnt, byte N_cnt[]);     // Basic constructor  (Not used)
    NN(const NN &obj);                // Copy constructor (NOT IMPLEMENTED)
    NN(const NN &&obj);               // Move constructor (NOT IMPLEMENTED)
    ~NN();                            // Destructor

    // Init/Setup
    void init(byte l_cnt);                        // same as Minimal Constructor
    void setNumOfLayers(byte l_cnt);              // set number of layers in the NN
    void setNumOfNodes(byte l_idx, byte n_cnt);   // set number of nodes for particukar layer (n_cnt is the number of nodes in layer l_idx)
    void initEmptyNN();                           // allocate and populate NN layers          
    void randomizeWeights(byte seed);             // set all input weights = some random vqlue between -1 and 1

    // Getters / Settters
    byte getCntL();                                   // Get number of layers
    byte getCntN(byte l);                             // Get number of nodes in l-th layer 
    byte getCntW(byte l);                             // Get number of input weights for nodes in l-th layer
    double getA(byte l, byte n);                      // Get l-th layer n-th node activation value                     
    double getB(byte l, byte n);                      // Get l-th layer n-th bias node value  
    double getD(byte l, byte n);                      // Get l-th layer n-th node delta value 
    double getWb(byte l, byte n);                     // Get l-th layer n-th node bias weight value 
    double getW(byte l, byte n, byte w);              // Get l-th layer n-th node w-th weight value
    double getOut(byte n);                            // Get n-th node of output layer activation
    void setA(double x, byte l, byte n);              // Set l-th layer n-th node activation value = x
    void setB(double x, byte l, byte n);              // Set l-th layer n-th bias node value = x
    void setD(double x, byte l, byte n);              // Set l-th layer n-th node delta value = x
    void setWb(double x, byte l, byte n);             // Set l-th layer n-th bias value = x
    void setW(double x, byte l, byte n, byte w);      // Set l-th layer n-th node w-th weight value = x

    double getHLDomMin();                             // Get min bound of Hidden Layer activation function domain
    double getHLDomMax();                             // Get max bound of Hidden Layer activation function domain
    double getOLDomMin();                             // Get min bound of Output Layer activation function domain
    double getOLDomMax();                             // Get max bound of Output Layer activation function domain
   
    // Math
    double fun_normRnd();                                                             // rand value between 0.000 and 1.000
    double fun_map(double x, double x_min, double x_max, double y_min, double y_max); // same as arduino map() but for doubles

    #if defined(NN_LOSSFUN_MSE)
      double fun_MSE(double o, double y);
      double fun_dMSE(double o, double y);
    #endif

    #if defined(NN_HL_AFUN_SIGMOID) || defined(NN_OL_AFUN_SIGMOID) || defined(NN_HL_AFUN_SOFTPLUS) || defined(NN_OL_AFUN_SOFTPLUS)
      double fun_Sigmoid(double x);
      double fun_invSigmoid(double y);
      double fun_dSigmoid(double x);
    #endif

    #if defined(NN_HL_AFUN_TANH) || defined(NN_OL_AFUN_TANH) 
      double fun_Tanh(double x);
      double fun_invTanh(double y);
      double fun_dTanh(double x);
    #endif

    #if defined(NN_HL_AFUN_ID) || defined(NN_OL_AFUN_ID) 
      double fun_Identity(double x);
      double fun_invIdentity(double y);
      double fun_dIdentity(double x);
    #endif

    #if defined(NN_HL_AFUN_RELU) || defined(NN_OL_AFUN_RELU)
      double fun_ReLU(double x);
      double fun_invReLU(double y);
      double fun_dReLU(double x);
    #endif

    #if defined(NN_HL_AFUN_SOFTPLUS) || defined(NN_OL_AFUN_SOFTPLUS)
      double fun_Softplus(double x);
      double fun_invSoftplus(double y);
      double fun_dSoftplus(double x);
    #endif

    #if defined(NN_HL_AFUN_LEAKYRELU) || defined(NN_OL_AFUN_LEAKYRELU) 
      double fun_LeakyReLU(double x);
      double fun_invLeakyReLU(double y);
      double fun_dLeakyReLU(double x);
    #endif

    #if defined(NN_HL_AFUN_ELU) || defined(NN_OL_AFUN_ELU)
      double fun_ELU(double x);
      double fun_invELU(double y);
      double fun_dELU(double x);
    #endif
    
    double fun_act_H(double x);           // "#defined" activation function for hidden layer
    double fun_act_O(double x);           // "#defined" activation function for output layer
    double fun_inv_H(double x);           // inv of "#defined" activation function for hidden layer
    double fun_inv_O(double x);           // inv of "#defined" activation function for output layer
    double fun_dAct_H(double x);          // derivative of "#defined" activation function for hidden layer
    double fun_dAct_O(double x);          // derivative of "#defined" activation function for output layer
    double fun_Loss(double o, double y);  // "#defined" loss function
    double fun_dLoss(double o, double y);  // derivative of "#defined" loss function

    // Scaling input/output
    double scale_to_hidden_domain(double in_val, double in_range_min, double in_range_max);      // Scaling val from [in_range_min, in_range_max] into domain of INPUT/HIDDEN layers activation function
    double scale_to_output_domain(double tgt_val, double tgt_range_min, double tgt_range_max);  // Scaling val from [tgt_range_min, tgt_range_max] into domain of OUTPUT layers activation function
    double scale_to_hidden_domain_and_calc_activation(double in_val, double in_range_min, double in_range_max);  // Scaling + applying INPUT/HIDDEN layers activation function
    double scale_to_output_domain_and_calc_activation(double tgt_val, double tgt_range_min, double tgt_range_max);  // Scaling + applying OUTPUT layers activation function
    double descale_output(byte out_idx, double tgt_range_min, double tgt_range_max);  // Applying inverse of INPUT/HIDDEN layers activation function and re-scaling back to [in_range_min, in_range_max]
    double calc_output_inv_activation_and_descale(byte out_idx, double tgt_range_min, double tgt_range_max); // Applying inverse of OUTPUT layers activation function and re-scaling back to [tgt_range_min, tgt_range_max]

  

    // Core functions
    double calcNeuroneNetInput(byte l_idx, byte n_idx);
    void propagate_layer(byte idx);                                   // aux function for propagate_network()
    void propagate_network(double* X); 
    void calc_output_layer_deltas(double* Y);                         // aux function for backpropagate_network()
    void calc_hidden_layer_deltas(byte l_idx);                        // aux function for backpropagate_network()
    void update_layer_weights(byte l_idx);                            // aux function for backpropagate_network()
    void backpropagate_network(double* Y);
    double single_train(double* X, double* Y);                        // one data row training
    double list_train_dynamic_data(double** XList, double** YList, byte r_cnt);    // list training, inputs are double pointer arrays

    // implementation will be here, in .h file
    template<size_t xrows, size_t xcols, size_t yrows, size_t ycols>
    double list_train_static_data(double (&XList)[xrows][xcols], double (&YList)[yrows][ycols], byte r_cnt);    // list training, inputs are static arrays 

    // Aux
    void copy_node_activation(byte from_l, byte from_n, byte to_l, byte to_n, double coef);   // for recurent network implementation
    
    
    void debugWaitSerialOutAvailable();     // wait until output buffer available

    // Display
    void disp_layer(byte l_idx);
    void disp_NN();
    void disp_Outputs();

    // Memory usage estimation
    int estimateHiddenLayerRAM(int n_cnt, int w_cnt); // aux function for estimateTotalLayersRAM()
    int estimateInputLayerRAM(int n_cnt);             // aux function for estimateTotalLayersRAM()
    int estimateTotalLayersRAM();
     
};

template<size_t xrows, size_t xcols, size_t yrows, size_t ycols>
double NN::list_train_static_data(double (&XList)[xrows][xcols], double (&YList)[yrows][ycols], byte r_cnt)
{
    
   // This function performs train of the network on the full training set
   // Arguments:

  #ifdef NN_REPORT_MODE
      Serial.println(F("NN::list_train_static start"));
  #endif
  
  double loss = 0;  // network loss value

  // loop through training dataset to finish one training
  for (int j = 0; j < NN_EPOCHS_CNT; j++) {
    #ifdef NN_REPORT_MODE
      Serial.print(F("."));
    #endif

    loss = 0; // clear loss value

    // for each row of training dataset
    for (byte r = 0; r < r_cnt; r++) {
      // For each training cycle
      for (int i = 0; i < NN_TRAINING_CYCLES_CNT; i++) {
        // perform training and accumulate loss
        loss += NN::single_train(&XList[r][0], &YList[r][0]);
      }
    }

    // average loss
    loss /= (double)r_cnt;

    #ifdef NN_REPORT_MODE
      Serial.print(F("-Loss:"));
      Serial.println(loss, DEC);
      Serial.flush();
    #endif   

    // break loop if error is already in desired range
    if (loss <= NN_MAX_LOSS) break;    
  }

  #ifdef NN_REPORT_MODE
    Serial.println(F("NN::list_train_static end"));
  #endif
  
  return loss;
}

#endif
