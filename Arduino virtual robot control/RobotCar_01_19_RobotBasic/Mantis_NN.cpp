#include <Arduino.h>
#include "Mantis_NN.h"

// ==========================================================================================
// NN_Layer
// ==========================================================================================


// ------------------------------------------------------------------------------------------
// Dummy Constructor
NN_Layer::NN_Layer(){

  // Dummy constructor.
  // Use it to create dynamic array of NN_Layer objects.
  // Objects in a dynamic array will be initialised (allocated) later using init() method
  // (by some reason it is impossible to use other constructors 
  // to create dynamic array of NN_Layer objects)
   
  #ifdef NN_DEBUG_MODE
    Serial.println(F("NNL:Def cnst bgn"));
    Serial.flush();
  #endif

  is_input_layer = false;   
  n_cnt = 0;                
  w_cnt = 0;                
 
  A = NULL;                 
  B = NULL;                 
  D = NULL;                 
  Wb = NULL;                
  W = NULL;                

  #ifdef NN_DEBUG_MODE
    Serial.println(F("NNL:Def cnst end"));
    Serial.flush();
  #endif
}


// ------------------------------------------------------------------------------------------
// Constructor
NN_Layer::NN_Layer(bool is_input, byte n_cnt, byte w_cnt){
  
  #ifdef NN_DEBUG_MODE
    Serial.println(F("NNL:Cnst bgn"));
    Serial.flush();
  #endif

  NN_Layer::init(is_input, n_cnt, w_cnt);

  #ifdef NN_DEBUG_MODE
    Serial.println(F("NNL:Cnst end"));
    Serial.flush();
  #endif
}

// ------------------------------------------------------------------------------------------
// Destructor
NN_Layer::~NN_Layer() {
  
  #ifdef NN_DEBUG_MODE
    Serial.println(F("NNL:Dst bgn"));
    Serial.flush();
  #endif

  if(is_input_layer) {
    
    // Delete 1D arrays
    delete[] A;

    // Delete pointers to not allocated arrays
    delete B;
    delete D;
    delete Wb;
    delete W;    
  }
  else{
    
    // Deleta 1D arrays
    delete[] A;
    delete[] B;
    delete[] D;
    delete[] Wb;

    // Delete 2D array
    for(byte i = 0; i < n_cnt; i++){
      
      delete[] W[i]; 
    }
    delete[] W;   
  }
  
  #ifdef NN_DEBUG_MODE
    Serial.println(F("NNL:Dst end"));
    Serial.flush();
  #endif
  
}

// ------------------------------------------------------------------------------------------
// Initialisation
void NN_Layer::init(bool is_input, byte n_cnt, byte w_cnt){
  
  #ifdef NN_DEBUG_MODE
    Serial.println(F("NNL:init bgn"));
    Serial.flush();
  #endif

  this->is_input_layer = is_input;
  this->n_cnt = n_cnt;
  this->w_cnt = w_cnt;

  // 1D arrays
  if(is_input){
    #ifdef NN_DEBUG_MODE
      Serial.println(F("-is Inp L-r"));
      Serial.flush();
    #endif
      
    A = new double[n_cnt];
    
    // set all values to zero
    for(byte n = 0; n < n_cnt; n++){
      A[n] = 0;
    }

    #ifdef NN_DEBUG_MODE
      Serial.println(F("-Inp L-r ok"));
      Serial.flush();
    #endif
    
  }
  else{
    #ifdef NN_DEBUG_MODE
      Serial.println(F("-gen L-r"));
      Serial.flush();
    #endif

    // 1D arrays
    A = new double[n_cnt];
    B = new double[n_cnt];
    D = new double[n_cnt];
    Wb = new double[n_cnt];

    #ifdef NN_DEBUG_MODE
      Serial.println(F("-1D arr OK"));
      Serial.flush();
    #endif
      
    // 2D array
    W = new double*[n_cnt];
    for(byte i = 0; i < n_cnt; i++){
      W[i] = new double[w_cnt];
    }  
    
    // set all values to zero
    for(byte n = 0; n < n_cnt; n++){
      A[n] = 0;
      B[n] = 0;
      D[n] = 0;
      Wb[n] = 0;

      for(byte w = 0; w < w_cnt; w++){
        W[n][w] = 0;
      }
    }
    
    #ifdef NN_DEBUG_MODE
      Serial.println(F("-2D arr OK"));
      Serial.println(F("-gen L-r done"));
      Serial.flush();
    #endif
  }

  #ifdef NN_DEBUG_MODE
    Serial.flush();
    Serial.println(F("NNL:Init end"));
    Serial.flush();
  #endif 
}


// ------------------------------------------------------------------------------------------
// Getters / Setters
byte NN_Layer::getCntN(){
  return this->n_cnt;
}

byte NN_Layer::getCntW(){
  return this->w_cnt;
}

double NN_Layer::getA(byte n){
  return A[n];
}

double NN_Layer::getB(byte n){
  return B[n];
}

double NN_Layer::getD(byte n){
  return D[n];  
}
    
double NN_Layer::getWb(byte n){
  return Wb[n]; 
}

double NN_Layer::getW(byte n, byte w){
  return W[n][w]; 
}

void NN_Layer::setA(double x, byte n){
  A[n] = x;
}

void NN_Layer::setB(double x, byte n){
  B[n] = x;
}

void NN_Layer::setD(double x, byte n){
  D[n] = x;
}

void NN_Layer::setWb(double x, byte n){
  Wb[n] = x;
}

void NN_Layer::setW(double x, byte n, byte w){
  W[n][w] = x;
}


// ==========================================================================================
// NN
// ==========================================================================================

// Default/ Dummy Constructor
NN::NN(){
  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::Def cnst bgn"));
    Serial.flush();
  #endif
  
  this->l_cnt = 0;
  LNodesCnt = new byte[l_cnt];
  L = NULL;

  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::Def cnst end"));
    Serial.flush();
  #endif  
}

// ------------------------------------------------------------------------------------------
// Minimalistic constructor
NN::NN(byte l_cnt){
  
  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::Min cnst bgn"));
    Serial.print(F("-l_cnt="));
    Serial.println(this->l_cnt);
    Serial.flush();
  #endif
  
  NN::init(l_cnt);

  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::Min cnst end"));
    Serial.flush();
  #endif
}

// ------------------------------------------------------------------------------------------
// Basic Constructor
NN::NN(byte l_cnt, byte N_cnt[]){
  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::Basic cnst bgn"));
    Serial.flush();
  #endif

  this->l_cnt = l_cnt;

  #ifdef NN_DEBUG_MODE
    Serial.print(F("-l_cnt="));
    Serial.println(this->l_cnt);
    Serial.flush();
  #endif

  LNodesCnt = new byte[l_cnt];

  for (byte l_idx = 0; l_idx < l_cnt; l_idx++){
    LNodesCnt[l_idx] = N_cnt[l_idx];
    #ifdef NN_DEBUG_MODE
      Serial.print(F("L"));
      Serial.print(l_idx);
      Serial.print(F(": n_cnt="));
      Serial.println(LNodesCnt[l_idx]);
      Serial.flush();
    #endif
  }

  L = new NN_Layer[l_cnt];

  NN::initEmptyNN();

  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::Basic cnst end"));
    Serial.flush();
  #endif
  
}


// Copy Constructor
// ------------------------------------------------------------------------------------------
NN::NN(const NN &obj){
  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::Cpy cnst bgn"));
    Serial.flush();
  #endif
}


// Move Constructor
// ------------------------------------------------------------------------------------------
NN::NN(const NN &&obj){
  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::Move cnst bgn"));
    Serial.flush();
  #endif 
}


// Destructor
// ------------------------------------------------------------------------------------------
NN::~NN(){
  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::Dstr bgn"));
    Serial.flush();
  #endif

  delete[] LNodesCnt;
  delete[] L;

  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::Dstr end"));
    Serial.flush();
  #endif
}

// Init/Setup
// ------------------------------------------------------------------------------------------
void NN::init(byte l_cnt){

  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::init() bgn"));
    Serial.flush();
  #endif
  
  this->l_cnt = l_cnt;

  #ifdef NN_DEBUG_MODE
    Serial.print(F("-L cnt="));
    Serial.println(this->l_cnt);
    Serial.flush();
  #endif

  LNodesCnt = new byte[l_cnt];
  L = new NN_Layer[l_cnt];

  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::init() end"));
    Serial.flush();
  #endif
  
}

// ------------------------------------------------------------------------------------------
void NN::setNumOfLayers(byte l_cnt){
  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::setNumOfLayers() bgn"));
    Serial.flush();
  #endif

  this->l_cnt = l_cnt;
  LNodesCnt = new byte[l_cnt];

  L = new NN_Layer[l_cnt];

  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::setNumOfLayers() end"));
    Serial.flush();
  #endif  
}

// ------------------------------------------------------------------------------------------
void NN::setNumOfNodes(byte l_idx, byte n_cnt){
  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::setNumOfNodes() bgn"));
    Serial.flush();
  #endif
  
  if((l_idx >= 0) && (l_idx < l_cnt)){
    LNodesCnt[l_idx] = n_cnt;
  }
  
  #ifdef NN_DEBUG_MODE
    Serial.print(F("L"));
    Serial.print(l_idx);
    Serial.print(F(", n_cnt="));
    Serial.print(n_cnt);
    Serial.println(F("NN::setNumOfNodes() end"));
    Serial.flush();
  #endif
}

// ------------------------------------------------------------------------------------------
void NN::initEmptyNN(){
  
  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::initEmptyNN() bgn"));
    Serial.flush();
  #endif

  // for all layers
  for(byte l_idx = 0; l_idx < this->l_cnt; l_idx++){
    // if first (input) layer
    if(l_idx == 0){ 
      // no input weights    
      L[l_idx].init(true, LNodesCnt[l_idx], 0);
    }
    else{
      // number of input weights same as number of nodes in previous layer
      L[l_idx].init(false, LNodesCnt[l_idx],LNodesCnt[l_idx - 1]);
    }
    #ifdef NN_DEBUG_MODE
      Serial.print(F("-L"));
      Serial.print(l_idx);
      Serial.println(F(" init ok"));
      Serial.flush();
    #endif   
  }

  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::initEmptyNN() end"));
    Serial.flush();
  #endif  
}

// -----------------------------------------------------------------------------------------------------------------------
void NN::randomizeWeights(byte seed){
  // initialise NN with zero values and random weights
  // if seed = 0 - random timer value will be the seed
  // if deed !=0 - seed actual valu evill be used

    #ifdef NN_DEBUG_MODE
      Serial.println(F("NN::randomizeWeights() bgn"));
      Serial.flush();
    #endif

   if(seed){
      #ifdef NN_DEBUG_MODE
        Serial.print(F("-seed="));
        Serial.println(seed);
        Serial.flush();
      #endif
      randomSeed(seed);
   }
   else{
      #ifdef NN_DEBUG_MODE
        Serial.println(F("-rnd seed"));
        Serial.flush();
      #endif
      randomSeed(millis());
   }

   // initialise input layer activation values
   // with zeros
   for(byte n = 0; n < L[0].getCntN(); n++){
      L[0].setA(0, n);
   }

   // initialise hidden and output layers:
   // activation values = 0
   // biases - according to specified value
   // deltas = 0
   // bias weights - random
   // input weights - random  
   // for each layer (after input layer)
   for(byte l_idx = 1; l_idx < l_cnt; l_idx++){
      // for each node
      for(byte n = 0; n < L[l_idx].getCntN(); n++){
        L[l_idx].setA(0, n);                    // set node activation value = 0
        L[l_idx].setB(NN_BIAS_INIT_VAL, n);        // set node bias value
        L[l_idx].setD(0, n);                    // set node delta value
        L[l_idx].setWb((2 * NN::fun_normRnd() - 1)*NN_WEIGHTS_INIT_COEF, n);   // set node weight for bias // "2*" , "-1" to make weights zero-centered, transform 0...1 into -1...1

        // for each input weight
        for(byte w = 0; w < L[l_idx].getCntW(); w++){
          L[l_idx].setW((2 * NN::fun_normRnd() - 1)*NN_WEIGHTS_INIT_COEF, n, w); // "2*" , "-1" to make weights zero-centered, transform 0...1 into -1...1
        }        
      }    
   }
    #ifdef NN_DEBUG_MODE
      Serial.println(F("NN::randomizeWeights() end"));
      Serial.flush();
    #endif  
}


// Getters/Setters
// ------------------------------------------------------------------------------------------
byte NN::getCntL(){
  return this->l_cnt;
}

byte NN::getCntN(byte l){
  return L[l].getCntN();
}

byte NN::getCntW(byte l){
  return L[l].getCntW();
}

double NN::getA(byte l, byte n){
  return L[l].getA(n);
}

double NN::getB(byte l, byte n){
  return L[l].getB(n);
}

double NN::getD(byte l, byte n){
  return L[l].getB(n);
}

double NN::getWb(byte l, byte n){
  return L[l].getWb(n);
}

double NN::getW(byte l, byte n, byte w){
  return L[l].getW(n, w);  
}

double NN::getOut(byte n){
  return L[l_cnt - 1].getA(n);
}

void NN::setA(double x, byte l, byte n){
  L[l].setA(x, n);
}

void NN::setB(double x, byte l, byte n){
  L[l].setB(x, n);
}

void NN::setD(double x, byte l, byte n){
  L[l].setD(x, n);
}

void NN::setWb(double x, byte l, byte n){
  L[l].setWb(x, n);
}

void NN::setW(double x, byte l, byte n, byte w){
  L[l].setW(x, n, w);
}

double NN::getHLDomMin(){
  return NN_HL_DOM_MIN;
}

double NN::getHLDomMax(){
  return NN_HL_DOM_MAX;  
}

double NN::getOLDomMin(){
  return NN_OL_DOM_MIN;
}

double NN::getOLDomMax(){
  return NN_OL_DOM_MAX;
}


// MATH FUNCTIONS
// -----------------------------------------------------------------------------------------------------------------------
double NN::fun_normRnd(){
  
  //randomSeed(millis());
  double rnd = (double) random(0, 1000);
  return rnd/1000;
}

// -----------------------------------------------------------------------------------------------------------------------
double NN::fun_map(double x, double x_min, double x_max, double y_min, double y_max){    
  // same as Arduino map() function, but for double type 
  // Formula here:
  // https://docs.arduino.cc/language-reference/en/functions/math/map/

  if(x_max - x_min == 0){
    return 0.0;
  }
  else{
    return (x - x_min) * (y_max - y_min) / (x_max - x_min) + y_min;
  }
}


// -----------------------------------------------------------------------------------------------------------------------
#ifdef NN_LOSSFUN_MSE
double NN::fun_MSE(double o, double y){
  return (o - y) * (o - y);
}

double NN::fun_dMSE(double o, double y){
  return 2.0 * (o - y);
}
#endif

// -----------------------------------------------------------------------------------------------------------------------
#if defined(NN_HL_AFUN_SIGMOID) || defined(NN_OL_AFUN_SIGMOID) || defined(NN_HL_AFUN_SOFTPLUS) || defined(NN_OL_AFUN_SOFTPLUS)

  // derivative of softplus is sigmoid

double NN::fun_Sigmoid(double x){
  // domain: -ing, inf
  // range: (0, 1)
  return 1.0 / (1.0 + exp(-x));
}

double NN::fun_invSigmoid(double y){
  if((y == 0) || ((1.0 - y) <= 0)){
    return 0;
  }
  else{
    return -log((1.0 - y) / y);
  }
}

double NN::fun_dSigmoid(double x){
  return NN::fun_Sigmoid(x) * (1.0 - NN::fun_Sigmoid(x));
}
#endif

// -----------------------------------------------------------------------------------------------------------------------
#if defined(NN_HL_AFUN_TANH ) || defined(NN_OL_AFUN_TANH )
double NN::fun_Tanh(double x){
  // domain: -ing, inf
  // range: (-1, 1)

  return 2/(1 + exp(-2*x)) - 1;  
}


double NN::fun_invTanh(double y){

  return -log((1-y)/(1+y))/2;  
}


double NN::fun_dTanh(double x){

  return 4*exp(2*x)/((exp(2*x) + 1)*(exp(2*x) + 1));  
}


#endif

// -----------------------------------------------------------------------------------------------------------------------
#if defined(NN_HL_AFUN_ID) || defined(NN_OL_AFUN_ID) 
double NN::fun_Identity(double x){
  // domain: -inf, inf
  // range: -inf, inf
  
  return x;
}

double NN::fun_invIdentity(double y){
  // domain: -inf, inf
  // range: -inf, inf
  
  return y;
}

double NN::fun_dIdentity(double x){
  return 1.0;
}
#endif

// -----------------------------------------------------------------------------------------------------------------------
#if defined(NN_HL_AFUN_RELU) || defined(NN_OL_AFUN_RELU)
double NN::fun_ReLU(double x){
  // domain: -ing, inf
  // range: 0, inf
  
  if(x > 0){
    return x;
  }
  else{
    return 0;
  }    
}

double NN::fun_invReLU(double y)
{

  // for x <=0 information is lost
  // so for y = 0 just set x as 0
  if(y > 0){
    return y;
  }
  else{
    return 0;
  }    
}


double NN::fun_dReLU(double x)
{
  if(x > 0){
    return 1.0;
  }
  else{
    return 0;
  }    
}
#endif

// -----------------------------------------------------------------------------------------------------------------------
#if defined(NN_HL_AFUN_SOFTPLUS) || defined(NN_OL_AFUN_SOFTPLUS)
double NN::fun_Softplus(double x){
  // domain: (-inf, inf)
  // range: (0, inf)
  
  return log(1 + exp(x));
}

double NN::fun_invSoftplus(double y){
  if((1 - exp(y) <= 0){
    return 0;
  }
  esle{
    return log(1 - exp(y));
  }
}

double NN::fun_dSoftplus(double x){
  return NN::fun_Sigmoid(x);  
}
#endif

// -----------------------------------------------------------------------------------------------------------------------
#if defined(NN_HL_AFUN_LEAKYRELU) || defined(NN_OL_AFUN_LEAKYRELU) 
double NN::fun_LeakyReLU(double x){
  // domain: (-inf, inf)
  // range: (-inf, inf)
  
  if(x > 0){
    return x;
  }
  else{
    return x * NN_LEAKY_RELU_COEFF;
  }  
}

double NN::fun_invLeakyReLU(double y){
  if(y > 0){
    return y;
  }
  else{
    if(NN_LEAKY_RELU_COEFF == 0){
      return 0;
    }
    else{
      return y / NN_LEAKY_RELU_COEFF;
    }
  }  
}


double NN::fun_dLeakyReLU(double x){
  if(x > 0){
    return 1.0;
  }
  else if(x == 0){
    return 0.0;
  }
  else{
    return NN_LEAKY_RELU_COEFF;
  }  
}
#endif

// -----------------------------------------------------------------------------------------------------------------------
#if defined(NN_HL_AFUN_ELU) || defined(NN_OL_AFUN_ELU)
double NN::fun_ELU(double x){
  // domain: (-inf, inf)
  // range: (-NN_LEAKY_RELU_COEFF, inf)
  
  
  if(x > 0){
    return x;
  }
  else{
    return NN_LEAKY_RELU_COEFF * (exp(x) - 1.0);
  }    
}

double NN::fun_invELU(double y){
  if(y > 0){
    return y;
  }
  else{
    if(NN_LEAKY_RELU_COEFF == 0){
      return 0;
    }
    else{
      if(((y + NN_LEAKY_RELU_COEFF) / NN_LEAKY_RELU_COEFF) <= 0){
        return log((-0.95*NN_LEAKY_RELU_COEFF + NN_LEAKY_RELU_COEFF) / NN_LEAKY_RELU_COEFF);   //??? -0.95*coef instead of -y
      }
      else{
        return log((y + NN_LEAKY_RELU_COEFF) / NN_LEAKY_RELU_COEFF);
      }
    }
  }    
}

double NN::fun_dELU(double x){
  if(x > 0){
    return 1;
  }
  else{
    //return NN_LEAKY_RELU_COEFF * exp(x);
    return -1*NN_LEAKY_RELU_COEFF * exp(x);  // for negative x
  }    
}
#endif

// -----------------------------------------------------------------------------------------------------------------------
double NN::fun_act_O(double x){
  
  // apply selected activation function (Output layer)
  
  #ifdef NN_OL_AFUN_SIGMOID 
    return NN::fun_Sigmoid(x);
  #endif

  #ifdef NN_OL_AFUN_TANH
    return NN::fun_Tanh(x);
  #endif

  #ifdef NN_OL_AFUN_ID
    return NN::fun_Identity(x);
  #endif

  #ifdef NN_OL_AFUN_RELU
    return NN::fun_ReLU(x);
  #endif

  #ifdef NN_OL_AFUN_SOFTPLUS
    return NN::fun_Softplus(x);
  #endif

  #ifdef NN_OL_AFUN_LEAKYRELU
    return NN::fun_LeakyReLU(x);
  #endif

  #ifdef NN_OL_AFUN_ELU
    return NN::fun_ELU(x);
  #endif  
}

// -----------------------------------------------------------------------------------------------------------------------
double NN::fun_inv_O(double y){
  
  // apply inverse of selected activation function (Output layer)
  
  #ifdef NN_OL_AFUN_SIGMOID
    return NN::fun_invSigmoid(y);
  #endif

  #ifdef NN_OL_AFUN_TANH
    return NN::fun_invTanh(y);
  #endif

  #ifdef NN_OL_AFUN_ID
    return NN::fun_invIdentity(y);
  #endif

  #ifdef NN_OL_AFUN_RELU
    return NN::fun_invReLU(y);
  #endif

  #ifdef NN_OL_AFUN_SOFTPLUS
    return NN::fun_invSoftplus(y);
  #endif

  #ifdef NN_OL_AFUN_LEAKYRELU
    return NN::fun_invLeakyReLU(y);
  #endif

  #ifdef NN_OL_AFUN_ELU
    return NN::fun_invELU(y);
  #endif  
}

// -----------------------------------------------------------------------------------------------------------------------
double NN::fun_dAct_O(double x){
  
  // apply derivative of selected activation function
      
  #ifdef NN_OL_AFUN_SIGMOID
    return NN::fun_dSigmoid(x);
  #endif

  #ifdef NN_OL_AFUN_TANH
    return NN::fun_dTanh(x);
  #endif  

  #ifdef NN_OL_AFUN_ID
    return NN::fun_dIdentity(x);
  #endif

  #ifdef NN_OL_AFUN_RELU
    return NN::fun_dReLU(x);
  #endif

  #ifdef NN_OL_AFUN_SOFTPLUS
    return NN::fun_dSoftplus(x);
  #endif

  #ifdef NN_OL_AFUN_LEAKYRELU
    return NN::fun_dLeakyReLU(x);
  #endif

  #ifdef NN_OL_AFUN_ELU
    return NN::fun_dELU(x);
  #endif
}

// -----------------------------------------------------------------------------------------------------------------------
double NN::fun_act_H(double x){
  
  // apply selected activation function (hidden/imput layers)
  
  #ifdef NN_HL_AFUN_SIGMOID
    return NN::fun_Sigmoid(x);
  #endif

  #ifdef NN_HL_AFUN_TANH
    return NN::fun_Tanh(x);
  #endif

  #ifdef NN_HL_AFUN_ID
    return NN::fun_Identity(x);
  #endif

  #ifdef NN_HL_AFUN_RELU
    return NN::fun_ReLU(x);
  #endif

  #ifdef NN_HL_AFUN_SOFTPLUS
    return NN::fun_Softplus(x);
  #endif

  #ifdef NN_HL_AFUN_LEAKYRELU
    return NN::fun_LeakyReLU(x);
  #endif

  #ifdef NN_HL_AFUN_ELU
    return NN::fun_ELU(x);
  #endif  
}

// -----------------------------------------------------------------------------------------------------------------------
double NN::fun_inv_H(double y){
  
  // apply inverse of selected activation function (hidden/input layers)
  
  #ifdef NN_HL_AFUN_SIGMOID
    return NN::fun_invSigmoid(y);
  #endif

  #ifdef NN_HL_AFUN_TANH
    return NN::fun_invTanh(y);
  #endif

  #ifdef NN_HL_AFUN_ID
    return NN::fun_invIdentity(y);
  #endif

  #ifdef NN_HL_AFUN_RELU
    return NN::fun_invReLU(y);
  #endif

  #ifdef NN_HL_AFUN_SOFTPLUS
    return NN::fun_invSoftplus(y);
  #endif

  #ifdef NN_HL_AFUN_LEAKYRELU
    return NN::fun_invLeakyReLU(y);
  #endif

  #ifdef NN_HL_AFUN_ELU
    return NN::fun_invELU(y);
  #endif  
}

// -----------------------------------------------------------------------------------------------------------------------
double NN::fun_dAct_H(double x){
  
  // apply derivative of selected activation function (hidden/input layers)
  #ifdef NN_HL_AFUN_SIGMOID
    return NN::fun_dSigmoid(x);
  #endif

  #ifdef NN_HL_AFUN_TANH
    return NN::fun_dTanh(x);
  #endif

  #ifdef NN_HL_AFUN_ID
    return NN::fun_dIdentity(x);
  #endif

  #ifdef NN_HL_AFUN_RELU
    return NN::fun_dReLU(x);
  #endif

  #ifdef NN_HL_AFUN_SOFTPLUS
    return NN::fun_dSoftplus(x);
  #endif

  #ifdef NN_HL_AFUN_LEAKYRELU
    return NN::fun_dLeakyReLU(x);
  #endif

  #ifdef NN_HL_AFUN_ELU
    return NN::fun_dELU(x);
  #endif
}

// -----------------------------------------------------------------------------------------------------------------------
double NN::fun_Loss(double o, double y){

  // apply selected loss function 
  #ifdef NN_LOSSFUN_MSE
    return NN::fun_MSE(o, y);
  #endif
}

// -----------------------------------------------------------------------------------------------------------------------
double NN::fun_dLoss(double o, double y){

  // apply selected loss function 
  #ifdef NN_LOSSFUN_MSE
    return NN::fun_dMSE(o, y);
  #endif
}


//// Scaling input/output
//// -----------------------------------------------------------------------------------------------------------------------
double NN::scale_to_hidden_domain(double in_val, double in_range_min, double in_range_max){

  return NN::fun_map(in_val, in_range_min, in_range_max, NN_HL_DOM_MIN, NN_HL_DOM_MAX);  
}

double NN::scale_to_output_domain(double tgt_val, double tgt_range_min, double tgt_range_max){

  return NN::fun_map(tgt_val, tgt_range_min, tgt_range_max, NN_OL_DOM_MIN, NN_OL_DOM_MAX);
}


double NN::scale_to_hidden_domain_and_calc_activation(double in_val, double in_range_min, double in_range_max){

  return NN::fun_act_H(NN::scale_to_hidden_domain(in_val,in_range_min,in_range_max));
}

double NN::scale_to_output_domain_and_calc_activation(double tgt_val, double tgt_range_min, double tgt_range_max){

  return NN::fun_act_O(NN::scale_to_output_domain(tgt_val, tgt_range_min, tgt_range_max));
}


double NN::calc_output_inv_activation_and_descale(byte out_idx, double tgt_range_min, double tgt_range_max){

  double activation_val = L[l_cnt - 1].getA(out_idx);
  double scaled_val = NN::fun_inv_O(activation_val);
  double raw_val = NN::fun_map(scaled_val, NN_OL_DOM_MIN, NN_OL_DOM_MAX, tgt_range_min, tgt_range_max);

  return raw_val;  
}

double NN::descale_output(byte out_idx, double tgt_range_min, double tgt_range_max){

  double scaled_val = L[l_cnt - 1].getA(out_idx);
  double raw_val = NN::fun_map(scaled_val, NN_OL_DOM_MIN, NN_OL_DOM_MAX, tgt_range_min, tgt_range_max);

  return raw_val;  
}




//double NN::scale_raw_val_to_hidden_actfun_domain(double raw_val, double raw_val_range_min, double raw_val_range_max){
//
//  return NN::fun_map(raw_val, raw_val_range_min, raw_val_range_max, NN_HL_DOM_MIN, NN_HL_DOM_MAX);
//}
//
//
//double NN::scale_raw_val_to_output_actfun_domain(double raw_val, double raw_val_range_min, double raw_val_range_max){
//
//  return NN::fun_map(raw_val, raw_val_range_min, raw_val_range_max, NN_OL_DOM_MIN, NN_OL_DOM_MAX);  
//}
//
//
//double NN::scale_hidden_actfun_domain_to_raw_val(double raw_val, double raw_val_range_min, double raw_val_range_max);
//double NN::scale_output_actfun_domain_to_raw_val(double raw_val, double raw_val_range_min, double raw_val_range_max); 
//double NN::calc_hidden_layer_actfun(double val);
//double NN::calc_output_layer_actfun(double val);
//double NN::calc_hidden_layer_actfun_inverse(double val);
//double NN::calc_output_layer_actfun_inverse(double val);

// NN CORE FUNCTIONS
// -----------------------------------------------------------------------------------------------------------------------
// Calculate net input of neuron n in layer l
double NN::calcNeuroneNetInput(byte l_idx, byte n_idx){

    double z = 0.0;

    // pre-activation value for n-th node
    z = L[l_idx].getB(n_idx) * L[l_idx].getWb(n_idx);  // add value from bias node first

    // loop thought input weights of of n-th node
    for (byte w_idx = 0; w_idx < L[l_idx].getCntW(); w_idx++)  {
      // add product of w-th input and weight to pre-activation value        
      z += L[l_idx].getW(n_idx, w_idx) * L[l_idx - 1].getA(w_idx);    
    }

    return z;
}

// -----------------------------------------------------------------------------------------------------------------------
// Propagate layer
void NN::propagate_layer(byte idx){

  //  This function calculates activation values for
  //  each node in idx-th layer

  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::propagate_layer() bgn"));
    Serial.flush();
  #endif

  double a = 0; // to store activation of a node
  double z = 0; // to store weighted sum

  #ifdef NN_HL_NORMALISATION
    double mean = 0;
    double std = 0;
    double z_min = 0;
    double z_max = 0;
  #endif

  // Find sum of weighted inputs for each node
  // -----------------------------------------
  // loop layer nodes
  for (byte n = 0; n < L[idx].getCntN(); n++) {
//      // pre-activation value for n-th node
//      z = L[idx].getB(n) * L[idx].getWb(n);  // add value from bias node first
//
//      // loop thought input weights of of n-th node
//      for (byte w = 0; w < L[idx].getCntW(); w++)  {
//        // add product of w-th input and weight to pre-activation value        
//        z += L[idx].getW(n, w) * L[idx - 1].getA(w);    
//      }

      z = NN::calcNeuroneNetInput(idx, n);

      // Save sum of weighted inputs for n-th node
      L[idx].setA(z, n);  
  }

  #ifdef NN_HL_NORMALISATION
    // Normalisation of weighted sums
    // ------------------------------
  
    // find mean value weighted sums
    z = 0;  // z will be summ of all weighted sums  
    for (byte n = 0; n < L[idx].getCntN(); n++) {
      z += L[idx].getA(n);
    }
    mean = z/L[idx].getCntN();
  
    // find standard deviation
    z = 0; // z will be summ of squared defferences  
    for (byte n = 0; n < L[idx].getCntN(); n++) {
      z += (L[idx].getA(n) - mean)*(L[idx].getA(n) - mean);
    }
    std = sqrt(z/L[idx].getCntN());
  
    // find min and max val for weighted sums (2 sigma)
    z_min = mean - 2*std;
    z_max = mean + 2*std;
  
    // map weighted sums to activation function domain
    z = 0;  // z will be normalised weighted sum
    for (byte n = 0; n < L[idx].getCntN(); n++) {
      if(idx == l_cnt - 1){
        // If output layer
        z = NN::fun_map(L[idx].getA(n), z_min, z_max, NN_OL_DOM_MIN, NN_OL_DOM_MAX);              
      }
      else{
        // If hidden layer
        z = NN::fun_map(L[idx].getA(n), z_min, z_max, NN_HL_DOM_MIN, NN_HL_DOM_MAX);     
      } 
      L[idx].setA(z, n);   
    }
  #endif


  // Apply activation function and calculate activation for each node
  // ----------------------------------------------------------------
  // loop layer nodes
  for (byte n = 0; n < L[idx].getCntN(); n++) {
      
      // Calc activation of n-th node
      if(idx == l_cnt - 1){
        // If output layer
        a = NN::fun_act_O(L[idx].getA(n));              
      }
      else{
        // If hidden layer
        a = NN::fun_act_H(L[idx].getA(n));    
      }

      // Save activation of n-th node to the array
      L[idx].setA(a, n);   
  }

  
  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::propagate_layer() end"));
    Serial.flush();
  #endif   
}

/*
void NN::propagate_layer(byte idx){

  //  This function calculates activation values for
  //  each node in idx-th layer

  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::propagate_layer() bgn"));
    Serial.flush();
  #endif

  double a = 0; // to store activation of a node

  // loop layer nodes
  for (byte n = 0; n < L[idx].getCntN(); n++) {
      // pre-activation value for n-th node
      double z = L[idx].getB(n) * L[idx].getWb(n);  // add value from bias node first

      // loop thought input weights of of n-th node
      for (byte w = 0; w < L[idx].getCntW(); w++)  {
        // add product of w-th input and weight to pre-activation value        
        z += L[idx].getW(n, w) * L[idx - 1].getA(w);    
      }

      // Calc activation of n-th node
      if(idx == l_cnt - 1){
        // If output layer
        a = NN::fun_act_O(z);              
      }
      else{
        // If hidden layer
        a = NN::fun_act_H(z);    
      }

      // Save activation of n-th node to the array
      L[idx].setA(a, n);   
  }
  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::propagate_layer() end"));
    Serial.flush();
  #endif   
}
*/

// -----------------------------------------------------------------------------------------------------------------------
void NN::propagate_network(double* X){

  // This function propagates inputs through network 
  //
  // Arguments:
  // - double* X - 1D array with input values (number of elements in X = number of nodes in input layer)

  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::propagate_network() start"));
    Serial.print(F("NN in: "));
    Serial.flush();
  #endif
  
  // Copy received inputs to input layer
  for(byte n = 0; n < L[0].getCntN(); n++){

    L[0].setA(X[n],n); 

    #ifdef NN_DEBUG_MODE
      NN::debugWaitSerialOutAvailable();
      Serial.print(L[0].getA(n));
      Serial.print(F(";"));
      Serial.flush();
    #endif
  }
  
  #ifdef NN_DEBUG_MODE
    Serial.println();
    Serial.flush();
  #endif

  // propagate inputs through all hidden layers to output layer
  // (start calculations from first hidden layer)
  for(byte l_idx = 1; l_idx < l_cnt; l_idx++){
    #ifdef NN_DEBUG_MODE
      Serial.print(F("Rdy to propagate L"));
      Serial.println(l_idx);
      Serial.flush();
    #endif
    
    NN::propagate_layer(l_idx);
    
  }

  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::propagate_network() end"));
    Serial.flush();
  #endif
}

// -----------------------------------------------------------------------------------------------------------------------
void NN::calc_output_layer_deltas(double* Y){

   //  This function calculates Delta values for nodes in output layer
   //  
   //  Arguments:
   //  - double* Y - 1D array with target values (number of elements in Y = number of nodes in output layer)

  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::calc_output_layer_deltas() bgn"));
    Serial.flush();
  #endif
    
    double dJ = 0.0;    // Derivative of a loss function
    double dA = 0.0;    // Derivative of an activation function
    double z = 0.0;     // net input for n-th neurone

    // for each node in output (last) layer
    for(byte n = 0; n < L[l_cnt - 1].getCntN(); n++){
      
      // Calculate derivative of a loss function           
      dJ = NN::fun_dLoss(L[l_cnt - 1].getA(n), Y[n]);

      // Calculate erivative of an activation function
      z = calcNeuroneNetInput(l_cnt - 1, n);
      dA = NN::fun_dAct_O(z);
      //dA = NN::fun_dAct_O(L[l_cnt - 1].getA(n));

      // Update delta value of output array node
      L[l_cnt - 1].setD(dA * dJ, n);

      #ifdef NN_DEBUG_MODE
        NN::debugWaitSerialOutAvailable();
        Serial.print(F("-n"));
        Serial.print(n);
        Serial.print(F(": df(a)="));
        Serial.print(dA);
        Serial.print(F(", dJ(a)="));
        Serial.print(dJ);
        Serial.print(F(", D="));
        Serial.println(L[l_cnt - 1].getD(n));
        Serial.flush();
      #endif        
    }
  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::calc_output_layer_deltas() end"));
    Serial.flush();
    NN::debugWaitSerialOutAvailable();
  #endif  
}

// -----------------------------------------------------------------------------------------------------------------------
void NN::calc_hidden_layer_deltas(byte l_idx) {  
   
  //  This function calculates Delta values for nodes in hidden layers

  #ifdef NN_DEBUG_MODE
    NN::debugWaitSerialOutAvailable();
    Serial.println(F("NN::calc_hidden_layer_deltas() bgn"));
    Serial.flush();
  #endif

  // initialisations
  double summ = 0.0;  // summ therm
  double add = 0.0;   // therm to be added to summ
  //double a = 0.0;     // activation value of n-th node in given layer
  double dA = 0.0;    // derivative of activation function of n-th node in given layer
  double d = 0.0;     // Delta value of n-th node in given layer
  double w_nxt = 0.0; // weight of the connection to between node in given layer and in next layer
  double d_nxt = 0.0; // delta of a node from next layer
  double z = 0.0;     // net input for n-th neurone

  #ifdef NN_DEBUG_MODE
    NN::debugWaitSerialOutAvailable();
    Serial.print(F("L"));
    Serial.println(l_idx);
    Serial.flush();
  #endif
  
  // Loop nodes in given layer
  for(byte n = 0; n < L[l_idx].getCntN(); n++){

    #ifdef NN_DEBUG_MODE
      NN::debugWaitSerialOutAvailable();
      Serial.print(F("-n"));
      Serial.println(n);
      Serial.flush();
    #endif
    
    summ = 0.0; // clear summ therm

    // Find "summ therm" for n-th neurone
    // ----------------------------------
    // Loop nodes in next layer
    for(byte nxt = 0; nxt < L[l_idx + 1].getCntN(); nxt++){

      // delta of a node from next layer
      d_nxt = L[l_idx + 1].getD(nxt);
      // weight of the connection between current node node in given layer (n) and current node in next layer (nxt)
      w_nxt = L[l_idx + 1].getW(nxt, n);
      
      #ifdef NN_DEBUG_MODE
        NN::debugWaitSerialOutAvailable();
        Serial.print(F("--Nxt:n"));
        Serial.print(nxt);
        Serial.print(F("; nxtD="));
        Serial.print(d_nxt);
        Serial.print(F("; w(n->nxt)="));
        Serial.print(w_nxt);
        Serial.flush();
      #endif

      // updateting summ therm
      add = w_nxt * d_nxt;      
      summ = summ + add;
      
      #ifdef NN_DEBUG_MODE
        NN::debugWaitSerialOutAvailable();
        Serial.print(F("; add therm="));
        Serial.print(add);
        Serial.print(F("; summ therm="));
        Serial.println(summ);
        Serial.flush();
      #endif    
    } // end loop nodes in next layer

    #ifdef NN_DEBUG_MODE
      NN::debugWaitSerialOutAvailable();
      Serial.print(F("-n"));
      Serial.print(n);
      Serial.print(F(" tot summ="));
      Serial.print(summ);
      Serial.flush();
    #endif

    // Find "derivative of activation function for n-th neurone
    // --------------------------------------------------------
    // find z

    // activation value of n-th node in given layer
    //a = L[l_idx].getA(n);
    z = NN::calcNeuroneNetInput(l_idx, n);

    // Calculate derivative of an activation function
    dA = NN::fun_dAct_H(z);

    // Delta value of n-th node in given layer
    d = dA*summ;

    #ifdef NN_DEBUG_MODE
      NN::debugWaitSerialOutAvailable(); 
      Serial.print(F("; z="));
      Serial.print(z);
      Serial.print(F("; df(z)="));
      Serial.print(dA);
      Serial.print(F("; D="));
      Serial.println(d);
      Serial.flush();
    #endif

    // save delta value to array
    L[l_idx].setD(d, n);

  } // end loop nodes in current layer
  
  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::calc_hidden_layer_deltas() end"));
    Serial.flush();
    NN::debugWaitSerialOutAvailable();
  #endif
}

// -----------------------------------------------------------------------------------------------------------------------
void NN::update_layer_weights(byte l_idx){

  //  This function updaites input weights for given layer

  #ifdef NN_DEBUG_MODE
    NN::debugWaitSerialOutAvailable();
    Serial.println(F("NN::update_layer_weights() bgn"));
    Serial.print(F("L"));
    Serial.println(l_idx);
    Serial.flush();
  #endif

  double upd = 0;
  double gradient_step = 0;  

  #ifdef NN_USE_GRADIENT_CLIPPING
    double tmp = 0;
    double g2_sum_of_squares = 0;
    double g2 = 0;

    // Loop nodes in given layer
    for(byte n = 0; n < L[l_idx].getCntN(); n++){
      // Loop input weights
      for(byte w = 0; w < L[l_idx].getCntW(); w++){
        tmp = NN_TRAINING_SPEED * L[l_idx].getD(n) * L[l_idx - 1].getA(w);
        g2_sum_of_squares +=  tmp*tmp;  
      }
    }

    g2 = sqrt(g2_sum_of_squares);
  #endif

  // Loop nodes in given layer
  for(byte n = 0; n < L[l_idx].getCntN(); n++){
    #ifdef NN_DEBUG_MODE
      NN::debugWaitSerialOutAvailable();
      Serial.print(F("n"));
      Serial.print(n);
      Serial.print(F(":"));
      Serial.flush();
    #endif
    
    // Loop input weights
    for(byte w = 0; w < L[l_idx].getCntW(); w++){      
      
      gradient_step = NN_TRAINING_SPEED * L[l_idx].getD(n) * L[l_idx - 1].getA(w);

      #ifdef NN_USE_GRADIENT_CLIPPING
        if(NN_GRADIENT_CLIPPING_COEF/g2 < 1){
          gradient_step = gradient_step*NN_GRADIENT_CLIPPING_COEF/g2;  
        }
      #endif
      
      // ??? "-" or "+" ?
      upd = L[l_idx].getW(n, w) - gradient_step;
      L[l_idx].setW(upd, n, w);

      #ifdef NN_DEBUG_MODE
        NN::debugWaitSerialOutAvailable();
        Serial.print(F("w"));
        Serial.print(w);
        Serial.print(F("="));
        Serial.print(upd);
        Serial.print(F(";"));
        Serial.flush();
      #endif
    } // end loop input weights

    // update bias weight
    gradient_step = NN_TRAINING_SPEED * L[l_idx].getD(n) * L[l_idx].getB(n);

    #ifdef NN_USE_GRADIENT_CLIPPING
      if(NN_GRADIENT_CLIPPING_COEF/g2 < 1){
        gradient_step = gradient_step*NN_GRADIENT_CLIPPING_COEF/g2;  
      }
    #endif
    
    // ??? "-" or "+" ?
    //upd = L[l_idx].getWb(n) - NN_TRAINING_SPEED * L[l_idx].getD(n) * L[l_idx].getB(n);
    upd = L[l_idx].getWb(n) - gradient_step;
    L[l_idx].setWb(upd, n);

    #ifdef NN_DEBUG_MODE
      NN::debugWaitSerialOutAvailable();
      Serial.print(F("wb="));
      Serial.println(upd);
      Serial.flush();
    #endif
    
  } // end loop nodes in given layer

  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::update_layer_weights() end"));
    Serial.flush();
    NN::debugWaitSerialOutAvailable();
  #endif  
}

// -----------------------------------------------------------------------------------------------------------------------
void NN::backpropagate_network(double* Y){

   // This function backpropagates network
   // 
   // Arguments:
   // - double* Y - 1D array with target values (number of elements in Y = number of nodes in output layer)

  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::backpropagate_network() bgn"));
    Serial.flush();
  #endif
  
  // Calc Output layer deltas and update its weights
  NN::calc_output_layer_deltas(Y);
  NN::update_layer_weights(l_cnt - 1);
  
  // Calc deltas and update weights for hidden layers
  for(int idx = l_cnt - 2; idx >= 1; idx--){
    NN::calc_hidden_layer_deltas(idx);
    NN::update_layer_weights(idx);
  }
  
  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::backpropagate_network() end"));
    Serial.flush();
  #endif
}

// -----------------------------------------------------------------------------------------------------------------------
double NN::single_train(double* X, double* Y){

   // This function performs single train on sigle training record (X - inputs, Y - outputs)
   //  
   //  Arguments:
   //  - double* X - 1D array with input values (number of elements in X = number of nodes in input layer)
   //  - double* Y - 1D array with target values (number of elements in Y = number of nodes in output layer)


  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::single_train() bgn"));
    Serial.print(F("-X:"));
    Serial.flush();
    for(byte i = 0; i < L[0].getCntN(); i++){
      Serial.print(X[i]);
      Serial.print(F(" "));
      Serial.flush();
    }
    Serial.print(F("; Y:"));
    Serial.flush();
    for(byte i = 0; i < L[l_cnt - 1].getCntN(); i++){
      Serial.print(Y[i]);
      Serial.print(F(" "));
      Serial.flush();
    }
    Serial.println();
    Serial.flush();
  #endif
  
  NN::propagate_network(X);
  NN::backpropagate_network(Y);

  double loss = 0;

  // for each output node
  for(byte i = 0; i < L[l_cnt - 1].getCntN(); i++){
     loss +=  NN::fun_Loss(L[l_cnt - 1].getA(i), Y[i]);
  }

  #ifdef NN_DEBUG_MODE
    Serial.println(F("NN::single_train() end"));
    Serial.flush();
  #endif
  
  return loss/L[l_cnt - 1].getCntN();
}


// -----------------------------------------------------------------------------------------------------------------------
double NN::list_train_dynamic_data(double** XList, double** YList, byte r_cnt) {
  // This function performs train of the network on the full training set
  //  
  //  Arguments:
  //  - double** Xlist - 2D array with set of training inputs (num of rows = r_cnt, num of cols same as num of nodes in input layer)
  //  - double** Ylist - 2D array  with set of target outputs (num of rows = r_cnt, num of cols same as num of nodes in output layer)
  //  - byte r_cnt - number of records in training set

  #ifdef NN_REPORT_MODE
    Serial.println(F("NN::list_train_dynamic_data() bgn"));
    Serial.flush();
  #endif
  
  double loss = 0;  // network loss value




  // loop through training dataset to finish one training
  for (int j = 0; j < NN_EPOCHS_CNT; j++) {
    #ifdef NN_REPORT_MODE
      Serial.print(F("."));
      Serial.flush();
    #endif

    loss = 0; // clear loss value


    // for each row of training dataset
    for (byte r = 0; r < r_cnt; r++) {
      // For each training cycle
      for (int i = 0; i < NN_TRAINING_CYCLES_CNT; i++) {
        // perform training and accumulate loss
        loss += NN::single_train(XList[r], YList[r]);
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
    Serial.println(F("NN::list_train_dynamic_data() end"));
    Serial.flush();
  #endif
  
  return loss;
}


// Aux
// -----------------------------------------------------------------------------------------------------------------------
void NN::copy_node_activation(byte from_l, byte from_n, byte to_l, byte to_n, double coef){

  // Copy activation value of "from" l-th layer n-th node into "to" l-th layer n-th node, multiplied to coef value
  
  L[to_l].setA(coef * L[from_l].getA(from_n), to_n);
}


// Display
// -----------------------------------------------------------------------------------------------------------------------
void NN::disp_layer(byte l_idx){
  #ifdef NN_REPORT_MODE
    NN::debugWaitSerialOutAvailable();
    Serial.println();
    Serial.print(F("L:"));
    Serial.print(l_idx);
    Serial.println(F(" Tab1"));
    Serial.println(F(" N | A | Wb | D "));
    Serial.flush();
    for (byte i = 0; i < L[l_idx].getCntN(); i++){
      NN::debugWaitSerialOutAvailable();
      Serial.print(F("N:"));
      Serial.print(i);
      Serial.print(F(" | "));
      Serial.print(L[l_idx].getA(i),4);
      Serial.print(F(" | "));
      Serial.print(L[l_idx].getWb(i),4);
      Serial.print(F(" | "));
      Serial.print(L[l_idx].getD(i),4);
      Serial.println();
      Serial.flush();
    }
    NN::debugWaitSerialOutAvailable();
    Serial.print(F("L:"));
    Serial.print(l_idx);
    Serial.println(F(" wghts"));
    Serial.flush();
    for (byte i = 0; i < L[l_idx].getCntN(); i++){
      NN::debugWaitSerialOutAvailable();
      Serial.print(F("N:"));
      Serial.print(i);
      Serial.print(F(": "));
      Serial.flush();
      for (byte j = 0; j < L[l_idx].getCntW(); j++){
        NN::debugWaitSerialOutAvailable();
        Serial.print(F(" "));
        Serial.print(L[l_idx].getW(i,j),4); 
        Serial.flush();    
      }
      Serial.println();
      Serial.flush();
    }
  #endif
}

void NN::disp_NN(){
  #ifdef NN_REPORT_MODE
    Serial.println();
    Serial.println(F("L:0 (Input layer): Not to show."));
    Serial.flush();
    for(int l_idx = 1; l_idx < l_cnt; l_idx++){
      NN::disp_layer(l_idx);
    }
  #endif
}

void NN::disp_Outputs(){
  #ifdef NN_REPORT_MODE

    Serial.println(F("Outputs:"));

    for(byte i = 0; i < NN::getCntN(NN::getCntL() - 1); i++){

      Serial.println(NN::getA(NN::getCntL() - 1, i));      
    }

  #endif  
}




// Memory usage calculations
// -----------------------------------------------------------------------------------------------------------------------
int NN::estimateHiddenLayerRAM(int n_cnt, int w_cnt){

   // Formula explained:
   // - 3*sizeof(byte) - approx size of three NN_Layer private members: is_input_layer, n_cnt and w_cnt
   // - 4*(sizeof(int) + n_cnt*sizeof(double)) + (sizeof(int) - Related to 4 arrays A, B, D, Wb in NN_Layer
   //   each of these array has
   //   -- pointer sizeof(int)
   //   -- n_cnt values each sizeof(double)
   // - (sizeof(int) + n_cnt*sizeof(int) + n_cnt*w_cnt*sizeof(double)) - related to double-pointer array W:
   //   -- double pointer W** has sizeof(int)
   //   -- n_cnt of pointers for subarrays W*, each has sizeof(int)
   //   -- n_cnt*w_cnt cells with values sizeof(double)
   // - LAYER_EXTRA_RAM - extra RAM for safety, experementally tested value (Arduino Nano). For 10-node layer with
   //   10 weights for each node difference between actual and estimated value was 17 bytes (593 vs 610). Or, for example 
   //   for n_cnt = 6, w_cnt = 10 diff was 9 bytes (361 vs 370 bytes)

   return 3*sizeof(byte) + 4*(sizeof(int) + n_cnt*sizeof(double)) + (sizeof(int) + n_cnt*sizeof(int) + n_cnt*w_cnt*sizeof(double)) + LAYER_EXTRA_RAM;
}

int NN::estimateInputLayerRAM(int n_cnt){
   // Formula explained:
   // - 3*sizeof(byte) - approx size of three NN_Layer private members: is_input_layer, n_cnt and w_cnt
   // - 5*(sizeof(int) 5 pointers to A, B, D, Wb and W in NN_Layer
   // - n_cnt*sizeof(double) - n_cnt double values in array A
   // - LAYER_EXTRA_RAM - extra RAM for safety/
   
  return 3*sizeof(byte) + 5*sizeof(int) + n_cnt*sizeof(double) + LAYER_EXTRA_RAM;
}

int NN::estimateTotalLayersRAM(){

  int totRAM = 0;

  for(byte i = 0; i < l_cnt; i++){
    if(i == 0){
      totRAM += NN::estimateInputLayerRAM(LNodesCnt[i]);
    }
    else{
      totRAM += NN::estimateHiddenLayerRAM(LNodesCnt[i], LNodesCnt[i-1]);
    }
  }

  return totRAM;
}

void NN::debugWaitSerialOutAvailable(){
  #ifdef NN_DEBUG_MODE
  while(Serial.availableForWrite() < 40){
    Serial.flush();
  }  
  #endif
}
