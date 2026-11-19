#include "KB_RBrain.h"
#include <Arduino.h>


// Default constructor
// -----------------------------------------------------------------------------------------------------------------------
RBrain::RBrain(){
  
  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::Default constructor start"));
  #endif

  // NN 
  brNN = NULL;
  // Data arrays (Inputs to NN)
  X = NULL;
  V = NULL;
  Y = NULL;
  // Data tables
  S = NULL;           
  M = NULL;           
  C = NULL;           
  // Motor programm
  MPGEN = NULL;       
  MPSEL = NULL;

  // Data tables related
  allocateRefTables = false;

  // NN inputs related
  x_cnt = 0;     
  y_cnt = 0;    

  // Sensors related
  s_cnt = 0;       
     
  // Motor programm generator parameters
  mps_cnt = 0;     
  mpp_cnt = 0;       

  // Motor programm selector paramentrs
  outcome_threshold_1 = 0;     
  outcome_threshold_2 = 0;     
  att_cnt = 0;                 

  // Context neurones related
  c_cnt = 0;       
}


// Destructor
// -----------------------------------------------------------------------------------------------------------------------
RBrain::~RBrain(){
  
  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::Destructor start"));
  #endif

  // delete 1D arrays
  delete[] X;
  delete[] V;
  delete[] Y;

  // If 2D data arrays was dynamically allocated by RBrain
  // Deallocate 2D arrays
  if(allocateRefTables){
        
    // delete 2D array S
    if(s_cnt > 0){
      for(byte i = 0; i < s_cnt; i++){
        delete[] S[i]; 
      }
      delete[] S;     
    }
    else{
      delete S;
    }

    // delete 2D array M
    if(mpp_cnt > 0){
      for(byte i = 0; i < mpp_cnt; i++){
        delete[] M[i]; 
      }
      delete[] M;     
    }
    else{
      delete M;
    }

    // delete 2D array C
    if(c_cnt > 0){
      for(byte i = 0; i < c_cnt; i++){
        delete[] C[i]; 
      }
      delete[] C;     
    }
    else{
      delete C;
    }
  }
  else{
    // Clear pointers to static arrays
    S = NULL;
    M = NULL;
    C = NULL;
  } //end if(allocateRefTables)

  // delete 2D arrays MPGEN and MPSEL
  if(mps_cnt > 0){
    
    for(byte i = 0; i < mps_cnt; i++){
      delete[] MPGEN[i]; 
      delete[] MPSEL[i]; 
    }
    delete[] MPGEN;
    delete[] MPSEL;       
  }
  else{
    delete MPGEN;
    delete MPSEL;
  }
  
  // Clear pointer to NN object
  brNN = NULL;
}


// -----------------------------------------------------------------------------------------------------------------------
// init/setup
// -----------------------------------------------------------------------------------------------------------------------
void RBrain::init(bool allocateRefTables){
  
  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::init() start"));
  #endif

  this->allocateRefTables = allocateRefTables;
  
  // Allocate memory for 1D dynamic arrays
  X = new double[x_cnt];
  V = new double[x_cnt];
  Y = new double[y_cnt];

  //init 1D dynamic arrays
  for(byte i = 0; i < x_cnt; i++){
    X[i] = 0.0;
    V[i] = 0.0;
  }

  for(byte i = 0; i < y_cnt; i++){
    Y[i] = 0.0;
  }

  // If 2D data arrays was to be dynamically allocated here
  if(allocateRefTables){
    
    // Allocate 2D array S and init it elements with zeros
    if(s_cnt > 0){
      
      S = new double*[s_cnt];
      
      for(byte i = 0; i < s_cnt; i++){
        S[i] = new double[BRN_S_COL_CNT];
      }

      for(byte i = 0; i < s_cnt; i++){        
        for(byte j = 0; j < BRN_S_COL_CNT; j++){
          S[i][j] = 0.0;
        }
      }    
    }
    else{
      S = NULL;    
    }

    // Allocate 2D array M and init it elements with zeros
    if(mpp_cnt > 0){
      
      M = new int*[mpp_cnt];
      
      for(byte i = 0; i < mpp_cnt; i++){
        M[i] = new int[BRN_M_COL_CNT];
      }

      for(byte i = 0; i < mpp_cnt; i++){
        for(byte j = 0; j < BRN_M_COL_CNT; j++){
          M[i][j] = 0;
        }
      }    
    }
    else{
      M = NULL;    
    }

    // Allocate 2D array M and init it elements with zeros
    if(c_cnt > 0){
      
      C = new byte*[c_cnt];
      for(byte i = 0; i < c_cnt; i++){
        C[i] = new byte[BRN_C_COL_CNT];
      }

      for(byte i = 0; i < c_cnt; i++){
        for(byte j = 0; j < BRN_C_COL_CNT; j++){
          C[i][j] = 0;
        }
      }    
    }
    else{
      C = NULL;    
    }
  }
  else{
    // keep pointers cleared (they will be initialised by other methos)
    S = NULL;
    M = NULL;
    C = NULL;
  } // end if(allocateRefTables)
                       
  
  // Allocate 2D array MPGEN and init it elements with zeros
  if(mpp_cnt > 0){
    
    MPGEN = new int*[mpp_cnt];
    
    for(byte i = 0; i < mpp_cnt; i++){
      MPGEN[i] = new int[mps_cnt];
    }

    for(byte i = 0; i < mpp_cnt; i++){
      for(byte j = 0; j < mps_cnt; j++){
        MPGEN[i][j] = 0;
      }
    }    
  }
  else{
    MPGEN = NULL;    
  }
      
  // Allocate 2D array MPSEL and init it elements with zeros
  if(mpp_cnt > 0){
    
    MPSEL = new int*[mpp_cnt];
    
    for(byte i = 0; i < mpp_cnt; i++){
      MPSEL[i] = new int[mps_cnt];
    }

    for(byte i = 0; i < mpp_cnt; i++){
      for(byte j = 0; j < mps_cnt; j++){
        MPSEL[i][j] = 0;
      }
    }    
  }
  else{
    MPSEL = NULL;    
  }
}


// -----------------------------------------------------------------------------------------------------------------------
// // Settings BEFORE Initialisation init()
// -----------------------------------------------------------------------------------------------------------------------
void RBrain::setCntXY(byte x_cnt, byte y_cnt){
  
  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::setCntXY() start"));
  #endif
  
  this->x_cnt = x_cnt;
  this->y_cnt = y_cnt;

  #ifdef BRN_DEBUG_MODE
    Serial.print(F("this->x_cnt = "));
    Serial.print(this->x_cnt);
    Serial.print(F("; this->y_cnt = "));
    Serial.println(this->y_cnt);
  #endif
}


// -----------------------------------------------------------------------------------------------------------------------
void RBrain::setCntS(byte s_cnt){
  
  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::setCntS() start"));
  #endif
  
  this->s_cnt = s_cnt;

  #ifdef BRN_DEBUG_MODE
    Serial.print(F("this->s_cnt = "));
    Serial.println(this->s_cnt);
  #endif
}


// -----------------------------------------------------------------------------------------------------------------------
void RBrain::setMotProgGenerator(byte mpp_cnt, byte mps_cnt)
{ 
  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::setMotProgGenerator() start"));
  #endif
  
  this->mpp_cnt = mpp_cnt;
  this->mps_cnt = mps_cnt;

  #ifdef BRN_DEBUG_MODE
    Serial.print(F("this->mpp_cnt = "));
    Serial.print(this->mpp_cnt);
    Serial.print(F("; this->mps_cnt = "));
    Serial.println(this->mps_cnt);
  #endif
}


// -----------------------------------------------------------------------------------------------------------------------
void RBrain::setMotProgSelector(double thr1, double thr2, int att_cnt){

  // Arguments:
  //  - thr1 - first threshold for outcome rating score (save motor programm if predicted outcome rating is above this threshold)
  //  - thr2 - second threshold for outcome rating score (select motor programm if predicted environment rating is above this threshold)
  //  - att_cnt - number of attempts to generate motor programm
 
  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::setMotProgSelector() start"));
  #endif
  
  this->outcome_threshold_1 = thr1;
  this->outcome_threshold_2 = thr2;
  this->att_cnt = att_cnt;

  #ifdef BRN_DEBUG_MODE
    Serial.print(F("this->outcome_threshold_1 = "));
    Serial.print(this->outcome_threshold_1);
    Serial.print(F("; this->outcome_threshold_2 = "));
    Serial.print(this->outcome_threshold_2);
    Serial.print(F("; this->att_cnt = "));
    Serial.println(this->att_cnt);
  #endif
}


// -----------------------------------------------------------------------------------------------------------------------
void RBrain::setCntC(byte c_cnt){
    
  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::setCntC() start"));
  #endif
  
  this->c_cnt = c_cnt;

  #ifdef BRN_DEBUG_MODE
    Serial.print(F("this->c_cnt = "));
    Serial.println(this->c_cnt);
  #endif
}


// -----------------------------------------------------------------------------------------------------------------------
// Settings AFTER Initialisation init()
// -----------------------------------------------------------------------------------------------------------------------
void RBrain::addNN(NN* ptrNN){
 
  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::addNN() start"));
  #endif
  
  // pointer to NN
  brNN = ptrNN;
}


// -----------------------------------------------------------------------------------------------------------------------
void RBrain::addTableS(double arr[][BRN_S_COL_CNT], byte s_cnt){
  
  // Description:
  //  - Attaches external table S (if table S was not allocated during RBrain object initialisation)
  //
  // Purpose/Function:
  //  - Save addresses of arr[][] rows into array of pointers (double pointer) S
  //  - Save numer of rows 
  //
  // Arguments:
  //  - arr[][BRN_S_COL_CNT] - external table S
  //  - s_cnt - numer of rows 
  
  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::addTableS() start"));
  #endif
  
  this->s_cnt = s_cnt; 
  
  S = new double*[s_cnt];     // Array of pointers

  // for each pointer in the array
  for(byte row = 0; row < s_cnt; row++){
    // initialise it with address of first element of static array's row
    S[row] = &arr[row][0];
  }
}


// -----------------------------------------------------------------------------------------------------------------------
void RBrain::addTableM(int arr[][BRN_M_COL_CNT], byte mpp_cnt){
  
  // Description:
  //  - Attaches external table M (if own table M was not allocated during RBrain object initialisation)
  //
  // Purpose/Function:
  //  - Save addresses of arr[][] rows into array of pointers (double pointer) M
  //  - Save numer of rows 
  //
  // Arguments:
  //  - arr[][BRN_M_COL_CNT] - external table M
  //  - m_cnt - numer of rows 
  
  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::addTableM() start"));
  #endif

  this->mpp_cnt = mpp_cnt; 

  M = new int*[mpp_cnt];    // Array of pointers

  // for each pointer in the array
  for(byte row = 0; row < mpp_cnt; row++){
    // initialise it with address of first element of static array's row
    M[row] = &arr[row][0];
  }
}


// -----------------------------------------------------------------------------------------------------------------------
void RBrain::addTableC(byte arr[][BRN_C_COL_CNT], byte c_cnt){
  
  // Description:
  //  - Attaches external table C (if own table C was not allocated during RBrain object initialisation)
  //
  // Purpose/Function:
  //  - Save addresses of arr[][] rows into array of pointers (double pointer) C
  //  - Save numer of rows 
  //
  // Arguments:
  //  - arr[][BRN_C_COL_CNT] - external table C
  //  - c_cnt - numer of rows 
  
  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::addTableC() start"));
  #endif
  
  this->c_cnt = c_cnt;  // Array of pointers
  
  C = new byte*[c_cnt];
  
  // for each pointer in the array
  for(byte row = 0; row < c_cnt; row++){
    // initialise it with address of first element of static array's row
    C[row] = &arr[row][0];
  }
}


// -----------------------------------------------------------------------------------------------------------------------
void RBrain::fillRowS(byte s_idx, byte nnio_ref, 
                      double min_val, double max_val, 
                      double lbd_val, double ubd_val, double wgt_val, 
                      double rec_val, double vrt_val)
{
  // Description:
  //  - Fill row #s_idx in table S
  
  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::fillRowS() start"));
  #endif

  // if row index is in range
  if((s_idx >= 0) && (s_idx < s_cnt)){
    S[s_idx][BRN_S_COL_NNIO] = (double)nnio_ref;
    S[s_idx][BRN_S_COL_MIN] = min_val;
    S[s_idx][BRN_S_COL_MAX] = max_val;
    S[s_idx][BRN_S_COL_LBD] = lbd_val;
    S[s_idx][BRN_S_COL_UBD] = ubd_val;
    S[s_idx][BRN_S_COL_WGT] = wgt_val;
    S[s_idx][BRN_S_COL_REC] = rec_val;
    S[s_idx][BRN_S_COL_VRT] = vrt_val;
  }

  #ifdef BRN_DEBUG_MODE
    Serial.print(F("Sensor input #"));
    Serial.println(s_idx);
    RBrain::dispS();
  #endif
}


// -----------------------------------------------------------------------------------------------------------------------
void RBrain::fillRowM(byte mpp_idx, byte nnio_ref, int min_val, int max_val){
  
  // Description:
  //  - Fill row #mpp_idx in table M

  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::fillRowM() start"));
  #endif

  // if row index is in range
  if((mpp_idx >= 0) && (mpp_idx < mpp_cnt)){
    M[mpp_idx][BRN_M_COL_NNIO] = nnio_ref;
    M[mpp_idx][BRN_M_COL_MIN] = min_val;
    M[mpp_idx][BRN_M_COL_MAX] = max_val;
  }

  #ifdef BRN_DEBUG_MODE
    Serial.print(F("Motor programm paramente #"));
    Serial.print(mpp_idx);
    RBrain::dispM();
  #endif
}
    

// -----------------------------------------------------------------------------------------------------------------------
void RBrain::fillRowC(byte c_idx, byte nnio_ref, byte l_idx, byte n_idx){
  
  // Description:
  //  - Fill row #c_idx in table C
  
  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::fillRowC() start"));
  #endif

  // if row index is in range
  if((c_idx >= 0) && (c_idx < c_cnt)){
    C[c_idx][BRN_C_COL_NNIO] = nnio_ref;
    C[c_idx][BRN_C_COL_IDL] = l_idx;
    C[c_idx][BRN_C_COL_IDN] = n_idx;   
  }

  #ifdef BRN_DEBUG_MODE
    Serial.print(F("Context neuron #"));
    Serial.println(c_idx);
    RBrain::dispC();
  #endif  
}


// -----------------------------------------------------------------------------------------------------------------------
// Getters / Setters
// -----------------------------------------------------------------------------------------------------------------------
double RBrain::getelX(byte n){
  return X[n];
}

void RBrain::setelX(double x, byte n){
  X[n] = x;
}


// -----------------------------------------------------------------------------------------------------------------------
double RBrain::getelY(byte n){
  return Y[n];
}

void RBrain::setelY(double y, byte n){
  Y[n] = y;
}


// -----------------------------------------------------------------------------------------------------------------------
double RBrain::getelS(byte row, byte col){
  return S[row][col];
}

void RBrain::setelS(double val, byte row, byte col){
  S[row][col] = val;
}

double RBrain::getRecS(byte s_idx){
  return S[s_idx][BRN_S_COL_REC];
}

void RBrain::setRecS(double val, byte s_idx){
  S[s_idx][BRN_S_COL_REC] = val;
}


// -----------------------------------------------------------------------------------------------------------------------
double RBrain::getelM(byte row, byte col){
  return M[row][col];
}

void RBrain::setelM(double val, byte row, byte col){
  M[row][col] = val;
}


// -----------------------------------------------------------------------------------------------------------------------
byte RBrain::getCntC(){
  return c_cnt;
}

byte RBrain::getNnIORefC(byte c_idx){
  return C[c_idx][BRN_C_COL_NNIO];
}

double RBrain::getValC(byte c_idx){
  return brNN->getA(C[c_idx][BRN_C_COL_IDL], C[c_idx][BRN_C_COL_IDN]);
}

void RBrain::setValC(double val, byte c_idx){
  brNN->setA(val, C[c_idx][BRN_C_COL_IDL], C[c_idx][BRN_C_COL_IDN]);
}


// -----------------------------------------------------------------------------------------------------------------------
int RBrain::getMotProgParam(byte mpp_idx, byte mps_idx){
  return MPSEL[mpp_idx][mps_idx];
}

void RBrain::setMotProgParam(double val, byte mpp_idx, byte mps_idx){
  MPSEL[mpp_idx][mps_idx] = val;
}


// -----------------------------------------------------------------------------------------------------------------------
// Math
// -----------------------------------------------------------------------------------------------------------------------    
double RBrain::mapVal(double x, double x_min, double x_max, double y_min, double y_max){
 
 // same as Arduino map() function, but for double type 
 // Formula here:
 // https://docs.arduino.cc/language-reference/en/functions/math/map/

  return (x - x_min) * (y_max - y_min) / (x_max - x_min) + y_min;
}


// ----------------------------------------------------------------------------------------------------------------------- 
// Data conditioning and encoding/decoding
// -----------------------------------------------------------------------------------------------------------------------  
void RBrain::encSensRdsToNnIOArr(byte dest_arr_id){
  
  // Description:
  // Encodes all actual sensors records from table S[][] column BRN_S_COL_REC into one of NN's in/out data arrays specified by dest_arr_id;

  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::encSensRdsToNnIOArr(() start"));
  #endif

  // traverse actual sensors records
  for(byte i = 0; i < s_cnt; i++){    
    RBrain::encValToNnIOArr(S[i][BRN_S_COL_REC], S[i][BRN_S_COL_MIN], S[i][BRN_S_COL_MAX], dest_arr_id, S[i][BRN_S_COL_NNIO]);     
  } 

  #ifdef BRN_DEBUG_MODE
    RBrain::dispS();
  #endif
}

// ----------------------------------------------------------------------------------------------------------------------- 
void RBrain::encMotProgToNnIOArr(byte src_arr_id, byte dest_arr_id, byte mps_idx){

  // Description:
  // Encodes motor programm parameters (only one step specified by mps_idx) from tables MPGEN or MPSEL into one of NN's input data arrays specified by dest_arr_id;
  //
  // Arguments:
  // - src_arr_id  - id of source array
  // - dest_arr_id - id of destination array
  // - mps_idx - index of motor programm step (idx of column in source array)

  
  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::encodeArrM start"));
  #endif

  switch(src_arr_id){
    
    case BRN_MPGEN_ID:
      #ifdef BRN_DEBUG_MODE
        Serial.println(F(" - Encoding data from MPGEN"));
      #endif

      for(byte mpp_idx = 0; mpp_idx < mpp_cnt; mpp_idx++){
        
        RBrain::encValToNnIOArr(MPGEN[mpp_idx][mps_idx], M[mpp_idx][BRN_M_COL_MIN], M[mpp_idx][BRN_M_COL_MAX], dest_arr_id, M[mpp_idx][BRN_M_COL_NNIO]); 
      }
    break;

    case BRN_MPSEL_ID:
      #ifdef BRN_DEBUG_MODE
        Serial.println(F(" - Encoding data from MPSEL"));
      #endif

      for(byte mpp_idx = 0; mpp_idx < mpp_cnt; mpp_idx++){

        RBrain::encValToNnIOArr(MPSEL[mpp_idx][mps_idx], M[mpp_idx][BRN_M_COL_MIN], M[mpp_idx][BRN_M_COL_MAX], dest_arr_id, M[mpp_idx][BRN_M_COL_NNIO]); 
      }
    break;
  }
}

// ----------------------------------------------------------------------------------------------------------------------- 
void RBrain::encValToNnIOArr(double val, double val_min, double val_max, byte dest_arr_id, byte dest_el_idx){
  // Description:
  // Encodes arbitary value into arbitary NN's in/out array:
  //    - maps val to domain of corresponding activation function
  //    - apply activation function to mapped value
  //    - save calculated value to corresponding NN's input/output data array
  //
  // Arguments:
  //  - val - oroginal value to be encoded
  //  - val_min - lower bound of original value range
  //  - val_max - upper bound of original value range
  //  - dest_arr_id - id of destination array
  //  - dest_el_idx - index of element in destination array
  //
  // Return:
  //  - N/A

  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::encValToNnIOArr() start"));
  #endif

  switch(dest_arr_id){
    
    // encode into array X
    case BRN_X_ID:
      X[dest_el_idx] = brNN->fun_act_H(RBrain::mapVal(val, val_min, val_max, brNN->getHLDomMin(), brNN->getHLDomMax()));
      #ifdef BRN_DEBUG_MODE
        Serial.print(F(" - encoded to X["));
        Serial.print(dest_el_idx);
        Serial.print(F("]: "));
        Serial.print(val);
        Serial.print(F(" -> "));
        Serial.println(X[dest_el_idx]);
      #endif  
    break;

    // encode into array V
    case BRN_V_ID:
      V[dest_el_idx] = brNN->fun_act_H(RBrain::mapVal(val, val_min, val_max, brNN->getHLDomMin(), brNN->getHLDomMax())); 
      #ifdef BRN_DEBUG_MODE
        Serial.print(F(" - encoded to V["));
        Serial.print(dest_el_idx);
        Serial.print(F("]: "));
        Serial.print(val);
        Serial.print(F(" -> "));
        Serial.println(V[dest_el_idx]);
      #endif  
    break;

    // encode into NN output layer
    case BRN_O_ID:
      brNN->setA(brNN->fun_act_O(RBrain::mapVal(val, val_min, val_max, brNN->getOLDomMin(), brNN->getOLDomMax())),brNN->getCntL() - 1, dest_el_idx);
      #ifdef BRN_DEBUG_MODE
        Serial.print(F(" - encoded to NN's O["));
        Serial.print(dest_el_idx);
        Serial.print(F("]: "));
        Serial.print(val);
        Serial.print(F(" -> "));
        Serial.println(brNN->getA(brNN->getCntL() - 1, dest_el_idx));
      #endif  
    break;

    // encode into array Y
    case BRN_Y_ID:
      Y[dest_el_idx] = brNN->fun_act_O(RBrain::mapVal(val, val_min, val_max, brNN->getOLDomMin(), brNN->getOLDomMax()));
      #ifdef BRN_DEBUG_MODE
        Serial.print(F(" - encoded to Y["));
        Serial.print(dest_el_idx);
        Serial.print(F("]: "));
        Serial.print(val);
        Serial.print(F(" -> "));
        Serial.println(Y[dest_el_idx]);
      #endif
    break;

    default:
      // Do nothing
      #ifdef BRN_DEBUG_MODE
        Serial.println(F(" - N/A "));
      #endif
    break;
  } 
}


// ----------------------------------------------------------------------------------------------------------------------- 
double RBrain::decValFromNnIOArr(double val_min, double val_max, byte src_arr_id, byte scr_el_idx){
  
  // Description:
  // Decodes arbitary value from arbitary NN's in/out array 
  //    - applly inverse of corresponding activation function to encoded value
  //    - map inversed value from activation function domain to corresponding parameter range
  //
  // Arguments:
  //  - val_min - lower bound of original value range
  //  - val_max - upper bound of original value range
  //  - src_arr_id - id of source array
  //  - scr_el_idx - index of element in source array to decode from
  //
  // Return:
  //  - decoded value

   

  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::decValFromNnIOArr() start"));
    Serial.print(F(" - ready to decode value with idx = "));
    Serial.print(scr_el_idx);
    Serial.print(F(" from "));
  #endif
  
  switch(src_arr_id){
    
    case BRN_X_ID:
      #ifdef BRN_DEBUG_MODE
        Serial.print(F("X["));
        Serial.print(scr_el_idx);
        Serial.print(F("] = "));
        Serial.print(X[scr_el_idx]);
        Serial.println(F(" -> ... "));
      #endif
      return RBrain::mapVal(brNN->fun_inv_H(X[scr_el_idx]), brNN->getHLDomMin(), brNN->getHLDomMax(), val_min, val_max);
    break;

    case BRN_V_ID:
      #ifdef BRN_DEBUG_MODE
        Serial.print(F("V["));
        Serial.print(scr_el_idx);
        Serial.print(F("] = "));
        Serial.print(V[scr_el_idx]);
        Serial.println(F(" -> ... "));
      #endif
      return RBrain::mapVal(brNN->fun_inv_H(V[scr_el_idx]), brNN->getHLDomMin(), brNN->getHLDomMax(), val_min, val_max);
    break;

    case BRN_O_ID:
      #ifdef BRN_DEBUG_MODE
        Serial.print(F("NN's O["));
        Serial.print(scr_el_idx);
        Serial.print(F("] = "));
        Serial.print(brNN->getA(brNN->getCntL() - 1, scr_el_idx));
        Serial.println(F(" -> ... "));
      #endif
      return RBrain::mapVal(brNN->fun_inv_O(brNN->getA(brNN->getCntL() - 1, scr_el_idx)), brNN->getOLDomMin(), brNN->getOLDomMax(), val_min, val_max);
    break;

    case BRN_Y_ID:
      #ifdef BRN_DEBUG_MODE
        Serial.print(F("Y["));
        Serial.print(scr_el_idx);
        Serial.print(F("] = "));
        Serial.print(Y[scr_el_idx]);
        Serial.println(F(" -> ... "));
      #endif
      return RBrain::mapVal(brNN->fun_inv_O(Y[scr_el_idx]), brNN->getOLDomMin(), brNN->getOLDomMax(), val_min, val_max);
    break;

    default:
      // Do nothing
      #ifdef BRN_DEBUG_MODE
        Serial.println(F(" - N/A "));
      #endif
    break;
  }
}


// -----------------------------------------------------------------------------------------------------------------------    
// Core
// -----------------------------------------------------------------------------------------------------------------------    
double RBrain::evalRawSensData(byte s_col_id){
  
  // Description:
  //  - Core function, change its logic to change robot's behavior.
  //    Calculates outcome rating score according to sensors readings.
  //
  // Purpose/Function:
  //  - Calculate outcome rating score for actual of virt/temp sensors readings stored in table S
  //
  // Arguments:
  //  - s_col_id - id of column in table S (column with actual readings or column with virt/temp readings
  //
  // Return:
  //  - oucome rating score


  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::BRN_MPP_INTERVALS_CNT() start"));
  #endif
  
  double score = 0;

  #ifdef BRN_DEBUG_MODE
    Serial.println(F(" - checking sensor#"));
  #endif

  // check all sensors readings in table S
  for(byte s_idx = 0; s_idx < s_cnt; s_idx++){
    #ifdef BRN_DEBUG_MODE
      Serial.print(s_idx);
    #endif
    
    // if sensor readings is in allowed range
    if((S[s_idx][s_col_id] >= S[s_idx][BRN_S_COL_LBD]) && (S[s_idx][BRN_S_COL_REC] <= S[s_idx][BRN_S_COL_UBD])){
      #ifdef BRN_DEBUG_MODE
        Serial.println(F(" - OK! in range!"));
      #endif
      // add weight of sensor reading
      score += S[s_idx][BRN_S_COL_WGT];
    }
    else{
      // do nothing
      #ifdef BRN_DEBUG_MODE
        Serial.println(F(" - not in range..."));
      #endif
    }
  }

  #ifdef BRN_DEBUG_MODE
    Serial.print(F(" - return value = "));
    Serial.println(score);
  #endif

  return score;
}


// -----------------------------------------------------------------------------------------------------------------------  
double RBrain::evalEncSensData(byte src_arr_id){

  // Description:
  //  - Calculate outcome rating score using encoded values of sensor's readings tored in one of NN's in/out data arrays
  //    - decode values into virt/temp sensors readings column in table S
  //    - apply BRN_MPP_INTERVALS_CNT() function for values in virt/temp sensors readings column in table S
  //
  // Arguments:
  //  - src_arr_id - id of source array with encoded sensors readings
  //
  // Return:
  //  - outcome rating score

  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::evalEncSensData() start"));
  #endif

  // Decode encoded values from NN's array into table S
  #ifdef BRN_DEBUG_MODE
    Serial.print(F(" - decoded raw data "));
  #endif 
  
  for(byte s_idx = 0; s_idx < s_cnt; s_idx++){

    S[s_idx][BRN_S_COL_VRT] = decValFromNnIOArr(S[s_idx][BRN_S_COL_MIN], S[s_idx][BRN_S_COL_MAX], src_arr_id, S[s_idx][BRN_S_COL_NNIO]); 
    
    #ifdef BRN_DEBUG_MODE
      Serial.print(F(" S["));
      Serial.print(s_idx);
      Serial.print(F("]["));
      Serial.print(BRN_S_COL_VRT);
      Serial.print(F("]="));
      Serial.println(S[s_idx][BRN_S_COL_VRT]);
    #endif
  }
  
 
  #ifdef BRN_DEBUG_MODE
    Serial.println(F(" - ready to call RBrain::evalRawSensData()"));
  #endif

  return RBrain::evalRawSensData(BRN_S_COL_VRT);
}


// -----------------------------------------------------------------------------------------------------------------------    
double RBrain::evalActRawSensData(){
  
  // Description:
  //  - Calculate outcome rating score for actual sensore readings stored in table S 
  //
  // Arguments:
  //  - src_arr_id - id of source array with encoded sensors readings
  //
  // Return:
  //  - outcome rating score
  
  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::evalActRawSensData() start"));
  #endif

  return RBrain::evalRawSensData(BRN_S_COL_REC);
}

// -----------------------------------------------------------------------------------------------------------------------    
void RBrain::generateMotProg(){
  
  // Description:
  //  - Motor programm generator
  //
  // Purpose/Function:
  //  - Generates (by random) motor proramm according to settings in table M and save it into table MPGEN
  //    (also can generate multi-step motor programm)

  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::generateMotProg() start"));
  #endif
  
  randomSeed(millis());

  // for each motor programm parameter
  for(byte mpp_idx = 0; mpp_idx < mpp_cnt; mpp_idx++){
    //for each programm step
    for(byte mps_idx = 0; mps_idx < mps_cnt; mps_idx++){
      
      //Generated motor programm parameter between allowed min and max values
      //MPGEN[mpp_idx][mps_idx] = (int)random(M[mpp_idx][BRN_M_COL_MIN], M[mpp_idx][BRN_M_COL_MAX] + 1);    

      // same but allowed range divided to BRN_MPPLEVELS_CNT levels
      MPGEN[mpp_idx][mps_idx] = map((int)random(BRN_MPP_INTERVALS_CNT + 1), 0, BRN_MPP_INTERVALS_CNT, M[mpp_idx][BRN_M_COL_MIN], M[mpp_idx][BRN_M_COL_MAX]);
    }    
  }

  #ifdef BRN_DEBUG_MODE
    Serial.println(F(" - generated programm:"));
    RBrain::dispMPGEN();
  #endif
}


// -----------------------------------------------------------------------------------------------------------------------  
double RBrain::evalMPGEN(){

  
  // "Virtual maneuver"
  // For each motprog step
  for(byte mps_idx = 0; mps_idx < mps_cnt; mps_idx++){

    // Copy current motor programm step commands from MPGEN to V
    RBrain::encMotProgToNnIOArr(BRN_MPGEN_ID, BRN_V_ID, mps_idx);

    // Predicted outcome for maneuver step
    // propagate NN with updated V
    brNN->propagate_network(V);

    // Copy predicted sensors data to virtual/temporal NN input array
    // (predicted outputs will be inputs for next step)
    for(byte s_idx = 0; s_idx < s_cnt; s_idx++){        
      V[(byte)S[s_idx][BRN_S_COL_NNIO]] = brNN->getA(brNN->getCntL() - 1, (byte)S[s_idx][BRN_S_COL_NNIO]);
    }
      
    // context neurones feedback
    if(c_cnt > 0){
      RBrain::copyContNodesToV();
    }    
  }

  return RBrain::evalEncSensData(BRN_O_ID);
}


// -----------------------------------------------------------------------------------------------------------------------  
bool RBrain::proposeMotProg(){
  
  // Description:
  //  - Motor programm selector
  //
  // Purpose/Function:
  //  - Generate motor programms and check them with NN. Select motor programm for which NN predicts good outcome rating score:
  //    - Call motor programm generator (Generated motor programm will be sored in table MPGEN)
  //    - Copy/encode sensors readings, context neurones and generated motor programm into NN's "virtual" input array V
  //    - Run NN and calculate outcome rating score for predicted sensors readings
  //    - For multistep motor programm: copy predicted sensors readings back to virtual input V, 
  //      encode next programm step to V, feddback context neurones into V and repeat for all programm steps.
  //    - repeat untill good motor programm fount or untill exceed number of attempts
  //    - save selected motor programm into MPSEL table (or save last one, if no good programm found)
  //
  // Arguments:
  //  - N/A
  //
  // Return:
  //  - N/A

  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::proposeMotProg() start"));
  #endif
  
  //flag wether good/candidate motprog found
  bool found = false;
  double eval_result = 0;
  
  // For each attempt
  for(int a = 0; a < att_cnt; a++){
    
    #ifdef BRN_DEBUG_MODE
      Serial.print(F(" - attempt #"));
      Serial.println(a);
    #endif
    
    // copy actual NN input array to virtual/temporal NN input array
    // --------------------------------------------------
    for(int n = 0; n < x_cnt; n++){
      V[n] = X[n];
    }

    #ifdef BRN_DEBUG_MODE
      RBrain::dispX();
      RBrain::dispV();
    #endif

    // generate random motprog (updates MPGEN 2D array)
    // --------------------------------------------------
    RBrain::generateMotProg();

    #ifdef BRN_DEBUG_MODE
      RBrain::dispMPGEN();
    #endif

    eval_result = RBrain::evalMPGEN();
   
    if(eval_result >= outcome_threshold_2){
      // good notor programm found! 
      // save current programm to MPSEL and finish

      #ifdef BRN_DEBUG_MODE
        Serial.println(F(" - Good programm found!"));
      #endif

      found = true;

      RBrain::copyMPGENtoMPSEL();
      
      break;  // break attempts    
    }

    if(eval_result >= outcome_threshold_1){
      // candidate programm found, 
      // save current programm as selected programm but continue attempts.
      
      #ifdef BRN_DEBUG_MODE
        Serial.println(F(" - Candidate programm found"));
      #endif

      found = true;  

      RBrain::copyMPGENtoMPSEL();              
    }    
  } // End for: attempts

  // if no good or even no candidate programm found, save and propose last generated programm
  if(found == false){
    #ifdef BRN_DEBUG_MODE
      Serial.println(F(" - No programm found, using last one"));
    #endif

    RBrain::copyMPGENtoMPSEL();
  }  

  #ifdef BRN_DEBUG_MODE
    Serial.print(F(" - MPGEN after selection"));
    RBrain::dispMPGEN();
    Serial.print(F(" - MPSEL after selection"));
    RBrain::dispMPSEL();
  #endif

  return found;

}

/*
// -----------------------------------------------------------------------------------------------------------------------  
bool RBrain::proposeMotProg(){
  
  // Description:
  //  - Motor programm selector
  //
  // Purpose/Function:
  //  - Generate motor programms and check them with NN. Select motor programm for which NN predicts good outcome rating score:
  //    - Call motor programm generator (Generated motor programm will be sored in table MPGEN)
  //    - Copy/encode sensors readings, context neurones and generated motor programm into NN's "virtual" input array V
  //    - Run NN and calculate outcome rating score for predicted sensors readings
  //    - For multistep motor programm: copy predicted sensors readings back to virtual input V, 
  //      encode next programm step to V, feddback context neurones into V and repeat for all programm steps.
  //    - repeat untill good motor programm fount or untill exceed number of attempts
  //    - save selected motor programm into MPSEL table (or save last one, if no good programm found)
  //
  // Arguments:
  //  - N/A
  //
  // Return:
  //  - N/A

  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::proposeMotProg() start"));
  #endif
  
  //flag wether good/candidate motprog found
  bool found = false;
  
  // For each attempt
  for(int a = 0; a < att_cnt; a++){
    
    #ifdef BRN_DEBUG_MODE
      Serial.print(F(" - attempt #"));
      Serial.println(a);
    #endif
    
    // copy actual NN input array to virtual/temporal NN input array
    // --------------------------------------------------
    for(int n = 0; n < x_cnt; n++){
      V[n] = X[n];
    }

    #ifdef BRN_DEBUG_MODE
      RBrain::dispX();
      RBrain::dispV();
    #endif

    // generate random motprog (updates MPGEN 2D array)
    // --------------------------------------------------
    RBrain::generateMotProg();

    #ifdef BRN_DEBUG_MODE
      RBrain::dispMPGEN();
    #endif

    // Check current motprog 
    // ----------------------
    // For each motprog step
    for(byte mps_idx = 0; mps_idx < mps_cnt; mps_idx++){

      // Copy current motor programm step commands from MPGEN to V
      RBrain::encMotProgToNnIOArr(BRN_MPGEN_ID, BRN_V_ID, mps_idx);

      #ifdef BRN_DEBUG_MODE
        Serial.print(F(" - V[] before propagation"));
        RBrain::dispV();
      #endif
        
      // propagate NN with updated V
      brNN->propagate_network(V);

      // Copy predicted sensors data to virtual/temporal NN input array
      // (predicted outputs will be inputs for next step)
      for(byte s_idx = 0; s_idx < s_cnt; s_idx++){        
        V[(byte)S[s_idx][BRN_S_COL_NNIO]] = brNN->getA(brNN->getCntL() - 1, (byte)S[s_idx][BRN_S_COL_NNIO]);
      }
        
      // context neurones feedback
      if(c_cnt > 0){
        RBrain::copyContNodesToV();
      }    

      #ifdef BRN_DEBUG_MODE
        Serial.print(F(" - V[] after feedback of conext and sensors"));
        RBrain::dispV();
      #endif
    }

    // restore values of context nodes in NN as it was
    // before motor programm generation and evaluation
    // restore "real memories", do not "remember"
    // "virtual" move (Not necessary - activation values updates each cycle
    // and don't remember previous values)
    for(int i = 0; i < c_cnt; i++){
      RBrain::setValC(X[RBrain::getNnIORefC(i)], i); 
    }

    // evaluate sensors readings predicted by NN
    #ifdef BRN_DEBUG_MODE
      Serial.print(F(" - MPGEN befre selection"));
      RBrain::dispMPGEN();
      Serial.print(F(" - MPSEL befre selection"));
      RBrain::dispMPSEL();
    #endif
    
    if(RBrain::evalEncSensData(BRN_O_ID) >= outcome_threshold_2){
      // good notor programm found! 
      // save current programm to MPSEL and finish

      #ifdef BRN_DEBUG_MODE
        Serial.println(F(" - Good programm found!"));
      #endif

      found = true;

      RBrain::copyMPGENtoMPSEL();
      
      break;  // break attempts    
    }

    if(RBrain::evalEncSensData(BRN_O_ID) >= outcome_threshold_1){
      // candidate programm found, 
      // save current programm as selected programm but continue attempts.
      
      #ifdef BRN_DEBUG_MODE
        Serial.println(F(" - Candidate programm found"));
      #endif

      found = true;  

      RBrain::copyMPGENtoMPSEL();              
    }    
  } // End for: attempts

  // if no good or even no candidate programm found, save and propose last generated programm
  if(found == false){
    #ifdef BRN_DEBUG_MODE
      Serial.println(F(" - No programm found, using last one"));
    #endif

    RBrain::copyMPGENtoMPSEL();
  }  

  #ifdef BRN_DEBUG_MODE
    Serial.print(F(" - MPGEN after selection"));
    RBrain::dispMPGEN();
    Serial.print(F(" - MPSEL after selection"));
    RBrain::dispMPSEL();
  #endif

  return found;

}
*/

// -----------------------------------------------------------------------------------------------------------------------  
void RBrain::convertMpgenDataPointsIdxToDataValues(){

  // MPGEN now is filled not with actual parameters values, but with indexws of data points
  // This function converts indexes of data points stored in MPGEN into actual values

  for(int mpp_idx = 0; mpp_idx < mpp_cnt; mpp_idx++){
    for(int mps_idx = 0; mps_idx < mps_cnt; mps_idx++){
      MPGEN[mpp_idx][mps_idx] = map((int)MPGEN[mpp_idx][mps_idx], 0, BRN_MPP_INTERVALS_CNT, M[mpp_idx][BRN_M_COL_MIN], M[mpp_idx][BRN_M_COL_MAX]);
    }
  }

}

// -----------------------------------------------------------------------------------------------------------------------  
bool RBrain::proposeMotProgBruteForce(){

  int datapoints_cnt = BRN_MPP_INTERVALS_CNT + 1;
  int min_datapoint_idx = 0;            // MPG data points idx range is from 0 to datapoints_cnt
  bool found = false;                   //flag: wether good/candidate motor programm programm found
  
  bool generate_next_combination = true;     // "Odomenter" algorithm - wether to generate next combination      
  int carry_index = 0;                  // variable to store current index in carry array

  // Idx[] is an index array idx[0..s-1] where Idx[k] is in [0, datapoints_cnt-1], and the actual value for cell k is min_datapoint_idx + Idx[k].
  int s = this->mpp_cnt*this->mps_cnt;
  int* Idx = new int[s];  // DONT FORGET TO DELETE              

  int generated_motprogs_cnt = 0;       // counter how much motor programs (combinations of parameters) has been generated

  int mpp_idx = 0;
  int mps_idx = 0;



  for(int i = 0; i < s; i++){
    Idx[i] = 0;
  }

  // Generate combinations
  // ---------------------
  while(generate_next_combination){

    // copy actual NN input array to virtual/temporal NN input array
    for(int n = 0; n < x_cnt; n++){
      V[n] = X[n];
    }


    // Fill table with new combination
    // -------------------------------
    for(int k = 0; k < s; k++){

      mpp_idx = k/this->mps_cnt;
      mps_idx = k%this->mps_cnt;

      MPGEN[mpp_idx][mps_idx] = min_datapoint_idx + Idx[k];    
    }

    generated_motprogs_cnt++;

    // proceed generated combination
    // -----------------------------
    if(generated_motprogs_cnt%BRN_BRUTE_MPGEN_COMB_STEP_SIZE == 0){

      RBrain::convertMpgenDataPointsIdxToDataValues();

      if(RBrain::evalMPGEN() >= outcome_threshold_2){
        // good programm found, 
        // save current programm as selected programm but continue attempts.
        found = true;
        RBrain::copyMPGENtoMPSEL();
        generate_next_combination = false;              
      }              
    }

    // increment odometer
    // ------------------
    carry_index = 0;

    while(carry_index < s){

      Idx[carry_index] = Idx[carry_index] + 1;

      if(Idx[carry_index] < datapoints_cnt){
        // no further carry 
        break;
      }

      Idx[carry_index] = 0;
      carry_index++;

      if(carry_index == s){
        // all combinations generated
        generate_next_combination = false;
        break;
      }
      
    } // end while(carry_index < s) 
  } // end while(continue_to_generate)

  delete[] Idx;
      
  return found;
}


// -----------------------------------------------------------------------------------------------------------------------  
void RBrain::learn(){
    
  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::learn() start"));
  #endif
  
  brNN->single_train(X, Y);

}


// -----------------------------------------------------------------------------------------------------------------------  
double RBrain::estimate(){

  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::estimate() start"));
  #endif

  brNN->propagate_network(X);
  
  return RBrain::evalEncSensData(BRN_O_ID);
}


// -----------------------------------------------------------------------------------------------------------------------  
// Aux
// -----------------------------------------------------------------------------------------------------------------------  
void RBrain::encSensRdsToX(){

  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::encSensRdsToX() start"));
  #endif

  RBrain::encSensRdsToNnIOArr(BRN_X_ID); 
}


// ----------------------------------------------------------------------------------------------------------------------- 
void RBrain::encMotProgToX(byte mps_idx){
  
  // Description:
  // Encodes selected motor programm parameters (one step) from table MPSEL into  NN's input data array X

  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::encodeMPSELtoX start"));
  #endif

  RBrain::encMotProgToNnIOArr(BRN_MPSEL_ID, BRN_X_ID, mps_idx);
  
  #ifdef BRN_DEBUG_MODE
    RBrain::dispMPSEL();
  #endif
}


// -----------------------------------------------------------------------------------------------------------------------  
void RBrain::encSensRdsToY(){

  // Description:
  //  - encodes actual sensors readingss from table S into table Y

  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::encSensRdsToY() start"));
  #endif

  RBrain::encSensRdsToNnIOArr(BRN_Y_ID);
}


// ----------------------------------------------------------------------------------------------------------------------- 
void RBrain::ccopyContNodesToNnInpLayer(){

  // Description:
  //  - copies context neurones activation values into corresponding neurones in NN's input layer

  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::ccopyContNodesToNnInpLayer() start"));
  #endif
  
  for(int i = 0; i < c_cnt; i++){
    // copy value of context neuron to corresponding neuron in input layer
    brNN->copy_node_activation(C[i][BRN_C_COL_IDL], C[i][BRN_C_COL_IDN], 0, C[i][BRN_C_COL_NNIO], 1);
  }
}


// ----------------------------------------------------------------------------------------------------------------------- 
void RBrain::copyContNodesToX(){
  
  // Description:
  //  - copies context neurones activation values into corresponding elements of NN's input data array X
  
  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::copyContNodesToX() start"));
  #endif
  
  for(int i = 0; i < c_cnt; i++){
    X[C[i][BRN_C_COL_NNIO]] = brNN->getA(C[i][BRN_C_COL_IDL], C[i][BRN_C_COL_IDN]);
  }  
}


// ----------------------------------------------------------------------------------------------------------------------- 
void RBrain::copyContNodesToV(){
  
  // Description:
  //  - copies context neurones activation values into corresponding elements of NN's virtual input data array V


  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::copyContNodesToV() start"));
  #endif
  
  for(int i = 0; i < c_cnt; i++){
    V[C[i][BRN_C_COL_NNIO]] = brNN->getA(C[i][BRN_C_COL_IDL], C[i][BRN_C_COL_IDN]);
  }
}


// ----------------------------------------------------------------------------------------------------------------------- 
void RBrain::copyYtoX(){
  
  // Description:
  //  - copy values from array Y into corresponding elements of array X

  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::copyYtoX() start"));
  #endif
  
  for(int n = 0; n < y_cnt; n++){
    if(n < x_cnt){
      X[n] = Y[n];
    }    
  }

  #ifdef BRN_DEBUG_MODE
    RBrain::dispX();
    RBrain::dispY();
  #endif
}


// ----------------------------------------------------------------------------------------------------------------------- 
void RBrain::copyMPGENtoMPSEL(){

  // Purpose/Function:
  //  - copy all values from taple MPGEN into table MPSEL


  #ifdef BRN_DEBUG_MODE
    Serial.println(F("BRN::copyMPGENtoMPSEL() start"));
  #endif

  // for each motor programm parameter
  for(byte mpp_idx = 0; mpp_idx < mpp_cnt; mpp_idx++){
    //for each motor programm step
    for(byte mps_idx = 0; mps_idx < mps_cnt; mps_idx++){
      MPSEL[mpp_idx][mps_idx] = MPGEN[mpp_idx][mps_idx];
    }
  }

  #ifdef BRN_DEBUG_MODE
    RBrain::dispMPGEN();
    RBrain::dispMPGEN();
  #endif
  
}

// ----------------------------------------------------------------------------------------------------------------------- 
void RBrain::dispNN(){
  
  // Description:
  //  - display NN's information on serial monitor

  
  #ifdef BRN_REPORT_MODE

    brNN->disp_NN();
  #endif 
}


// ----------------------------------------------------------------------------------------------------------------------- 
void RBrain::dispX(){
  
  #ifdef BRN_REPORT_MODE
    Serial.print(F("Data array X: "));
    for(int i = 0; i < x_cnt; i++){
      Serial.print(X[i]);
      Serial.print(F("; "));
    }
    Serial.println();
  #endif  
}


// ----------------------------------------------------------------------------------------------------------------------- 
void RBrain::dispV(){
  #ifdef BRN_REPORT_MODE   
    Serial.print(F("Data array V: "));
    
    for(int i = 0; i < x_cnt; i++){
      Serial.print(V[i]);
      Serial.print(F("; "));
    }
    Serial.println();
  #endif  
}


// ----------------------------------------------------------------------------------------------------------------------- 
void RBrain::dispY(){
  #ifdef BRN_REPORT_MODE    
    Serial.print(F("Data array Y: "));
    for(int i = 0; i < y_cnt; i++){
      Serial.print(Y[i]);
      Serial.print(F("; "));
    }
    Serial.println();
  #endif  
}


// ----------------------------------------------------------------------------------------------------------------------- 
void RBrain::dispS(){
  #ifdef BRN_REPORT_MODE  
    Serial.println(F("Data array S: "));
    for(int i = 0; i < s_cnt; i++){
      for(int j = 0; j < BRN_S_COL_CNT; j++){
        Serial.print(S[i][j]);
        Serial.print(F("; "));
      }
      Serial.println();
    }
  #endif 
}


// ----------------------------------------------------------------------------------------------------------------------- 
void RBrain::dispM(){
  
  #ifdef BRN_REPORT_MODE
    Serial.println(F("Data array M: "));
    for(int i = 0; i < mpp_cnt; i++){
      for(int j = 0; j < BRN_M_COL_CNT; j++){
        Serial.print(M[i][j]);
        Serial.print(F("; "));
      }
      Serial.println();
    }
  #endif 
}


// ----------------------------------------------------------------------------------------------------------------------- 
void RBrain::dispC(){
  
  #ifdef BRN_REPORT_MODE
    Serial.println(F("Data array C: "));
    for(int i = 0; i < c_cnt; i++){
      for(int j = 0; j < BRN_C_COL_CNT; j++){
        Serial.print(C[i][j]);
        Serial.print(F("; "));
      }
      Serial.println();
    }
  #endif 
}


// ----------------------------------------------------------------------------------------------------------------------- 
void RBrain::dispMPGEN(){
  
  #ifdef BRN_REPORT_MODE
    Serial.println(F("Data array MPGEN: "));
    for(int i = 0; i < mpp_cnt; i++){
      for(int j = 0; j < mps_cnt; j++){
        Serial.print(MPGEN[i][j]);
        Serial.print(F("; "));
      }
      Serial.println();
    }
  #endif
}


// ----------------------------------------------------------------------------------------------------------------------- 
void RBrain::dispMPSEL()
{
  #ifdef BRN_REPORT_MODE
    Serial.println(F("Data array MPSEL: "));
    for(int i = 0; i < mpp_cnt; i++){
      for(int j = 0; j < mps_cnt; j++){
        Serial.print(MPSEL[i][j]);
        Serial.print(F("; "));
      }
      Serial.println();
    }
  #endif    
  
}


// ----------------------------------------------------------------------------------------------------------------------- 
void RBrain::dispBRN()
{
  // Description:
  //  - 
  //
  // Purpose/Function:
  //  - display all RBrain object information on serial monitor
  //
  // Arguments:
  //  - N/A
  //
  // Return:
  //  - N/A
  // 
  #ifdef BRN_REPORT_MODE
    Serial.println();
    Serial.println(F("----------------"));
    Serial.println(F("dispBRN() start"));
    Serial.print(F("x_cnt = "));
    Serial.println(x_cnt);
    Serial.print(F("y_cnt = "));
    Serial.println(y_cnt);
    Serial.print(F("s_cnt = "));
    Serial.println(s_cnt);
    Serial.print(F("mpp_cnt = "));
    Serial.println(mpp_cnt);
    Serial.print(F("mps_cnt = "));
    Serial.println(mps_cnt);
    Serial.print(F("outcome_threshold_1 = "));
    Serial.println(outcome_threshold_1);
    Serial.print(F("outcome_threshold_2 = "));
    Serial.println(outcome_threshold_2);
    Serial.print(F("att_cnt = "));
    Serial.println(att_cnt);
    Serial.print(F("c_cnt = "));
    Serial.println(c_cnt);

    RBrain::dispX();
    RBrain::dispV();
    RBrain::dispY();
    RBrain::dispS();
    RBrain::dispM();
    RBrain::dispC();
    RBrain::dispMPGEN();
    RBrain::dispMPSEL();

    Serial.println(F("dispBRN() end"));
    Serial.println(F("----------------"));
    Serial.println();    
  #endif 
}
