/*
"BRAINS" FOR ARDUINO ROBOT
Class to integrate NN (neural network) class from "KB_NN_3.h" with Arduino robot

Version 4.6 - 2026-07-31
 - proposeMotProgBruteForce() algorithm modified.
   Now it is iterative "odometer" (carry) algorithm, idea proposed by github copilot[3]
 - evaluateMpgen() added
 - convertMpgenDataPointsIdxToDataValues() added

Version 4.5 - 2026-07-12
 - void estimate() added

Version 4.4 - 2026-07-04
 - proposeMotProg() niw return bool value (1 if good motprog found)
 - proposeMotProgBruteForce() added

Version 4.3 - 2026-06-11
 - Activation function domain definitions moved to NN class
 - optimized methods that use activation function data

Version 4.2 - 2026-05-30
 - Added BRN_MPP_INTERVALS_CNT parameter:
   generateMotProg() method now generates not continious values for motor programm parameters, but values of discrete levels
   For example, instead of -250,-249,-248,..., 248, 249, 250 possible values it will generate  -250, -200, -150, -100, -50, 0, 50, 100, 150, 200, 250

Version 4.1 - 2026-04-28
 - Implements arbitary motor programm steps (applied and learned one by one)
 - added one more column to S table
 - re-naming of methods and members
 - code optimisation
 

Version 4.0 - 2026-04-24
 - Implements arbitary motor programm steps (applied and one by one learned in parallel)
  but it needs to enlarge NN's input layer, also need to research how to do it correctly

Version 3.0 - 2026-04-23
 - flexible: can be tuned to any number of sensors, motor programm paramenters, context neurones

Version 2.3 - 2026-04-19
 - double evalRawSensData(byte irc, double usf, double usl, double usr);
 - double evalEncSensData(byte data_source);
   istead of only one double evalEncSensData(byte data_source);

Version 2.2 - 2026-04-14
 - indexes of context neurones stored  in array, context neurones managed automatically

Version 2.1 - 2026-04-12
 - Some changes into constructor

Version 2.0 - 2026-04-07 
 - Realised as class
 
Version 1.0 - 
 - Basic
 
Features
 - Implements storage of sensors readings and its encoding/decoding into/from NN's Input/output data arrays;
 - Implements evaluation of actual and NN predicted sensors readings - "outcome rating";
 - Implements multi-step motor programm generation, selection and storage;
 - Implements  encoding/decoding sensors of motor programm parameters into/from NN's Input arrays
 - Implements some kind of NN's reccurent behavior (can copy "context"/"memory" neurones activation values back to NN's input);

Refs
[1] J. L. Elman, “Finding Structure in Time,” Cognitive Science, vol. 14, no. 2, pp. 179–211, Mar. 1990, doi: 10.1207/s15516709cog1402_1.
    Accesed: April 03, 2026. [Online]. Available: 
    https://onlinelibrary.wiley.com/doi/10.1207/s15516709cog1402_1
[2] helasraizam, “Converting two dimensional array to double pointer,” cplusplus.com. Accessed: Apr. 27, 2026. 
    [Online]. Available: https://cplusplus.com/forum/beginner/43003/
[3] https://github.com/copilot/c/63c85496-1bad-4fef-83d7-36db903c479d

 */

#ifndef KB_RBrain_H
#define KB_RBrain_H

#include <Arduino.h>
#include "KB_NN_3.h"

// RUN MODE
// #define BRN_DEBUG_MODE
// #define BRN_REPORT_MODE


// ---------------------------------
// Data arrays/tables specifications
// ---------------------------------

// Table S ("Sensors data table") 
#define BRN_S_COL_CNT 8       // number of columns in table (2D array) S;
#define BRN_S_COL_NNIO 0      // alias of column that holds indexes of neural net i/o channels for corresponding sensors (insexes of elements in NN i/o arrays X/V/O/Y);
#define BRN_S_COL_MIN 1       // alias of column that holds sensors min detectable values (according to datasheet);
#define BRN_S_COL_MAX 2       // alias of column that holds sensors max detectable values (according to datasheet);
#define BRN_S_COL_LBD 3       // alias of column that holds lower bounds of acceptable sensors readings;
#define BRN_S_COL_UBD 4       // alias of column that holds upper bounds of acceptable sensors readings;
#define BRN_S_COL_WGT 5       // alias of column that holds "weights" or "importance scores" of corresponding sensors (used to score environment conditions);
#define BRN_S_COL_REC 6       // alias of column that holds records of actual sensors raadings;
#define BRN_S_COL_VRT 7       // alias of column that holds temporary/virtual sensors raadings;

// Table M ("Motor programm settings table") 
#define BRN_M_COL_CNT 3       // number of columns in table (2D array) M;
#define BRN_M_COL_NNIO 0      // alias of column that holds indexes of neural net i/o channels for corresponding motor program parametets (insexes of elements in NN i/o arrays X/V);
#define BRN_M_COL_MIN 1       // alias of column that holds min allowable values of motor programm parameters;
#define BRN_M_COL_MAX 2       // alias of column that holds max allowable values of motor programm parameters;

// Table C ("Context neurones table") 
#define BRN_C_COL_CNT 3       // number of columns in table (2D array) C
#define BRN_C_COL_NNIO 0      // alias of column that holds indexes of neural net i/o channels to feedback context neurones activation values (insexes of elements in NN i/o arrays X/V);
#define BRN_C_COL_IDL 1       // alias of column that holds idx of NN's layer where context neurone is;
#define BRN_C_COL_IDN 2       // alias of column that holds idx of node inside layer that is context neurone;

// IDs of data arrays/tables
#define BRN_X_ID 0            // id of array X
#define BRN_O_ID 1            // id of NN'a output layer nodes array
#define BRN_Y_ID 2            // id of array Y
#define BRN_V_ID 3            // id of array V
#define BRN_S_ID 4            // id of table S
#define BRN_M_ID 5            // id of table M
#define BRN_C_ID 6            // id of table C
#define BRN_MPGEN_ID 7        // id of table MPGEN
#define BRN_MPSEL_ID 8        // id of table MPSEL


// ------------------------
// MOTOR PROGRAMM GENERATOR
// ------------------------
#define BRN_MPP_INTERVALS_CNT 10    // divides range of motor programm patamenters into intervals. It will create (BRN_MPP_INTERVALS_CNT + 1) points/values for each range.
#define BRN_BRUTE_MPGEN_COMB_STEP_SIZE 3  // step size for combinations to check motor program
                                

class RBrain
{
  private:

    // NN
    NN* brNN;           // pointer to NN (Neural net) object
    
    // NN's In/Out data
    double* X;          // array X[x_cnt] for actual NN input dara (number of elements must be equal to number of NN input nodes). Dynamically allocated.
    double* V;          // array V[x_cnt] for with virtual/simulated/temporary NN input dara (number of elements must be equal to number of NN input nodes). Dynamically allocated.
    double* Y;          // array Y[y_cnt] for NN target dara (number of elements must be equal to number of NN output nodes). Dynamically allocated.
    byte x_cnt;         // number of elements in array X (must be equal to number of NN input nodes).
    byte y_cnt;         // number of elements in array Y (must be equal to number of NN output nodes).
    
    // Reference/Data tables
    double** S;         // table S[s_cnt][BRN_S_COL_CNT] for data related to robot's sensors. Dynamically allocated or static array passed by pointer.
    int** M;            // table M[mpp_cnt][BRN_M_COL_CNT] for settings related to robot's motor programm. Dynamically allocated or static array passed by pointer.
    byte** C;           // table C[c_cnt][BRN_C_COL_CNT] for data related to NN's context neurones. Dynamically allocated or static array passed by pointer.
    byte s_cnt;         // number of rows in table S / number of sensor inputs
    byte c_cnt;         // number of rows in table C / number of context neurones to manage
    byte mpp_cnt;       // number of rows in tables M, MPGEN and MPSEL / number of motor program parameters
    byte mps_cnt;       // number of columns in tables MPGEN and MPSEL / number of motor program steps
    
    // Whether to allocate data tables S, M and C.
    bool allocateRefTables;   // If not - it must be static arrays passed by pointer.                             
    
    // Motor programm generator tables
    int** MPGEN;        // table MPGEN[mpp_cnt][mps_cnt] for temporary generated motor programs. Dynamically allocated.
    int** MPSEL;        // table MPSEL[mpp_cnt][mps_cnt] for proposed/selected/final motor programm. Dynamically allocated.
    
    // Motor programm selector paramentrs
    double outcome_threshold_1;     // threshold value for candidate motor programm
    double outcome_threshold_2;     // threshold value for selected motor programm
    int att_cnt;                    // number of attempts to select randomly generated motor program (number of randomly generated programs to check with NN)
    
    
  public:
  
    // Constructors, destructors
    // -------------------------
    RBrain();               // Default constructor
    ~RBrain();              // Destructor


    // Initialisation
    // --------------
    void init(bool allocateRefTables);    // argument allocateRefTables - wether to dynamicaly allocate reference tables S, M, C. 
                                          // If not - it must be static arrays passed by pointer.
    
    // Settings BEFORE Initialisation init()
    void setCntXY(byte x_cnt, byte y_cnt);                            // X,Y NN inputs: set number of elements for arrays X (and V), Y 
    void setCntS(byte s_cnt);                                         // Sensors: set number of sensors inputs (can be ommited if corresponding static data tables will be passed later)
    void setMotProgGenerator(byte mpp_cnt, byte mps_cnt);             // Motor Programm Generator: set number of motor programm parameters and number of motor programm steps
    void setMotProgSelector(double thr1, double thr2, int att_cnt);   // Motor Programm Selector: set thresholds values to evaluate motor programm and number of attempts to generate motor programm (can be ommited if corresponding static data tables will be passed later)
    void setCntC(byte c_cnt);                                         // Context neurones: set number of context neurones (can be ommited if corresponding static data tables will be passed later)
    
    // settings AFTER Initialisation init()
    void addNN(NN* ptrNN);                                      // attach Neural Network object
    void addTableS(double arr[][BRN_S_COL_CNT], byte s_cnt);    // attach static 2D array S (Optional, used if allocateRefTables == false) - it will be accesed via pointer S**
    void addTableM(int arr[][BRN_M_COL_CNT], byte mpp_cnt);     // attach static 2D array M (Optional, used if allocateRefTables == false) - it will be accesed via pointer M**
    void addTableC(byte arr[][BRN_C_COL_CNT], byte c_cnt);      // attach static 2D array C (Optional, used if allocateRefTables == false) - it will be accesed via pointer C**
    
       
    // Getters and Setters
    // -------------------
    double getelX(byte n);                        // Get X[n]
    void setelX(double x, byte n);                // Set X[n] = x
    
    double getelY(byte n);                        // Get Y[n]
    void setelY(double y, byte n);                // Set Y[n] = y

    double getelS(byte row, byte col);            // Get S[row][col]
    void setelS(double val, byte row, byte col);  // Set S[row][col] = val
    double getRecS(byte s_idx);                   // Get actual sensor record from table S (S[s_idx][BRN_S_COL_REC])
    void setRecS(double val, byte s_idx);         // Set actual sensor record in table (S[s_idx][BRN_S_COL_REC] = val)

    double getelM(byte row, byte col);            // Get M[row][col]
    void setelM(double val, byte row, byte col);  // Set M[row][col] = val
   
    byte getCntC();                               // Get number of context neurones
    byte getNnIORefC(byte c_idx);                 // Get ref idx of context neurone in NN's in/out data arrats
    double getValC(byte c_idx);                   // Get activation value of context neurone
    void setValC(double val, byte c_idx);         // Set activation value of context neurune

    int getMotProgParam(byte mpp_idx, byte mps_idx);              // Get value of motor programm parameter mpp_idx for step mps_idx (MPSEL[mpp_idx][mps_idx])
    void setMotProgParam(double val, byte mpp_idx, byte mps_idx); // Set value of motor programm parameter mpp_idx for step mps_idx (MPSEL[mpp_idx][mps_idx] = val)    


    // Updating rows in reference/data tables
    // --------------------------------------
    void fillRowS(byte s_idx, byte nnio_ref, double min_val, double max_val, 
                  double lbd_val, double ubd_val, double wgt_val,
                  double rec_val, double vrt_val);                              // fill a row in table S
    void fillRowM(byte mpp_idx, byte nnio_ref, int min_val, int max_val);       // fill a row in table M
    void fillRowC(byte c_idx, byte nnio_ref, byte l_idx, byte n_idx);           // fill a row in table C


    // Math
    // ----
    double mapVal(double x, double x_min, double x_max, double y_min, double y_max);  // mapping for doubles


    // Data conditioning/encoding/decoding/copying into/from NN's input/output arrays (X, V, Y...)
    // -------------------------------------------------------------------------------------------
    // Arbitary values
    void encValToNnIOArr(double val, double val_min, double val_max, byte dest_arr_id, byte dest_el_idx);    // Encode arbitary value into arbitary element of arbitary NN's in/out array 
    double decValFromNnIOArr(double val_min, double val_max, byte src_arr_id, byte scr_el_idx);              // Decode arbitary value from arbitary element of arbitary NN's in/out array 
   
    // Sensors
    void encSensRdsToNnIOArr(byte dest_arr_id);     // encode actual sensors records from table S into arbitary NN's in/out array   
    void encSensRdsToX();                           // encode actual sensors readings from table S into array X
    void encSensRdsToY();                           // encode actual sensors readings from table S into array Y 
   
    // Motor programm
    void encMotProgToNnIOArr(byte src_arr_id, byte dest_arr_id, byte mps_idx);    // Encode all motor programm params for step# mps_idx into NN's in arrays
    void encMotProgToX(byte mps_idx);                                             // Encode all motor programm params for step# mps_idx from MPSEL into array X
    void copyMPGENtoMPSEL();                                                      // copy values of table MPGEN into MPSEL

    // Context neurones
    void ccopyContNodesToNnInpLayer();  // copy context neurones activation values into corresponding neurones in NN input layer
    void copyContNodesToX();            // copy context neurones activation values into corresponding cells of array X
    void copyContNodesToV();            // copy context neurones activation values into corresponding cells of array V
    
    // Other
    void copyYtoX();                   // copy values from array Y into corresponding elements of array X
    
    
    // Core
    // ----
    // Methods marked with [!!!] in comments define robot's behavior
    // change its logic to change robot's behavior
    double evalRawSensData(byte s_col_id);        // [!!!] eval actual or virtual/temporal raw sensors data stored in table S (pass corresponding column number as argument)
    double evalActRawSensData();                  // eval actual raw sensors data stored in table S   
    double evalEncSensData(byte src_arr_id);      // eval encoded sensors data stored in one of NN's input/output arrays (pass corresponding index of a data array as argument) 
    double evalMPGEN(); 
    void generateMotProg();                       // [!!!] generate motor programm  (randomly) 
    bool proposeMotProg();                        // select and propose motor programm (use randomly generated motprog, number of atrempts limited by s_cnt)   
    bool proposeMotProgBruteForce();              // select and propose motor programm (check all possible values for all motprog parameters)
    void learn();                                 // train NN according to data stored in X and Y arrays
    double estimate();                            // propgate NN once (input data array is X[]) and decode/denormalise predicted values into S[][BRN_S_COL_VRT] column

    void convertMpgenDataPointsIdxToDataValues();

    // Disp
    // ----
    void dispNN();                          // display NN
    void dispX();                           // display 1D array X
    void dispV();                           // display 1D array V
    void dispY();                           // display 1D array Y
    void dispS();                           // display 2D array S
    void dispM();                           // display 2D array M
    void dispC();                           // display 2D array C
    void dispMPGEN();                       // display 2D array MPGEN
    void dispMPSEL();                       // display 2D array MPSEL
    void dispBRN();                         // display all Brain's data + arrays
     
};


#endif
