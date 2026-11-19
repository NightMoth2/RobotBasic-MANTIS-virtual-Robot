/* 

Robot Car Ver. 19 (2026-07-12)
1) FSM state update/rename
2) Transmit to LOG predictions for MPSEL, not predictions stored in output layer after learning process)
3) Transmit to LOG context neurones values.

Robot Car Ver. 18 (2026-07-10)
1) FSM state update/rename
2) Logic bug fix: INIT_MOVE_FW updates X[] with "Move Forward" motor program

Robot Car Ver. 17 (2026-07-05)
1) FSM state added: PROPOSE_NOTPROG_BRUTE

Robot Car Ver. 16 (2026-06-25)
1) Obstacle-free timer readings doesnt predicted by NN
2) Obstacle free timer result readings is NN's INPUT data (it is actual motor work time for "Move forward" motor program)
3) IR umper sensor data changed to bitmap again
4) NN layers, "S" table and scoring changed accordingly

Robot Car Ver. 15.2 (2026-06-21)
1) Bumper sensors: 
 - binary instead of bitmap: only 0 and 1 (0 - clear, 1 - any of bumpers activated) - updates in rbt_fullScan() and in table S
 - predictions: double instead of int (sigmoid between 0 and 1)
3) NN Training speed changed to 0.01 (anti overfitting, in some cases negative time predictes)

Robot Car Ver. 15 (2026-06-20)
1) Move forward is the continuation of evade maneuver

Robot Car Ver. 14 (2026-05-28)
1) Connected to virtual robot's body in RobotBasic environment via "RBSerial_02.h" serial communication protocol
2) FSM changes: FULL_SCAN state after RESTING state
3) re-evaluation of sensors "weights" to calculate outcome rating:
   Target:
   - any sensor's readings, but obstacle free timer readings > 1000mS (robot moved at least one sec before next obstacle)
   OR
   - free bumpers and free forward direction 
4) FSM changes:  "Move forward" motor programm to be saved as NN input data only if it is second 
   "Move forward" initialisation in a row (FSM state INIT_MOVE_FW).
   During first "Move forward" initialisation, previous "Evade maneuver" motor programm be saved as NN input data
   (it is to keep information about "Evade maneuver" programm that allowed the robot to evade last obstacle and move forward)
   Corresponding changes are in "INIT_MOVE_FW", "RESTING", "STOP_OBST" FSM states. bool rbt_is_moving_forward added.
5) "obstacle-free" timer is off during evade maneuver. During evade maneuver "obstacle-free" time is zero.
6) Related to (5): FULL_SCAN FSM state output "guards" updated (rbt_just_powered_on flag checked instead of obstacle free timer)
7) SAVE_INIT_COND output guards check bumper and forward sensot instead of outcome rating (function bool rbt_isFwPathFree())

Robot Car Ver. 13 (2026-04-29) with Neural Network for Arduino Nano
Features:
 - Finite state machine implementation;
 - Class NN - Neural network for Arduino;
 - Neural Network, can be saved to EEPROM and loaded from EEPROM on power-on;
 - "Context"/"Memory" neurones (kind of recurrent neural network implementation);
 - Class RBrain - "Interface" between robot sensors, motor drivers and Neural Network;
 - RBrain v4 allows to manage arbitary quantity and type of sensors, motor programm parameters, motor programm steps and context neurones;
 - Class Timer_R - timer to measure obstacle-free run time robot moves between obstacles;
 - display_freeram() function to check Arduino RAM usage;

========================================================================

Robot:
 - Two DC motors chassis;
 - Ultrasonic sonar mounted on servo - scans distance in three sectors (Forward, Left and Right);
 - Array of four IR bumper sensors 
 - Timer to measure how long robot moves forward before encounters an obstacle (obstacle-free time)

Soft:
 - 

Simplified Algorithm:
  Repeat{
    Move forward; 
    If(Obstacle){
      Repeat until forward path is free{
        Collect sensors readings (initial);
        Generate list of motor programs (random and/or by other method);
        Use NN to predict best motor program to evade the obstacle;
        Try to evade the obstacle using proposed motor programm;
        Collect sensors readings (result) after evasion attempt;
        Train NN (input: initial sensors readings, motor programm; target: result sensors readings);
      }
    }
  } 

Refs:
[1] J. Tani, вЂњModel-based learning for mobile robot navigation from the dynamical systems perspective,вЂќ 
    IEEE Trans. Syst., Man, Cybern. B, vol. 26, no. 3, pp. 421вЂ“436, Jun. 1996, doi: 10.1109/3477.499793.
[2] D. JanglovГЎ, вЂњNeural Networks in Mobile Robot Motion,вЂќ International Journal of Advanced Robotic Systems, 
    vol. 1, no. 1, p. 2, Mar. 2004, doi: 10.5772/5615.
[3] J. L. Elman, вЂњFinding Structure in Time,вЂќ Cognitive Science, vol. 14, no. 2, pp. 179вЂ“211, Mar. 1990, doi: 10.1207/s15516709cog1402_1.
    Accesed: April 03, 2026. [Online]. 
    Available: https://onlinelibrary.wiley.com/doi/10.1207/s15516709cog1402_1
[4] Arduino Docs, вЂњPROGMEM,вЂќ Arduino Docs. Accessed: Apr. 20, 2026. [Online]. 
    Available: https://docs.arduino.cc/language-reference/en/variables/utilities/PROGMEM/#the-f-macro     
[5] H. Talk, вЂњSave SRAM with the F() macro,вЂќ Tech Explorations. Accessed: Apr. 20, 2026. [Online]. 
    Available: https://techexplorations.com/guides/arduino/programming/f-macro/     
[6] M. Girish, вЂњWhat are the traditional ways to optimize program memory usage?,вЂќ Arduino Stack Exchange. Accessed: Apr. 20, 2026. [Online]. 
    Available: https://arduino.stackexchange.com/questions/76619/what-are-the-traditional-ways-to-optimize-program-memory-usage     
[7] Arduino, J. Bagur, and T. Chung, вЂњArduino Memory Guide,вЂќ Arduino Docs. Accessed: Apr. 20, 2026. [Online]. 
    Available: https://docs.arduino.cc/learn/programming/memory-guide/
[8] Arduino Learning, вЂњState Machines in Arduino Projects Explained (With Examples),вЂќ Arduino Learning. Accessed: Apr. 20, 2026. [Online]. 
    Available: https://www.arduinolearning.com/learning/state-machines-in-arduino-projects-explained-with-examples.php
[9] K.-S. (KC) Chang and D. Zhu, вЂњFinite State Machine (FSM) Concept and Implementation,вЂќ Standford University. Accessed: Apr. 20, 2026. [Online]. 
    Available: https://web.stanford.edu/class/cs123/
[10] C. Terman, вЂњFinite State Machines,вЂќ Computation Structures. Accessed: Apr. 20, 2026. [Online]. 
     Available: https://computationstructures.org/lectures/fsm/fsm.html
[11] Arduino, вЂњA guide to EEPROM,вЂќ Arduino Docs. Accessed: Apr. 24, 2026. [Online]. 
     Available: https://docs.arduino.cc/learn/programming/eeprom-guide/
[12] A. Haber, вЂњClear and Detailed Explanation of Kinematics, Equations, and Geometry of Motion of Differential Wheeled Robot 
     (Differential Drive Robot),вЂќ Aleksandar Haber. Accessed: May 24, 2026. [Online]. Available: 
     https://aleksandarhaber.com/clear-and-detailed-explanation-of-kinematics-equations-and-geometry-of-motion-of-differential-wheeled-robot-differential-drive-robot/
*/

#include <EEPROM.h>           // To save NN
#include "KB_NN_3.h"          // !!! Set NN params here
#include "KB_RBrain.h"        // !!! Set NN and the Robot interfacng paramenters here
#include "KB_Timer_R.h"       // timer with pause
#include "RBSerial_02.h"      // Serial protocol

// --------------------
// Define working modes
// --------------------
#define VIRTUAL_ROBOT
#define MIN_REPORT_MODE       // write only minimal info Serial
//#define REPORT_MODE           // write more info to Serial for debugging purposes
#define USE_RESTING           // allows robot to rest periodically (in order to cool down power supply module)
//#define RND_AFTER_REST      // Robot to turn in random directions on waking-up after resting
#define SAVE_NN_TO_EEPROM     // save NN and related data to EEPROM
#define LOAD_NN_FROM_EEPROM   // load NN and related data from EEPROM at power up
#define NOT_LOAD_CONTEXT_NEURONES // not load context neurones from EEPROM

// [VIRTUAL ROBOT]
// SERIAL PROTOCOL - Specifications for messages to be sent/received
#define RBS_LBL_LMS  'A'          // Label for Left motor speed
#define RBS_LBL_RMS  'B'          // Label for Right motor speed
#define RBS_LBL_MWT  'C'          // Label for Motor working time
#define RBS_LBL_IRB  'D'          // Label for Infrared Bumpers
#define RBS_LBL_USF  'E'          // Label for Ultrasonic sensor forward sector
#define RBS_LBL_USL  'F'          // Label for Ultrasonic sensor left sector
#define RBS_LBL_USR  'G'          // Label for Ultrasonic sensor right sector
#define RBS_LBL_OFT  'H'          // Label for obstacle-free time
#define RBS_LBL_ALS  'I'          // Label for "all sensors"
#define RBS_LBL_RBT  'J'          // Label for "robot"
#define RBS_LBL_MSG  'K'          // Label "message"
#define RBS_LBL_OBS  'L'          // Label for obstacles counter
#define RBS_LBL_NNL  'M'          // Label for nn learnings counter
#define RBS_LBL_LOG  'N'          // Label for commands related to logging
#define RBS_LBL_CN1  'O'          // Label for context neurone 1
#define RBS_LBL_CN2  'P'          // Label for context neurone 2

#define RBS_ARG_EXE  'A'          // Argument for virual robot command - exec motor programm
#define RBS_ARG_FWD  'B'          // Argument for virual robot command - go forward
#define RBS_ARG_STP  'C'          // Argument for virual robot command - stop
#define RBS_ARG_RGO  'D'          // Argument for virual robot command - robot GO!
#define RBS_ARG_INI_S 'E'         // Argument for Logging command: save initial sensors data to log table
#define RBS_ARG_INI_M 'F'         // Argument for Logging command: save initial motor programm to log table
#define RBS_ARG_LST_M 'G'         // Argument for Logging command: save last/previous motor programm to log table
#define RBS_ARG_RES_S 'H'         // Argument for Logging command: save result sensors data to log table
#define RBS_ARG_FWD_M 'I'         // Argument for Logging command: save "Move forward" motor program to log table
#define RBS_ARG_MWT_U 'J'         // Argument for Logging command: Update motor working time in log table
#define RBS_ARG_NXT 'K'           // Argument for Logging command: start next row in log table

// --------------
// Neural network
// -------------- 
// Input, 9 items:
// - IR bumper sensors array (4 sensors). Bitmap value (0...15)
// - Ultrasonic sensor, forward sector distance, cm
// - Ultrasonic sensor, left sector distance, cm
// - Ultrasonic sensor, right sector distance, cm
// - Left motor speed (-250...250)
// - Right motor speed (-250...250)
// - Motors worktime, mS
// - Context neurone #1 activation
// - Context neurone #2 activation
// Output, 4 items:
// - Predicted bitmap value for IR bumper sensors array (4 sensors). Bitmap value (0...15)
// - Predicted forward sector distance, cm
// - Predicted left sector distance, cm
// - Predicted right sector distance, cm
const byte rbt_nn_layers_cnt = 5;     // number of NN layers 
const byte rbt_nn_input_layer_nodes_cnt = 9;     // number of nodes in input layer
const byte rbt_nn_hidden_layer1_nodes_cnt = 6;     // number of nodes in hidden layer
const byte rbt_nn_hidden_layer2_nodes_cnt = 5;     // number of nodes in hidden layer
const byte rbt_nn_hidden_layer3_nodes_cnt = 4;     // number of nodes in hidden layer
const byte rbt_nn_output_layer_nodes_cnt = 4;     // number of nodes in output layer

// -------
// Sensors
// -------
const byte rbt_s_cnt = 4;         // number of sensors inputs: 3 for sonar (F, L and R), 1 for bitmap array of IR bumpers
const byte s_row_irb = 0;         // row idx in table S for IR bumper sensor data (in table S, not in NN's in/out data arrays)
const byte s_row_usf = 1;         // row idx in table S for US forward distance readings (in table S, not in NN's in/out data arrays)
const byte s_row_usl = 2;         // row idx in table S for US left distance readings (in table S, not in NN's in/out data arrays)
const byte s_row_usr = 3;         // row idx in table S for US right distance readings (in table S, not in NN's in/out data arrays)
//const byte s_row_oft = 4;       // row idx in table S for "obstacle free" timer readings (in table S, not in NN's in/out data arrays)

// Table S:
// holds data related to sensors
// it can be arbitary number of rows
// columns descriptions:
// [][0] holds indexes of neural net i/o channels
// [][1] holds min value detectable by sensor (inclusive);
// [][2] holds max value detectable by sensor (inclusive);
// [][3] holds lower bound of acceptable range (inclusive);
// [][4] holds upper bound of acceptable range (inclusive);
// [][5] holds "importance score" of "weight" of a sensor - this value will be added to outcome rating if the readings of this sensor is in acceptable range;
// [][6] to store actual records of sensors readings (updates after each scan);
// [][7] to store temporal/virtual sensors raadings (used for calculations);
// +------------------------------------------------------------------------------------------------+
// | NN I/O | min val | max val | lower bound | upper bound | weight | actual val | temp/virt val | 
// +------------------------------------------------------------------------------------------------+
double rbt_Table_S[rbt_s_cnt][BRN_S_COL_CNT] = 
/*row 0*/{{0,   0.0,       15.0,  -10000.0,       0.4,    30.0,   0.0,    0.0},      // IR bumpers array - encoded binary value (one bit for each of four sensors, DEC 0...15)
/*row 1*/ {1,   0.0,       40.0,      15.0,   10000.0,    30.0,   0.0,    0.0},      // ultrasonic sensor - readings in forward direction, cm
/*row 2*/ {2,   0.0,       40.0,      15.0,   10000.0,    10.0,   0.0,    0.0},      // ultrasonic sensor - readings in left direction, cm
/*row 3*/ {3,   0.0,       40.0,      15.0,   10000.0,    10.0,   0.0,    0.0}};     // ultrasonic sensor - readings in right direction, cm 


const double rbt_outcome_thr_1 = 60;                     // !!! outcome threshold to accept generated motor programm as candidate (see sensor weights in Table S)
const double rbt_outcome_thr_2 = 80;                     // !!! outcome threshold to accept motor programm (see sensor weights in Table S)

// ---------------------------------
// Motor programm generator/selector
// ---------------------------------
const byte rbt_mpparam_cnt = 3;                     // number of motor programm parameters (left and right motors speed, motors working time)
const byte rbt_mpsteps_cnt = 1;                     // number of motor programm steps 
const int rbt_mpgenatt_cnt = 444;                   // number of attempts motor programm generator generates a motor programm (1/3 of 11*11*11 possible motor programms)
const byte m_row_lms = 0;                           // idx of left motor speed parameter in tables M, MPGEN, MPSEl(!! not in NN's in/out data arrays)
const byte m_row_rms = 1;                           // idx of right motor speed parameter in tables M, MPGEN, MPSEl(!! not in NN's in/out data arrays)
const byte m_row_mwt = 2;                           // idx of motor working time parameter in tables M, MPGEN, MPSEl(!! not in NN's in/out data arrays)

// Table M - motor programm settings
// it can be arbitary number of rows
// columns description:
// [][0] holds indexes of neural net i/o channels
// [][1] holds min value of a parameter to be generated
// [][2] holds max value of a parameter to be generated
// +----------------------------+
// | NN I/O | min val | max val |
// +----------------------------+
int rbt_Table_M[rbt_mpparam_cnt][BRN_M_COL_CNT] = 
/*row 0*/ {{4, -250,  250},           // Settings for Left motor programm 
/*row 1*/  {5, -250,  250},           // Settings for Right motor programm
/*row 2*/  {6,  100, 1600}};          // Settings for Motors working time programm (enough to make 0...540deg turns for my robot, see kinematics calculations [12])


// ----------------
// Context neurones
// ----------------
const byte rbt_c_cnt = 2;   // number of context neurones    
const double c_init_val = 0.5; // init value for context neurone

// Table C columns description:
// it can be arbitary number of rows
// [][0] holds indexes of neural net i/o channels
// [][1] holds idx of NN's layer with a context neurone
// [][2] holds idx of context neuron node inside a layer
// +------------------------+
// | NN I/O | NN L# | NN N# |
// +------------------------+                
byte rbt_Table_C[rbt_c_cnt][BRN_C_COL_CNT] = 
/*row 0*/ {{7, 2, 2},           // context neuron #0 indexes
/*row 1*/  {8, 2, 3}};          // context neuron #1 indexes

// ---------------------------------------------
// Variables and timers to control state machine
// and robot behavior
// ---------------------------------------------

const unsigned long rbt_max_working_time = 15000;   // mS, max motors working time before pause (in order to cool-down motors and power supply)
const int rbt_rest_time = 4000;                     // mS, robot resting time (in order to cool-down motors and power supply)
const int rbt_idle_time = 1000;                     // mS, robot startup idle time
const byte rbt_trainings_per_event = 1;            // number of NN trainigs per event

bool rbt_candidate_mpg_found = false;               // flag, wether candidate motor programm found by motor programm generator/selector
double rbt_actual_mwt = 0;                          // mS, to store actual motors working time (before obstacle)
double outcome_rating = 0;                          // stores rating score for sensors readings
byte rbt_motprogstep_idx = 0;                       // stores current motor programm steps
bool rbt_motprog_accomplished = false;              // flag, robot accomplished all motor programm steps
bool rbt_is_moving_forward = false;                 // flag, is robot performing "move forward" motor programm
//bool rbt_just_powered_on = true;                  // to chek is this first FSM cycle

unsigned long tmr_wt = 0;     // Timer to control robot working time and resting (to measure how many time robot works without a rest)

#ifdef VIRTUAL_ROBOT
  // Virtual timer used
#else
  Timer_R tmr_oft;        // Timer with pause to measure "obstacle free" time robot moves between obstacles
#endif

// --------
// Counters
// --------
// statistics
unsigned int cnt_obstacle_alerts = 0;    // counts encountered obstacles
unsigned int cnt_learnings_total = 0;    // increments each time after NN training


// --------------
// EEPROM related
// --------------
const byte eepromControl1 = 1;
const byte eepromControl2 = 2;
const byte eepromControl3 = 3;
const byte eepromControl4 = 4;
bool isNNLoadedFromEEPROM = false;  // flag wether NN has been loaded from EEPROM


// ------------
// Sonar object
// ------------
// (Not used for virtual robot)
//SNR Robo_Sonar(A0, A1, 9);


// ---------------------------
// NN (Neural Network) Object and RBrain (Robot's Brain) object 
// ---------------------------
RBrain Robo_Brain;
NN Robo_NN;


#ifdef VIRTUAL_ROBOT
  // ---------------------------
  // RBS NPROTOCOL OBJECT 
  // ---------------------------
  // Need to control virtual robot
  // Hardware serial object
  RBSerial commProtocol;
#endif

bool isStarted = false; // "Start" command received via serial

// -------------
// State machine
// -------------
enum State {
  EVADE_MANEUVER,                       // Try to evade an obstacle
  EVADE_MANEUVER_INIT,                  // Evade maneuver initialization
  EVADE_MANEUVER_UPD_PROGRESS,          // Update Evade maneuver progress and status
  EVALUATE_SENSORS_DATA,                // Evaluate sensors readings
  FULL_SCAN,                            // Full environment scan
  MOVE_FW,                              // Moving forward  
  MOVE_FW_INIT,                         // Moving forward initialization
  NN_SAVE,                              // Saving NN weights and some statistics to EEPROM
  NN_TRAIN,                             // Train NN 
  NN_UPD_ESTIMATION_FOR_MPSEL,          // Update NN estimations for selected motor programm
  NN_UPD_INPUT_DATA_ACTUAL_MWT,         // Update NN input data with actual motor working time
  NN_UPD_INPUT_DATA_MOTPROG_MANEUVER,   // Update NN input data with Evade maneuver motor programm
  NN_UPD_INPUT_DATA_MOTPROG_MOVE_FW,    // Update NN input data with "Move Forward" motor programm
  NN_UPD_INPUT_DATA_SENSORS,            // Update NN input data with sensors readings
  NN_UPD_TARGET_DATA_SENSORS,           // Update NN target data with sensors readings
  PREP_TO_START,                        // Prepare to start
  QUICK_SCAN,                           // Bumpers and forward direction check
  RESTING,                              // Resting 
  SELECT_MOTPROG_BRUTE,                 // Propose motor programm, brute force
  SELECT_MOTPROG_MOVE_FW,               // Propose "Move Forward" motor programm
  SELECT_MOTPROG_RANDOM,                // Propose Evade Maneuver motor programm by selection from random generated programs 
  STOP_OBST                             // Stop robot because of obstacle 
};
State robo_state = PREP_TO_START;


// ---------------------
// FUNCTION DECLARATIONS
// ---------------------

// Behavior 
// (update code of these functions for real/virtual robot)
bool checkIsStarted();
void rbt_takeARestIfNeed();
void rbt_fullScan();
int rbt_quickCheckBumpers();
double rbt_quickCheckFwDir();
bool rbt_isFwPathFree();
bool rbt_stopRobot();
bool rbt_moveForward();
bool rbt_maneuver(int spdL, int spdR, int ms);


// EEPROM Related
int saveToEEPROM_double2D(double** Arr, int r_cnt, int c_cnt, int add);
int readFromEEPROM_double2D(double** Arr, int r_cnt, int c_cnt, int add);
int saveNNToEEPROM(unsigned long cnt_alerts, unsigned long cnt_learn);
int loadNNFromEEPROM(unsigned long& cnt_alerts, unsigned long& cnt_learn);

// RAM control
void display_freeram();
int freeRam();


#ifdef VIRTUAL_ROBOT
  // Send statistics
  void transmitObstaclesCnt();
  void transmitLearningsCnt();
  void transmitNnPredictions();
  void transmitContextNeurones();
  void transmitCommandToSaveInitSens();
  void transmitCommandToSaveInitMPG();
  void transmitCommandToCopyLastMPG();
  void transmitCommandToSaveResultSens();
  void transmitCommandNnLogNextRow();
  void transmitCommandToSaveMoveFwMPG();
  void transmitCommandToUpdateInitMwt();
#endif


// ------------------------------------------------------------------------------------------------------------
// SETUP() AND LOOP()
// ------------------------------------------------------------------------------------------------------------
void setup() {


  Serial.begin(115200);

  #ifdef VIRTUAL_ROBOT
    commProtocol.addSerial(&Serial);
  #endif
   
  delay(100);  

  
  #ifdef MIN_REPORT_MODE
    Serial.println();
    Serial.print(F("RAM at setup() start"));
    display_freeram();
  #endif

  // -----------
  // Seup NN
  // -----------

  // Set NN layers number and sizes
  Robo_NN.setNumOfLayers(rbt_nn_layers_cnt);    
  Robo_NN.setNumOfNodes(0, rbt_nn_input_layer_nodes_cnt);  
  Robo_NN.setNumOfNodes(1, rbt_nn_hidden_layer1_nodes_cnt);  
  Robo_NN.setNumOfNodes(2, rbt_nn_hidden_layer2_nodes_cnt);  
  Robo_NN.setNumOfNodes(3, rbt_nn_hidden_layer3_nodes_cnt);  
  Robo_NN.setNumOfNodes(4, rbt_nn_output_layer_nodes_cnt);
  //Robo_NN.setNumOfNodes(3, rbt_nn_output_layer_nodes_cnt);    

  // initialise empty NN (allocate memory according to specified NN's size)
  Robo_NN.initEmptyNN();   

  #ifdef MIN_REPORT_MODE
    Serial.println();
    Serial.print(F("RAM after Empty NN created"));
    display_freeram();
  #endif



  // ---------------------
  // Setup Robot's "Brain"
  // ---------------------

  // set parameters and initialise the "Brain"
  Robo_Brain.setCntXY(rbt_nn_input_layer_nodes_cnt, rbt_nn_output_layer_nodes_cnt);                                // set number of elements in data arrays X (Same as NN In layer), Y(Same as NN Out layer)   
  Robo_Brain.setMotProgGenerator(rbt_mpparam_cnt, rbt_mpsteps_cnt);             // sent number of motor programm parameters and number of motor programm steps
  Robo_Brain.setMotProgSelector(rbt_outcome_thr_1, rbt_outcome_thr_2, rbt_mpgenatt_cnt);  // set threshold values to evaluate motor programm, set number of attempts to generate motor programm
  
  Robo_Brain.init(false);                                                       // allocate memory and init the "Brain" ("false" argument - pre-defined static data tables 
                                                                                // will be passed by reference
  
  Robo_Brain.addNN(&Robo_NN);                                     // "Attach" NN to Robo_Brain
  Robo_Brain.addTableS(rbt_Table_S, rbt_s_cnt);                   // "Attach" sensors data table
  Robo_Brain.addTableM(rbt_Table_M, rbt_mpparam_cnt);             // "Attach" motor programm settings table
  Robo_Brain.addTableC(rbt_Table_C, rbt_c_cnt);                   // "Attach" context neurones data table

  #ifdef MIN_REPORT_MODE
    Serial.println(F("BRN set"));  
    Robo_Brain.dispBRN();
    Serial.println();
    Serial.print(F("RAM After initialising of BRN"));
    display_freeram();
  #endif


  // ------------------------------------------
  // Load NN weights from EEPROM or Init random
  // ------------------------------------------
  
  // Load NN weighta, context neurones data and statistics from EEPROM
  #ifdef LOAD_NN_FROM_EEPROM
    
    loadNNFromEEPROM(cnt_obstacle_alerts, cnt_learnings_total);
    
    #ifdef NOT_LOAD_CONTEXT_NEURONES
      // Rewrite context neurones with default values
      for(int i = 0; i < Robo_Brain.getCntC(); i++){
        Robo_Brain.setValC(c_init_val, i);
      }
    #endif
  #endif

  // If NN is not loaded from EEPROM
  // - randomize weights
  // - initialise context neurones with defined value
  if(isNNLoadedFromEEPROM == false){

    // init NN weights with random values
    Robo_NN.randomizeWeights(0); 

    // Init context neurones with pre-defined values
    for(int i = 0; i < Robo_Brain.getCntC(); i++){
      Robo_Brain.setValC(c_init_val, i);
    }
    
    #ifdef MIN_REPORT_MODE
      Serial.println();
      Serial.println(F("New NN: OK")); 
    #endif 
  }
  else{
    #ifdef MIN_REPORT_MODE
      Serial.println();
      Serial.println(F("EEPROM->NN: OK")); 
    #endif
  }


  #ifdef MIN_REPORT_MODE
    Serial.println();    
    Serial.println(F("NN and statistics before start:")); 
    Serial.print(F("Obs#/Tot: ")); 
    Serial.print(cnt_obstacle_alerts); 
    Serial.print(F("/")); 
    Serial.println(cnt_learnings_total);
    
    Robo_NN.disp_NN();
    
    Serial.println();
    Serial.print(F("RAM at setup() end"));
    display_freeram();
  #endif    


  // -----------------
  // Starting sequence
  // -----------------

  #ifdef MIN_REPORT_MODE
    Serial.println();
    Serial.println(F("Go!"));
    Serial.println();
  #endif

  // Wait after sonar is in forward position
  delay(2*rbt_idle_time);

}


void loop() {

// Check for "Start" command received via serial
if(isStarted){

  // -------------------------
  // State machine realisation
  // -------------------------
  
  switch(robo_state){
    
   // ----------------------------
    case EVADE_MANEUVER:{

      #ifdef VIRTUAL_ROBOT
        Serial.println(F("{MNVR}"));
      #endif

      #ifdef VIRTUAL_ROBOT
        // virtual timer data will be used
      #else
        tmr_oft.reset();
        tmr_oft.start();
      #endif
     
      rbt_maneuver(Robo_Brain.getMotProgParam(m_row_lms, rbt_motprogstep_idx), 
                   Robo_Brain.getMotProgParam(m_row_rms, rbt_motprogstep_idx), 
                   Robo_Brain.getMotProgParam(m_row_mwt, rbt_motprogstep_idx));

      #ifdef VIRTUAL_ROBOT
        // virtual timer data will be used
      #else
        tmr_oft.stop();
      #endif

      // Next State
      robo_state = NN_UPD_INPUT_DATA_MOTPROG_MANEUVER;

      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();
     
      break;
    }


    // ----------------------------
    case EVADE_MANEUVER_INIT:{

      #ifdef VIRTUAL_ROBOT
        Serial.println(F("{MNVR_INIT}"));
      #endif

      rbt_motprogstep_idx = 0;
      rbt_motprog_accomplished = false;

      // Next State
      robo_state = EVADE_MANEUVER;
      
      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();
                  
      break;
    }


    // ----------------------------
    case EVADE_MANEUVER_UPD_PROGRESS:{

      #ifdef VIRTUAL_ROBOT
        Serial.println(F("{MNVR_UPD_PROGRESS}"));
      #endif

      rbt_motprogstep_idx++;

      if(rbt_motprogstep_idx >= rbt_mpsteps_cnt){
        rbt_motprog_accomplished = true;
        rbt_motprogstep_idx = 0;
      }
      else{
        rbt_motprog_accomplished = false; 
      }

      // Next State
      robo_state = FULL_SCAN;

      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();

      break;
    }


    // ----------------------------
    case EVALUATE_SENSORS_DATA:{

      #ifdef VIRTUAL_ROBOT
        // Calc outcome rating
        // !!! Not necessary, for report/debug purposes
        Serial.println(F("{EVAL_SENS_RDS}"));
        //Robo_Brain.dispS();
        outcome_rating = Robo_Brain.evalActRawSensData();
        String valStr = "OUT:";
        valStr.concat(outcome_rating);
        Serial.println(valStr.c_str());
      #endif

      // Next state
      if(rbt_isFwPathFree()){
        robo_state = SELECT_MOTPROG_MOVE_FW;
      }
      else{

        if(rbt_motprog_accomplished){
          robo_state = SELECT_MOTPROG_RANDOM;  
        }
        else{
          robo_state = EVADE_MANEUVER;
        }        
      }
      
      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();
      
      break;
    }

    
    // ----------------------------
    case FULL_SCAN:{  

      #ifdef VIRTUAL_ROBOT
        Serial.println(F("{FULL_SCAN}"));
      #endif

      // Scan using all sensors (table "S[][]" and rbt_actual_mwt will be updated by this function)
      rbt_fullScan(); 

      #ifdef VIRTUAL_ROBOT
        //Robo_Brain.dispS();
      #endif
      
      // Next state
      // Check tmr_wt (from Version 14)
      if(tmr_wt == 0){        
        // if robot just powered on
        robo_state = NN_UPD_INPUT_DATA_SENSORS;
      }
      else{        
        // if robot already moved
        robo_state = NN_UPD_INPUT_DATA_ACTUAL_MWT;  
      }
      
      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();      
      
      break;
    }


    // ----------------------------
    case MOVE_FW:{
      
      // check working time timer while moving forward
      #ifdef VIRTUAL_ROBOT
        Serial.println(F("{MOVE_FW}"));
      #endif
      
      // Next state
      if(millis() - tmr_wt < (unsigned long)rbt_max_working_time){
        
        // working time is OK
        robo_state = QUICK_SCAN;
      }
      else{
        
        // working time too long
        robo_state = RESTING;
      }
                
      break;
    }

    // ----------------------------
    case MOVE_FW_INIT:{
      
      #ifdef VIRTUAL_ROBOT
        Serial.println(F("{MOVE_FW_INIT}"));
      #endif

      // Clear motor programm accomplished flag
      rbt_motprog_accomplished = false;

      // Restart obstacle free timer
      #ifdef VIRTUAL_ROBOT
        // virtual timer data will be used
      #else
        tmr_oft.reset();
        tmr_oft.start();
      #endif

      // Restart worktime timer
      tmr_wt = millis();

      // Send programm to motor driver
      rbt_moveForward();

      // Next state
      robo_state = MOVE_FW;
      
      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();   
         
      break;
    }

    // ----------------------------
    case NN_SAVE:{

      #ifdef VIRTUAL_ROBOT
        Serial.println(F("{NN_SAVE}"));
      #endif
      
      // Save NN to EEPROM
      #ifdef SAVE_NN_TO_EEPROM
        saveNNToEEPROM(cnt_obstacle_alerts, cnt_learnings_total);
      #endif

      #ifdef VIRTUAL_ROBOT        
        transmitLearningsCnt();         // Send counters by serial        
        //transmitNnPredictions();        // Send predicted values by serial       
        transmitCommandNnLogNextRow();  // Send command to start next row in log table
      #endif

      // Next state
      robo_state = NN_UPD_INPUT_DATA_SENSORS;

      #ifdef VIRTUAL_ROBOT
        //Robo_NN.disp_NN();
      #endif
      
      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();      
      
      break;
    }


    // ----------------------------
    case NN_TRAIN:{

      #ifdef VIRTUAL_ROBOT 
        Serial.println(F("{NN_TRAIN}"));
      #endif
      
      for(int i = 0; i < rbt_trainings_per_event; i++){
        Robo_Brain.learn();       
        cnt_learnings_total++;
      }

      Robo_Brain.copyContNodesToX();

      // Next state
      robo_state = NN_SAVE;
      
      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();     
       
      break;
    }


    // ----------------------------
    case NN_UPD_ESTIMATION_FOR_MPSEL:{

      #ifdef VIRTUAL_ROBOT
        Serial.println(F("{NN_EST_OUTCOME}"));
      #endif

      #ifdef VIRTUAL_ROBOT
        double pred_scr = Robo_Brain.estimate();
        
        //Serial.println(F("{RAM:}"));
        //display_freeram();

        Serial.println(F("{EVAL_PREDICTIONS}"));
        String valStr = "ESTIM:";
        valStr.concat(pred_scr);
        Serial.println(valStr.c_str());
       
        transmitNnPredictions();
      
      #endif

      // Next state
      robo_state = EVADE_MANEUVER_UPD_PROGRESS;

      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();

      break;
    }

    
    // ----------------------------
    case NN_UPD_INPUT_DATA_ACTUAL_MWT:{

      #ifdef VIRTUAL_ROBOT
        Serial.println(F("{NN_UPD_INP_MWT}"));
      #endif

      // Update NN input data with actual motor working time (Ver 16)
      Robo_Brain.encValToNnIOArr(rbt_actual_mwt, 
                                 rbt_Table_M[m_row_mwt][BRN_M_COL_MIN], 
                                 rbt_Table_M[m_row_mwt][BRN_M_COL_MAX], 
                                 BRN_X_ID, 
                                 rbt_Table_M[m_row_mwt][BRN_M_COL_NNIO]);

      #ifdef VIRTUAL_ROBOT
        transmitCommandToUpdateInitMwt();
      #endif

      
      #ifdef VIRTUAL_ROBOT
        //Robo_Brain.dispX();
      #endif
      
      // Next state
      robo_state = NN_UPD_TARGET_DATA_SENSORS;  

      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();

      break;
    }


    // ----------------------------
    case NN_UPD_INPUT_DATA_MOTPROG_MANEUVER:{

      #ifdef VIRTUAL_ROBOT
        Serial.println(F("{NN_UPD_INP_MNVR}"));
      #endif

      Robo_Brain.encMotProgToX(rbt_motprogstep_idx);

      #ifdef VIRTUAL_ROBOT
        transmitCommandToSaveInitMPG();
      #endif

      #ifdef VIRTUAL_ROBOT
        //Robo_Brain.dispX();
      #endif
      
      // Next state
      robo_state = NN_UPD_ESTIMATION_FOR_MPSEL;

      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();

      break;
    }


    // ----------------------------
    case NN_UPD_INPUT_DATA_MOTPROG_MOVE_FW:{

      #ifdef VIRTUAL_ROBOT
        Serial.println(F("{NN_UPD_INP_MVFW}"));
      #endif

      Robo_Brain.encMotProgToX(rbt_motprogstep_idx);

      #ifdef VIRTUAL_ROBOT
         transmitCommandToSaveMoveFwMPG();
      #endif

      #ifdef VIRTUAL_ROBOT
        //Robo_Brain.dispX();
      #endif

      // Next state
      robo_state = MOVE_FW_INIT;

      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();

      break;
    }


    // ----------------------------
    case NN_UPD_INPUT_DATA_SENSORS:{

      #ifdef VIRTUAL_ROBOT
        Serial.println(F("{NN_UPD_INP_SENS}"));
      #endif
      
      // convert sensors readings to NN input data
      Robo_Brain.encSensRdsToX();

      #ifdef VIRTUAL_ROBOT
        transmitCommandToSaveInitSens();
        transmitContextNeurones();
      #endif

      #ifdef VIRTUAL_ROBOT
        //Robo_Brain.dispX();
      #endif

      // Next state
      robo_state = EVALUATE_SENSORS_DATA;

      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();
      
      break;
    }


    // ----------------------------
    case NN_UPD_TARGET_DATA_SENSORS:{

      #ifdef VIRTUAL_ROBOT
        Serial.println(F("{NN_UPD_TGT}"));
      #endif
      
      // Convert sensors readings to NN's target data
      Robo_Brain.encSensRdsToY();

      #ifdef VIRTUAL_ROBOT
        transmitCommandToSaveResultSens();
      #endif

      #ifdef VIRTUAL_ROBOT
        //Robo_Brain.dispY();
      #endif
           
      // Next state
      robo_state = NN_TRAIN;

      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();
      
      break;
    }


    // ----------------------------
    case PREP_TO_START:{
      
      // Prepare robot to Start

      #ifdef VIRTUAL_ROBOT
        Serial.println(F("{PREP_START}"));
      #endif
      
      // Reset timers
      tmr_wt = 0;

      #ifdef VIRTUAL_ROBOT
        // virtual timer data will be used
      #else
        tmr_oft.reset();
      #endif

      // Clear sensors readings
      Robo_Brain.encSensRdsToX();

      // Copy values of context neurones to input array X
      Robo_Brain.copyContNodesToX();

      // Save "move forward" programm and encode it to input array X
      // (robot is stopped before start)
      Robo_Brain.setMotProgParam(rbt_Table_M[m_row_lms][BRN_M_COL_MAX], m_row_lms, 0);
      Robo_Brain.setMotProgParam(rbt_Table_M[m_row_rms][BRN_M_COL_MAX], m_row_rms, 0);
      Robo_Brain.setMotProgParam((double)rbt_max_working_time, m_row_mwt, 0);
      Robo_Brain.encMotProgToX(0); 

      // set motor programm status
      rbt_motprog_accomplished = true;
      
      #ifdef VIRTUAL_ROBOT
        // Send cnt_obstacle_alerts counter info by serial
        // (comment  for real robot)
        transmitObstaclesCnt();
  
        // [VIRTUAL ROBOT]
        // Send cnt_learnings_total counters info by serial
        // (comment for real robot)
        transmitLearningsCnt();
      #endif

      // Next state
      robo_state = FULL_SCAN;

      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();
            
      break;
    }


   // ----------------------------
    case QUICK_SCAN:{
      
      // Quick bumpers check and scan in forward direction

      #ifdef VIRTUAL_ROBOT
        Serial.println(F("{QUICK_SCAN}"));
      #endif

      // Next state (may be changed by below checks)
      robo_state = MOVE_FW;
      
      // Check IR bumpers
      if(rbt_quickCheckBumpers() > 0){
        
        // EMERGENCY STOP
        rbt_stopRobot();
                   
        // Next state
        robo_state = STOP_OBST;    

        break;        
      }

      // Check Ultrasonic sensor FW direction
      if(rbt_quickCheckFwDir() < rbt_Table_S[s_row_usf][BRN_S_COL_LBD]){
        
        // EMERGENCY STOP
        rbt_stopRobot();
              
        // Next state
        robo_state = STOP_OBST;        

        break;
      }        
  
      break;
    }


    // ----------------------------
    case RESTING:{

      #ifdef VIRTUAL_ROBOT
        Serial.println(F("{REST}"));
      #endif
      
      #ifdef USE_RESTING

        #ifdef VIRTUAL_ROBOT
          // virtual timer data will be used
        #else
          tmr_oft.stop();
        #endif
        
        rbt_takeARestIfNeed();

      #endif

      // Next state
      robo_state = FULL_SCAN;

      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();
      
      break;
    }


    // ----------------------------
    case SELECT_MOTPROG_BRUTE:{

      #ifdef VIRTUAL_ROBOT
        Serial.println(F("{SEL_MPG_BRUTE}"));
      #endif

      rbt_candidate_mpg_found = Robo_Brain.proposeMotProgBruteForce();

      #ifdef VIRTUAL_ROBOT
        //Robo_Brain.dispMPSEL();
      #endif

      // Next state
      robo_state = EVADE_MANEUVER_INIT;

      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();

      break;
    }


    // ----------------------------
    case SELECT_MOTPROG_MOVE_FW:{

      #ifdef VIRTUAL_ROBOT
        Serial.println(F("{SEL_MPG_MVFW}"));
      #endif

      // Save "Move forward" motor programm to MPSEL table in Brain
      Robo_Brain.setMotProgParam(rbt_Table_M[m_row_lms][BRN_M_COL_MAX], m_row_lms, 0);
      Robo_Brain.setMotProgParam(rbt_Table_M[m_row_rms][BRN_M_COL_MAX], m_row_rms, 0);
      Robo_Brain.setMotProgParam(rbt_Table_M[m_row_mwt][BRN_M_COL_MAX], m_row_mwt, 0);

      #ifdef VIRTUAL_ROBOT
        //Robo_Brain.dispMPSEL();
      #endif      

      // Next state
      robo_state = NN_UPD_INPUT_DATA_MOTPROG_MOVE_FW;

      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();

      break;
    }


    // ----------------------------
    case SELECT_MOTPROG_RANDOM:{
      
      // Propose motor programm 

      #ifdef VIRTUAL_ROBOT
        Serial.println(F("{SEL_MPG_RND}"));
      #endif

      rbt_candidate_mpg_found = Robo_Brain.proposeMotProg();

      #ifdef VIRTUAL_ROBOT
        //Robo_Brain.dispMPSEL();
      #endif
      
      // Next state
      if(rbt_candidate_mpg_found){
        robo_state = EVADE_MANEUVER_INIT;  
      }
      else{
        robo_state = SELECT_MOTPROG_BRUTE;
      }

      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();
           
      break;
    }


    // ----------------------------
    case STOP_OBST:{

      #ifdef VIRTUAL_ROBOT 
        Serial.println(F("{STOP}"));
      #endif

      #ifdef VIRTUAL_ROBOT
        // virtual timer data will be used
      #else
        tmr_oft.stop();
      #endif
      
      rbt_stopRobot();

      // Set motprog_accomplished flag and reset counter
      rbt_motprog_accomplished = true;
      rbt_motprogstep_idx = 0;

      // ncrement obstacles counter
      cnt_obstacle_alerts++; 

      #ifdef VIRTUAL_ROBOT 
        // Send counter info by serial
        // (delete for real rorob)
        transmitObstaclesCnt();
      #endif
      
      // Next state
      robo_state = FULL_SCAN;

      //Serial.println(F("{EXIT-RAM:}"));
      //display_freeram();
     
      break;
    }


    // ----------------------------
    default:{

      //commProtocol.sendFrame("{DEF}");
      // Next state - restart
      robo_state = PREP_TO_START;
      
      break;
    }

  } // end switch

}
else{
  #ifdef VIRTUAL_ROBOT
    // chaeck for "start" message received
    commProtocol.receiveARQ();
    isStarted = checkIsStarted();  
  #else
    // Start real robot
    isStarted = true;
  #endif    
} // End if(isStarted)

} // End loop()


// ------------------------------------------------------------------------------------------------------------
// FUNCTION DEFINITIONS
// ------------------------------------------------------------------------------------------------------------

// CONNECT TO RB FUNCTIONS
// ------------------------------------------------------------------------------------------------------------
void rbt_fullScan(){

  #ifdef VIRTUAL_ROBOT
    // Inquiry IR bumper readings and save received data to table S
    //while(!commProtocol.commInquiryData(RBS_LBL_IRB)){}
    //Robo_Brain.setRecS(commProtocol.argArrToDbl(), s_row_irb);
    if(commProtocol.commInquiryData(RBS_LBL_IRB)){
      Robo_Brain.setRecS(commProtocol.argArrToDbl(), s_row_irb);  
    }
    else{
      // "Sensor missed obstacle"
      Robo_Brain.setRecS(rbt_Table_S[s_row_irb][BRN_S_COL_MIN], s_row_irb);  
    }
  
    // Inquiry Ultrasonic sensor forward sector readings and save received data to table S
    //while(!commProtocol.commInquiryData(RBS_LBL_USF)){}
    //Robo_Brain.setRecS(commProtocol.argArrToDbl(), s_row_usf);
    if(commProtocol.commInquiryData(RBS_LBL_USF)){
      Robo_Brain.setRecS(commProtocol.argArrToDbl(), s_row_usf); 
    }
    else{
      // "Sensor missed obstacle"
      Robo_Brain.setRecS(rbt_Table_S[s_row_usf][BRN_S_COL_MAX], s_row_usf);  
    }
    
    // Inquiry Ultrasonic sensor left sector readings and save received data to table S
    //while(!commProtocol.commInquiryData(RBS_LBL_USL)){}
    //Robo_Brain.setRecS(commProtocol.argArrToDbl(), s_row_usl);
    if(commProtocol.commInquiryData(RBS_LBL_USL)){
      Robo_Brain.setRecS(commProtocol.argArrToDbl(), s_row_usl); 
    }
    else{
      // "Sensor missed obstacle"
      Robo_Brain.setRecS(rbt_Table_S[s_row_usl][BRN_S_COL_MAX], s_row_usl);  
    }    
  
    // Inquiry Ultrasonic sensor right sector readings and save received data to table S
    //while(!commProtocol.commInquiryData(RBS_LBL_USR)){}
    //Robo_Brain.setRecS(commProtocol.argArrToDbl(), s_row_usr); 
    if(commProtocol.commInquiryData(RBS_LBL_USR)){
      Robo_Brain.setRecS(commProtocol.argArrToDbl(), s_row_usr); 
    }
    else{
      // "Sensor missed obstacle"
      Robo_Brain.setRecS(rbt_Table_S[s_row_usr][BRN_S_COL_MAX], s_row_usr);  
    }  
        
    // Version 16
    // Inquiry obstacle free timer and update motor working time INPUT data with it
    //while(!commProtocol.commInquiryData(RBS_LBL_OFT)){}
    //rbt_actual_mwt = commProtocol.argArrToDbl();
    if(commProtocol.commInquiryData(RBS_LBL_OFT)){
      rbt_actual_mwt = commProtocol.argArrToDbl();
    }
    else{
      // "Timer didt work"
      rbt_actual_mwt = 0;
    }      
  #else
    // Code for Real robot

    // Version 16
    // Inquiry obstacle free timer and update motor working time INPUT data with it
    rbt_actual_mwt = (double)tmr_oft.getWorkTime(false);  

  #endif


  // Version 15.2
  // Make binary value from IR bumpers bitmap
  //if(Robo_Brain.getRecS(s_row_irb) >= 1.0){
  //  Robo_Brain.setRecS(1.0, s_row_irb);  
  //}
  //else{
  //  Robo_Brain.setRecS(0.0, s_row_irb);    
  //}
}


// -------------------------------------------------------
int rbt_quickCheckBumpers(){

  #ifdef VIRTUAL_ROBOT
    if(commProtocol.commInquiryData(RBS_LBL_IRB)){
      return commProtocol.argArrToInt();
    }
    else{
      return 0;
    }  
  #else
    // Code for Real robot
    
  #endif
}


// -------------------------------------------------------
double rbt_quickCheckFwDir(){

  #ifdef VIRTUAL_ROBOT
    // Quick check Sonar in forward direction
    if(commProtocol.commInquiryData(RBS_LBL_USF)){
      return commProtocol.argArrToDbl();
    }
    else{
      return 0;
    } 
  #else
    // Code for Real robot
    
  #endif        
}


// -------------------------------------------------------
bool rbt_isFwPathFree(){

  // if no bumper sensors active and distance to obstacle in fw direction >= threshold
  if((rbt_Table_S[s_row_irb][BRN_S_COL_REC]) == 0 && (rbt_Table_S[s_row_usf][BRN_S_COL_REC]  >= rbt_Table_S[s_row_usf][BRN_S_COL_LBD])){
    return true;
  }
  else{    
    return false;
  }
}


// -------------------------------------------------------
bool rbt_stopRobot(){

  #ifdef VIRTUAL_ROBOT
    //while(!commProtocol.commSendCommand(RBS_LBL_RBT, RBS_ARG_STP)){}
    //return true;
    if(commProtocol.commSendCommand(RBS_LBL_RBT, RBS_ARG_STP)){
      return true;  
    }
    else{
      return false;  
    }
  #else
    // Code for Real robot
    
  #endif       
}

// -------------------------------------------------------
bool rbt_moveForward(){

  #ifdef VIRTUAL_ROBOT
    //while(!commProtocol.commSendCommand(RBS_LBL_RBT, RBS_ARG_FWD)){} 
    //return true;
    if(commProtocol.commSendCommand(RBS_LBL_RBT, RBS_ARG_FWD)){
      return true;  
    }
    else{
      return false;  
    }    
  #else
    // Code for Real robot
    
  #endif  
}

// -------------------------------------------------------
bool rbt_maneuver(int spdL, int spdR, int ms){

  #ifdef VIRTUAL_ROBOT
    // Command for left motor
    bool commandSent = commProtocol.commSendCommand(RBS_LBL_LMS, spdL); 
    
    // do not proceed if failed to send
    if(!commandSent){
      return false;
    }
  
    // Command for right motor
    commandSent = commProtocol.commSendCommand(RBS_LBL_RMS, spdR);
  
    // do not proceed if failed to send
    if(!commandSent){
      return false;
    }
  
    // Set motors working time
    commandSent = commProtocol.commSendCommand(RBS_LBL_MWT, ms);
  
    // do not proceed if failed to send
    if(!commandSent){
      return false;
    }
  
    // execute motor programm
    commandSent = commProtocol.commSendCommand(RBS_LBL_RBT, RBS_ARG_EXE);
  
    return commandSent;
  #else
    // Code for Real robot
    
  #endif 
    
}




// ------------------------------------------------------------------------------------------------------------
void rbt_takeARestIfNeed(){

    // Stop motors for defined resting time

    #ifdef VIRTUAL_ROBOT
      Serial.println(F("{SLEEP}"));
    #endif
    
    rbt_stopRobot();
    delay(rbt_rest_time);

    #ifdef VIRTUAL_ROBOT
      Serial.println(F("{WAKE_UP}"));
    #endif
    
    // decide turn direction (In a case of ambiguity) 
    #ifdef RND_AFTER_REST
      byte turnDir = random(0,5);
    #else
      byte turnDir = 4;
    #endif

    // move according to direction decided on waking up
    switch(turnDir){
      case 0:{
        rbt_maneuver(rbt_Table_M[m_row_lms][BRN_M_COL_MIN] , rbt_Table_M[m_row_rms][BRN_M_COL_MAX] , rbt_Table_M[m_row_mwt][BRN_M_COL_MAX]/2);
        break;
      }

      case 1:{
        rbt_maneuver(rbt_Table_M[m_row_lms][BRN_M_COL_MAX], rbt_Table_M[m_row_rms][BRN_M_COL_MIN], rbt_Table_M[m_row_mwt][BRN_M_COL_MAX]/2);
        break;
      }

      case 2:{
        rbt_maneuver(rbt_Table_M[m_row_lms][BRN_M_COL_MAX], rbt_Table_M[m_row_rms][BRN_M_COL_MAX], rbt_Table_M[m_row_mwt][BRN_M_COL_MAX]/2);
        break;
      }

      case 3:{
        rbt_maneuver(rbt_Table_M[m_row_lms][BRN_M_COL_MIN], rbt_Table_M[m_row_rms][BRN_M_COL_MIN], rbt_Table_M[m_row_mwt][BRN_M_COL_MAX]/2);
        break;
      }

      case 4:{
        // Do nothing
        break;
      }

      default:{
        // Do nothing
        break;
      }
    }
}


// -------------------------------------------------------
bool checkIsStarted(){

  #ifdef VIRTUAL_ROBOT
    if(commProtocol.checkArgChar(RBS_ARG_RGO) && commProtocol.checkLbl(RBS_LBL_MSG)){
      return true;
    }
    else{
      return false;
    }
  #else
    // Code for Real robot
    // NOT USED
    
  #endif 
    
}

// ------------------------------------------------------------------------------------------------------------
int saveToEEPROM_double2D(double** Arr, int r_cnt, int c_cnt, int add){
  
  //
  // Save 2D Array passed as double pointer to EEPROM
  // Based on Arduino documentation on EEPROM
  // https://docs.arduino.cc/learn/programming/eeprom-guide
  // 
  // Arguments:
  // - double** Arr - 2D array as double pointer
  // - int r_cnt - number of rows
  // - int c_cnt - number of columns
  // - int add - EEPROM adress to start with
  // 
  // Return:
  // - int add - next EEPROM adress just after saved array 
  // 
  // Depends/Updates
  //  - library <EEPROM.h>
  // 
  //
  for(int r = 0; r < r_cnt; r++){
    for(int c = 0; c < c_cnt; c++){
      EEPROM.put(add, Arr[r][c]);
      add += sizeof(double);
    }
  } 

  return add;
}

// ------------------------------------------------------------------------------------------------------------
int readFromEEPROM_double2D(double** Arr, int r_cnt, int c_cnt, int add){

  // Read data from EEPROM into 2D Array (passed as double pointer)
  // Based on Arduino documentation on EEPROM
  // https://docs.arduino.cc/learn/programming/eeprom-guide
  // 
  // Arguments:
  // - double** Arr - 2D array as double pointer
  // - int r_cnt - number of rows
  // - int c_cnt - number of columns
  // - int add - EEPROM adress to start with
  // 
  // Return:
  // - int add - next EEPROM adress just after finishing reading array Arr 
  // 
  // Depends/Updates
  //  - library <EEPROM.h>
  //
   
  for(int r = 0; r < r_cnt; r++){
    for(int c = 0; c < c_cnt; c++){
      EEPROM.get(add, Arr[r][c]);
      add += sizeof(double);
    }
  } 

  return add;
}

// ------------------------------------------------------------------------------------------------------------
int saveNNToEEPROM(unsigned int cnt_alerts, unsigned int cnt_learn){

  // Save the robot's NN weights and some related data to EEPROM

  // reset starting address
  int eeprom_addr = 0; 

  // Save control bytes */
  EEPROM.put(eeprom_addr, eepromControl1);
  eeprom_addr += sizeof(byte);
  EEPROM.put(eeprom_addr, eepromControl2);
  eeprom_addr += sizeof(byte);
  EEPROM.put(eeprom_addr, eepromControl3);
  eeprom_addr += sizeof(byte);
  EEPROM.put(eeprom_addr, eepromControl4);
  eeprom_addr += sizeof(byte);

  // Save each layer nodes biases, bias weights and input weights (excluding input layer)   
  // for each layer (excluding input layer
  for(byte l_idx = 1; l_idx < Robo_NN.getCntL(); l_idx++){
    // for each node
    for(byte n = 0; n < Robo_NN.getCntN(l_idx); n++){
      // save biases
      EEPROM.put(eeprom_addr, Robo_NN.getB(l_idx, n));
      eeprom_addr += sizeof(double);  
        
      // save biases weights
      EEPROM.put(eeprom_addr, Robo_NN.getWb(l_idx, n));
      eeprom_addr += sizeof(double);  

      // for each input weight
      for(byte w = 0; w < Robo_NN.getCntW(l_idx); w++){
        // save inputs weights
        EEPROM.put(eeprom_addr, Robo_NN.getW(l_idx, n , w));
        eeprom_addr += sizeof(double);    
      } // end looping input weights  
    } // end looping nodes    
  } // end looping layers
   
  // save context neurones values
  for(byte i = 0; i < Robo_Brain.getCntC(); i++){
    EEPROM.put(eeprom_addr, Robo_Brain.getValC(i)); 
    eeprom_addr += sizeof(double); 
  }
        
  // save counters
  EEPROM.put(eeprom_addr, cnt_alerts);
  eeprom_addr += sizeof(unsigned int);
  EEPROM.put(eeprom_addr, cnt_learn);
  eeprom_addr += sizeof(unsigned int);

  return eeprom_addr;  
}

int loadNNFromEEPROM(unsigned int& cnt_alerts, unsigned int& cnt_learn){

  // Loads the robot's NN weights and some related data from EEPROM
  // Arguments:
  // - unsigned int cnt_alerts - counnter, stores quantity of obstackes encountered by robot
  // - unsigned int cnt_learn - counnter, stores quantity of single learnings
  // 
  // Return:
  // - int eeprom_addr - next EEPROM adress just after finishing reading NN data

  // set starting address
  int eeprom_addr = 0; 

  // temp variable
  double temp = 0;

  // Load eeprom control bytes and set succes flag if they are OK
  byte controlCheck1;
  byte controlCheck2;
  byte controlCheck3;
  byte controlCheck4;
  EEPROM.get(eeprom_addr, controlCheck1);
  eeprom_addr += sizeof(byte);
  EEPROM.get(eeprom_addr, controlCheck2);
  eeprom_addr += sizeof(byte);
  EEPROM.get(eeprom_addr, controlCheck3);
  eeprom_addr += sizeof(byte);
  EEPROM.get(eeprom_addr, controlCheck4);
  eeprom_addr += sizeof(byte);

  if((controlCheck1==eepromControl1) && (controlCheck2==eepromControl2) && (controlCheck3==eepromControl3) && (controlCheck4==eepromControl4)){
     isNNLoadedFromEEPROM = true;
  }
  else{
     isNNLoadedFromEEPROM = false;
  }

  // Load each layer nodes biases, bias weights and input weights (excluding input layer)   
  // for each layer (excluding input layer
  for(byte l_idx = 1; l_idx < Robo_NN.getCntL(); l_idx++){
    // for each node
    for(byte n = 0; n < Robo_NN.getCntN(l_idx); n++){
      // load biases
      EEPROM.get(eeprom_addr, temp);
      Robo_NN.setB(temp, l_idx, n);
      eeprom_addr += sizeof(double);
        
      // load biases weights
      EEPROM.get(eeprom_addr, temp);
      Robo_NN.setWb(temp, l_idx, n);
      eeprom_addr += sizeof(double);

      // for each input weight
      for(byte w = 0; w < Robo_NN.getCntW(l_idx); w++){
        // load inputs weights
        EEPROM.get(eeprom_addr, temp);
        Robo_NN.setW(temp, l_idx, n, w);
        eeprom_addr += sizeof(double);            
      } // end looping weights   
    } // end looping nodes   
  } // end looping layers

  // load context neurones
  for(byte i = 0; i < Robo_Brain.getCntC(); i++){
    EEPROM.get(eeprom_addr, temp);
    Robo_Brain.setValC(temp, i);
    eeprom_addr += sizeof(double);
  }
  
  // load counters
  EEPROM.get(eeprom_addr, cnt_alerts);
  eeprom_addr += sizeof(unsigned int);
  EEPROM.get(eeprom_addr, cnt_learn);
  eeprom_addr += sizeof(unsigned int);

  return eeprom_addr;  
}


// ------------------------------------------------------------------------------------------------------------
void display_freeram() {
  #ifdef MIN_REPORT_MODE
    Serial.print(F(" - SRAM left: "));
    Serial.println(freeRam());
  #endif
}

int freeRam() {
  extern int __heap_start,*__brkval;
  int v;
  return (int)&v - (__brkval == 0  ? (int)&__heap_start : (int) __brkval);  
}



#ifdef VIRTUAL_ROBOT
// ------------------------------------------------------------------------------------------------------------
void transmitObstaclesCnt(){

  // Send cnt_obstacle_alerts counter info by serial
  // (comment  for real robot)
  commProtocol.setParsedLbl(RBS_LBL_OBS);
  commProtocol.setParsedType(RBS_TYPE_AT);
  commProtocol.ulongToArgArr((unsigned long)cnt_obstacle_alerts);
  commProtocol.commSendParsedFrame(); 
 
}


void transmitLearningsCnt(){

  // Send cnt_learnings_total counters info by serial
  // (comment for real robot)
  commProtocol.setParsedLbl(RBS_LBL_NNL);
  commProtocol.setParsedType(RBS_TYPE_AT);
  commProtocol.ulongToArgArr((unsigned long)cnt_learnings_total);
  commProtocol.commSendParsedFrame(); 
  
}


void transmitNnPredictions(){

  // This will decode NN's output into temporal/virtual sensors raadings column of rbt_Table_S
  //Robo_Brain.evalEncSensData(BRN_O_ID);  

  // Predicted outcom values stored
  // in virtual/temporary column of table S 

  //Serial.println(F("{DEBUG:}"));
  //String valStr = "S[irb][vrt]:";
  //valStr.concat(rbt_Table_S[s_row_irb][BRN_S_COL_VRT]);
  //Serial.println(valStr.c_str());
  
  //valStr = "S[usf][vrt]:";
  //valStr.concat(rbt_Table_S[s_row_usf][BRN_S_COL_VRT]);
  //Serial.println(valStr.c_str());

  //valStr = "S[usl][vrt]:";
  //valStr.concat(rbt_Table_S[s_row_usl][BRN_S_COL_VRT]);
  //Serial.println(valStr.c_str());

  //valStr = "S[usr][vrt]:";
  //valStr.concat(rbt_Table_S[s_row_usr][BRN_S_COL_VRT]);
  //Serial.println(valStr.c_str());

  // Ver 15.2 - also double, not int
  // Send predicted IR bitmap
  commProtocol.setParsedLbl(RBS_LBL_IRB);
  commProtocol.setParsedType(RBS_TYPE_AT);
  if(isnan(rbt_Table_S[s_row_irb][BRN_S_COL_VRT])){
    commProtocol.dblToArgArr((double)0.0);  
  }
  else{
    commProtocol.dblToArgArr((double)rbt_Table_S[s_row_irb][BRN_S_COL_VRT]);
  }
  
  commProtocol.commSendParsedFrame(); 

  // Send predicted distance for forward sector
  commProtocol.setParsedLbl(RBS_LBL_USF);
  commProtocol.setParsedType(RBS_TYPE_AT);
  if(isnan(rbt_Table_S[s_row_usf][BRN_S_COL_VRT])){
    commProtocol.dblToArgArr((double)0.0);  
  }
  else{
    commProtocol.dblToArgArr((double)rbt_Table_S[s_row_usf][BRN_S_COL_VRT]);
  }
  commProtocol.commSendParsedFrame(); 

  // Send predicted distance for left sector
  commProtocol.setParsedLbl(RBS_LBL_USL);
  commProtocol.setParsedType(RBS_TYPE_AT);
  if(isnan(rbt_Table_S[s_row_usl][BRN_S_COL_VRT])){
    commProtocol.dblToArgArr((double)0.0);  
  }
  else{
    commProtocol.dblToArgArr((double)rbt_Table_S[s_row_usl][BRN_S_COL_VRT]);
  }
  commProtocol.commSendParsedFrame(); 

  // Send predicted distance for right sector
  commProtocol.setParsedLbl(RBS_LBL_USR);
  commProtocol.setParsedType(RBS_TYPE_AT);
  if(isnan(rbt_Table_S[s_row_usr][BRN_S_COL_VRT])){
    commProtocol.dblToArgArr((double)0.0);    
  }
  else{
    commProtocol.dblToArgArr((double)rbt_Table_S[s_row_usr][BRN_S_COL_VRT]);  
  }  
  commProtocol.commSendParsedFrame();

  // Send predicted distance for obstacle free run timer
  // Send as double, not convert to ulong
  //commProtocol.setParsedLbl(RBS_LBL_OFT);
  //commProtocol.setParsedType(RBS_TYPE_AT);
  //commProtocol.dblToArgArr(rbt_Table_S[s_row_oft][BRN_S_COL_VRT]);
  //commProtocol.commSendParsedFrame(); 
  
}


void transmitContextNeurones(){

  // Send context neurone 1
  commProtocol.setParsedLbl(RBS_LBL_CN1);
  commProtocol.setParsedType(RBS_TYPE_AT);
  commProtocol.dblToArgArr(Robo_Brain.getelX(Robo_Brain.getNnIORefC(0)));
  commProtocol.commSendParsedFrame();

  // Send context neurone 2
  commProtocol.setParsedLbl(RBS_LBL_CN2);
  commProtocol.setParsedType(RBS_TYPE_AT);
  commProtocol.dblToArgArr(Robo_Brain.getelX(Robo_Brain.getNnIORefC(1)));
  commProtocol.commSendParsedFrame(); 
}


void transmitCommandToSaveInitSens(){

  commProtocol.commSendCommand(RBS_LBL_LOG, RBS_ARG_INI_S);  
}


void transmitCommandToSaveInitMPG(){

  commProtocol.commSendCommand(RBS_LBL_LOG, RBS_ARG_INI_M); 
}


void transmitCommandToCopyLastMPG(){

  commProtocol.commSendCommand(RBS_LBL_LOG, RBS_ARG_LST_M);   
}


void transmitCommandToSaveResultSens(){
  
  commProtocol.commSendCommand(RBS_LBL_LOG, RBS_ARG_RES_S);   
}

void transmitCommandNnLogNextRow(){

  commProtocol.commSendCommand(RBS_LBL_LOG, RBS_ARG_NXT); 
}

void transmitCommandToSaveMoveFwMPG(){

  commProtocol.commSendCommand(RBS_LBL_LOG, RBS_ARG_FWD_M); 
}

void transmitCommandToUpdateInitMwt(){

  commProtocol.commSendCommand(RBS_LBL_LOG, RBS_ARG_MWT_U);
}

// ------------------------------------------------------------------------------------------------------------
void exportNN(){
  
}
#endif
