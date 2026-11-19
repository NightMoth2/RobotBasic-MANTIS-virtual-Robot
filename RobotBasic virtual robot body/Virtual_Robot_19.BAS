// Version 19-3 2026-08-17
// - Sectors for IR sensors added (now each IR sensor "sees" ~20deg sector
//
// Version 19-2 2026-07-24
// - FSM nodes update and rename (improve self-documenting, re-arranging more logically)
//
// Version 19 2026-07-12
// - Save Terminal screen Robot coordinates and heading into NN Log table
// - Save two context neurones inro NN Log table
// - Save NN log after every 10 obstacles
//
// Version 16 2026-07-06
// - IR bumpers readings is bitmap value again
//
// Version 10 2026-06-22
// - IR bumpers readings now binary value instead of bitmap
//   (0 - all bumpers clear, 1 - any is bumper active)
//   Function RBT_calcIRBumperBitmap() changed
//
// Version 9 2026-06-02
// - FSM realisation
// - new control variables : RBT_motors_work_steps_cnt, RBT_mnvr_params_set_cnt, ... etc
// - reneming of some methods and vars
// - cleaning code
// - auto exit if stucked
// - taking into account programm's generator min motor working time parameter (RBT_MIN_MANEUVER_MWT)
//
// Version 7 2026-05-28
// - Collision error preveintion implemented via error handling procedure
//
// Experiment design [3]
// m - num of samples (rows)
// n - num of features (cols)
// d - model parameters (num of weights? )
// c - num of classes
// w - num of weights
//
// NN:
// Input - 9 nodes
// Hidden 1 - 6 nodes
// Hidden 2 - 6 nodes
// Output - 4 nodes
// weights = 9*6 + 6*6 + 4*6 = 114
// bias weights = 6 + 6 + 4  = 16
// total weights w = d = 130
// 
// "classes":
// fw_free, fw_obst, lft_free, lft_obst, rght_free, rght_obst, 
// irb_free, irb_obst = 8
//
// m (classification) >= 10*n*c (class) = 10*9*8 = 720 
// m (regression) >= 50*n = 450 (per parameter? if yes - 4*450 = 1800?)
// d <= m/10 (or m/50) => 
// m >= 10...50*d >= 1300...6500
//
// => training data (= row trainings) ~= 1500
//
// Refs:
// Kinematics
// [1] A. Haber, “Clear and Detailed Explanation of Kinematics, Equations, and Geometry of Motion of Differential Wheeled Robot 
//    (Differential Drive Robot),” Aleksandar Haber. Accessed: May 24, 2026. [Online]. Available: 
//    https://aleksandarhaber.com/clear-and-detailed-explanation-of-kinematics-equations-and-geometry-of-motion-of-differential-wheeled-robot-differential-drive-robot/
//
// Simulator collision-error prevention (before Version 7):
// [2] K. Brik, “Distance between points on two circles,” Physics Forums. Accessed: May 24, 2026. [Online]. Available: 
//     https://www.physicsforums.com/threads/distance-between-points-on-two-circles.1085029/
//
// [3] https://towardsdatascience.com/machine-learning-rules-of-thumb-b50232b4b2f8/
//
// [4] https://www.geeksforgeeks.org/machine-learning/metrics-for-machine-learning-model/



//==============================================================================================
#include "LOGGING_01.BAS"   // Logging purposes
#include "GENERAL_02.BAS"   // date string, functions for coordinate system and vector transformations
#include "RBSERIAL_02.BAS"  // serial ARQ protocol
//==============================================================================================

// --------------------------
// GLOBAL CONSTANTS/VARIABLES
// -------------------------- 
//==============================================================================================
// ROBOT BASIC - UART/Serial comm related
UART_BAUD_RATE = br115200   // UART baud rate
UART_COM_PORT_DEF = 4       // Default COM port
UART_FLAG_CONNECTED = 0     // Flag, indicates UART connection established

//==============================================================================================
// ROBOT BASIC - Logging
TMR_PROG_START = Timer()                    // Log timer
LOG_WRITE_FLAG = true                       // write log flag

Call GEN_makeDateTimeString()
strDateTime = GEN_makeDateTimeString__Result    // DateTime string to label log filename

LOG_FN = strDateTime + " - " + "LOG.txt"    // log file name
LOG_FH = Filecreate(LOG_FN)                 // log file handler 

//==============================================================================================
// Labels/Tags for comm. protocol messages
// (Same as in connected device)
RBS_LBL_LMS = "A"          // Label for Left motor speed commands
RBS_LBL_RMS = "B"          // Label for Right motor speed commands
RBS_LBL_MWT = "C"          // Label for Motor working timer commands
RBS_LBL_IRB = "D"          // Label for Infrared Bumpers commands
RBS_LBL_USF = "E"          // Label for Ultrasonic sensor forward sector commands
RBS_LBL_USL = "F"          // Label for Ultrasonic sensor left sector commands
RBS_LBL_USR = "G"          // Label for Ultrasonic sensor right sector commands
RBS_LBL_OFT = "H"          // Label for obstacle-free time commands
RBS_LBL_ALS = "I"          // Label for "All sensors" commands
RBS_LBL_RBT = "J"          // Label for "Robot" commands
RBS_LBL_MSG = "K"          // Label for "Message" commands
RBS_LBL_OBS = "L"          // Label for obstacles counter
RBS_LBL_NNL = "M"          // Label for nn learnings counter
RBS_LBL_LOG = "N"          // Label for commands related to logging
RBS_LBL_CN1 = "O"          // Label for context neurone 1
RBS_LBL_CN2 = "P"          // Label for context neurone 2

RBS_ARG_EXE = "A"          // Argument for robot command - exec motor programm
RBS_ARG_FWD = "B"          // Argument for robot command - go forward
RBS_ARG_STP = "C"          // Argument for robot command - stop
RBS_ARG_RGO = "D"          // Argument for robot command - robot GO!
RBS_ARG_INI_S = "E"        // Argument for Logging command: save initial sensors data to log table                                
RBS_ARG_INI_M = "F"        // Argument for Logging command: save initial motor programm to log table
RBS_ARG_LST_M = "G"        // Argument for Logging command: save last/previous motor programm to log table
RBS_ARG_RES_S = "H"        // Argument for Logging command: save result sensors data to log table
RBS_ARG_FWD_M = "I"        // Argument for Logging command: save "Move forward" motor program to log table
RBS_ARG_MWT_U = "J"        // Argument for Logging command: Update motor working time in log table
RBS_ARG_NXT = "K"          // Argument for Logging command: start next row in log table

//==============================================================================================
// SIMULATION CONSTANTS
SIM_MS_PER_TICK = 250.0          // mS/step, approx ratio of real time per simulator step, picked experementally
SIM_MM_PER_PX = 5.0                 // mm per pixel
SIM_PX_PER_MM = 1/SIM_MM_PER_PX     // pixel per mm

//==============================================================================================
// ROBOT CONSTANTS
// Size
RBT_LENGTH_MM = 240.0                                   // mm, robot length
RBT_WIDTH_MM = 160.0                                    // mm, wheel to wheel, robot width
RBT_WIDTH_PX = RBT_WIDTH_MM*SIM_PX_PER_MM               // px, robot width
RBT_RADIUS_MM = RBT_WIDTH_MM/2                          // mm, radius of simulated robot
RBT_RADIUS_PX = RBT_RADIUS_MM*SIM_PX_PER_MM             // px, radius of simulated robot, must be 5-50 pixels, 20 pixels default
RBT_WHEEL_RADIUS_MM = 33.0                              // mm, radius of robot's wheels

// Speed
RBT_MIN_SPD_PWM = 0.0                                   // PWM (Arduino PWM control is on a scale 0-255)
RBT_MAX_SPD_PWM = 250.0                                 // PWM (Arduino PWM control is on a scale 0-255)
RBT_MAX_SPD_MMMS = 0.5                                  // mm/mS, max rorot speed (found experementally)
RBT_MIN_SPD_PXTICK = 0.0                                // px/tick, min rorot speed
RBT_MAX_SPD_PXTICK = SIM_MS_PER_TICK*(RBT_MAX_SPD_MMMS*SIM_PX_PER_MM)  // px/tick, max rorot speed  (mS/step)*((mm/mS)*(px/mm)) = px
RBT_MAX_BATTERY_TIME = 3000000                          // mS, approx, used as default value for motors workttime 
                                                        // in "move forward" motor programm
// Sensors 1
RBT_IR_RANGE_MM = 50                                    // mm, detection range of IR bumper sensors, bumper fires alert if there is an obstacle within this range
RBT_IR_RANGE_PX = Round(RBT_IR_RANGE_MM*SIM_PX_PER_MM)  // px, detection range of IR bumper sensors
RBT_US_RANGE_MM = 400                                   // mm, max range of ultrasonic sensor, detection on longer distances will be trimmed (not realable)
RBT_US_RANGE_PX = Round(RBT_US_RANGE_MM*SIM_PX_PER_MM)  // px, max range of ultrasonic sensor

// Sensors 2
// Specific for Arduino robot "RobotCar_01": 
// Sonar on "RobotCar_01" checks 9 angles: three sectors (FW, L, R), three angles in each sector
RBT_US_ANGS_PER_SECTOR = 3                          // number of angles in each sector
Dim RBT_US_FORWARD_ANGLES[RBT_US_ANGS_PER_SECTOR]   // array of angles in forward sector
RBT_US_FORWARD_ANGLES[0] = -20
RBT_US_FORWARD_ANGLES[1] = 0
RBT_US_FORWARD_ANGLES[2] = 20
Dim RBT_US_LEFT_ANGLES[RBT_US_ANGS_PER_SECTOR]      // array of angles in left sector
RBT_US_LEFT_ANGLES[0] = -90
RBT_US_LEFT_ANGLES[1] = -80
RBT_US_LEFT_ANGLES[2] = -70
Dim RBT_US_RIGHT_ANGLES[RBT_US_ANGS_PER_SECTOR]   // array of angles in right sector
RBT_US_RIGHT_ANGLES[0] = 70
RBT_US_RIGHT_ANGLES[1] = 80
RBT_US_RIGHT_ANGLES[2] = 90

RBT_IR_ANGS_PER_SECTOR = 3
Dim RBT_IR_FORWARD_ANGLES[RBT_IR_ANGS_PER_SECTOR]   // array of angles in forward sector
RBT_IR_FORWARD_ANGLES[0] = -10
RBT_IR_FORWARD_ANGLES[1] = 0
RBT_IR_FORWARD_ANGLES[2] = 10
Dim RBT_IR_LEFT_ANGLES[RBT_IR_ANGS_PER_SECTOR]      // array of angles in left sector
RBT_IR_LEFT_ANGLES[0] = -70
RBT_IR_LEFT_ANGLES[1] = -80
RBT_IR_LEFT_ANGLES[2] = -90
Dim RBT_IR_RIGHT_ANGLES[RBT_IR_ANGS_PER_SECTOR]   // array of angles in right sector
RBT_IR_RIGHT_ANGLES[0] = 70
RBT_IR_RIGHT_ANGLES[1] = 80
RBT_IR_RIGHT_ANGLES[2] = 90

// Motor programm
RBT_MPROG_PARAMS_TOT_CNT = 3    // Number of motor programm parameters (speed for left and right motor, motors working time)
RBT_MIN_MANEUVER_MWT = 100.0    // mS, minimum motor working time during maneuver, specific for Arduino robot "RobotCar_01" 
                                // (to round-up if less than SIM_MS_PER_TICK)
                                
// RobotBasic setting for robot
RBT_START_POSX = 700.0          // px, starting pos x on terminal screen in lho system
RBT_START_POSY = 550.0          // px, starting pos y on terminal screen in lho system
RBT_START_ANG = 0               // deg, robot stating heading in nlho system
RBT_DISP_COLOR = Red            // robot color on terminal screen
RBT_RBASIC_SPD = 3.0            // simulated robot speed on terminal screen, 
                                // "rSpeed" 0 - max speed, The larger the number the slower the robot. Picked manually
// Robot stuck prevention / exit
RBT_STUCK_TICKS_THRESHOLD = 10  // Finish simulation if the robot is in "MOVE FORWARD" mode
                                // but position of the robot doesnt change more than threshold ticks

//==============================================================================================
// ROBOT FSM
FSM_BATTERY_CHECK = 1
FSM_EXIT = 2
FSM_MANEUVER_CONTINUE = 3
FSM_MANEUVER_FINISHED = 4
FSM_MANEUVER_INIT = 5
FSM_MANEUVER_MODE_ENTER = 6
FSM_MANEUVER_PROGRAM_CHECK = 7 
FSM_MANEUVER_TICK = 8
FSM_MOVE_FORWARD_INIT = 9 
FSM_MOVE_FORWARD_MODE_ENTER = 10 
FSM_MOVE_FORWARD_TICK = 11 
FSM_MOVEMENT_TICK_START = 12
FSM_MOVEMENT_TICK_END = 13
FSM_QUERY_EXECUTE = 14 
FSM_QUERY_NOT_RECEIVED = 15 
FSM_QUERY_RECEIVE = 16
FSM_RESTART_CYCLE = 17
FSM_SETUP = 18
FSM_STOP_INIT = 19
FSM_STOP_MODE_ENTER = 20
FSM_STOP_TICK = 21

FSM_state = FSM_SETUP   // Set starting state
FSM_run = true          // Set Run fsm flag

//==============================================================================================                                
// Robot control variables
// Motors:
// - Real (commands to real notors received from Arduino by serial)
RBT_left_motor_spdpwm = 0.0       // PWM 0-255
RBT_right_motor_spdpwm = 0.0      // PWM 0-255
RBT_motors_work_time = 0.0        // mS
// - Simulated (to be calculated based on commands to real notors)
RBT_motors_work_ticks_cnt = 0.0   // steps
RBT_move_per_tick_px = 0.0        // px/tick (float to allow float calculations)
RBT_turn_per_tick = 0.0           // deg/tick (float to allow float calculations)
// Sensors:
// (simulated sensors data to be coverted from px to cm and to be sent via serial to Arduino)
RBT_ir_bumper_bitmap = 0        // bitmap |L|R|F|T|, IR bumper data, NOT the same as RobotBasic default bumper
RBT_us_forward_dist_px = 0.0    // px, ultrasonic, forward sector
RBT_us_forward_dist_cm = 0.0    // cm, ultrasonic, forward sector
RBT_us_left_dist_px = 0.0       // px, ultrasonic, left sector
RBT_us_left_dist_cm = 0.0       // cm, ultrasonic, left sector
RBT_us_right_dist_px = 0.0      // px, ultrasonic, right sector
RBT_us_right_dist_cm = 0.0      // cm, ultrasonic, right sector
RBT_obst_free_ticks = 0         // fsm ticks, robot moves before detects and obstacle
RBT_obst_free_ms = 0.0          // mS, robot moves before detects and obstacle
// Status/Mode (USED BY FSM)
RBT_MODE_STR_STOP = "STOP"                  // robot stopped
RBT_MODE_STR_MANEUVER = "MANEUVER"          // robot performing evade maneuver
RBT_MODE_STR_MVFORWARD = "MOVE FORWARD"     // robot moving forward
RBT_is_moving_fw = false                    // flag, is moving forward
RBT_movement_mode_str = RBT_MODE_STR_STOP   // default movement mode
RBT_mnvr_continue = false                   // flag, need to continue maneuver (motor working time/steps has not been elapsed)
RBT_mnvr_ticks_done_cnt = 0                 // counter: maneuver movement ticks competed
RBT_mnvr_params_set_cnt = 0                 // number of motor programm parameters received, 
                                            // motor programm ready if it is == RBT_MPROG_PARAMS_TOT_CNT
// Robot stuck prevention / exit
// (XY position of the robot to track wether robot moves is stuck)
RBT_control_Xpos = 0
RBT_control_Ypos = 0

//==============================================================================================
// Simulation control variables
SIM_ticks_cnt = 0          // counts FSM cycles (robot movement cycles)


//==============================================================================================
// Statistics
STAT_comm_fails_tot = 0          // counter: total number of communication fails/cycles without incoming communication

STAT_total_obst_cnt = 0                             // counter, total number of obstacles
STAT_total_obst_cnt_onstart = 0                     // total number of obstacles at power-ON
STAT_total_obst_cnt_received_flag = false           // flag, true if initial counter data received via serial
STAT_total_nn_learnings_cnt = 0                     // counter, total number of NN learnings
STAT_total_nn_learnings_cnt_onstart = 0             // total number of NN learnings at power-ON
STAT_total_nn_learnings_cnt_received_flag = false   // flag, true if initial counter data received via serial

STAT_session_obst_cnt = 0                           // counter, number of obstacles during session
STAT_session_nn_learnings_cnt = 0                   // counter, number of NN learnings during session
STAT_session_oft_ticks_cnt = 0                      // counter, number of obstacle-free ticks during session
STAT_session_mnvr_ticks_cnt = 0                     // counter, number of maneuver ticks during session

//==============================================================================================
// NN performance log table 
//nnlog_row_started = false         // flag to indicate that new row has beed started
nnlog_row_idx = 0                   // idx of current row to save data
nnlog_rows_cnt = 5000               // number of rows
nnlog_cols_cnt = 27                 // number of columns
Dim NN_Log_Tbl[nnlog_rows_cnt,nnlog_cols_cnt]
// Columns indexes and definitions:
nnlog_col_timestamp = 0             // column to save filename datastamp
nnlog_col_obst_num = 1              // current obstacle number
nnlog_col_lrn_num = 2               // current learnings number
nnlog_col_row_number = 3            // row number (session event number)
nnlog_col_irb_ini = 4               // bitmap, IR bumbers bitmap/score  (initial)
nnlog_col_usf_ini = 5               // cm, input, distance to obstacle in forward direction (initial)
nnlog_col_usl_ini = 6               // cm, input, distance to obstacle in left direction (initial)
nnlog_col_usr_ini = 7               // cm, input, distance to obstacle in right direction (initial)
nnlog_col_oft_ini = 8               // mS, input, readings of obstacle-free timer (how much time robot moves free before current obstackle)
nnlog_col_mpl_ini = 9               // PWM, input, left motor speed (initial)
nnlog_col_mpr_ini = 10              // PWM, input, right motor speed (initial)
nnlog_col_mwt_ini = 11              // mS, input, motor working time (initial)
nnlog_col_irb_est = 12              // bitmap, IR bumbers bitmap/score  (estimated/predicted)
nnlog_col_usf_est = 13              // cm, input, distance to obstacle in forward direction (estimated/predicted)
nnlog_col_usl_est = 14              // cm, input, distance to obstacle in left direction (estimated/predicted)
nnlog_col_usr_est = 15              // cm, input, distance to obstacle in right direction (estimated/predicted)
nnlog_col_oft_est = 16              // mS, input, readings of obstacle-free timer (estimated/predicted)
nnlog_col_irb_res = 17              // bitmap, IR bumbers bitmap/score  (result)
nnlog_col_usf_res = 18              // cm, input, distance to obstacle in forward direction (result)
nnlog_col_usl_res = 19              // cm, input, distance to obstacle in left direction (result)
nnlog_col_usr_res = 20              // cm, input, distance to obstacle in right direction (result)
nnlog_col_oft_res = 21              // mS, input, readings of obstacle-free timer (estimated/predicted)
nnlog_col_posX_ini = 22             // Robot position on terminal screen coordinate X, lho system
nnlog_col_posY_ini = 23             // Robot position on terminal screen coordinate Y, lho system
nnlog_col_hdir_ini = 24             // degrees, Robot heading direction (0-359) 0= North.
nnlog_col_cn1_ini = 25              // value of context neuron 1
nnlog_col_cn2_ini = 26              // value of context neuron 2

// First row initialisation

// Init first row with zeroes
For j = 0 to nnlog_cols_cnt - 1
    NN_Log_Tbl[0, j] = 0
Next

// Init first row with default values
NN_Log_Tbl[0, nnlog_col_timestamp] = strDateTime
NN_Log_Tbl[0, nnlog_col_row_number] = nnlog_row_idx
NN_Log_Tbl[0, nnlog_col_mpl_ini] = RBT_MAX_SPD_PWM
NN_Log_Tbl[0, nnlog_col_mpr_ini] = RBT_MAX_SPD_PWM
NN_Log_Tbl[0, nnlog_col_mwt_ini] = RBT_MAX_BATTERY_TIME
NN_Log_Tbl[0, nnlog_col_posX_ini] = RBT_START_POSX
NN_Log_Tbl[0, nnlog_col_posY_ini] = RBT_START_POSY 
NN_Log_Tbl[0, nnlog_col_hdir_ini] = RBT_START_ANG
nnlog_row_idx++


//==============================================================================================
// NEURAL NETWORK EXPORT (Must be same as on Arduino)
// (NOT implemented yet)
NN_L_cnt = 3
NN_L0N_cnt = 10
NN_L1N_cnt = 7
NN_L2N_cnt = 5

NN_L1B_cnt = NN_L1N_cnt
NN_L2B_cnt = NN_L2N_cnt

Dim NN_L1_A[NN_L1N_cnt]
Dim NN_L1_B[NN_L1N_cnt]
Dim NN_L1_W[NN_L1N_cnt, NN_L0N_cnt]

Dim NN_L2_A[NN_L2N_cnt]
Dim NN_L2_B[NN_L2N_cnt]
Dim NN_L2_W[NN_L2N_cnt, NN_L1N_cnt]

//==============================================================================================



//==============================================================================================
MainProgramm:

    Call LOG_write("Main program started!")
    
    // "Libraries"
    gosub Init_lib_RBS             // Init communication protocol variables (from #include "RBSERIAL_02.BAS")
    gosub Init_lib_GEN             // Init general library (from #include "GENERAL_02.BAS")
    
    // Terminal screen
    Call InitGUI()               // Place GUI elements
    Call InitScr()               // Draw grid, put objects, locate robot
    Call initRobot()             // Place and initialize the Robot 
    
    // event handlers
    OnError eHandler                // erroe handler
    onbutton bHandler               // GUI button pressed interrupt
    OnAbort abortHandler            // on abort handler
    
    // FSM
    Call LOG_write("Starting FSM...")
    
    FSM_run = true
    FSM_state = FSM_SETUP
    
    while FSM_run == true

        // ------------------------------------
        if FSM_state == FSM_BATTERY_CHECK
        
            Call LOG_write("FSM_BATTERY_CHECK ")
            
            // Next state
            if rChargeLevel() > 0            
                FSM_state = FSM_MOVEMENT_TICK_START
            else
                FSM_state = FSM_EXIT
            endif                 
        // END FSM_BATTERY_CHECK   


        // ------------------------------------
        elseif FSM_state == FSM_EXIT

            Call LOG_write("FSM_EXIT")
                                                         
            FSM_run = false
        // END FSM_EXIT 
        
        
        // ------------------------------------
        elseif FSM_state == FSM_MANEUVER_CONTINUE
        
            Call LOG_write("FSM_MANEUVER_CONTINUE") 
            
            RBT_mnvr_continue = true
            
            // Next state
            FSM_state =  FSM_RESTART_CYCLE                    
        // END FSM_MANEUVER_CONTINUE
        
        
        // ------------------------------------
        elseif FSM_state == FSM_MANEUVER_FINISHED
        
            Call LOG_write("FSM_MANEUVER_FINISHED")
            
            RBT_mnvr_continue = false
            RBT_mnvr_params_set_cnt = 0
            
            // Next state
            FSM_state =  FSM_STOP_MODE_ENTER       
        // END FSM_MANEUVER_FINISHED        

        
        // ------------------------------------
        elseif FSM_state == FSM_MANEUVER_INIT

            Call LOG_write("FSM_MANEUVER_INIT") 
            
            RBT_is_moving_fw = false
            RBT_obst_free_ticks = 0
            RBT_mnvr_continue = true
            RBT_mnvr_ticks_done_cnt = 0                
            Call RBT_calcSimMove()
            Call SCR_displayData()
            
            rPen Down,red
            
            // Next state
            FSM_state =  FSM_MANEUVER_TICK
        // END FSM_MANEUVER_INIT
        
                                
        // ------------------------------------
        elseif FSM_state == FSM_MANEUVER_MODE_ENTER

            Call LOG_write("FSM_MANEUVER_MODE_ENTER") 

            // Next state
            if RBT_mnvr_continue == true
                FSM_state = FSM_MANEUVER_TICK
            else
                FSM_state = FSM_MANEUVER_PROGRAM_CHECK
            endif               
        // END FSM_MANEUVER_MODE_ENTER                                                       

                                                                                
        // ------------------------------------
        elseif FSM_state == FSM_MANEUVER_PROGRAM_CHECK

            Call LOG_write("FSM_MANEUVER_PROGRAM_CHECK") 

            // Next state
            if RBT_mnvr_params_set_cnt == RBT_MPROG_PARAMS_TOT_CNT
                FSM_state = FSM_MANEUVER_INIT
            else
                FSM_state = FSM_STOP_MODE_ENTER
            endif               
        // END FSM_MANEUVER_PROGRAM_CHECK                                                                                                                                                                                                                                                 


        // ------------------------------------
        elseif FSM_state == FSM_MANEUVER_TICK
        
            Call LOG_write("FSM_MANEUVER_TICK")
            
            rTurn Roundup(_RBT_turn_per_tick)
            rForward _RBT_move_per_tick_px
        
            RBT_mnvr_ticks_done_cnt++
            RBT_obst_free_ticks++
            STAT_session_mnvr_ticks_cnt++
            
            Call LOG_write("Steps total, cnt = " + ToString(RBT_motors_work_ticks_cnt))
            Call LOG_write("Steps done, cnt = " + ToString(RBT_mnvr_ticks_done_cnt))  
            
            // Next state
            if RBT_mnvr_ticks_done_cnt >= RBT_motors_work_ticks_cnt
               FSM_state = FSM_MANEUVER_FINISHED
            else
               FSM_state = FSM_MANEUVER_CONTINUE
            endif                   
        // END FSM_MANEUVER_TICK                                                                                                                                                                                                                                                                                                                                                                                                                                  
                                                                                                                                

        // ------------------------------------
        elseif FSM_state == FSM_MOVE_FORWARD_INIT

            Call LOG_write("FSM_MOVE_FORWARD_INIT")
            
            RBT_is_moving_fw = true
            RBT_obst_free_ticks = 0
            RBT_mnvr_params_set_cnt = 0 
            
            RBT_left_motor_spdpwm = RBT_MAX_SPD_PWM
            RBT_right_motor_spdpwm = RBT_MAX_SPD_PWM
            RBT_motors_work_time = RBT_MAX_BATTERY_TIME
                                    
            rGPS RBT_control_Xpos, RBT_control_Ypos
            
            Call RBT_calcSimMove()
            Call SCR_displayData() 
            
            rPen Down,cyan                
        
            // Next state
            FSM_state =  FSM_MOVE_FORWARD_TICK
        // END FSM_MOVE_FORWARD_INIT                                                                                                                                                                                                                                                                                                                
                                                                                                                                                                                                        
                        
        // ------------------------------------
        elseif FSM_state == FSM_MOVE_FORWARD_MODE_ENTER

            Call LOG_write("FSM_MOVE_FORWARD_MODE_ENTER")
            
            // Next state
            if RBT_is_moving_fw == true
                FSM_state = FSM_MOVE_FORWARD_TICK            
            else
                FSM_state = FSM_MOVE_FORWARD_INIT             
            endif              
        // END FSM_MOVE_FORWARD_MODE_ENTER

        
        // ------------------------------------
        elseif FSM_state == FSM_MOVE_FORWARD_TICK
        
            Call LOG_write("FSM_MOVE_FORWARD_TICK")
            
            rForward _RBT_move_per_tick_px     
            
            RBT_obst_free_ticks++ 
            STAT_session_oft_ticks_cnt++ 
            rGPS newX, newY
            
            // Next state            
            if (RBT_control_Xpos == newX) && (RBT_control_Ypos == newY) && (RBT_obst_free_ticks >= RBT_STUCK_TICKS_THRESHOLD) 
                Call LOG_write("ROBOT STUCK DETECTED!")
                FSM_state = FSM_EXIT
            else        
                FSM_state = FSM_MOVEMENT_TICK_END
            endif 
        // END FSM_MOVE_FORWARD_TICK                

                                
        // ------------------------------------
        elseif FSM_state == FSM_MOVEMENT_TICK_START
        
            Call LOG_write("FSM_MOVEMENT_TICK_START")
                        
            // Next state            
            if RBT_movement_mode_str == RBT_MODE_STR_MVFORWARD 
                FSM_state = FSM_MOVE_FORWARD_MODE_ENTER
            elseif RBT_movement_mode_str == RBT_MODE_STR_MANEUVER
                FSM_state = FSM_MANEUVER_MODE_ENTER
            elseif RBT_movement_mode_str == RBT_MODE_STR_STOP
                FSM_state = FSM_STOP_MODE_ENTER
            else
                FSM_state = FSM_MOVEMENT_TICK_END
            endif
        // END FSM_MOVEMENT_TICK_START                                                                                                   
                
                        
        // ------------------------------------
        elseif FSM_state == FSM_MOVEMENT_TICK_END
        
            Call LOG_write("FSM_MOVEMENT_TICK_END")
                        
            // Next state            
            FSM_state = FSM_QUERY_RECEIVE
        // END FSM_MOVEMENT_TICK_END                                  

                                        
        // ------------------------------------
        elseif FSM_state == FSM_QUERY_EXECUTE
        
            Call LOG_write("FSM_QUERY_EXECUTE") 
        
            Call RBT_execQuery()
            
            // Next state
            FSM_state = FSM_RESTART_CYCLE         
        // END FSM_QUERY_EXECUTE                                                                                

                                                                                                                        
        // ------------------------------------
        elseif FSM_state == FSM_QUERY_NOT_RECEIVED
        
            Call LOG_write("FSM_QUERY_NOT_RECEIVED")
            
            STAT_comm_fails_tot++
            
            // Next state
            FSM_state = FSM_RESTART_CYCLE                    
        // END FSM_QUERY_NOT_RECEIVED                                                                                                                                                                                                                                                                                                                                                                          

                                                                                                                                                                
        // ------------------------------------
        elseif FSM_state == FSM_QUERY_RECEIVE
        
            Call LOG_write("FSM_QUERY_RECEIVE") 
            
            Call RBS_commReceive()
            
            Call LOG_write("SER IN: " + ToString(_RBS_RAW_IN))
            _RBS_RAW_IN = "" 
            
            // Next state
            if RBS_commReceive__Result == true        
                FSM_state = FSM_QUERY_EXECUTE
            else
                FSM_state = FSM_QUERY_NOT_RECEIVED
            endif                                                            
        // END FSM_QUERY_RECEIVE                                                                                                                                                                                                                                                                                                                                


        // ------------------------------------
        elseif FSM_state == FSM_RESTART_CYCLE
        
            Call LOG_write("FSM_RESTART_CYCLE")
            
            SIM_ticks_cnt++
            
            Call LOG_write("-----------------------") 
            Call LOG_write("STEP # " + ToString(SIM_ticks_cnt))
            
            Call SCR_displayData()
                       
            // Next state
            FSM_state = FSM_BATTERY_CHECK                   
        // END FSM_RESTART_CYCLE                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
                                                                                                                                                                                                                                                
                                                
        // ------------------------------------
        elseif FSM_state == FSM_SETUP
        
            Call LOG_write("FSM_SETUP")
            
            SIM_ticks_cnt = 0
            STAT_comm_fails_tot = 0
            RBT_mnvr_params_set_cnt = 0
            RBT_mnvr_ticks_done_cnt = 0
            RBT_obst_free_ticks = 0
            RBT_mnvr_continue = false
            RBT_is_moving_fw = false
            RBT_movement_mode_str = RBT_MODE_STR_STOP
            
            // Next state
            FSM_state = FSM_RESTART_CYCLE
        // END FSM_SETUP                                                        

 
        // ------------------------------------
        elseif FSM_state == FSM_STOP_INIT

            Call LOG_write("FSM_STOP_INIT")
            
            RBT_is_moving_fw = false
            RBT_mnvr_continue = false
            RBT_mnvr_params_set_cnt = 0
            RBT_turn_per_tick = 0
            RBT_move_per_tick_px = 0
            Call SCR_displayData()                                             
        
            // Next state
            FSM_state = FSM_STOP_TICK
        // END FSM_STOP_INIT
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
                                                                                                                                                                               
        // ------------------------------------
        elseif FSM_state == FSM_STOP_MODE_ENTER
        
            Call LOG_write("FSM_STOP_MODE_ENTER")
            
            RBT_movement_mode_str = RBT_MODE_STR_STOP
            
            // Next state
            if RBT_is_moving_fw == true
                FSM_state = FSM_STOP_INIT            
            else
                FSM_state = FSM_STOP_TICK             
            endif             
        // END FSM_STOP_MODE_ENTER                         

                                
        // ------------------------------------
        elseif FSM_state == FSM_STOP_TICK
        
            Call LOG_write("FSM_STOP_TICK")
            
            // Do nothing
            
            // Next state
            FSM_state = FSM_MOVEMENT_TICK_END
        // END FSM_STOP_TICK        
        
                                                                                          
        // ------------------------------------
        else

            Call LOG_write("FSM_DEFAULT") 
            
            // Do nothing
            
            // Next state
            FSM_state = FSM_RESTART_CYCLE       
        // END DEFAULT
            
        endif
        // END FSM IF/ELSE
                                        
    wend // END FSM LOOP
    
    // Auto exit after FSM stopped
    WriteScr  strDateTime + " - " + "SCR_FSM_END.bmp" 
    //mWrite NN_Log_Tbl, strDateTime + " - " + "NNLOG_FSM_END.txt"  
    Call STAT_save_NN_log_tbl()    
    Call STAT_calcStat()
    Call STAT_statToLog()      
    FileClose(_LOG_FH) // Close LOG file 
    SetCommPort 0      // Close serial connection       
    Terminate          // Terminate program
         
end


//==============================================================================================
Sub InitGUI()

    //Call LOG_write("PROCSTART: InitGUI")

    // GUI UART related
    addbutton "Connect to UART",10,10               // Click on this button to connect to Uart
    SerPorts vs_PortsList                           // List of available COM ports
    addlistbox "COMList", 130,10, 100, vs_PortsList // Select COM port to connect here
    
    // Send 
    xyText 560,115,"Wait 5-10 sec"
    xyText 560,130,"after serial comm"
    addbutton "Ready", 500, 120
    
    //Call LOG_write("PROCEND: InitGUI")

return
//==============================================================================================
Sub InitScr()
    
    //Call LOG_write("FSTART: InitScr()")
               
    //SCR_drawGrid_mm(lhx,lhy,majxmm,minxmm,majymm,minymm,pix_per_mm)
    Call SCR_drawGrid_mm(250,150,500,100,500,100,_SIM_PX_PER_MM)
    
    // MAZE
    //Line ne_X1,ne_Y1,ne_X2,ne_Y2{,ne_PenWidth{,ne_PenColor}}
    Line 350,450,550,600,4,Black           // line 3
    Line 350,250,350,450,4,Black           // line 3
    Line 350,250,450,250,4,Black           // line 4
    Line 450,250,450,450,4,Black           // line 5
    Line 450,450,650,450,4,Black           // line 6
    Line 650,450,650,600,4,Black           // line 7
    Line 650,350,800,350,4,Black           // line 8
    Line 650,250,650,350,4,Black           // line 9
    Line 650,250,800,250,4,Black           // line 10
    Line 525,300,575,300,4,Black           // line 11
    Line 525,375,575,375,4,Black           // line 11
    Line 525,300,525,375,4,Black           // line 11
    Line 575,300,575,375,4,Black           // line 11
    Line 250,500,350,600,4,Black           // line 11
     
    // For future purposes                                                       
                                                                                                                                                                       
    // Draw labels for putting circle edit boxes
    //xyText 420,10,"pv X, mm"
    //xyText 420,30,"pv Y, mm"
    //xyText 420,50,"radius mm." 
               
    // Draw labels for putting line edit boxes
    //xyText 580,10,"pv X1, mm"
    //xyText 710,10,"pv Y1, mm"
    //xyText 580,30,"pv X2, mm"
    //xyText 710,30,"pv Y2, mm"
    //xyText 580,50,"width mm."
       
    //Call LOG_write("FEND: InitScr()")
    
return


//==============================================================================================
Sub initRobot()

    rLocate _RBT_START_POSX,_RBT_START_POSY,_RBT_START_ANG,_RBT_RADIUS_PX,_RBT_DISP_COLOR
    rInvisible Green, White, Cyan, Red
    rSpeed _RBT_RBASIC_SPD
    rPen Down,cyan
    rSlip 10
    rIgnoreCharge false    
    rCharge 100

return

//==============================================================================================
Sub SCR_displayData()
    
    // INDICATOR - UART connection status        
    if(_UART_FLAG_CONNECTED==0)
        Circle 240,10,260,30,,Red
    else
        Circle 240,10,260,30,,Green
    endif
      
    // Data sent/received via UART
    emptyString1 = "          " // empty string
    xystring 10,40,"Last ser in: ", Left(_RBS_RAW_IN, 32),emptyString1
    xystring 10,55,"Last ser out: ", Left(_RBS_RAW_OUT,32),emptyString1   
    xystring 10,70,"In command parsed: ",_RBS_parsed_lbl,_RBS_parsed_type,_RBS_parsed_arg_str,emptyString1
    xystring 10,85,"Frame received: ",_RBS_frame_received,emptyString1 
    xystring 10,100,"Curr prot. fails: ",ToString(_RBS_rx_not_received_cnt),emptyString1
    xystring 10,115,"Total prot. fails: ",ToString(_STAT_comm_fails_tot),emptyString1  
    
    startXPos = 10
    startYPos = 135
    yStep = 15
    emptyString2 = "     " 
    
    yPos = startYPos
    xystring startXPos, yPos , "------------" \ yPos += yStep 
    xystring startXPos, yPos , "Sim:"  \ yPos += yStep
    xystring startXPos, yPos , "rPoints:",ToString(rPoints()),emptyString2 \ yPos += yStep
    xystring startXPos, yPos , "Real time, ms:", ToString(Timer() - _TMR_PROG_START),emptyString2 \ yPos += yStep
    xystring startXPos, yPos , "Ticks:",ToString(_SIM_ticks_cnt),emptyString2 \ yPos += yStep
    xystring startXPos, yPos , "Sim time, ms:",ToString(_SIM_ticks_cnt*_SIM_MS_PER_TICK),emptyString2 \ yPos += yStep   
    xystring startXPos, yPos , "Real mS/tick:",ToString(Round((Timer() - _TMR_PROG_START)/_SIM_ticks_cnt)),emptyString2 \ yPos += yStep
    
    xystring startXPos, yPos , "------------" \ yPos += yStep  
    xystring startXPos, yPos , "RBASIC sens:" \ yPos += yStep
    xystring startXPos, yPos , "Range F, px:",ToString(rRange(0)),emptyString2  \ yPos += yStep
    xystring startXPos, yPos , "Range L, px:",ToString(rRange(-90)),emptyString2   \ yPos += yStep
    xystring startXPos, yPos , "Range R, px:",ToString(rRange(90)),emptyString2 \ yPos += yStep   
        
    xystring startXPos, yPos , "------------"  \ yPos += yStep
    xystring startXPos, yPos , "Robot:"  \ yPos += yStep
    xystring startXPos, yPos , "L mot, pwm:", ToString(_RBT_left_motor_spdpwm),emptyString2 \ yPos += yStep
    xystring startXPos, yPos , "R mot, pwm:", ToString(_RBT_right_motor_spdpwm),emptyString2 \ yPos += yStep
    xystring startXPos, yPos , "Real wt, mS:", ToString(_RBT_motors_work_time),emptyString2 \ yPos += yStep
    
    xystring startXPos, yPos , "------------" \ yPos += yStep
    xystring startXPos, yPos , "Status:", ToString(_RBT_movement_mode_str),emptyString2 \ yPos += yStep
    xystring startXPos, yPos , "Battery:", ToString(rChargeLevel()),emptyString2  \ yPos += yStep
    xystring startXPos, yPos , "Sim v, px/tick:", ToString(_RBT_move_per_tick_px),emptyString2   \ yPos += yStep
    xystring startXPos, yPos , "Turn rate, deg:", ToString(_RBT_turn_per_tick),emptyString2  \ yPos += yStep
    xystring startXPos, yPos , "Sim OFT, ticks:", ToString(_RBT_obst_free_ticks),emptyString2 \ yPos += yStep 
    
    xystring startXPos, yPos , "------------" \ yPos += yStep
    xystring startXPos, yPos , "IR bitmap:", ToString(Bin(_RBT_ir_bumper_bitmap)),emptyString2 \ yPos += yStep
    xystring startXPos, yPos , "US F, px:", ToString(_RBT_us_forward_dist_px),"; cm:", ToString(_RBT_us_forward_dist_cm), emptyString2 \ yPos += yStep
    xystring startXPos, yPos , "US L, px:", ToString(_RBT_us_left_dist_px),"; cm:",ToString(_RBT_us_left_dist_cm),emptyString2 \ yPos += yStep
    xystring startXPos, yPos , "US R, px:", ToString(_RBT_us_right_dist_px),"; cm:",ToString(_RBT_us_right_dist_cm),emptyString2  \ yPos += yStep
  
    Call STAT_calcStat()
    xystring 500, 15, "Total obstacles:", ToString(_STAT_total_obst_cnt)
    xystring 500, 30, "Session obstacles:", ToString(_STAT_session_obst_cnt)
    xystring 500, 45, "Total nn learnings:", ToString(_STAT_total_nn_learnings_cnt)
    xystring 500, 60, "Session nn learnings:", ToString(_STAT_session_nn_learnings_cnt)
    xystring 500, 75, "Session oft steps:", ToString(_STAT_session_oft_ticks_cnt)
    xystring 500, 90, "Session mnvr steps:", ToString(_STAT_session_mnvr_ticks_cnt)
    
Return


//==============================================================================================
Sub SCR_drawGrid_mm(lhx,lhy,majxmm,minxmm,majymm,minymm,pix_per_mm)
    
    // Draws grid on terminal screen
    //
    // Arguments
    // - lhx -  px lho, x of left high corner
    // - lhy -  px lho, y of left high corner
    // - majxmm - mm, width between major x lines
    // - minxmm - mm, width between minor x lines
    // - majymm - mm, width between major y lines
    // - minymm - mm, width between minor y lines
    // - pix_per_mm - pixels per mm
               
    
    // define Grid parameters
    // -1 = default
    x = lhx        // x of left high corner
    y = lhy        // y of left high corner (leave some room for GUI)
    width = 545    // as screen width
    height = 445   // from lhy to screen bottom
    border_forecolor = Black
    border_backcolor = White
    border_width = 4
    x_minor_line_freq = minxmm*pix_per_mm     
    x_major_line_freq = majxmm*pix_per_mm     
    y_minor_line_freq = minymm*pix_per_mm     
    y_major_line_freq = majymm*pix_per_mm    
    x_minor_line_width = -1
    x_major_line_width = -1
    y_minor_line_width = -1
    y_major_line_width = -1
    x_minor_line_color =  Green
    x_major_line_color =  Green
    y_minor_line_color =  Green
    y_major_line_color =  Green

    // put parameters into Data array
    data a_Specs; x, y, width, height, border_forecolor, border_backcolor, border_width
    data a_Specs; x_minor_line_freq, x_major_line_freq, y_minor_line_freq, y_major_line_freq
    data a_Specs; x_minor_line_width, x_major_line_width, y_minor_line_width, y_major_line_width
    data a_Specs; x_minor_line_color, x_major_line_color, y_minor_line_color, y_major_line_color
    
    // Grid
    mGraphPaper a_Specs
Return

//==============================================================================================
Sub connectToUART(portNo,baudRate)
 
    // Last update: 2025-06-10, 13:20
    //
    // Connect to COM port
    //
    // Arguments:
    // - portNo - integer, COM port number
    // - baudRate - baud rate
    // Return:
    // - 0 if not succes, 1 if succes
    //
    // Calls/depends on these functions/global variables
    // UART_FLAG_CONNECTED
    // LOG_write()
    
    Call LOG_write("FSTART: connectToUART(UART_COM_PORT)")
    Call LOG_write("Argument: portNo = " + ToString(portNo))
    Call LOG_write("Argument: baudRate = " + ToString(baudRate))
    
    if IsNumber(portNo)

         if _UART_FLAG_CONNECTED == 0
            //setcommport portNo,baudRate,8,pNone,sbOne,fcNone
            
            setcommport portNo,baudRate,8,pNone,sbOne,fcNone
            //SetTimeOut _RBS_SER_TIMEOUT
            
            ClearSerBuffer 1
            ClearSerBuffer 2
         
            _UART_FLAG_CONNECTED = 1
            SetButtonCaption "Connect to UART", "Disconnect"
            Call LOG_write("Connected to UART")
         else
            setcommport 0
            _UART_FLAG_CONNECTED = 0
            SetButtonCaption "Connect to UART", "Connect to UART"
            Call LOG_write("Disconnected from UART")
         endif
    else
    
        setcommport 0
        _UART_FLAG_CONNECTED = 0
        Call LOG_write("Wrong port number")
        Call LOG_write("FRETURN: 0")
    
    endif
    
    Call LOG_write("FRETURN: 1")            
    Call LOG_write("FEND: connectToUART(UART_COM_PORT)")
    
return 1


//==============================================================================================
eHandler:

    GetError vn_ErrNo, vs_ErrMessage, vn_LineNo

    if vn_ErrNo == 79   // OBSTACLE COLLISION
        Call LOG_write("ERROR #" + ToString(vn_ErrNo))
        Call LOG_write("ERROR MSG:" + ToString(vs_ErrMessage))
    elseif vn_ErrNo == 93  // SERIAL
        UART_FLAG_CONNECTED = 0
        ErrMsg("No serial connection","Error",MB_ERROR)    
    else
        WriteScr  _strDateTime + " - " + "SCR_ERROR.bmp"
        //mWrite NN_Log_Tbl, _strDateTime + " - " + "NNLOG_ERROR.txt" 
        Call STAT_save_NN_log_tbl()
        Call STAT_calcStat()
        Call STAT_statToLog()  
        ErrMsg("ERROR #" + ToString(vn_ErrNo) + " " + ToString(vs_ErrMessage) + " Line#" + ToString(vn_LineNo))        
        Terminate
    endif

    OnError eHandler

Return


//==============================================================================================
abortHandler:

    //Call LOG_write("FSTART: abortHandler()")
    
    WriteScr _strDateTime + " - " + "SCR_ABORT.bmp"
    //mWrite NN_Log_Tbl, strDateTime + " - " + "NNLOG_ABORT.txt" 
    Call STAT_save_NN_log_tbl()
    Call STAT_calcStat()
    Call STAT_statToLog()             
    FileClose(_LOG_FH) // Close LOG file
    SetCommPort 0      // Close serial connection  
            
    Terminate          // Terminate program
         
    OnAbort abortHandler

Return
//==============================================================================================
bHandler:

    
    //Call LOG_write("Button has been pressed")
    //Call LOG_write("FSTART: bHandler()")

    lb = LastButton()   //must be done at least once before returning
    
    //Call LOG_write("Last button code = " + ToString(lb))
    
    // Connect to com port selected in drop box
    if lb ==  "Connect to UART"
        comPortNo = GetListBoxText("COMList")
        Call connectToUART(ToNumber(comPortNo),_UART_BAUD_RATE)
        delay(300)
    endif
    
    // proceed only if connection established
    if _UART_FLAG_CONNECTED
        
        // Send RDY signal
        if lb == "Ready"
             // Send message to Arduino via serial
             _RBS_parsed_lbl = _RBS_LBL_MSG
             _RBS_parsed_type = _RBS_TYPE_AT
             _RBS_parsed_arg_str = _RBS_ARG_RGO
             ClearSerBuffer
             Call RBS_transmitARQ() 
             // Reset FSM and timers/counters
             _FSM_state = _FSM_RESTART_CYCLE
             _SIM_ticks_cnt = 1   // 1 to avoid division by zero
             _TMR_PROG_START = Timer()
        endif
               
    // end if "connected to UART"        
    endif
         
    // Other buttons (For future purposes)
                 
    // Put a circle on the screen     
    if lb == "Put circle"
        pvX_mm = ToNumber(GetEdit("pvX_mm"))
        pvY_mm = ToNumber(GetEdit("pvY_mm"))
        r_mm = ToNumber(GetEdit("circleRad_mm")) 
        Call SNR_putCircle2(pvX_mm,pvY_mm,r_mm,1,Brown)
    endif
    
    // Put a line on the screen     
    if lb == "Put line"
        pvX1_mm = ToNumber(GetEdit("pvX1_mm"))
        pvY1_mm = ToNumber(GetEdit("pvY1_mm"))
        pvX2_mm = ToNumber(GetEdit("pvX2_mm"))
        pvY2_mm = ToNumber(GetEdit("pvY2_mm"))
        lineWidth_mm = ToNumber(GetEdit("lineWidth_mm"))
        Call SNR_putLine2(pvX1_mm,pvY1_mm,pvX2_mm,pvY2_mm,lineWidth_mm,Brown)
    endif
                        
    //Call LOG_write("FEND: bHandler()")
    
    onbutton bHandler

Return

//==============================================================================================
Sub RBT_calcIRBumperBitmap()

    // This function simulates IR bumper sensors mounted on real robot,
    // and bitmap used in real robot 
    // bitmap |Left|Right|Forward|Top| NOT the same as Simulator default
    
    _RBT_ir_bumper_bitmap = 0 
    
    // FW IR SENSOR
    For idx = 0 To MaxDim(RBT_IR_FORWARD_ANGLES) - 1
        curr_dist_px = rRange(RBT_IR_FORWARD_ANGLES[idx])
        if curr_dist_px <=_RBT_IR_RANGE_PX
           _RBT_ir_bumper_bitmap = _RBT_ir_bumper_bitmap | 3 
        endif
    Next
    
    // LEFT IR SENSOR
    For idx = 0 To MaxDim(RBT_IR_LEFT_ANGLES) - 1
        curr_dist_px = rRange(RBT_IR_LEFT_ANGLES[idx])
        if curr_dist_px <=_RBT_IR_RANGE_PX
           _RBT_ir_bumper_bitmap = _RBT_ir_bumper_bitmap | 8 
        endif
    Next
    
    // RIGHT IR SENSOR
    For idx = 0 To MaxDim(RBT_IR_RIGHT_ANGLES) - 1
        curr_dist_px = rRange(RBT_IR_RIGHT_ANGLES[idx])
        if curr_dist_px <=_RBT_IR_RANGE_PX
           _RBT_ir_bumper_bitmap = _RBT_ir_bumper_bitmap | 4 
        endif
    Next                             
    
    // Left -45 deg      
    //if rRange(315) <=_RBT_IR_RANGE_PX
    //   _RBT_ir_bumper_bitmap = _RBT_ir_bumper_bitmap | 8 
       
       // Ver 10
       //_RBT_ir_bumper_bitmap = 1 
    //endif
    
    // Forward 0 deg      
    //if rRange(0) <=_RBT_IR_RANGE_PX
    //   _RBT_ir_bumper_bitmap = _RBT_ir_bumper_bitmap | 3 
       
       // Ver 10
       //_RBT_ir_bumper_bitmap = 1       
   // endif
    
    // Right +45 deg      
    //if rRange(45) <=_RBT_IR_RANGE_PX
    //   _RBT_ir_bumper_bitmap = _RBT_ir_bumper_bitmap | 4
       
       // Ver 10
       //_RBT_ir_bumper_bitmap = 1         
    //endif
    
    // Ver 10.1
    //if rRange(315) <=_RBT_IR_RANGE_PX || rRange(45) <=_RBT_IR_RANGE_PX
    //    _RBT_ir_bumper_bitmap = 2
    //endif
    
    //if rRange(315) <=_RBT_IR_RANGE_PX && rRange(45) <=_RBT_IR_RANGE_PX
    //    _RBT_ir_bumper_bitmap = 2
    //endif
    
    //if rRange(0) <=_RBT_IR_RANGE_PX
    //    _RBT_ir_bumper_bitmap = 3
    //endif
    
    //if (rRange(0) <=_RBT_IR_RANGE_PX && rRange(315) <=_RBT_IR_RANGE_PX) || (rRange(0) <=_RBT_IR_RANGE_PX && rRange(45) <=_RBT_IR_RANGE_PX)
    //    _RBT_ir_bumper_bitmap = 4
    //endif     

    //if rRange(0) <=_RBT_IR_RANGE_PX && rRange(315) <=_RBT_IR_RANGE_PX && rRange(45) <=_RBT_IR_RANGE_PX
    //    _RBT_ir_bumper_bitmap = 5
    //endif     

Return _RBT_ir_bumper_bitmap


//==============================================================================================
Sub RBT_calcUltrasonicDist()

    // Same as in real Arduino robot
    
    // Check forward sector
    _RBT_us_forward_dist_px = _RBT_US_RANGE_PX
    curr_dist_px = _RBT_US_RANGE_PX 
    
    For idx = 0 To MaxDim(RBT_US_FORWARD_ANGLES) - 1
        curr_dist_px = rRange(RBT_US_FORWARD_ANGLES[idx])
        if curr_dist_px < _RBT_us_forward_dist_px
           _RBT_us_forward_dist_px = curr_dist_px
        endif
    Next
    
    _RBT_us_forward_dist_cm = (_RBT_us_forward_dist_px*_SIM_MM_PER_PX)/10
    
    
    // Check left sector
    _RBT_us_left_dist_px = _RBT_US_RANGE_PX
    curr_dist_px = _RBT_US_RANGE_PX
    
    For idx = 0 To MaxDim(RBT_US_LEFT_ANGLES) - 1
        curr_dist_px = rRange(RBT_US_LEFT_ANGLES[idx])
        if curr_dist_px < _RBT_us_left_dist_px
           _RBT_us_left_dist_px = curr_dist_px
        endif
    Next
    
    _RBT_us_left_dist_cm = (_RBT_us_left_dist_px*_SIM_MM_PER_PX)/10
    
    
    // Check right sector
    _RBT_us_right_dist_px = _RBT_US_RANGE_PX
    curr_dist_px = _RBT_US_RANGE_PX
    
    For idx = 0 To MaxDim(RBT_US_RIGHT_ANGLES) - 1
        curr_dist_px = rRange(RBT_US_RIGHT_ANGLES[idx])
        if curr_dist_px < _RBT_us_right_dist_px
           _RBT_us_right_dist_px = curr_dist_px
        endif
    Next
    
    _RBT_us_right_dist_cm = (_RBT_us_right_dist_px*_SIM_MM_PER_PX)/10
    
Return


//==============================================================================================
Sub RBT_calcObstFreeTime()

    _RBT_obst_free_ms = _RBT_obst_free_ticks*_SIM_MS_PER_TICK
    
Return


//==============================================================================================
Sub RBT_calcSimMove()
    
     left_motor_spdpx = Map(_RBT_left_motor_spdpwm,_RBT_MIN_SPD_PWM, _RBT_MAX_SPD_PWM, _RBT_MIN_SPD_PXTICK, _RBT_MAX_SPD_PXTICK)
                            
     right_motor_spdpx = Map(_RBT_right_motor_spdpwm, _RBT_MIN_SPD_PWM, _RBT_MAX_SPD_PWM, _RBT_MIN_SPD_PXTICK, _RBT_MAX_SPD_PXTICK)
                
     _RBT_turn_per_tick = - Round(RtoD((right_motor_spdpx - left_motor_spdpx)/_RBT_WIDTH_PX))                          
    
     _RBT_move_per_tick_px = Round((right_motor_spdpx + left_motor_spdpx)/2)
     
     if (_RBT_motors_work_time >= _RBT_MIN_MANEUVER_MWT) && (_RBT_motors_work_time < _SIM_MS_PER_TICK)
        _RBT_motors_work_ticks_cnt = 1
     else
        _RBT_motors_work_ticks_cnt = RoundUp(_RBT_motors_work_time/_SIM_MS_PER_TICK)
     endif
     
     //Call LOG_write("Turn rate = " + ToString(_RBT_turn_per_tick))
     //Call LOG_write("Step, px = " + ToString(_RBT_move_per_tick_px))
     //Call LOG_write("Steps, cnt = " + ToString(_RBT_motors_work_ticks_cnt))
                           
Return




//==============================================================================================
Sub RBT_execQuery()


    // LEFT MOTOR SPEED
    if _RBS_parsed_lbl == _RBS_LBL_LMS
        // Check command type
        if _RBS_parsed_type == _RBS_TYPE_GET
            // prepare parsed values and send
            _RBS_parsed_type = _RBS_TYPE_AT
            _RBS_parsed_arg_str = ToString(_RBT_left_motor_spdpwm)
            Call RBS_transmitARQ()
        elseif _RBS_parsed_type == _RBS_TYPE_SET
            // Set corresponding value
            _RBT_left_motor_spdpwm = ToNumber(_RBS_parsed_arg_str)
            _RBT_mnvr_params_set_cnt++            
        elseif _RBS_parsed_type == _RBS_TYPE_AT
            // Do nothing
        else
            // Do nothing
        endif
    // End _RBS_LBL_LM
    
     .
    // RIGHT MOTOR SPEED     
    elseif _RBS_parsed_lbl == _RBS_LBL_RMS
        // Check command type
        if _RBS_parsed_type == _RBS_TYPE_GET
            // prepare parsed values and send
            _RBS_parsed_type =  _RBS_TYPE_AT
            _RBS_parsed_arg_str = ToString(_RBT_right_motor_spdpwm)
            Call RBS_transmitARQ()
        elseif _RBS_parsed_type == _RBS_TYPE_SET
            // Set corresponding value
            _RBT_right_motor_spdpwm = ToNumber(_RBS_parsed_arg_str)
            _RBT_mnvr_params_set_cnt++
        elseif _RBS_parsed_type == _RBS_TYPE_AT
            // Do nothing
        else
            // Do nothing
        endif
    // End _RBS_LBL_RMS
    
     
    //  MOTORS WORKING TIME              
    elseif _RBS_parsed_lbl == _RBS_LBL_MWT
        // Check command type
        if _RBS_parsed_type == _RBS_TYPE_GET
            // prepare parsed values and send
            _RBS_parsed_type =  _RBS_TYPE_AT
            _RBS_parsed_arg_str = ToString(_RBT_motors_work_time)
            Call RBS_transmitARQ()            
        elseif _RBS_parsed_type == _RBS_TYPE_SET
            // Set corresponding value
            _RBT_motors_work_time = ToNumber(_RBS_parsed_arg_str)
            _RBT_mnvr_params_set_cnt++
        elseif _RBS_parsed_type == _RBS_TYPE_AT
            // Do nothing
        else
            // Do nothing
        endif
    // End _RBS_LBL_MWT 
      
     
    // IR SENSORS BITMAP             
    elseif _RBS_parsed_lbl == _RBS_LBL_IRB
        // Check command type
        if _RBS_parsed_type == _RBS_TYPE_GET
            // prepare parsed values and send
            Call RBT_calcIRBumperBitmap()
            _RBS_parsed_type =  _RBS_TYPE_AT
            _RBS_parsed_arg_str = ToString(_RBT_ir_bumper_bitmap)            
            Call RBS_transmitARQ()
        elseif _RBS_parsed_type == _RBS_TYPE_SET
            // Set corresponding value
            _RBT_ir_bumper_bitmap = ToNumber(_RBS_parsed_arg_str)
        elseif _RBS_parsed_type == _RBS_TYPE_AT
            // Value predictrd by NN
            NN_Log_Tbl[_nnlog_row_idx,_nnlog_col_irb_est] = ToNumber(_RBS_parsed_arg_str) 
        else
            // Do nothing
        endif 
    // End _RBS_LBL_IRB
    
   
    // ULTRASONIC SENSOR FORWARD DIR DISTANCE                                                                    
    elseif _RBS_parsed_lbl == _RBS_LBL_USF
        // Check command type
        if _RBS_parsed_type == _RBS_TYPE_GET
            // prepare parsed values and send
            Call RBT_calcUltrasonicDist()
            _RBS_parsed_type =  _RBS_TYPE_AT
            _RBS_parsed_arg_str = ToString(_RBT_us_forward_dist_cm)            
            Call RBS_transmitARQ()
        elseif _RBS_parsed_type == _RBS_TYPE_SET
            // Set corresponding value
            _RBT_us_forward_dist_cm = ToNumber(_RBS_parsed_arg_str)
        elseif _RBS_parsed_type == _RBS_TYPE_AT
            // Value predicted by NN
            NN_Log_Tbl[_nnlog_row_idx,_nnlog_col_usf_est] = ToNumber(_RBS_parsed_arg_str) 
        else
            // Do something
        endif        
    // End _RBS_LBL_USF
    
     
    // ULTRASONIC SENSOR LEFT DIR DISTANCE                                                                                     
    elseif _RBS_parsed_lbl == _RBS_LBL_USL
        // Check command type
        if _RBS_parsed_type == _RBS_TYPE_GET
            // prepare parsed values and send
            Call RBT_calcUltrasonicDist()
            _RBS_parsed_type =  _RBS_TYPE_AT
            _RBS_parsed_arg_str = ToString(_RBT_us_left_dist_cm)           
            Call RBS_transmitARQ()
        elseif _RBS_parsed_type == _RBS_TYPE_SET
            // Set corresponding value
            _RBT_us_left_dist_cm = ToNumber(_RBS_parsed_arg_str)
        elseif _RBS_parsed_type == _RBS_TYPE_AT
            // Value predicted by NN
            NN_Log_Tbl[_nnlog_row_idx,_nnlog_col_usl_est] = ToNumber(_RBS_parsed_arg_str) 
        else
            // Do something
        endif
    // End _RBS_LBL_USL   
            
     
    // ULTRASONIC SENSOR RIGHT DIR DISTANCE               
    elseif _RBS_parsed_lbl == _RBS_LBL_USR
        // Check command type
        if _RBS_parsed_type == _RBS_TYPE_GET
             // prepare parsed values and send
            Call RBT_calcUltrasonicDist()
            _RBS_parsed_type =  _RBS_TYPE_AT
            _RBS_parsed_arg_str = ToString(_RBT_us_right_dist_cm)           
            Call RBS_transmitARQ()
        elseif _RBS_parsed_type == _RBS_TYPE_SET
            // Set corresponding value
            _RBT_us_right_dist_cm = ToNumber(_RBS_parsed_arg_str)
        elseif _RBS_parsed_type == _RBS_TYPE_AT
            // Value predicted by NN
            NN_Log_Tbl[_nnlog_row_idx,_nnlog_col_usr_est] = ToNumber(_RBS_parsed_arg_str) 
        else
            // Do something
        endif
    // End _RBS_LBL_USR  
    
       
    // OBSTACLE FREE RUN TIMER                    
    elseif _RBS_parsed_lbl == _RBS_LBL_OFT
        // Check command type
        if _RBS_parsed_type == _RBS_TYPE_GET
            Call RBT_calcObstFreeTime()
             // prepare parsed values and send
            _RBS_parsed_type =  _RBS_TYPE_AT
            _RBS_parsed_arg_str = ToString(_RBT_obst_free_ms)         
            Call RBS_transmitARQ()
        elseif _RBS_parsed_type == _RBS_TYPE_SET
            // Set corresponding value
            _RBT_obst_free_ms = ToNumber(_RBS_parsed_arg_str)
        elseif _RBS_parsed_type == _RBS_TYPE_AT
            // Value predicted by NN
            NN_Log_Tbl[_nnlog_row_idx,_nnlog_col_oft_est] = ToNumber(_RBS_parsed_arg_str) 
        else
            // Do something
        endif       
    // End _RBS_LBL_OFT
    
    
    // CONTEXT NEURONE #1                  
    elseif _RBS_parsed_lbl == _RBS_LBL_CN1
        // Check command type
        if _RBS_parsed_type == _RBS_TYPE_GET
            // do nothing
        elseif _RBS_parsed_type == _RBS_TYPE_SET
            // do nothing
        elseif _RBS_parsed_type == _RBS_TYPE_AT
            // Save received value to LOG table
            NN_Log_Tbl[_nnlog_row_idx,_nnlog_col_cn1_ini] = ToNumber(_RBS_parsed_arg_str) 
        else
            // do nothing
        endif       
    // End _RBS_LBL_CN1
    
    
    // CONTEXT NEURONE #2                  
    elseif _RBS_parsed_lbl == _RBS_LBL_CN2
        // Check command type
        if _RBS_parsed_type == _RBS_TYPE_GET
            // do nothing
        elseif _RBS_parsed_type == _RBS_TYPE_SET
            // do nothing
        elseif _RBS_parsed_type == _RBS_TYPE_AT
            // Save received value to LOG table
            NN_Log_Tbl[_nnlog_row_idx,_nnlog_col_cn2_ini] = ToNumber(_RBS_parsed_arg_str) 
        else
            // do nothing
        endif       
    // End _RBS_LBL_CN1
    
    
   
    
    // ROBOT                                                
    elseif _RBS_parsed_lbl == _RBS_LBL_RBT
        // Check command type
        if _RBS_parsed_type == _RBS_TYPE_GET
            // Do something
        elseif _RBS_parsed_type == _RBS_TYPE_SET
            if _RBS_parsed_arg_str == _RBS_ARG_EXE
                _RBT_movement_mode_str = _RBT_MODE_STR_MANEUVER             
            elseif _RBS_parsed_arg_str == _RBS_ARG_FWD
                _RBT_movement_mode_str = _RBT_MODE_STR_MVFORWARD      
            elseif _RBS_parsed_arg_str == _RBS_ARG_STP
                _RBT_movement_mode_str = _RBT_MODE_STR_STOP
            else
                // Do nothing
            endif
        elseif _RBS_parsed_type == _RBS_TYPE_AT
            // Do something
        else
            // Do something
        endif
    // End _RBS_LBL_RBT


    // STAT/LOG OBSTACLES COUNTER
    elseif _RBS_parsed_lbl == _RBS_LBL_OBS
        // Check command type
        if _RBS_parsed_type == _RBS_TYPE_GET
            // Do nothing
        elseif _RBS_parsed_type == _RBS_TYPE_SET
            // Do nothing
        elseif _RBS_parsed_type == _RBS_TYPE_AT
            // Update corresponding value
            if _STAT_total_obst_cnt_received_flag == false
                _STAT_total_obst_cnt_received_flag = true
                _STAT_total_obst_cnt_onstart = ToNumber(_RBS_parsed_arg_str)
                _STAT_total_obst_cnt = _STAT_total_obst_cnt_onstart 
            else
                _STAT_total_obst_cnt = ToNumber(_RBS_parsed_arg_str)
            endif
        else
            // Do nothing
        endif
    // End _RBS_LBL_OBS
    
    
    // STAT/LOG NN LEARNINGS COUNTER
    elseif _RBS_parsed_lbl == _RBS_LBL_NNL
        // Check command type
        if _RBS_parsed_type == _RBS_TYPE_GET
            // Do nothing
        elseif _RBS_parsed_type == _RBS_TYPE_SET
            // Do nothing
        elseif _RBS_parsed_type == _RBS_TYPE_AT
            // Update corresponding value
            if _STAT_total_nn_learnings_cnt_received_flag == false
                _STAT_total_nn_learnings_cnt_received_flag = true
                _STAT_total_nn_learnings_cnt_onstart = ToNumber(_RBS_parsed_arg_str)
                _STAT_total_nn_learnings_cnt = _STAT_total_nn_learnings_cnt_onstart  
            else
                _STAT_total_nn_learnings_cnt = ToNumber(_RBS_parsed_arg_str)
            endif
        else
            // Do nothing
        endif
    // End _RBS_LBL_NNL
    
    
    // STAT/LOG 
    elseif _RBS_parsed_lbl == _RBS_LBL_LOG
        // Check command type
        if _RBS_parsed_type == _RBS_TYPE_GET
            // Do nothing
        elseif _RBS_parsed_type == _RBS_TYPE_SET
            // Do nothing
            if _RBS_parsed_arg_str == _RBS_ARG_INI_S
                Call STAT_save_init_sens(_nnlog_row_idx)
            elseif _RBS_parsed_arg_str == _RBS_ARG_INI_M
                Call STAT_save_init_mpg(_nnlog_row_idx)
            elseif _RBS_parsed_arg_str == _RBS_ARG_LST_M
                Call STAT_copy_last_mpg(_nnlog_row_idx)
            elseif _RBS_parsed_arg_str == _RBS_ARG_RES_S            
                Call STAT_save_result_sens(_nnlog_row_idx)
            elseif _RBS_parsed_arg_str == _RBS_ARG_FWD_M
                Call STAT_save_move_fw_mpg(_nnlog_row_idx)
            elseif _RBS_parsed_arg_str == _RBS_ARG_MWT_U
                Call STAT_update_init_mwt(_nnlog_row_idx)
            elseif _RBS_parsed_arg_str == _RBS_ARG_NXT
                // Save NN log after every 10 obstacles
                if _STAT_session_obst_cnt#10 == 0 && _STAT_session_obst_cnt >= 10
                    Call STAT_save_NN_log_tbl() 
                endif
                
                NN_Log_Tbl[_nnlog_row_idx, _nnlog_col_timestamp] = _strDateTime
                NN_Log_Tbl[_nnlog_row_idx, _nnlog_col_row_number] = _nnlog_row_idx
                NN_Log_Tbl[_nnlog_row_idx, _nnlog_col_obst_num] = _STAT_total_obst_cnt
                NN_Log_Tbl[_nnlog_row_idx, _nnlog_col_lrn_num] = _STAT_total_nn_learnings_cnt
                
                _nnlog_row_idx++
                
                if _nnlog_row_idx >= _nnlog_rows_cnt - 1
                     _nnlog_row_idx = _nnlog_rows_cnt - 1
                endif               
            else
                // Do nothing
            endif
        elseif _RBS_parsed_type == _RBS_TYPE_AT
            // Do nothing
        else
            // Do nothing
        endif
    // End _RBS_LBL_LOG
    
    
    //2026-08-01
    // Comm reeset
    //elseif (_RBS_parsed_lbl == _RBS_LBL_HDS) && (_RBS_parsed_arg_str == _RBS_ARG_RST)
    //        _RBT_movement_mode_str = _RBT_MODE_STR_STOP            
    //        _RBT_is_moving_fw = false
    //        _RBT_mnvr_continue = false
    //        _RBT_mnvr_params_set_cnt = 0
    //        _RBT_turn_per_tick = 0
    //        _RBT_move_per_tick_px = 0
    //        Call SCR_displayData()  
    
    // DEFAULT                                        
    else
        // Do something
    // End default
    
    endif   
Return

//==============================================================================================
Sub STAT_calcStat()

    _STAT_session_obst_cnt = _STAT_total_obst_cnt  - _STAT_total_obst_cnt_onstart
    _STAT_session_nn_learnings_cnt = _STAT_total_nn_learnings_cnt - _STAT_total_nn_learnings_cnt_onstart
    
Return


//==============================================================================================
Sub STAT_statToLog()

    Call LOG_write("------------")
    Call LOG_write("STAT ON EXIT")
    Call LOG_write("------------") 
    Call LOG_write("Timestamp: " + ToString(_strDateTime))   
    Call LOG_write("Total ticks: " + ToString(_SIM_ticks_cnt))  
    Call LOG_write("Comm errors: " + ToString(_STAT_comm_fails_tot))  
    Call LOG_write("Session obstacles: " + ToString(_STAT_session_obst_cnt))
    Call LOG_write("Session learnings: " + ToString(_STAT_session_nn_learnings_cnt))
    Call LOG_write("Session obstacle free ticks: " + ToString(_STAT_session_oft_ticks_cnt))
    Call LOG_write("Session evade maneuver ticks: " + ToString(_STAT_session_mnvr_ticks_cnt))        
    Call LOG_write("Total obstacles: " + ToString(_STAT_total_obst_cnt))
    Call LOG_write("Total learnings: " + ToString(_STAT_total_nn_learnings_cnt))
    Call LOG_write("Sim ms/tick: " + ToString(_SIM_MS_PER_TICK))
    Call LOG_write("Stat data row:")
    stat_row_str = ToString(_strDateTime) + ToString(Char(9)) +\
                   ToString(_SIM_ticks_cnt) + ToString(Char(9)) +\
                   ToString(_STAT_comm_fails_tot) + ToString(Char(9)) +\
                   ToString(_STAT_session_obst_cnt) + ToString(Char(9)) +\
                   ToString(_STAT_session_nn_learnings_cnt) + ToString(Char(9)) +\
                   ToString(_STAT_session_oft_ticks_cnt) + ToString(Char(9)) +\
                   ToString(_STAT_session_mnvr_ticks_cnt) + ToString(Char(9)) +\
                   ToString(_STAT_total_obst_cnt) + ToString(Char(9)) +\
                   ToString(_STAT_total_nn_learnings_cnt) + ToString(Char(9))
   Call LOG_write(stat_row_str)               
Return



//==============================================================================================
Sub STAT_save_init_sens(row_idx)

    NN_Log_Tbl[row_idx, _nnlog_col_irb_ini] = _RBT_ir_bumper_bitmap
    NN_Log_Tbl[row_idx, _nnlog_col_usf_ini] = _RBT_us_forward_dist_cm
    NN_Log_Tbl[row_idx, _nnlog_col_usl_ini] = _RBT_us_left_dist_cm
    NN_Log_Tbl[row_idx, _nnlog_col_usr_ini] = _RBT_us_right_dist_cm  
    NN_Log_Tbl[row_idx, _nnlog_col_oft_ini] = _RBT_obst_free_ms
    // Robot basic coordinates
    NN_Log_Tbl[row_idx, _nnlog_col_posX_ini] = rGpsX()
    NN_Log_Tbl[row_idx, _nnlog_col_posY_ini] = rGpsY() 
    NN_Log_Tbl[row_idx, _nnlog_col_hdir_ini] = rCompass()
    
    
Return


//==============================================================================================
Sub STAT_copy_last_est_sens_if_empty(row_idx)

    if mType(NN_Log_Tbl[row_idx, _nnlog_col_irb_est]) == 0
        NN_Log_Tbl[row_idx, _nnlog_col_irb_est] = NN_Log_Tbl[row_idx - 1, _nnlog_col_irb_est]
    endif
    
    if mType(NN_Log_Tbl[row_idx, _nnlog_col_usf_est]) == 0
        NN_Log_Tbl[row_idx, _nnlog_col_usf_est] = NN_Log_Tbl[row_idx - 1, _nnlog_col_usf_est]
    endif
    
    if mType(NN_Log_Tbl[row_idx, _nnlog_col_usl_est]) == 0
        NN_Log_Tbl[row_idx, _nnlog_col_usl_est] = NN_Log_Tbl[row_idx - 1, _nnlog_col_usl_est]
    endif
    
    if mType(NN_Log_Tbl[row_idx, _nnlog_col_usr_est]) == 0
        NN_Log_Tbl[row_idx, _nnlog_col_usr_est] = NN_Log_Tbl[row_idx - 1, _nnlog_col_usr_est] 
    endif

    if mType(NN_Log_Tbl[row_idx, _nnlog_col_oft_est]) == 0
        NN_Log_Tbl[row_idx, _nnlog_col_oft_est] = NN_Log_Tbl[row_idx - 1, _nnlog_col_oft_est]
    endif
    
Return

//==============================================================================================
Sub STAT_save_init_mpg(row_idx)

    NN_Log_Tbl[row_idx, _nnlog_col_mpl_ini] = _RBT_left_motor_spdpwm
    NN_Log_Tbl[row_idx, _nnlog_col_mpr_ini] = _RBT_right_motor_spdpwm
    NN_Log_Tbl[row_idx, _nnlog_col_mwt_ini] = _RBT_motors_work_time
    
Return

//==============================================================================================
Sub STAT_update_init_mwt(row_idx)

    NN_Log_Tbl[row_idx, _nnlog_col_mwt_ini] = _RBT_obst_free_ms

Return

//==============================================================================================
Sub STAT_save_move_fw_mpg(row_idx)

    NN_Log_Tbl[row_idx, _nnlog_col_mpl_ini] = _RBT_MAX_SPD_PWM
    NN_Log_Tbl[row_idx, _nnlog_col_mpr_ini] = _RBT_MAX_SPD_PWM
    NN_Log_Tbl[row_idx, _nnlog_col_mwt_ini] = _RBT_MAX_BATTERY_TIME
    
Return

//==============================================================================================
Sub STAT_copy_last_mpg(row_idx)

    NN_Log_Tbl[row_idx, _nnlog_col_mpl_ini] = NN_Log_Tbl[row_idx - 1, _nnlog_col_mpl_ini]
    NN_Log_Tbl[row_idx, _nnlog_col_mpr_ini] = NN_Log_Tbl[row_idx - 1, _nnlog_col_mpr_ini]
    NN_Log_Tbl[row_idx, _nnlog_col_mwt_ini] = NN_Log_Tbl[row_idx - 1, _nnlog_col_mwt_ini]
    
Return

//==============================================================================================
Sub STAT_save_init_cond(row_idx)

    Call STAT_save_init_sens(row_idx)
    Call STAT_save_init_mpg(row_idx)
    
Return


//==============================================================================================
Sub STAT_save_result_sens(row_idx)

    NN_Log_Tbl[row_idx, _nnlog_col_irb_res] = _RBT_ir_bumper_bitmap
    NN_Log_Tbl[row_idx, _nnlog_col_usf_res] = _RBT_us_forward_dist_cm
    NN_Log_Tbl[row_idx, _nnlog_col_usl_res] = _RBT_us_left_dist_cm
    NN_Log_Tbl[row_idx, _nnlog_col_usr_res] = _RBT_us_right_dist_cm  
    NN_Log_Tbl[row_idx, _nnlog_col_oft_res] = _RBT_obst_free_ms
    
Return


//==============================================================================================
Sub STAT_save_NN_log_tbl()

    NN_LOG_FN = _strDateTime + " - " + "NNLOG_.txt"         // log file name
    NN_LOG_FH = Filecreate(NN_LOG_FN)    // log file handler 

    
    row_str = ""
    For i = 0 to _nnlog_rows_cnt - 1
    
        // stop write to file if first calumn in current row
        // is empty (no more data to save)
        if(mType(NN_Log_Tbl[i,0]) == 0) 
            break
        endif
    
        For j = 0 to _nnlog_cols_cnt - 1
            if(mType(NN_Log_Tbl[i,j]) != 0)
                row_str +=  ToString(NN_Log_Tbl[i,j])
            else
                //row_str += ToString(Char(9)) 
            endif
                
            if j == _nnlog_cols_cnt - 1
                row_str += CrLf()
            else
                row_str += ToString(Char(9))
            endif            
        Next
        FileWrite(NN_LOG_FH,row_str)
        row_str = ""
    Next
    FileClose(NN_LOG_FH)

Return

// ===================================================

