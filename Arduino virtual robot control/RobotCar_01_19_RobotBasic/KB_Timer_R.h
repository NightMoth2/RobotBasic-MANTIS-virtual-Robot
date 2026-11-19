/*
 * Class to implement timer for arduino with "on-pause" feature
 * 
 * Ver 2.0 - 2026-04-20
 * 
 * Supposed usage
 * ======================================================
 * Timer_R testTimer;
 * 
 * testTimer.reset();      // clear all values and stop
 * testTimer.start();      // Start timer
 * 
 * // some other process here
 * 
 * testTimer.pause();      // pause (pause-resume can be used many times)
 * 
 * // some other process here
 * 
 * testTimer.resume();     // OR testTimer.start();
 * 
 * // some other process here
 * 
 *  * testTimer.pause();      // pause (pause-resume can be used many times)
 * 
 * // some other process here
 * 
 * testTimer.resume();     // OR testTimer.start();
 * 
 * // some other process here
 * 
 * testTimer.stop();       // stop timer
 * 
 * // Get information
 * unsigned long totTime = testTimer.getTotTime();        // time between start and stop including pauses
 * unsigned long idleTime = testTimer.getIdleTime();      // time the timer was on pause
 * unsigned long activeTime = testTimer.getWorkTime(0);   // timer active time (NOT including pause time)
 * 
 * ======================================================
 * 
 */


#ifndef KB_TIMER_R_H
#define KB_TIMER_R_H

#include <Arduino.h>



class Timer_R
{
  private: 

    bool is_paused;
    bool is_started;
    unsigned long t_start;
    unsigned long t_stop;
    unsigned long t_pause;
    unsigned long t_resume;
    unsigned long pause_length;

  public:

    // Constructor
    Timer_R();

    void start();
    void reset();
    void stop();
    void pause();
    void resume();
    unsigned long getTotTime();
    unsigned long getIdleTime();
    unsigned long getWorkTime(bool roundup);


  
};



#endif
