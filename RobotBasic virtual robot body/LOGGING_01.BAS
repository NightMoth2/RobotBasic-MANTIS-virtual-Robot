//==============================================================================================
Sub LOG_write(txtToWrite)

    // Updated on 2026-06-10, 10:30
    // 
    // Write string to log file
    //
    // Arguments:
    //  - txtToWrite - text to write into logfile
    //
    // Return: NONE
    //
    // Calls/depends on these functions/global variables
    // LOG_WRITE_FLAG
    // TMR_PROG_START
    // LOG_FH

    
    if _LOG_WRITE_FLAG
        FileWrite(_LOG_FH,ToString( Timer() - _TMR_PROG_START) + "ms: " + ToString(txtToWrite) + CrLf())
    endif
Return
//==============================================================================================
