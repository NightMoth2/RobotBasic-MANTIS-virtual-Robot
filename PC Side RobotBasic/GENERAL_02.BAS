//==============================================================================================
// Some functions for RB general use
// Author: NightMoth (Kyrylo)
//
// Ver 2 - 2026-06-04
// - Function to create Datetime string added
//
// Ver 1
// Created on 01 JUNE 2025, 23:46
// FInshed on 10 JUNE 2025, 16:56
//
// rho - "right-handed" orientation of cartesian coordinates
// servo motor, and RB cartesian-polar coordinate-conversion functions based on rho system
//  y
//  |
//  |
//  |
//  0-------x
//
// lho - "left-handed" orientation of cartesian coordinates
// RB screen terminal based on lho system
//  0-------x
//  |
//  |
//  |
//  y
//
// nlho - "north-left-handed" orientation of cartesian coordinates
// RB robot compass based on nlho system
//  x
//  |
//  |
//  |
//  0-------y
//
// THESE GLOBAL FLAGS AND VARS MUST BE DECLARED IN MAIN PROGRAMM:
// SCR_XW - terminal screen width in pixels
// SCR_YW - terminal screen height in pixels
//
// Refs:
// [1] D. J. Griffiths, Introduction to electrodynamics, Fourth edition. Boston: Pearson, 2013.
// [2] S. J. Leon, Linear algebra with applications, 9. ed., Global ed. in Always learning. Boston Munich: Pearson, 2015.


//==============================================================================================
Init_lib_GEN:

    GEN_SCR_XW = 800    // pixels, width of terminal screen
    GEN_SCR_YW = 600    // pixels, height of terminal screen

Return


//==============================================================================================
Sub GEN_makeDateTimeString()

    startDateTime = Now()

    strYear = ToString(Year(startDateTime))

    strMon = ToString(Month(startDateTime))
    if Length(strMon) < 2
        strMon = "0" + strMon
    endif

    strDay = ToString(Day(startDateTime))
    if Length(strDay) < 2
        strDay = "0" + strDay
    endif

    strHour = ToString(Hour(startDateTime))
    if Length(strHour) < 2
        strHour = "0" + strHour
    endif

    strMin = ToString(Minute(startDateTime))
    if Length(strMin) < 2
        strMin = "0" + strMin
    endif

    strSec = ToString(Second(startDateTime))
    if Length(strSec) < 2
        strSec = "0" + strSec
    endif

    strDateTime = strYear + "-" + strMon + "-" + strDay + "-" + strHour + "-" + strMin + "-" + strSec  

Return strDateTime


//==============================================================================================
Sub GEN_lho2rhoX(X_lho)

    // Updated on 2025-06-02, 10:31
    //
    // conwert lho X to rho X
    // (RB terminal screen X to regular X)
    //
    // Arguments:
    // - X_lho - X in lho
    //
    // Return:
    // - X_rho - X in rho
    //
    // Calls/depends on these functions/global variables:
    // - NONE

    X_rho = X_lho

Return X_rho
//==============================================================================================
Sub GEN_lho2rhoY(Y_lho, Y_lho_max)

    // Updated on 2025-06-02, 10:31
    //
    // conwert lho Y to rho Y
    // (RB terminal screen Y to regular Y)
    //
    // Arguments:
    // - Y_lho - Y in lho
    // - Y_lho_max - maximum Y value (for example terminal screen height)
    //
    // Return:
    // - Y_rho - Y in rho
    //
    // Calls/depends on these functions/global variables:
    // - GEN_SCR_YW

    // if Y_lho_max not passed as argument, make Y_lho_max = GEN_SCR_YW
    if !vType(Y_lho_max) then Y_lho_max = _GEN_SCR_YW
    Y_rho =  Y_lho_max - Y_lho

Return Y_rho
//==============================================================================================
Sub GEN_rho2lhoX(X_rho)

    // Updated on 2025-06-02, 10:31
    //
    // conwert rho X to lho X
    // (regular X to RB terminal screen X)
    //
    // Arguments:
    // - X_rho - X in rho
    //
    // Return:
    // - X_lho - X in lho
    //
    // Calls/depends on these functions/global variables:
    // - NONE

    X_lho = X_rho

Return X_lho
//==============================================================================================
Sub GEN_rho2lhoY(Y_rho, Y_rho_max)

    // Updated on 2025-06-02, 10:31
    //
    // conwert rho Y to lho Y
    // (regular Y to RB terminal screen Y)
    //
    // Arguments:
    // - Y_rho - Y in rho
    // - Y_rho_max - maximum Y value (for example terminal screen height)
    //
    // Return:
    // - Y_lho - Y in lho
    //
    // Calls/depends on these functions/global variables:
    // - GEN_SCR_YW

    // if Y_rho_max not passed as argument, make Y_rho_max = GEN_SCR_YW
    if !vType(Y_rho_max) then Y_rho_max = _GEN_SCR_YW
    Y_lho =  Y_rho_max - Y_rho

Return Y_lho
//==============================================================================================
Sub GEN_nlhoAng2rhoAng(ang_nlho)

    // Updated on 2025-06-02, 10:31
    //
    // nlho angle to rho angle
    // Convert robot heading angle into right-handed cartesian coord angle
    // Robot heading: N: 0deg, E: 90deg, S: 180deg, W: 270deg
    // RHO heading: N: 90deg, E: 0deg, S: 270deg, W: 180deg
    //
    // Arguments:
    // - ang_nlho - angle in nlho system
    //
    // Return:
    // - ang_rho - angle in rho system 
    //
    // Calls/depends on these functions/global variables:
    // - NONE

    ang_rho = (90 + (360 - ang_nlho)) # 360 


Return ang_rho
//==============================================================================================
Sub GEN_rhoAng2nlhoAng(ang_rho)


    // Updated on 2026-05-04
    // ang_nlho = 90 - ang_rho
    //
    // Updated on 2025-06-02, 10:31
    // formula ang_nlho = ((360 - ang_rho) + 90) # 360 
    //
    //
    // rho angle to nlho angle:
    // converts right-handed orientation cartesian coord angle into 
    // north-left-handed (RB robot compass heading angle)
    // Robot heading: N: 0deg, E: 90deg, S: 180deg, W: 270deg
    // RHO heading: N: 90deg, E: 0deg, S: 270deg, W: 180deg
    //
    // Arguments:
    // - ang_rho - angle in rho system
    //
    // Return:
    // - ang_nlho - angle in nlho system (RB robot compass heading angle)
    //
    // Calls/depends on these functions/global variables:
    // - NONE

    ang_nlho = 90 - ang_rho


Return ang_nlho
//==============================================================================================
Sub GEN_getCSRotX(x,y,phi_deg)

    // Updated on 2026-06-24, 12:08
    //
    // Coordinate system rotation
    // Rotate coordinate system by angle phi_deg (in degrees)
    // and return x' - vector coordinate in rotated coordinate system
    //
    // Arguments:
    // - x - original vector x component
    // - y - original vector y component
    // - phi_deg - deg. rho, angle to rotate coordinate system
    // Return:
    // - x_prime - vector x component after rotation of coordinate system
    //
    // Ref: 
    // [1] D. J. Griffiths, Introduction to electrodynamics, Fourth edition. Boston: Pearson, 2013.

    phi_rad = DtoR(phi_deg)

    x_prime = x*COS(phi_rad) + y*SIN(phi_rad)

Return x_prime
//==============================================================================================
Sub GEN_getCSRotY(x,y,phi_deg)

    // Updated on 2026-06-24, 12:08
    //
    // Coordinate system rotation
    // Rotate coordinate system by angle phi_deg (in degrees)
    // and return y' - vector coordinate in rotated coordinate system
    //
    // Arguments:
    // - x - original vector x component
    // - y - original vector y component
    // - phi_deg - deg. rho, angle to rotate coordinate system
    // Return:
    // - y_prime - vector y component after rotation of coordinate system
    // 
    //[1] D. J. Griffiths, Introduction to electrodynamics, Fourth edition. Boston: Pearson, 2013.

    phi_rad = DtoR(phi_deg)

    y_prime = -x*SIN(phi_rad) + y*COS(phi_rad)

Return y_prime
//==============================================================================================
Sub GEN_getVecRotX(x,y,phi_deg)

    // Updated on 2026-06-24, 12:08
    //
    // Vector rotation
    // Rotate vector by angle phi_deg (in degrees)
    // and return x' - vector coordinate after rotation
    // |x'|   |cos(phi)   -sin(phi)| |x|
    // |  | = |                    | | |
    // |y'|   |sin(phi)    cos(phi)| |y|
    //
    // Arguments:
    // - x - original vector x component
    // - y - original vector y component
    // - phi_deg - deg. rho, angle to rotate origina vector
    // Return:
    // - x_prime - rotated vector x component 
    // Ref:
    // S. J. Leon, Linear algebra with applications, 9. ed., Global ed. in Always learning. Boston Munich: Pearson, 2015.

    phi_rad = DtoR(phi_deg)

    x_prime = x*COS(phi_rad) - y*SIN(phi_rad)

Return x_prime
//==============================================================================================
Sub GEN_getVecRotY(x,y,phi_deg)

    // Updated on 2026-06-24, 12:08
    //
    // Vector rotation
    // Rotate vector by angle phi_deg (in degrees)
    // and return y' - vector coordinate after rotation
    // |x'|   |cos(phi)   -sin(phi)| |x|
    // |  | = |                    | | |
    // |y'|   |sin(phi)    cos(phi)| |y|
    //
    // Arguments:
    // - x - original vector x component
    // - y - original vector y component
    // - phi_deg - deg. rho, angle to rotate origina vector
    // Return:
    // - y_prime - rotated vector y component 
    // Ref:
    // S. J. Leon, Linear algebra with applications, 9. ed., Global ed. in Always learning. Boston Munich: Pearson, 2015.

    phi_rad = DtoR(phi_deg)

    y_prime = x*SIN(phi_rad) + y*COS(phi_rad)

Return y_prime
//==============================================================================================
Sub GEN_rvtA(ang,dist,rotAng,shiftX,shiftY)

    // Updated on 2026-06-24, 11:38
    //
    // Vector transformation (rotation and shift), radial coordinates
    // rotates and shifts vector defined by (ang,dist) pair
    // returns new heading angle
    //
    // Arguments:
    // - ang - deg. rho, original vector heading angle
    // - dist - original vector length
    // - rotAng - deg. rho, rotation angle
    // - shiftX - distance to shift original vector along x coordinate
    // - shiftY - distance to shift original vector along y coordinate
    // Return:
    // - ang_prime - deg., vectot heading angle after transform
    
    // To cartesian
    angRad = DtoR(ang)
    X = CartX(dist,angRad)
    Y = CartY(dist,angRad)
    
    // Rotate vector
    Call GEN_getVecRotX(X,Y,rotAng)
    X_prime = GEN_getVecRotX__Result                        
    Call GEN_getVecRotY(X,Y,rotAng)
    Y_prime = GEN_getVecRotY__Result
    
    // Shift vector
    X_prime = X_prime + shiftX
    Y_prime = Y_prime + shiftY
    
    // Back to polar
    //dist_prime = Round(PolarR(X_prime,Y_prime))
    ang_prime = Round(RtoD(PolarA(X_prime,Y_prime)))

Return ang_prime
//==============================================================================================
Sub GEN_rvtD(ang,dist,rotAng,shiftX,shiftY)

    // Updated on 2026-06-24, 11:38
    //
    // Vector transformation (rotation and shift), radial coordinates
    // rotates and shifts vector defined by (ang,dist) pair
    // returns new vector radius
    //
    // Arguments:
    // - ang - deg. rho, original vector heading angle
    // - dist - original vector length
    // - rotAng - deg. rho, rotation angle
    // - shiftX - distance to shift original vector along x coordinate
    // - shiftY - distance to shift original vector along y coordinate
    // Return:
    // - dist_prime - vector radius after transform
    
    // To cartesian
    angRad = DtoR(ang)
    X = CartX(dist,angRad)
    Y = CartY(dist,angRad)
    
    // Rotate vector
    Call GEN_getVecRotX(X,Y,rotAng)
    X_prime = GEN_getVecRotX__Result                        
    Call GEN_getVecRotY(X,Y,rotAng)
    Y_prime = GEN_getVecRotY__Result
    
    // Shift vector
    X_prime = X_prime + shiftX
    Y_prime = Y_prime + shiftY
    
    // Back to polar
    dist_prime = Round(PolarR(X_prime,Y_prime))
    //ang_prime = Round(RtoD(PolarA(X_prime,Y_prime)))

Return dist_prime
