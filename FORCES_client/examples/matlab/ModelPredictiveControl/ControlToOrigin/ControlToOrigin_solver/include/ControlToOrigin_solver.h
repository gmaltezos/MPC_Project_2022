/*
ControlToOrigin_solver : A fast customized optimization solver.

Copyright (C) 2013-2021 EMBOTECH AG [info@embotech.com]. All rights reserved.


This software is intended for simulation and testing purposes only. 
Use of this software for any commercial purpose is prohibited.

This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/

/* Generated by FORCESPRO v5.1.0 on Saturday, April 30, 2022 at 4:17:25 PM */
#ifndef ControlToOrigin_solver_H
#define ControlToOrigin_solver_H

#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#endif


#ifndef SOLVER_STANDARD_TYPES
#define SOLVER_STANDARD_TYPES

typedef signed char solver_int8_signed;
typedef unsigned char solver_int8_unsigned;
typedef char solver_int8_default;
typedef signed short int solver_int16_signed;
typedef unsigned short int solver_int16_unsigned;
typedef short int solver_int16_default;
typedef signed int solver_int32_signed;
typedef unsigned int solver_int32_unsigned;
typedef int solver_int32_default;
typedef signed long long int solver_int64_signed;
typedef unsigned long long int solver_int64_unsigned;
typedef long long int solver_int64_default;

#endif


/* DATA TYPE ------------------------------------------------------------*/
typedef double ControlToOrigin_solver_float;


typedef double ControlToOrigin_solverinterface_float;

/* SOLVER SETTINGS ------------------------------------------------------*/

/* MISRA-C compliance */
#ifndef MISRA_C_ControlToOrigin_solver
#define MISRA_C_ControlToOrigin_solver (0)
#endif

/* restrict code */
#ifndef RESTRICT_CODE_ControlToOrigin_solver
#define RESTRICT_CODE_ControlToOrigin_solver (0)
#endif

/* print level */
#ifndef SET_PRINTLEVEL_ControlToOrigin_solver
#define SET_PRINTLEVEL_ControlToOrigin_solver    (2)
#endif

/* timing */
#ifndef SET_TIMING_ControlToOrigin_solver
#define SET_TIMING_ControlToOrigin_solver    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define SET_MAXIT_ControlToOrigin_solver         (50)	

/* scaling factor of line search (affine direction) */
#define SET_LS_SCALE_AFF_ControlToOrigin_solver  (ControlToOrigin_solver_float)(0.9)      

/* scaling factor of line search (combined direction) */
#define SET_LS_SCALE_ControlToOrigin_solver      (ControlToOrigin_solver_float)(0.95)  

/* minimum required step size in each iteration */
#define SET_LS_MINSTEP_ControlToOrigin_solver    (ControlToOrigin_solver_float)(1E-08)

/* maximum step size (combined direction) */
#define SET_LS_MAXSTEP_ControlToOrigin_solver    (ControlToOrigin_solver_float)(0.995)

/* desired relative duality gap */
#define SET_ACC_RDGAP_ControlToOrigin_solver     (ControlToOrigin_solver_float)(0.0001)

/* desired maximum residual on equality constraints */
#define SET_ACC_RESEQ_ControlToOrigin_solver     (ControlToOrigin_solver_float)(1E-06)

/* desired maximum residual on inequality constraints */
#define SET_ACC_RESINEQ_ControlToOrigin_solver   (ControlToOrigin_solver_float)(1E-06)

/* desired maximum violation of complementarity */
#define SET_ACC_KKTCOMPL_ControlToOrigin_solver  (ControlToOrigin_solver_float)(1E-06)

/* desired maximum violation of stationarity (only checked if value is > 0) */
#define SET_ACC_KKTSTAT_ControlToOrigin_solver  (ControlToOrigin_solver_float)(-1)

/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define OPTIMAL_ControlToOrigin_solver      (1)

/* maximum number of iterations has been reached */
#define MAXITREACHED_ControlToOrigin_solver (0)

/* solver has stopped due to a timeout */
#define TIMEOUT_ControlToOrigin_solver   (2)

/* no progress in line search possible */
#define NOPROGRESS_ControlToOrigin_solver   (-7)

/* fatal internal error - nans occurring */
#define NAN_ControlToOrigin_solver  (-10)

/* invalid values in parameters */
#define PARAM_VALUE_ERROR_ControlToOrigin_solver   (-11)

/* too small timeout given */
#define INVALID_TIMEOUT_ControlToOrigin_solver   (-12)

/* thread error */
#define THREAD_FAILURE_ControlToOrigin_solver  (-98)

/* locking mechanism error */
#define LOCK_FAILURE_ControlToOrigin_solver  (-99)

/* licensing error - solver not valid on this machine */
#define LICENSE_ERROR_ControlToOrigin_solver  (-100)



/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct ControlToOrigin_solver_params
{
    /* vector of size 2 */
    ControlToOrigin_solver_float minusA_times_x0[2];

} ControlToOrigin_solver_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct ControlToOrigin_solver_output
{
    /* vector of size 1 */
    ControlToOrigin_solver_float u0[1];

} ControlToOrigin_solver_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct ControlToOrigin_solver_info
{
    /* iteration number */
    solver_int32_default it;

	/* number of iterations needed to optimality (branch-and-bound) */
	solver_int32_default it2opt;
	
    /* inf-norm of equality constraint residuals */
    ControlToOrigin_solver_float res_eq;
	
    /* inf-norm of inequality constraint residuals */
    ControlToOrigin_solver_float res_ineq;

    /* primal objective */
    ControlToOrigin_solver_float pobj;	
	
    /* dual objective */
    ControlToOrigin_solver_float dobj;	

    /* duality gap := pobj - dobj */
    ControlToOrigin_solver_float dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    ControlToOrigin_solver_float rdgap;		

	/* infinity norm of gradient of Lagrangian*/
	ControlToOrigin_solver_float gradient_lag_norm;

    /* duality measure */
    ControlToOrigin_solver_float mu;

	/* duality measure (after affine step) */
    ControlToOrigin_solver_float mu_aff;
	
    /* centering parameter */
    ControlToOrigin_solver_float sigma;
	
    /* number of backtracking line search steps (affine direction) */
    solver_int32_default lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    solver_int32_default lsit_cc;
    
    /* step size (affine direction) */
    ControlToOrigin_solver_float step_aff;
    
    /* step size (combined direction) */
    ControlToOrigin_solver_float step_cc;    

	/* solvertime */
	ControlToOrigin_solver_float solvetime;   

} ControlToOrigin_solver_info;









/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* Time of Solver Generation: (UTC) Saturday, April 30, 2022 4:17:25 PM */
/* User License expires on: (UTC) Friday, June 3, 2022 9:00:00 PM (approx.) (at the time of code generation) */
/* Solver Static License expires on: (UTC) Friday, June 3, 2022 9:00:00 PM (approx.) */
/* Solver Generation Request Id: 8b7df736-5ead-4d38-972c-0243ee9d6744 */
/* examine exitflag before using the result! */
#ifdef __cplusplus
extern "C" {
#endif
extern solver_int32_default ControlToOrigin_solver_solve(ControlToOrigin_solver_params *params, ControlToOrigin_solver_output *output, ControlToOrigin_solver_info *info, FILE *fs);

;



#ifdef __cplusplus
}
#endif

#endif
