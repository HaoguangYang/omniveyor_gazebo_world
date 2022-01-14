#ifndef OMNIVEYOR_DEFINITIONS_H
#define OMNIVEYOR_DEFINITIONS_H

/********************************** MOTOR ******************************************/
#define NUM_MOTORS					(8)
#define QUEUE_SIZE					(32)
#define QUEUE_HIGH_WATER_MARK       (28)
#define MAX_NAME_LEN				(25)

/* Constants for unit conversion */
#define ENCODER_TICKS 				(4096.0)
#define SLOW_LOOP_SAMP_PERIOD  		(1.0/2000.0)  		// [s]
// Magmotor S28-E-200X
#define TORQUE_CONSTANT 			(0.12357715708333275) 			// [Nm/A]
#define CURRENT_PEAK				(20.0)    			// [A], limited by the controller.
#define CURRENT_CONT				(8.0)               // [A], limited by the controller.
#define CURRENT_NUM					(2.0)
#define CURRENT_DENOM				(65472.0) //(65520.0)
#define VOLTAGE_NUM					(102.0)
#define VOLTAGE_DENOM				(65520.0)
#define TORQUE_PEAK					(TORQUE_CONSTANT * CURRENT_PEAK)
#define TORQUE_CONT 				(TORQUE_CONSTANT * CURRENT_CONT)

/* timer */
#define MS_TO_NS(a)					((a)*1000*1000)
#define MSG_TIMEOUT					(2)  				// [s]
#define HOME_TIMEOUT				(20)				// [s]
#define HEARTBEAT_TIMEOUT			(2)					// [s]

/* motor home offsets */
#define PI                      	(M_PI)
#define TWO_PI                  	(2 * PI)
#define HOME_OFFSET_MTR_1       	(-3.*PI/4.-0.13)//(-3 *PI / 4)
#define HOME_OFFSET_MTR_3       	(3.*PI/4.-0.18)//(3 * PI / 4)
#define HOME_OFFSET_MTR_5       	(PI/4.-0.16)//(PI / 4)
#define HOME_OFFSET_MTR_7       	(-PI/4.-0.15)//(-PI/ 4)

/* velocity and torque filter coeffecients: the smaller value, the less filtration */
#define LP_VEL_FILTER_COEFF			(0.75)
#define LP_TRQ_FILTER_COEFF			(0.75)

/********************************** CASTER ******************************************/
/* Gear ratios */
#define Ns							(4.0)               // Steer Gear Ratio
#define Nr 							(85.0/35.0)         // Roll Gear Ratio
#define Nw 							(2.0)				// Wheel Gear Ratio

/* Caster measurements */
#define PC_r 						(0.055*1.000) 		// [m] -- Wheel radius  * Empirical_Cal_Factor
#define PC_b 						(-0.020*0.995) 		// [m] -- Caster offset * Empirical_Cal_Factor
#define PC_h 						( 0.2934) 			// [m] -- FROM BOB's CODE. Never used in program. Originally 0.2159 for a smaller base

/********************************** VEHICLE *****************************************/
#define NUM_CASTERS 				4
#define DIST_TO_CASTER_X 			0.2075     	// from cad. need to verify (Haley's old number: 0.1300) 0.1375
#define DIST_TO_CASTER_Y 			0.2075  	// from cad. need to verify (Haley's old number: 0.1325) 0.147114
#define DIST_TO_CASTER 				0.2934		// radial dist calculated from X and Y above (used for max angular vel) (Haley's old number: 0.1856) 0.201

// Max change in velocity for one control loop (inc = increase, dec = decrease)
#define MAX_VEL_TRANS_INC   		(MAX_ACCEL_TRANS * CONTROL_PERIOD_s)
#define MAX_VEL_ROT_INC  			(MAX_ACCEL_ROT   * CONTROL_PERIOD_s)
#define MAX_VEL_TRANS_DEC   		(MAX_DECEL_TRANS * CONTROL_PERIOD_s)
#define MAX_VEL_ROT_DEC  			(MAX_DECEL_ROT   * CONTROL_PERIOD_s)

// FROM CONSTANTS.TXT
/** Powered Caster -- PC CONSTANTS **/
#define PC_f  						(-0.010)       		// [m] -- fork CoM
#define PC_e  						(PC_f - PC_b)  		// from PCV_Dynamics notes
#define PC_Mf 						(2.720)        		// [kg] -- Mass of fork
#define PC_If 						(3.32e-3)      		// [kg m2] -- Steering Inertia of fork at CoM

#define PC_Ih 						(4.78e-4)     		// [kg m2] -- Inertia of cluster gear
#define PC_Ii 						(1.74e-5)     		// [kg m2] -- Inertia of idle shaft & gearing
#define PC_Is 						(7.11e-5)     		// [kg m2] -- Inertia of steering motor rotor
#define PC_It 						(8.53e-5)     		// [kg m2] -- Inertia of traction motor rotor
#define PC_Ij 						(4.00e-4)     		// [kg m2] -- Rolling Inertia of wheel

#define PC_px 						(0.0)        		// [m]     -- pumpkin CoM x-coord
#define PC_py 						(0.0)        		// [m]     -- pumpkin CoM y-coord
#define PC_Mp 						(5.2)        		// [kg]    -- pumpkin Mass (non-moving in pumpkin frame)
#define PC_Ip 						(0.022)      		// [kg m2] -- pumpkin Inertia (non-moving in pumpkin frame)

// Vehicle CONSTANTS.  Change these for CM (x,y) Mass and Inertia
#define PC_length					(0.60)              // [m]     -- length of vehicle along the x and y directions. Originally 0.465
#define PC_height 					(0.35)              // [m]     -- height of vehicle
#define PC_Vx 						(0.0)      			// [m]     -- Vehicle CoM x-coord
#define PC_Vy 						(0.0)      			// [m]     -- Vehicle CoM y-coord
#define PC_Mv 						(61.5 - 4.0 * (PC_Mp + PC_Mf))           // [kg]  -- Vehicle Mass
#define PC_Iv 						(1.0/12.0 * PC_Mv * (pow(PC_length,2) + pow(PC_length,2))) // (1/12)*m*(.30^2 + .465^2) // [kg m2] -- Vehicle Inertia, Bob uses 2.1kg


// Control loop times for vehicle
#define CONTROL_PERIOD_ns 			(10000000)
#define CONTROL_PERIOD_s 			(0.01)

#define UNUSED_NODE_ID				(0xF)
#define MAX_ACCEL_TRANS     		(1.0)								// [m/s^2]
#define MAX_DECEL_TRANS     		(2.0)								// [m/s^2]
#define MAX_ACCEL_ROT       		(MAX_ACCEL_TRANS/DIST_TO_CASTER) 	// [rad/s^2]
#define MAX_DECEL_ROT       		(MAX_DECEL_TRANS/DIST_TO_CASTER)	// [m/s^2]
#define MAX_VEL_TRANS       		(1.0) 								// [m/s]
#define MAX_VEL_ROT         		(MAX_VEL_TRANS/DIST_TO_CASTER)  	// [rad/s]
#define MAX_VEL_X 					(0.3)								// [m/s]
#define MAX_VEL_Y					(0.2)								// [m/s]
#define MAX_VEL_TH					(0.2)								// [rad/s]
#define MAX_STEER_TORQUE 			(8.0)                               // [N/m]
#define MAX_ROLL_TORQUE 			(8.0)                               // [N/m]

#define BIT21HI                     (0x200000)

#endif
