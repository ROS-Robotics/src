/*
 * bridge.h
 *
 *  Created on: Dec 12, 2016
 *      Author: webot
 */

#ifndef BRIDGE_INCLUDE_BRIDGE_BRIDGE_H_
#define BRIDGE_INCLUDE_BRIDGE_BRIDGE_H_



#define IR_DATA   0
#define IR_DATA_LF   command_long_msg.param1     // POS: left front
#define IR_DATA_CF   command_long_msg.param2     // POS: center front
#define IR_DATA_RF   command_long_msg.param3     // POS: right front

#define IR_NUM 3
#define IR_PCL_HEIGHT 1

#define BARRIER_DISTANCE 0.3


#define ODOM_DATA 1
#define LINEAR_X_SPEED  command_long_msg.param1
#define ANGULAR_Z_SPEED command_long_msg.param2
#define Pos_X command_long_msg.param4
#define Pos_Y command_long_msg.param3



#endif /* BRIDGE_INCLUDE_BRIDGE_BRIDGE_H_ */
