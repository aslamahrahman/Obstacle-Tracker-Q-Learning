/*BEGIN CONTROLS---------------------------------------------*/
#define LED_BUILTIN 2
#define DEBUG 1
/*END CONTROLS-----------------------------------------------*/

/*BEGIN TRAINING STUFF---------------------------------------*/
const int pin_training = 36;
const int pin_training_indicator = 22;
/*END TRAINING STUFF-----------------------------------------*/
  
/*BEGIN ULTRASONIC STUFF--------------------------------*/
const int pin_trigger = 25;
const int pin_echo = 26;
volatile long duration_isr = 0;
volatile long start_time = micros();
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
long int interruptCounter = 0;
/*END ULTRASONIC STUFF----------------------------------*/

/*BEGIN MOTOR STUFF-------------------------------------*/
//define motor variables
int motor1_pwm = 0;
int motor2_pwm = 0;
int motor1_dir1 = 0;
int motor1_dir2 = 0;
int motor2_dir1 = 0;
int motor2_dir2 = 0;

const int pin_motor1_pwm = 23;
const int pin_motor2_pwm = 22;
const int pin_motor1_dir1 = 19;
const int pin_motor1_dir2 = 18;
const int pin_motor2_dir1 = 17;
const int pin_motor2_dir2 = 16;

//constant speed for stage 1
const int drive_speed = 200;

//define thresholds to remove noise from pot readings
const int drive_thresh = 50;
const int drive_thresh_forward = drive_thresh;
const int drive_thresh_backward = -drive_thresh;
const int turn_thresh = 25;
const int dir_thresh_right = turn_thresh;
const int dir_thresh_left = -turn_thresh;

//direction pins values, caster wheel on left, motor 2 nearer to you, forward is anti-clockwise rotation, backward is clockwise rotation
const byte bw_dir1 = LOW;
const byte bw_dir2 = HIGH;
const byte fw_dir1 = HIGH;
const byte fw_dir2 = LOW;
const int is_turn_linear = 1;
/*END MOTOR STUFF---------------------------------------*/

/*BEGIN LEDC STUFF----------------------------------------*/
const int freq = 1200;
const int led1_channel = 0;
const int led2_channel = 1;
const int resolution = 8;
/*END LEDC STUFF-------------------------------------------*/

/*BEGIN Q LEARNING STUFF------------------------------*/
QL_MODEL agent;
const int num_states = 9;
const int num_actions = 3;
int goal = 1;
int max_epsilon = 80;
int min_epsilon = 1;
float decay_rate = 0.0001f;
float learning_rate = 0.1;
float discount_rate = 0.9; 
float reward_amplifier = 1.0;
float Q[num_states][num_actions] = {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};
float R[num_states][num_actions] = {{-1,1,0}, {-1, -1, 100}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, -1, 0}} ;

int state_now, state_prev, state_next;
long int counter = 0;
/*END Q LEARNING STUFF--------------------------------*/
