#include "q_learning.h"
#include "vars.h"

void setup() {
  Serial.begin(115200);
  initialize();

  /*BEGIN LEDC STUFF--------------------------------------*/
  ledcSetup(led1_channel, freq, resolution);
  ledcAttachPin(pin_motor1_pwm, led1_channel);
  ledcSetup(led2_channel, freq, resolution);
  ledcAttachPin(pin_motor2_pwm, led2_channel);
  /*END LEDC STUFF---------------------------------------*/

  /*BEGIN ULTRASONIC STUFF---------------------------------*/
  attachInterrupt(digitalPinToInterrupt(pin_echo), ISR_ultrasonic, CHANGE);
  /*END ULTRASONIC STUFF-----------------------------------*/

  Serial.println("Begin!");
  state_now = get_state();
}

void loop() {
  /*BEGIN Q LEARNING STUFF------------------------------*/
  counter++;
  int action = 0;
  if(digitalRead(pin_training) == HIGH) {
    //Train
    if(state_now != goal) { 
      //Choose
      int random_number = random(0,100);
      if(random_number < agent.epsilon) {
        //Exploitaition
        action = agent.exploit(state_now);
      }
      else {
        //Exploration
        action = agent.explore();
      }

      //Perform
      if(action == 0) {
        drive(drive_speed);
      }
      else if(action == 1) {
        drive(-drive_speed);
      }
      else {
        drive(0);
      }
      delay(100);
      stop_now();

      #ifdef DEBUG
        Serial.print("State: ");
        Serial.println(state_now);
        Serial.print("Action: ");
        Serial.println(action);
        Serial.print("Epsilon: ");
        Serial.println(agent.epsilon);
      #endif
      
      //Evaluate
      state_next = get_state();
      agent.R[state_next*agent.num_actions + action] = float(-(state_next - state_now))*agent.reward_amplifier;
      agent.update_Q(state_now, action, state_next);
      state_now = state_next;
    }
    else {
      state_now = get_state();
    }
  }
  else {
    //Test
  }
  /*END Q LEARNING STUFF------------------------------*/
}

void initialize() {
  /*BEGIN TRAINING STUFF---------------------------------------*/
  pinMode(pin_training, INPUT_PULLDOWN);
  pinMode(pin_training_indicator, OUTPUT);
  /*END TRAINING STUFF-----------------------------------------*/
  
  /*BEGIN ULTRASONIC STUFF--------------------------------*/
  pinMode(pin_trigger, OUTPUT);
  pinMode(pin_echo, INPUT);
  /*END ULTRASONIC STUFF----------------------------------*/

  /*BEGIN MOTOR STUFF-------------------------------------*/
  pinMode(pin_motor1_pwm, OUTPUT);
  pinMode(pin_motor2_pwm, OUTPUT);
  pinMode(pin_motor1_dir1, OUTPUT);
  pinMode(pin_motor1_dir2, OUTPUT);
  pinMode(pin_motor2_dir1, OUTPUT);
  pinMode(pin_motor2_dir2, OUTPUT);
  /*END MOTOR STUFF-------------------------------------*/

  /*BEGIN Q LEARNING STUFF------------------------------*/
  int dims[2] = {num_states, num_actions};
  agent.num_states = num_states;
  agent.num_actions = num_actions;
  agent.Q = agent.allocate(agent.Q, 2, dims);
  agent.R = agent.allocate(agent.R, 2, dims);

  for(int i=0; i<dims[0]; i++) {
    for(int j=0; j<dims[1]; j++) {
      agent.Q[i*dims[1] + j] = Q[i][j];
      agent.R[i*dims[1] + j] = R[i][j];
    }
  }
  agent.min_epsilon = min_epsilon;
  agent.max_epsilon = max_epsilon;
  agent.epsilon = max_epsilon;
  agent.learning_rate = learning_rate;
  agent.discount_rate = discount_rate;
  agent.decay_rate = decay_rate;
  agent.reward_amplifier = reward_amplifier;
  /*END Q LEARNING STUFF------------------------------*/
}

/*BEGIN ULTRASONIC STUFF--------------------------------*/
int get_distance() {
  int duration, cm = 0;
  int num_samples = 10;
  for(int i=0; i<num_samples; i++) {
    digitalWrite(pin_trigger, LOW);
    delayMicroseconds(2);
    digitalWrite(pin_trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin_trigger, LOW);
    delayMicroseconds(2);
    duration = pulseIn(pin_echo, HIGH);
    cm += (duration/29)/2;
  }
  cm = cm/num_samples;
  return cm;
}

void pulse_ultrasonic() {
  digitalWrite(pin_trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(pin_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin_trigger, LOW);
  delayMicroseconds(2);
}

void ISR_ultrasonic() {
  if(digitalRead(pin_echo) == LOW) {
    portENTER_CRITICAL(&mux);
      duration_isr = micros() - start_time;
    portEXIT_CRITICAL(&mux); 
  }
  else {
    start_time = micros();
  }
}

int get_state() {
//  pulse_ultrasonic();
//  static int dist_prev;
//  int dist = duration_isr/29/2;
//  if(dist > 500) {
//    dist = dist_prev;
//  } 
//  dist_prev = dist;

  int dist = get_distance();

  #ifdef DEBUG
    Serial.print("Distance: ");
    Serial.println(dist);
  #endif 
  
  int state = dist/10;
  if(state < 8) {
    return state;
  }
  else {
    return 7;
  }
}
/*END ULTRASONIC STUFF----------------------------------*/

/*BEGIN MOTOR STUFF-------------------------------------*/
void drive(int velocity) {
  Serial.println("driving");
  if (velocity > drive_thresh_forward) {
    Serial.println("Forward");
    //go forward
    motor1_pwm = abs(velocity);
    motor2_pwm = abs(velocity);

    digitalWrite(pin_motor1_dir1, fw_dir1);
    digitalWrite(pin_motor1_dir2, fw_dir2);
    ledcWrite(led1_channel, motor1_pwm);

    digitalWrite(pin_motor2_dir1, fw_dir1);
    digitalWrite(pin_motor2_dir2, fw_dir2);
    ledcWrite(led2_channel, motor2_pwm);
  }
  else if (velocity < drive_thresh_backward) {
    Serial.println("Backward");
    //go backward
    motor1_pwm = abs(velocity);
    motor2_pwm = abs(velocity);

    digitalWrite(pin_motor1_dir1, bw_dir1);
    digitalWrite(pin_motor1_dir2, bw_dir2);
    ledcWrite(led1_channel, motor1_pwm);

    digitalWrite(pin_motor2_dir1, bw_dir1);
    digitalWrite(pin_motor2_dir2, bw_dir2);
    ledcWrite(led2_channel, motor2_pwm);
  }
  else {
    Serial.println("Stop");
    //stay right there
    motor1_pwm = 0;
    motor2_pwm = 0;
    ledcWrite(led1_channel, motor1_pwm);
    ledcWrite(led2_channel, motor2_pwm);
  }
}

void stop_now(void) {
  motor1_pwm = 0;
  motor2_pwm = 0;
  ledcWrite(led1_channel, motor1_pwm);
  ledcWrite(led2_channel, motor2_pwm);

  return;
}

void turn_left(int velocity, int pot_turn) {
  if (velocity > drive_thresh_forward) {
    //go forward left
    motor1_pwm = abs(velocity);
    motor2_pwm = is_turn_linear*(abs(velocity)*pot_turn/(3*turn_thresh) + 4*abs(velocity)/3);

    digitalWrite(pin_motor1_dir1, fw_dir1);
    digitalWrite(pin_motor1_dir2, fw_dir2);
    ledcWrite(led1_channel, motor1_pwm);

    digitalWrite(pin_motor2_dir1, fw_dir1);
    digitalWrite(pin_motor2_dir2, fw_dir2);
    ledcWrite(led2_channel, motor2_pwm);
  }
  else if (velocity < drive_thresh_backward) {
    //go backward left
    motor1_pwm = abs(velocity);
    motor2_pwm = is_turn_linear*(abs(velocity)*pot_turn/(3*turn_thresh) + 4*abs(velocity)/3);

    digitalWrite(pin_motor1_dir1, bw_dir1);
    digitalWrite(pin_motor1_dir2, bw_dir2);
    ledcWrite(led1_channel, motor1_pwm);

    digitalWrite(pin_motor2_dir1, bw_dir1);
    digitalWrite(pin_motor2_dir2, bw_dir2);
    ledcWrite(led2_channel, motor2_pwm);
  }
  else {
    //stay right there
    motor1_pwm = 0;
    motor2_pwm = 0;
    ledcWrite(led1_channel, motor1_pwm);
    ledcWrite(led2_channel, motor2_pwm);
  }
}

void turn_right(int velocity, int pot_turn) {
  if (velocity > drive_thresh_forward) {
    //go forward right
    motor1_pwm = is_turn_linear*(-abs(velocity)*pot_turn/(3*turn_thresh) + 4*abs(velocity)/3);
    motor2_pwm = abs(velocity);

    digitalWrite(pin_motor1_dir1, fw_dir1);
    digitalWrite(pin_motor1_dir2, fw_dir2);
    ledcWrite(led1_channel, motor1_pwm);

    digitalWrite(pin_motor2_dir1, fw_dir1);
    digitalWrite(pin_motor2_dir2, fw_dir2);
    ledcWrite(led2_channel, motor2_pwm);
  }
  else if (velocity < drive_thresh_backward) {
    //go backward right
    motor1_pwm = is_turn_linear*(-abs(velocity)*pot_turn/(3*turn_thresh) + 4*abs(velocity)/3);
    motor2_pwm = abs(velocity);
    
    digitalWrite(pin_motor1_dir1, bw_dir1);
    digitalWrite(pin_motor1_dir2, bw_dir2);
    ledcWrite(led1_channel, motor1_pwm);

    digitalWrite(pin_motor2_dir1, bw_dir1);
    digitalWrite(pin_motor2_dir2, bw_dir2);
    ledcWrite(led2_channel, motor2_pwm);
  }
  else {
    //stay right there
    motor1_pwm = 0;
    motor2_pwm = 0;
    ledcWrite(led1_channel, motor1_pwm);
    ledcWrite(led2_channel, motor2_pwm);
  }
}
/*END MOTOR STUFF---------------------------------------*/

/*PRINT MISCELLANEOUS STUFF--------------------------------*/
void show(float *obj, int nx) {
  int i;
  for(i=0; i<nx; i++) {
    Serial.print(obj[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
  return;
}

void show(float *obj, int nx, int ny) {
  int i, j;
  for(i=0; i<nx; i++) {
    for(j=0; j<ny; j++) {
      Serial.print(obj[i*ny+j]);
      Serial.print(" ");
    }
    Serial.println("");
  }
  return;
}
/*END MISCELLANEOUS STUFF----------------------------------*/
