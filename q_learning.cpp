#include "q_learning.h"

float* QL_MODEL::allocate(float *obj, int num_dims, int dims[]) {
  int total_memory = 0;
  for(int i=0; i<dims[0]; i++) {
    total_memory += dims[1]; 
  }
  obj = new float [total_memory];
  return obj;
}

int QL_MODEL::explore(void) {
  return (rand()%3);
}

int QL_MODEL::exploit(int state) {
  float max_action = -1000.0;
  int max_index;
  for(int i=0; i<this->num_actions; i++) {
    if(this->Q[state*this->num_actions] > max_action) {
      max_action = Q[state*this->num_actions];
      max_index = i;
    }
  }
  return max_index;
}

void QL_MODEL::update_Q(int state, int action, int state_new) {
  float max_Q = 0.0;
  for(int i=0; i<this->num_actions; i++) {
    if(this->Q[state_new*this->num_actions + i] > max_Q)
      max_Q = this->Q[state_new*this->num_actions + i];
  }
  this->Q[state*this->num_actions + action] = (1.0 - this->learning_rate)*this->Q[state*this->num_actions + action] + \
                                              this->learning_rate*(this->R[state*this->num_actions + action] + \
                                              this->discount_rate*max_Q);
  return;
}

void QL_MODEL::free_model() {
  delete this->Q;
  delete this->R;
  return;
}
