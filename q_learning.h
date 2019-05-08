#include <math.h>
#include <stdlib.h>
#include <stdio.h>

class QL_MODEL {
  public:
    int num_states;
    int num_actions;
    float *Q = NULL;
    float *R = NULL;
    int epsilon;
    int max_epsilon;
    int min_epsilon;
    float learning_rate;
    float discount_rate; 
    float decay_rate;
    float reward_amplifier;

    float* allocate(float *obj, int num_dims, int dims[]);
    int exploit(int state);
    int explore(void);
    void update_Q(int state, int action, int state_new);
    void free_model(void);
};
