#ifndef slip_h__
#define slip_h__
 

#include <stdbool.h>

#define nQ 6
#define nU 2

typedef struct state_t {
	double q[nQ];
	double qd[nQ];
	double qdd[nQ];
	double u[nU];
	//double M[nQ*nQ];
} state_t;


typedef struct pos_limits_t {
	double lb[nQ];
	double ub[nQ];
} pos_limits_t;

typedef struct motor_limits_t {
	double lb[nU];
	double ub[nU];
} motor_limits_t;

typedef struct slip_t slip_t;
typedef struct slip_vis_t slip_vis_t;


slip_t *init(void);
void forward(slip_t* s, state_t* state);
void step(slip_t* s, state_t* state);
void run_forward(slip_t* s, state_t* state, double DT);
void step_ctrl(slip_t* s);
//int get_stationary_init(slip_t *s, state_t *state);
void get_joint_limits(pos_limits_t *lim);
void get_motor_limits(motor_limits_t *lim);


slip_vis_t *vis_init(void);
bool vis_draw(slip_vis_t *v, slip_t *s, bool bWaitUser);
void vis_close(slip_vis_t *v);
void Keyboard(int key, int scancode, int act, int mods, slip_vis_t* v);
void Scroll(double xoffset, double yoffset, slip_vis_t* v);
void MouseMove(double xpos, double ypos, slip_vis_t* v);
void MouseButton(int button, int act, int mods, slip_vis_t* v);

#endif  // slip_h__

