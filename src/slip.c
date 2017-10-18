#include <stdio.h>
#include <stdbool.h>
#include "mujoco.h"
#include "slip.h"
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include "glfw3.h"
#include "stdlib.h"
#include "string.h"


static bool m_bMJActivated = false;
static bool glfw_initialized = false;

typedef struct slip_t {
        mjData *d;
}slip_t;

struct slip_vis_t {
	GLFWwindow* window;
	mjvCamera cam;
	mjvOption opt;
	mjvScene scn;
	mjrContext con;
};

static mjModel* m;

slip_t* init(void)
{
	if (!m_bMJActivated)
		mj_activate("mjpro150/bin/mjkey.txt");
	m_bMJActivated = true;

    char error[1000] = "Could not load binary model";
    m = mj_loadXML("models/Monopod.xml", 0, error, 1000); // allocates and initializes mjModel and returns a pointer to it. 
    if (!m)
    {    	
        mju_error("Failed to Load xml model");
    	return NULL;
    }

    slip_t *s = calloc(1, sizeof(slip_t));
    s->d = mj_makeData(m); // d is a pointer to mjData
// Once both mjModel and mjData are allocated and initialized, we can call the various simulation functions. 

    return s;
}

void forward(slip_t* s, state_t* state)
{

	mju_zero(s->d->xfrc_applied, m->nbody*6); //clear applied forces
	//wipe out data and replace with state info
	for (int i = 0; i < nQ; i++)
	{
		s->d->qpos[i] = state->q[i];
		s->d->qvel[i] = state->qd[i];
	}

	for (int i = 0; i < nU; i++)
		s->d->ctrl[i] = state->u[i];


	mju_zero(s->d->qacc, m->nv);
	mju_zero(s->d->qacc_warmstart, m->nv);
	mju_zero(s->d->qfrc_applied, m->nv);

	mj_forward(m, s->d);

	mj_forward(m, s->d);

	// extra solver iterations to improve warmstart (qacc) at center point
	for( int rep=1; rep<3; rep++ )
		mj_forwardSkip(m, s->d, mjSTAGE_VEL, 1);

	for (int i = 0; i < nQ; i++)
	{
		state->qdd[i] = s->d->qacc[i];
	}

	//mass matrix
	//mjtNum M[nQ*nQ];
	//mj_fullM(m, M, s->d->qM);
	//for (int i = 0; i < nQ*nQ; i++)
	//	state->M[i] = M[i];

}

void step(slip_t* s, state_t* state)
{
	mj_step(m, s->d); // pass the mjModel and mjData
//	for (int i = 0; i < nQ; i++)
//		printf("%f\t", s->d->qpos[i]);
//	printf("\n");
}

void run_forward(slip_t* s, state_t* state, double DT)
{
	//wipe out data and replace with state info
	for (int i = 0; i < nQ; i++)
	{
		s->d->qpos[i] = state->q[i];
		s->d->qvel[i] = state->qd[i];
	}

	for (int i = 0; i < nU; i++)
		s->d->ctrl[i] = state->u[i];

	mju_zero(s->d->qacc, m->nv);
	mju_zero(s->d->qacc_warmstart, m->nv);
	mju_zero(s->d->qfrc_applied, m->nv);
	mju_zero(s->d->xfrc_applied, m->nbody*6);

	mj_forward(m, s->d);

	int iters = (int)(DT/m->opt.timestep);

	for (int i = 0; i < iters; i++)
		mj_step(m, s->d);

	for (int i = 0; i < nQ; i++)
	{
		state->q[i] = s->d->qpos[i];
		state->qd[i] = s->d->qvel[i];
		state->qdd[i] = s->d->qacc[i];
	}

}

void step_ctrl(slip_t* s)
{
	mj_step1(m, s->d);
	int count;
	const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &count);
	for (int i = 0; i < nU; i++)
		s->d->ctrl[i] = 150*(double)(axes[i]);

	mj_step2(m, s->d);
}

void get_joint_limits(pos_limits_t *lim)
{
	for (int i = 0; i < nQ; i++)
	{
		if (!m->jnt_limited[i])
		{
			lim->lb[i] = -1e20;
			lim->ub[i] = 1e20;
		}
		else {
			lim->lb[i] = m->jnt_range[i*2];
			lim->ub[i] = m->jnt_range[i*2+1];
		}
	}
}

void get_motor_limits(motor_limits_t *lim)
{
	for (int i = 0; i < nU; i++)
	{
		if (!m->actuator_ctrllimited[i])
		{
			lim->lb[i] = -1e20;
			lim->ub[i] = 1e20;
		}
		else {
			lim->lb[i] = m->actuator_ctrlrange[i*2];
			lim->ub[i] = m->actuator_ctrlrange[i*2+1];
		}
	}
}

/*
int get_stationary_init(slip_t *s, state_t *state)
{
	//given desired root height find u and f to balance forces

	double root_z = state->q[1];

	for (int i = 0; i < nQ; i++)
		state->q[i] = 0.0;

	state->q[1] = root_z;

	double foot_targ_z = 0.0;

	double body_force_N = mj_getTotalmass(m)*9.806;
	double stiffness_Nm = m->jnt_stiffness[mj_name2id(m, mjOBJ_JOINT, "right_spring")];
	double spring_def = body_force_N/(2*stiffness_Nm);

//	printf("%f\t%f\t%f\n", body_force_N, stiffness_Nm, spring_def);

	double tempq;
	int idx = 5;
	tempq = spring_def;
	if (tempq > m->jnt_range[idx*2+1] || tempq < m->jnt_range[idx*2])
	{
		printf("Idx: %d | Range: %f|%f : %f\n", idx, m->jnt_range[idx*2], m->jnt_range[idx*2+1], tempq);
		return 1;
	}
	state->q[idx] = tempq;

	idx = 8;
	tempq = spring_def;
	if (tempq > m->jnt_range[idx*2+1] || tempq < m->jnt_range[idx*2])
	{
		printf("Idx: %d | Range: %f|%f : %f\n", idx, m->jnt_range[idx*2], m->jnt_range[idx*2+1], tempq);
		return 1;
	}
	state->q[idx] = tempq;

	idx = 4;
	tempq = root_z + m->body_pos[m->jnt_bodyid[5]*3 + 2] + state->q[5] + m->body_pos[m->jnt_bodyid[4]*3 + 2] + m->site_pos[2];
	tempq *= -1;
	if (tempq > m->jnt_range[idx*2+1] || tempq < m->jnt_range[idx*2])
	{
		printf("Idx: %d | Range: %f|%f : %f\n", idx, m->jnt_range[idx*2], m->jnt_range[idx*2+1], tempq);
		return 1;
	}
	state->q[idx] = tempq;

	idx = 7;
	tempq = root_z + m->body_pos[m->jnt_bodyid[8]*3 + 2] + state->q[8] - foot_targ_z + m->body_pos[m->jnt_bodyid[7]*3 + 2] + m->site_pos[2];
	tempq *= -1;
	if (tempq > m->jnt_range[idx*2+1] || tempq < m->jnt_range[idx*2])
	{
		printf("Idx: %d | Range: %f|%f : %f\n", idx, m->jnt_range[idx*2], m->jnt_range[idx*2+1], tempq);
		return 1;
	}
	state->q[idx] = tempq;

	for (int i = 0; i < nQ; i++)
	{
		state->qd[i] = 0.0;
		state->qdd[i] = 0.0;
	}

	state->u[0] = 0.0;
	state->u[2] = 0.0;

	state->u[1] = -body_force_N/2.0;
	state->u[3] = -body_force_N/2.0;

	state->f[0] = 0.0;
	state->f[2] = 0.0;
	state->f[1] = -state->u[1];
	state->f[3] = -state->u[3];

	return 0;
}
*/

static void window_close_callback(GLFWwindow* window)
{
    vis_close(glfwGetWindowUserPointer(window));
}

static void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
	Scroll(xoffset, yoffset, glfwGetWindowUserPointer(window));
}
static void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
	MouseMove(xpos, ypos, glfwGetWindowUserPointer(window));
}
static void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
	MouseButton(button, act, mods, glfwGetWindowUserPointer(window));
}

static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
	Keyboard(key, scancode, act, mods, glfwGetWindowUserPointer(window));
}


slip_vis_t* vis_init()
{
    // Check if model has been loaded
    if (!m) {
        mju_error("init must be called before vis_init");
        return NULL;
    }

    // Initialize GLFW if this is the first visualization window
    if (!glfw_initialized) {
        if (!glfwInit()) {
            mju_error("Could not initialize GLFW");
            return NULL;
        }
        glfw_initialized = true;
    }

    // Allocate visualization structure
    slip_vis_t *v = malloc(sizeof (slip_vis_t));

    // Create window
    v->window = glfwCreateWindow(1200, 900, "Slip", NULL, NULL);
    glfwMakeContextCurrent(v->window);
    glfwSwapInterval(1);

    // Set up mujoco visualization objects
    v->cam.lookat[0] = m->stat.center[0];
	v->cam.lookat[1] = m->stat.center[1];
	v->cam.lookat[2] = 0.8 + m->stat.center[2];
	v->cam.type = mjCAMERA_FREE;

	v->cam.distance = 0.5 * m->stat.extent;
	mjv_moveCamera(m, mjMOUSE_ROTATE_H, 0.75, 0.0, &v->scn, &v->cam);

    mjv_defaultOption(&v->opt);
    mjr_defaultContext(&v->con);
    mjv_makeScene(&v->scn, 1000);
    mjr_makeContext(m, &v->con, mjFONTSCALE_100);

    // Set callback for user-initiated window close events
    glfwSetWindowUserPointer(v->window, v);
    glfwSetWindowCloseCallback(v->window, window_close_callback);
	glfwSetCursorPosCallback(v->window, mouse_move);
	glfwSetMouseButtonCallback(v->window, mouse_button);
	glfwSetScrollCallback(v->window, scroll);
	glfwSetKeyCallback(v->window, keyboard);

    return v;
}

static bool bUserInput = false;

bool vis_draw(slip_vis_t *v, slip_t *s, bool bWaitUser)
{
	bool doOnce = true;

	while (bWaitUser || doOnce)
	{
		doOnce = false;
		// Return early if window is closed
		if (!v->window)
			return false;

		// Set up for rendering
		glfwMakeContextCurrent(v->window);
		mjrRect viewport = {0, 0, 0, 0};
		glfwGetFramebufferSize(v->window, &viewport.width, &viewport.height);

		// Render scene
		mjv_updateScene(m, s->d, &v->opt, NULL, &v->cam, mjCAT_ALL, &v->scn);
		mjr_render(viewport, &v->scn, &v->con);

		// Show updated scene
		glfwSwapBuffers(v->window);
		glfwPollEvents();

		if (bUserInput)
		{
			bUserInput = false;
			break;
		}

	}
	return true;
}

static bool button_left = false;
static bool button_middle = false;
static bool button_right =  false;
static double cursor_lastx = 0;
static double cursor_lasty = 0;


// mouse button
void MouseButton(int button, int act, int mods, slip_vis_t* v)
{
    // update button state
    button_left =   (glfwGetMouseButton(v->window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(v->window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(v->window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(v->window, &cursor_lastx, &cursor_lasty);
}



// mouse move
void MouseMove(double xpos, double ypos, slip_vis_t* v)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - cursor_lastx;
    double dy = ypos - cursor_lasty;
    cursor_lastx = xpos;
    cursor_lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(v->window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(v->window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(v->window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    mjv_moveCamera(m, action, dx/height, dy/height, &v->scn, &v->cam);
}

void Scroll(double xoffset, double yoffset, slip_vis_t* v)
{
    // scroll: emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &v->scn, &v->cam);
}

// keyboard
void Keyboard(int key, int scancode, int act, int mods, slip_vis_t* v)
{
    // do not act on release
    if( act==GLFW_RELEASE )
        return;

    bUserInput = true;
}

void vis_close(slip_vis_t *v)
{
    if (!v || !v->window)
        return;

    // Free mujoco objects
    mjv_freeScene(&v->scn);
    mjr_freeContext(&v->con);

    // Close window
    glfwDestroyWindow(v->window);
    v->window = NULL;

}

