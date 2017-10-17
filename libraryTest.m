clear
close all

s = SLIP();

state.q = [0,0,0,0,0,0];
state.qd = state.q;
state.qdd = state.q;
state.f = [0,0,0,0];
state.u = state.f;
state = libpointer('state_t', state);
state = s.dynamics(state);

real_state = state.Value;

fprintf('qdd: ')
real_state.qdd

for i = 1:1:1000
    s.draw();
end

s.close()

