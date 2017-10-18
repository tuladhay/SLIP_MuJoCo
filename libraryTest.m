clear
close all

s = SLIP(0);

state.q = [0,0,0,0,0,0];
state.qd = state.q;
state.qdd = state.q;
state.u = [0,0];
state = libpointer('state_t', state); % creates pointer initialized to a copy of Value.
state = s.dynamics(state);

real_state = state.Value;

fprintf('qdd: ')
real_state.qdd

for i = 1:1:1000
    s.draw();
end

s.close()

