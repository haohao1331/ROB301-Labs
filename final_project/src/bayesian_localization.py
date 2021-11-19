#!/usr/bin/env python
import numpy as np
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
def normalize(a):
    return a / np.sum(a)

(b, g, y, r) = (0, 1, 2, 3)
colors = {0: "blue", 1: 'green', 2: 'yellow', 3: 'orange'}

state_color_map = np.array([y, g, b, r, r, g, b, r, y, g, b])
state_to_meas = np.array([
    [0.6, 0.2, 0.05, 0.05, 0.1], 
    [0.2, 0.6, 0.05, 0.05, 0.1],
    [0.05, 0.05, 0.65, 0.15, 0.1],
    [0.05, 0.05, 0.2, 0.6, 0.1]
])
# p(meas | state) = state_to_meas[state][meas]
state_model = np.array([
    [0.85, 0.1, 0.05],  # u = -1
    [0.05, 0.9, 0.05],  # u = 0
    [0.05, 0.1, 0.85]   # u = 1
])
# p(next state | u) = state_model[u][next state]
states = len(state_color_map)

actions = np.array([1,1,1,1,1,1,1,1,0,1,1,1,np.nan])
meas = np.array([np.nan,r,y,g,b,np.nan,g,b,g,r,y,g,b])
print(meas[1])

# a = 1/0

# initialize the state with uniform probability
state = normalize(np.ones(states))
all_states = []

for i in range(len(actions) - 1):
    print("step: ", i)
    # apriori update
    u = actions[i] + 1
    new_state = np.zeros(states)
    print(state_model[u])
    for j in range(states):
        for k in [0, 1, 2]:
            new_state[j] += state_model[u][2-k] * state[(j+k-1)%states]
    state = new_state
    print("apriori update: ", new_state)
    
    # posterior update
    m = meas[i+1]
    if not np.isnan(m):
        for j in range(states):
            color = state_color_map[j]
            new_state[j] = new_state[j] * state_to_meas[color][m]
    state = normalize(new_state)
    print("posterior update: ", state)
    all_states.append(state)
    
for i in range(len(all_states)):
    c = np.nan
    if not np.isnan(meas[i+1]) and i != len(all_states):
        c = colors[meas[i+1]]
    print("step: ", i, "color: ", c)
    print(all_states[i])
    
print("done")
    


