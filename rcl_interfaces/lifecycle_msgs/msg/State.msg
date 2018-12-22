# Primary state definitions
# as depicted in
# http://design.ros2.org/articles/node_lifecycle.html

# These are the primary states
# State changes can only be requested
# when the node is in one of these states.
uint8 PRIMARY_STATE_UNKNOWN = 0
uint8 PRIMARY_STATE_UNCONFIGURED = 1
uint8 PRIMARY_STATE_INACTIVE = 2
uint8 PRIMARY_STATE_ACTIVE = 3
uint8 PRIMARY_STATE_FINALIZED = 4

# Temporary intermediate states
# When a transition is requested, the
# node changes its state into one
# of these states.
uint8 TRANSITION_STATE_CONFIGURING = 10
uint8 TRANSITION_STATE_CLEANINGUP = 11
uint8 TRANSITION_STATE_SHUTTINGDOWN = 12
uint8 TRANSITION_STATE_ACTIVATING = 13
uint8 TRANSITION_STATE_DEACTIVATING = 14
uint8 TRANSITION_STATE_ERRORPROCESSING = 15

uint8 id
string label
