# An action goal can be in one of these states after it is accepted by an action server.
# For more information, see http://design.ros2.org/articles/actions.html
int8 STATUS_UNKNOWN   = 0
int8 STATUS_ACCEPTED  = 1
int8 STATUS_EXECUTING = 2
int8 STATUS_CANCELING = 3
int8 STATUS_SUCCEEDED = 4
int8 STATUS_CANCELED  = 5
int8 STATUS_ABORTED   = 6

# Goal info (contains ID and timestamp)
GoalInfo goal_info

# Goal status
int8 status
