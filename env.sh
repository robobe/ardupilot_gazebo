source /usr/share/gazebo/setup.bash

export GAZEBO_PLUGIN_PATH=/workspace/bin:$GAZEBO_PLUGIN_PATH
export GAZEBO_RESOURCE_PATH=/workspace/src/ardupilot_gazebo/worlds:$GAZEBO_RESOURCE_PATH
export GAZEBO_MODEL_PATH=/workspace/src/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH

# Function to get git branch
parse_git_branch() {
    git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/[\1]/'
}
export PS1="🟧 \[\033[32m\]\u@\h\[\033[00m\]:\[\033[34m\]\w\[\033[33m\]\$(parse_git_branch)\[\033[00m\]\$ "

bind '"\C-b": "gazebo --verbose iris_arducopter_runway.world"'