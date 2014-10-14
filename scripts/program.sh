MSPDEBUG=~/data/workspaces/mspdebug/code/mspdebug
TILIB_PATH=~/data/workspaces/msp/MSPDebugStack_OS_Package


export LD_LIBRARY_PATH=$TILIB_PATH
cmd="$MSPDEBUG tilib"

echo $cmd
$cmd


