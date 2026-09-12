TSTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
tmux new -s $TSTAMP

TSTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
cd dev/mpose_rt
conda activate mpose_pipe
bash start_inference.bash 2>&1 | tee "logs/run_$TSTAMP.log"
