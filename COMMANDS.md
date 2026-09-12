# Real time megapose
To run this code, type this command to store it's logs too:

```bash
TSTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
bash start_inference.bash 2>&1 | tee "logs/run_$TSTAMP.log"
```