# Buliding the program

- `git clone git.uwaterloo.ca:akarmano/cs452-trains.git`
- `cd cs452-trains/kernel`
- `make OPT="-O3" DATA_CACHE="true" INSTRUCTION_CACHE="true" PERF_TEST="false" RPS_TEST="false" CLOCK_TEST="true"` // default (shell only)
- `../upload.sh aaa.img <pi mac address>`

# running CLOCK tests

make CLOCK_TEST="true"
