# Buliding the program

- `git clone git.uwaterloo.ca:akarmano/cs452-trains.git`
- `cd cs452-trains/kernel`
- `make OPT="-O3" DATA_CACHE="true" INSTRUCTION_CACHE="true" PERF_TEST="false" RPS_TEST="false"` // default (shell only)
- `../upload.sh iotest.img <pi mac address>`

# running RPS tests
make RPS_TEST="true"
# running the performance tests

copy paste for testing different configurations

make OPT="-O3" DATA_CACHE="false" INSTRUCTION_CACHE="false" PERF_TEST="true"

make OPT="-O3" DATA_CACHE="false" INSTRUCTION_CACHE="true" PERF_TEST="true"

make OPT="-O3" DATA_CACHE="true" INSTRUCTION_CACHE="false" PERF_TEST="true"

make OPT="-O3" DATA_CACHE="true" INSTRUCTION_CACHE="true" PERF_TEST="true"

make OPT="-O0" DATA_CACHE="false" INSTRUCTION_CACHE="false" PERF_TEST="true"

make OPT="-O0" DATA_CACHE="false" INSTRUCTION_CACHE="true" PERF_TEST="true"

make OPT="-O0" DATA_CACHE="true" INSTRUCTION_CACHE="false" PERF_TEST="true"

make OPT="-O0" DATA_CACHE="true" INSTRUCTION_CACHE="true" PERF_TEST="true"

