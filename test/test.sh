cd "$(dirname "$0")/.."

cc -std=c11 -Wall -I main_tx -o /tmp/fdirtest \
   test/fault_inject.c main_tx/voting.c main_tx/fdir.c -lm && /tmp/fdirtest
