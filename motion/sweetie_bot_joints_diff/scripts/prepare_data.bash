#! /bin/bash

cat input.out | sed 's/(time)/ /g' | sed 's/servo1/ /g' | sed 's/servo2/ /g' | tr -s " " > B.out
cat output.out | sed 's/(time)/ /g' | sed 's/servo1/ /g' | sed 's/servo2/ /g' | tr -s " " > A.out


