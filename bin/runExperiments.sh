#!/bin/bash
trap "exit" INT

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 beginInt endInt"
    exit
fi

for i in $(seq $1 $2)
do
    echo $i | ./cameraBasicExperiments.sh
done
