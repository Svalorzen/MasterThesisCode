#!/bin/bash
set -e
set -x

AIPATH=~/Libraries/Self/AIToolbox

if [ $# -ge 1 ]
then
    if [ $# -ge 2 ]
    then
        ( cd "$AIPATH"/build && make -j )
        rm -rf include/AIToolbox
        cp -ru "$AIPATH"/include .
        rm ./lib/*
        cp -u "$AIPATH"/build/lib* ./lib/
    fi

    cd build
    rm -r *
    cmake -DCMAKE_BUILD_TYPE=Release ..
    cd ..
fi

( cd build && make -j )
