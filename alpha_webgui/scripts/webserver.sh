#!/bin/bash

ARGS=()

while [[ $# -gt 0 ]]
do

case "$1" in
    _*)
    shift
    ;;
    *)
    ARGS+=("$1")
    shift
    ;;
esac
done


python3 -m http.server "${ARGS[@]}"
