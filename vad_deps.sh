#!/bin/bash

# FIXME use path from cmake cache
dir=$(pwd)
for e in external/* ; do
  cd $e ; git diff --exit-code > /dev/null
  if [ $? -ne 0 ]; then
    echo "$e has changes"
  fi
  cd $dir
done
