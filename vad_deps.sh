#!/bin/bash

# FIXME use path from cmake cache
dir=$(pwd)
for e in external/* ; do
  cd $e
  rootdir=$(git rev-parse --show-toplevel)
  if [ "$rootdir" != "$dir" ]; then
    git diff --exit-code > /dev/null
    changes=$?
    if [ $changes -ne 0 ]; then
      echo "$e has changes"
    fi
  fi
  cd $dir
done
