#!/usr/bin/bash

echo "pulling git repo and rebuilding..."

git pull && cd pi_build && make all -j4
