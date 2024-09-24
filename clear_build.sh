#!/bin/bash

# Directories to be deleted
dirs=("install" "build" "log")

for dir in "${dirs[@]}"; do
  if [ -d "$dir" ]; then
    echo "Deleting directory: $dir"
    rm -rf "$dir"
    if [ $? -eq 0 ]; then
      echo "$dir deleted successfully."
    else
      echo "Failed to delete $dir."
    fi
  else
    echo "$dir does not exist."
  fi
done
