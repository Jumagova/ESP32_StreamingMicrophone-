#!/bin/bash

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
echo $PROJECT_ROOT
source ${PROJECT_ROOT}/config.sh

echo "Building Docker image..."
docker build -t ${PROJECT_NAME} ${PROJECT_ROOT}