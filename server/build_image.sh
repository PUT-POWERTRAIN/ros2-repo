#!/bin/bash

# Build the Docker image with the name powertrain-gz
docker build -t powertrain-gz --ulimit nofile=1024 .
