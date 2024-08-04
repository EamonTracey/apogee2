#!/bin/bash

export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH=src

python src/scripts/apogee.py $@
