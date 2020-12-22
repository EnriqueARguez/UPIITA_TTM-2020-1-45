#!/usr/bin/env bash

#Instalamos pip
cd
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python get-pip.py

pip install -U scikit-learn