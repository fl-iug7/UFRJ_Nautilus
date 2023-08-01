#!/bin/bash

user_name=$USER
host_name=$(hostname)

echo "Bem-vindo $user_name ao terminal do $host_name"

curl wttr.in/?0

echo "$(date) - $user_name" >> ~/.welcome.data
