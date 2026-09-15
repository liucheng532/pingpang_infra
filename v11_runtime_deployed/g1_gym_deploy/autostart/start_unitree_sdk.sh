#!/bin/bash
sudo docker stop foxy_controller || true
sudo docker rm foxy_controller || true
sudo kill $(ps aux |grep lcm_position | awk '{print $2}')
cd ~/g1_deploy/g1_gym_deploy/unitree_legged_sdk_bin/
yes "" | sudo ./lcm_position &