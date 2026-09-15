#!/bin/bash
sudo docker stop foxy_controller || true
sudo docker rm foxy_controller || true
cd ~/g1_deploy/g1_gym_deploy/docker/
sudo make autostart