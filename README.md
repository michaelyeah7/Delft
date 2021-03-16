
# Hermes: Delft Branch

This code is the original Delft trajectory controller that runs on Flame.
It was pulled from the Delft repository by Mark Locascio on 5-25-10
Several very small edits have been made to this code by Jane Miller
This code is intended to only be run on the physical robot, and no in simulation.

Guo Ye made some modifications to make it work on a virtualbox running on laptop. By such way we can get rid of previous heavy mother PC and update code using prefered editor. (10/2020)

## Installation

### Virtual Box Setting
Download debian3.1 iso and launch an instance in virtual box. Connect robot and laptop to the same router. Set the networks as following:

Adapter1: NAT port forwarding 2222 to 22

Adapter2: bridged adapter en0: WI-FI

On host machine(your laptop):
```
ssh-keygen -t rsa
cat .ssh/id_rsa.pub | ssh root@robot 'cat >> .ssh/authorized_keys'
```

### Steps to burn into robot
```
#copy code to VM
scp -P 2222 -r Delft/ root@127.0.0.1:/root/Delft_ws

#make and make install, this will copy built programs into robot
make clean && make && make install

#On a new terminal, run program
ssh root@127.0.0.1
ssh root@robot
run_Flame
```

