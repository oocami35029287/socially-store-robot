#!/bin/bash
sleep 2
#killall gzserver gzclient
# 启动第一个 tmux 会话并在其中运行指令
tmux new-session -d -s session1
tmux send-keys -t session1 "cd ~/socially-store-robot && source run_docker.sh cuda10" C-m
tmux send-keys -t session1 "cmd1" C-m
sleep 1

# 启动第二个 tmux 会话并在其中运行指令
tmux new-session -d -s session2
tmux send-keys -t session2 "cd ~/socially-store-robot && source run_docker.sh same" C-m
tmux send-keys -t session2 "cmd2" C-m
sleep 3
tmux new-session -d -s session3
tmux send-keys -t session3 "cd ~/socially-store-robot && source run_docker.sh same" C-m
tmux send-keys -t session3 "cmd3" C-m
sleep 3
tmux new-session -d -s session4
tmux send-keys -t session4 "cd ~/socially-store-robot && source run_docker.sh same" C-m
tmux send-keys -t session4 "cmd4" C-m
sleep 1
tmux new-session -d -s session5
tmux send-keys -t session5 "cd ~/socially-store-robot && source run_docker.sh same" C-m
tmux send-keys -t session5 "cmd5" C-m
sleep 1
tmux new-session -d -s session6
tmux send-keys -t session6 "cd ~/socially-store-robot && source run_docker.sh same" C-m
tmux send-keys -t session6 "cmd6" C-m
sleep 1
tmux new-session -d -s session7
tmux send-keys -t session7 "cd ~/socially-store-robot && source run_docker.sh same" C-m
tmux send-keys -t session7 "cmd7" C-m
sleep 1
tmux new-session -d -s session8
tmux send-keys -t session8 "cd ~/socially-store-robot && source run_docker.sh same" C-m
tmux send-keys -t session8 "wmctrl -r \"Gazebo\" -b remove,maximized_vert,maximized_horz" C-m
tmux send-keys -t session8 "wmctrl -r \"Gazebo\" -e 0,960,0,960,1080" C-m
tmux send-keys -t session8 "wmctrl -r \"Rviz\" -b remove,maximized_vert,maximized_horz" C-m
tmux send-keys -t session8 "wmctrl -r \"Rviz\" -e 0,0,540,960,540" C-m
sleep 3
tmux send-keys -t session8 "wmctrl -r \"Yolo demo\" -b remove,maximized_vert,maximized_horz" C-m
tmux send-keys -t session8 "wmctrl -r \"Yolo demo\" -e 0,0,0,0,960,540" C-m






# # 连接到第一个 tmux 会话以查看输出
# tmux attach-session -t session1
# tmux send-keys -t session2 "cd ~/socially-store-robot && source run_docker.sh same" C-m
# tmux send-keys -t session1 "cmd3" C-m


#tmux kill-session -a

