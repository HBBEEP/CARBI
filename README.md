# CARBI
## _carbi_

## prerequisite

RUN in terminal
```
xhost local:root
```

## Docker

```
cd ~/CARBI-1/docker
```

```
docker compose up -d
```
```
docker exec -it docker-ros2-1 bash
```
then RUN this command in root
```
cd ros2/carbi
```

## Sim Go to goal

```
ros2 launch carbi_visualization display.launch.py 
```

```
ros2 launch carbi_bridge carbi.launch.py 
```

```
ros2 run carbi_navigation go_to_goal.py 
```

## Robot Control

```
ros2 run carbi_control carbi_controller.py
```
