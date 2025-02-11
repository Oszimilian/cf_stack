# Crazy Fly Stack
![alt text](image.png)
## Starting
```
ros2 launch cf_launch cf_launch.launch.py input_grid_path:=/home/maximilian/Git/cf_stack/grids/arena.json

ros2 launch tha_example tha_framework.launch.py type:=1 backend:=hardware id:=0

ros2 topic pub /safeflie0/land std_msgs/Empty
```

## Models
### Domain Model
![alt text](domain.drawio.png)

### Path Patterns
![alt text](path_patterns.drawio.png)

### State Chart
![alt text](Statechart.drawio.png)

### Node Graph
![alt text](image-1.png)