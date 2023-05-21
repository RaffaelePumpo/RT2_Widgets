# RT_2 Widgets

## Initialize node, libraries, class visualizer, useful variables


```python
import jupyros as jr
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import actionlib
import actionlib.msg
import matplotlib.pyplot as plt
import tf
from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation

from assignment_2_2022.msg import Info_rob 
import assignment_2_2022.msg
import ipywidgets as widgets
from ipywidgets import Button, HBox, VBox

# Variables to count the number of goals reached and canceled
canceled = 0
reached = 0

# Lists of all targets 
targets_x = []
targets_y = []


status_canc = 2
status_reach = 3

def callback(msg):


    global canceled, reached

    # Get the status 
    status = msg.status.status

    # Goal canceled
    if status == status_canc:
        x.disabled = False
        y.disabled = False
        button_start.disabled = False
        button_stop.disabled = True
        canceled = canceled + 1

    # Goal reached
    elif status == status_reach:
        x.disabled = False
        y.disabled = False
        button_start.disabled = False
        button_stop.disabled = True
        reached = reached + 1


    
class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots() 
        self.ln, = plt.plot([], [], 'ro') 
        self.x_data, self.y_data = [] , []
        
    def plot_init(self): 
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Position ')
        return self.ln

    def odom_callback(self, msg):
        #Get the position and velocity from the message
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        self.y_data.append(pos.y) 
        self.x_data.append(pos.x)
        global pub
         # Create the custom message
        info_rob = Info_rob()
        info_rob.x = pos.x
        info_rob.y = pos.y
        info_rob.vel_x = vel.x
        info_rob.vel_y = vel.y

        # Publish it
        pub.publish(info_rob)
        
    def update_plot(self, frame): 
        self.ln.set_data(self.x_data, self.y_data) 
        return self.ln
    
        
# Initializes a rospy node 
rospy.init_node('client_node_a')

# Publisher to publish the custom message
pub = rospy.Publisher("/info_rob", Info_rob, queue_size=10)
```

## Create buttons for Goals


```python
# Create start and stop buttons
button_start = Button(description="START!", button_style="success")
button_stop = Button(description="STOP!", disabled=True, button_style="danger")

# Assign functions to buttons click

def on_button_start_clicked(b):
    global targets_x, targets_y
    
    goal.target_pose.pose.position.x = x.value
    goal.target_pose.pose.position.y = y.value

    # Update target lists
    targets_x.append(x.value)
    targets_y.append(y.value)

    # Send the goal 
    client.send_goal(goal)

    x.disabled = True
    y.disabled = True
    button_start.disabled = True
    button_stop.disabled = False


def on_button_stop_clicked(b):
    client.cancel_goal()
    x.disabled = False
    y.disabled = False
    button_start.disabled = False
    button_stop.disabled = True
    
# Create the action client
client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2022.msg.PlanningAction)

# Wait for the server to be ready
client.wait_for_server()

# Create the goal object
goal = assignment_2_2022.msg.PlanningGoal()




x = widgets.FloatText(description="x:")
y = widgets.FloatText(description="y:")

# Assign functions to button clicks
button_start.on_click(on_button_start_clicked)
button_stop.on_click(on_button_stop_clicked)


```

## Create button and figure for target cancelled and reached


```python
# Subscribe to the result topic
jr.subscribe('/reaching_goal/result', assignment_2_2022.msg.PlanningActionResult, callback)

get_ipython().run_line_magic('matplotlib', 'widget')

# Create the button to update the graph
button_update = widgets.Button(description = "Update graph!", button_style = "success")

 
def on_button_update_clicked(b):
    plt.figure(2)
    values = [canceled,reached]
    labels = ['Canceled', 'Reached']
    colors = ['red', 'green']

    plt.bar(labels, values, color = colors)
    
    plt.ylabel('Numbers')
    plt.title('Targets')
    
    plt.yticks(range(int(min(values)), int(max(values))+1))
    plt.show()

 
button_update.on_click(on_button_update_clicked)

```

## Create button and figure for all targets set


```python
get_ipython().run_line_magic('matplotlib', 'widget')

# Create the button to update the graph
button_targets = Button(description = "Update graph!", button_style = "success")

def on_button_update_targets(b):
    plt.figure(3)
    
    global targets_x ,targets_y 

    plt.scatter(targets_x, targets_y)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('All Targets set!')
    plt.show()

  
button_targets.on_click(on_button_update_targets)


```

## Create widget for see distance and angle of closest obstacle


```python
# Create the labels
d_label = widgets.Label(value="Distance:", layout=widgets.Layout(width='150px'))
ang_label = widgets.Label(value="Angle:", layout=widgets.Layout(width='150px'))

# Create the widgets
dist = widgets.HTML(layout=widgets.Layout(width='100px'))
ang = widgets.HTML(layout=widgets.Layout(width='100px'))

# Set the thresholds fqor the distance and angle
d_threshold = 1
angle_threshold = 0.5

def laserCallback(scan):
    d_range = 100
    a_range = 100
    # Find the closest obstacle
    for at, x in enumerate(scan.ranges):
        # Check if the distance is smaller than the current minimum
        if x < d_range and x > scan.range_min:
            d_range = x
            a_range = scan.angle_min + scan.angle_increment * at
    # Update the values of the widgets
    
    dist.value = f"<span style='color: {'red' if d_range < d_threshold else 'green'};'>{d_range}</span>"
    ang.value = f"<span style='color: {'red' if abs(a_range) < angle_threshold else 'green'};'>{a_range}</span>"


# Subscribe to the laser scan topic
jr.subscribe('/scan', LaserScan, laserCallback)

# Create the widget for th title
title_label = widgets.Label(value="Closest Obstacle", layout=widgets.Layout(width='200px'))



```

## Display all widgets


```python
print("Insert the x and y position of the target or c to cancel")
# Create the widget layout
HBox([VBox([x, y]), VBox([button_start, button_stop])])
```

    Insert the x and y position of the target or c to cancel



    HBox(children=(VBox(children=(FloatText(value=0.0, description='x:'), FloatText(value=0.0, description='y:')))…



```python
get_ipython().run_line_magic('matplotlib', 'widget')

# Create the visualiser
vis = Visualiser()

# Subscribe to the odometry topic
jr.subscribe('/odom', Odometry, vis.odom_callback)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.figure(1)
plt.show(block=True)
```


    Canvas(toolbar=Toolbar(toolitems=[('Home', 'Reset original view', 'home', 'home'), ('Back', 'Back to previous …



```python
display(button_update)
```


    Button(button_style='success', description='Update graph!', style=ButtonStyle())



    Canvas(toolbar=Toolbar(toolitems=[('Home', 'Reset original view', 'home', 'home'), ('Back', 'Back to previous …



```python
display(button_targets)
```


    Button(button_style='success', description='Update graph!', style=ButtonStyle())



    Canvas(toolbar=Toolbar(toolitems=[('Home', 'Reset original view', 'home', 'home'), ('Back', 'Back to previous …



```python
display(widgets.VBox([title_label, widgets.HBox([d_label, dist]), widgets.HBox([ang_label, ang])]))
```


    VBox(children=(Label(value='Closest Obstacle', layout=Layout(width='200px')), HBox(children=(Label(value='Dist…

