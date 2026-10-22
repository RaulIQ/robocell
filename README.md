# robocell

## Run the simulation

Start the TurtleBot3 Waffle Pi in the MuJoCo viewer from any folder:

```console
python /ws/src/robocell_sim/robocell_sim/main.py
```

Steer the robot with the keyboard while the viewer window has focus.
Every press changes the speed by one step, up to the robot limits.

| Key            | Action                              |
| -------------- | ----------------------------------- |
| Up / Down      | forward speed one step up or down   |
| Left / Right   | turn one step to the left or right  |
| Enter          | stop                                |
| Space          | pause or resume the simulation      |

Every half second of simulated time the console logs both wheel encoders
in ticks, at 4096 ticks per wheel revolution, as the Dynamixel XM430 counts them.

A second window draws the route of the robot as it drives: the dashed line is
where the wheel encoders put it, the solid line is where MuJoCo actually has it,
and the title tells how far apart they are in millimeters.
