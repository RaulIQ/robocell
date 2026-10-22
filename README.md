# robocell

## Run the simulation

Start the TurtleBot3 Waffle Pi in the MuJoCo viewer from the package folder:

```console
cd /ws/src/robocell_sim
python -m robocell_sim.main
```

Steer the robot with the keyboard while the viewer window has focus.
Every press changes the speed by one step, up to the robot limits.

| Key            | Action                              |
| -------------- | ----------------------------------- |
| Up / Down      | forward speed one step up or down   |
| Left / Right   | turn one step to the left or right  |
| Enter          | stop                                |
| Space          | pause or resume the simulation      |
