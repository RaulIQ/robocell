# Position accuracy

How far the reckoned position drifts from the MuJoCo ground truth
as the robot gains sensors, one step per sensor.

| Step | Sensors           | Drift, mm | Commit  | Route                                  |
| ---- | ----------------- | --------- | ------- | -------------------------------------- |
| 1    | encoders          | 205       | d1394b9 | [01-encoders.png](01-encoders.png)     |
| 2    | encoders and gyro | 4         | 867ac25 | [02-gyroscope.png](02-gyroscope.png)   |

## 1. Encoders

Dead reckoning from both wheel encoders alone, after a manual drive
of a few loops about one and a half meters across.

![Encoders only](01-encoders.png)

## 2. Gyroscope

The encoders still give the distance, but the turn now comes from the gyro,
whose yaw rate adds up on every physics step. The route window still labels
the dashed line as encoders. After a manual drive of loops and figure
eights about one and a half meters across, the reckoned route ends 4 mm
from the ground truth.

![Encoders and gyro](02-gyroscope.png)
