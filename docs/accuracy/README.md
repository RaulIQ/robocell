# Position accuracy

How far the reckoned position drifts from the MuJoCo ground truth
as the robot gains sensors, one step per sensor.

| Step | Sensors  | Drift, mm | Commit  | Route                                 |
| ---- | -------- | --------- | ------- | ------------------------------------- |
| 1    | encoders | 205       | d1394b9 | [01-encoders.png](01-encoders.png)    |

## 1. Encoders

Dead reckoning from both wheel encoders alone, after a manual drive
of a few loops about one and a half meters across.

![Encoders only](01-encoders.png)
