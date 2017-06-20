# Mono-Slam Implementation in ROS

This repository implements the mono-slam algorithm first introduced by [Andrew Davison](https://www.doc.ic.ac.uk/~ajd/Publications/davison_etal_pami2007.pdf),
and is the final outcome of the M.Sc. Thesis of [Ludovico O. Russo](https://github.com/ludusrusso)
in the 2013.

[Reference: Russo L.O., Rosa S., Bona B., Matteucci M., "A ROS implementation of the mono-slam algorithm",
International Journal of Computer Science & Information Technology, Vol. 6 Issue 1, p339](https://www.researchgate.net/publication/269200654_A_ROS_Implementation_of_the_Mono-Slam_Algorithm)


## Usage:

```bash
$ rosrun mono-slam mono-slam configuration.cfg /image:=/your_image_topic
```

> configuration file contains parameters for the mono-slam algorithm
  and camera calibration.
  Sample configuration files are provided in the "conf" folder

## Example videos

 - https://www.youtube.com/watch?v=Cf0iKfhnyu4
 - https://www.youtube.com/watch?v=Jjmm9yZY3kA
 - https://www.youtube.com/watch?v=ANNwb4NqlIM
