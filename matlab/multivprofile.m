clear all
mjt = multiJointTrajectory(2)
mjt.setAMax([3 3])
mjt.setVMax([4 4])
[s, pd]=mjt.calculate([0 0], [4 2], [0 .5], [4 -1], [5.4 1.1])
[s, pd]=mjt.calculate([0 0], [4 2], [0 .5], [4 -1], .5)
[s, pd]=mjt.calculate([4 6], [4 2], [0 .5], [4 -1], .5) % no success