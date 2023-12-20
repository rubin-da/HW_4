import rosbag
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

bag_file_path = '/home/davide/Documents/BAG/totTraj.bag'

with rosbag.Bag(bag_file_path, 'r') as bag:
    positions_x = []
    positions_y = []
    positions_z = []

    for topic, msg, t in bag.read_messages(topics=['/fra2mo/pose']):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        positions_x.append(x)
        positions_y.append(y)
        positions_z.append(z)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(positions_x, positions_y, positions_z, label='Field Position')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Field Position in 3D Cartesian Coordinates')
plt.legend()
plt.show()
