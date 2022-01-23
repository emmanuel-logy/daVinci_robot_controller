from __future__ import print_function
import time
import rospy
from ambf_client import Client
import os
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#### Mapping for psm_full
class PSMJointMappingFull:
    def __init__(self):
        self.idx_to_name = {0: 'baselink-yawlink',
                            1: 'yawlink-pitchbacklink',
                            2: 'pitchendlink-maininsertionlink',
                            3: 'maininsertionlink-toolrolllink',
                            4: 'toolrolllink-toolpitchlink',
                            5: 'toolpitchlink-toolgripper1link',
                            6: 'toolpitchlink-toolgripper2link',
                            }

        self.name_to_idx = {'baselink-yawlink': 0,
                            'yawlink-pitchbacklink': 1,
                            'pitchendlink-maininsertionlink': 2,
                            'maininsertionlink-toolrolllink': 3,
                            'toolrolllink-toolpitchlink': 4,
                            'toolpitchlink-toolgripper1link': 5,
                            'toolpitchlink-toolgripper2link': 6,
                            }

pjm = PSMJointMappingFull()


class connectionTestFull():

    def __init__(self, client, name='psm1', csv_file_name='JointPos.csv'):
        self.client = client
        self.name = name
        self.parent_folder = os.getcwd()
        self.csv_name = csv_file_name
        self.base = self.client.get_obj_handle(name + '/baselink')
        self.tip = self.client.get_obj_handle(name + '/toolpitchlink')
        time.sleep(2)
        num_joint = self.base.get_num_joints()
        joint_name = self.base.get_joint_names()
        self.jp_values = []
        self.counter = 0
        print(num_joint)
        print(joint_name)
        self.x = []
        self.y = []
        self.z = []

    def load_data(self):
        csv_file = self.parent_folder + '/' + self.csv_name
        csv_output = []
        #
        with open(csv_file) as f:
            content = f.readlines()
            csv_modify = [[]] * 6
            csv_output = []
            for num_joint in range(6):
                csv_modify[num_joint] = content[num_joint].split('\n')[0].split(',')
                # csv_modify[num_joint] = csv_modify[num_joint]

        for num_pts in range(len(csv_modify[0])):
            jp1 = float(csv_modify[0][num_pts])
            jp2 = float(csv_modify[1][num_pts])
            jp3 = float(csv_modify[2][num_pts])
            jp4 = float(csv_modify[3][num_pts])
            jp5 = float(csv_modify[4][num_pts])
            jp6 = float(csv_modify[5][num_pts])
            jp_v = [jp1, jp2, jp3, jp4, jp5, jp6]
            csv_output.append(jp_v)
        self.jp_values = csv_output
        return csv_output

    def load_path(self, file_name):
        csv_file = self.parent_folder + '/' + file_name
        x_d = []
        y_d = []
        z_d = []
        #
        with open(csv_file) as f:
            content = f.readlines()
            for num_pts in range(len(content)):
                content_row = content[num_pts].split('\n')[0].split(',')
                x = float(content_row[0])
                y = float(content_row[1])
                z = float(content_row[2])
                x_d.append(x)
                y_d.append(y)
                z_d.append(z)
        return x_d, y_d, z_d

    def servo_jp(self, jp):
        self.base.set_joint_pos(0, -jp[0])
        self.base.set_joint_pos(1, -jp[1])
        self.base.set_joint_pos(2, jp[2])
        self.base.set_joint_pos(3, jp[3])
        self.base.set_joint_pos(4, jp[4])
        self.base.set_joint_pos(5, 0)
        self.base.set_joint_pos(6, 0)
        rospy.sleep(0.02)

    def measured_jp(self):
        j0 = self.base.get_joint_pos(0)
        j1 = self.base.get_joint_pos(1)
        j2 = self.base.get_joint_pos(2)
        j3 = self.base.get_joint_pos(3)
        j4 = self.base.get_joint_pos(4)
        j5 = self.base.get_joint_pos(5)
        j6 = self.base.get_joint_pos(6)
        return [j0, j1, j2, j3, j4, j5, j6]

    def update_pose(self):
        num_data = len(self.jp_values)
        if self.counter == num_data:
            print('Finish  running! \n')
            self.counter = self.counter + 1
        elif self.counter < num_data:
            self.servo_jp(self.jp_values[self.counter])
            self.counter = self.counter + 1
            coord = self.tip.get_pos()
            self.x.append(coord.x)
            self.y.append(coord.y)
            self.z.append(coord.z - 0.0102) # 0.0102m is the distance between the toolpitch link and the control point
        else:
            pass


    def run(self):
        self.load_data()
        self.update_pose()
        return self.x, self.y, self.z



if __name__ == '__main__':
    ### draw a circle
    joint_file = 'JointPos_full_circle.csv'
    path_file = 'Path_full_circle.csv'

    ### draw a straight line
    # joint_file = 'JointPos_full_line.csv'
    # path_file = 'Path_full_line.csv'

    c = Client()
    c.connect()
    Test = connectionTestFull(c, 'psm', joint_file)
    rate = rospy.Rate(200)

    # while not rospy.is_shutdown():
    #     try:
    #         Test.servo_jp([0,0,0,0,0,0])
    #     except KeyboardInterrupt:
    #         print('stop!')
    #     rate.sleep()
    #
    # n_iter = 0
    # while n_iter < 500:
    #     x, y, z = Test.run()
    #     n_iter = n_iter + 1
    #     rate.sleep()

    x_d, y_d, z_d = Test.load_path(path_file)
    fig = plt.figure(1)
    plt.rcParams.update({'font.size': 14})
    ax = plt.axes(projection='3d')
    N_remove = 40
    ax.scatter3D(x[N_remove:], y[N_remove:], z[N_remove:], 'r', label='actual')
    ax.scatter3D(x_d[N_remove:], y_d[N_remove:], z_d[N_remove:], 'b', label='desired')
    # ax.set_aspect('equal', 'box') # can uncomment when plotting circle
    ax.legend(fontsize=16)
    ax.set_title('path', fontsize=24)
    ax.set_xlabel('X (m)', fontsize=20)
    ax.set_ylabel('Y (m)', fontsize=20)
    ax.set_zlabel('Z (m)', fontsize=20)

    fig2 = plt.figure(2)
    plt.rcParams.update({'font.size': 12})
    ax1 = fig2.add_subplot(311)
    ax1.plot(x[N_remove:], 'r', label='actual')
    ax1.plot(x_d[N_remove:], 'b', label='desired')
    ax1.legend(fontsize=12)
    ax1.set_title('X', fontsize=20)
    ax1.set_ylabel('value (m)', fontsize=14)
    ax1.grid()
    ax2 = fig2.add_subplot(312)
    ax2.plot(y[N_remove:], 'r', label='actual')
    ax2.plot(y_d[N_remove:], 'b', label='desired')
    ax2.legend(fontsize=12)
    ax2.set_title('Y', fontsize=20)
    ax2.set_ylabel('value (m)', fontsize=14)
    ax2.grid()
    ax3 = fig2.add_subplot(313)
    ax3.plot(z[N_remove:], 'r', label='actual')
    ax3.plot(z_d[N_remove:], 'b', label='desired')
    ax3.legend(fontsize=12)
    ax3.set_title('Z', fontsize=20)
    ax3.set_xlabel('time (ms)', fontsize=14)
    ax3.set_ylabel('value (m)', fontsize=14)
    ax3.grid()

    plt.show()
