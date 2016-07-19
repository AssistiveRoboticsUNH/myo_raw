import re
import os
import glob
import rosbag

class makeData(object):
    def __init__(self, emg_file='emg.dat', imu_file='imu.dat', ort_file='ort.dat', identifier='l', out_path='./'):
        self.emg_file = emg_file
        self.imu_file = imu_file
        self.ort_file = ort_file
        self.identifier = identifier
        self.out_path = out_path

    def makeEMG(self):
        fout = open(os.path.join(self.out_path, 'emg_'+self.identifier+'.mat'),'w')
        with open(self.emg_file) as f:
            for line in f:
                m = re.search(r'\[(\d+[\d,\s]+)\]', line)
                if m:
                    data = m.group(1)
                    fout.write(data+'\n')
        fout.close()

    def makeACC(self):
        fout = open(os.path.join(self.out_path, 'acceleration_'+self.identifier+'.mat'),'w')
        with open(self.imu_file) as f:
            writing = False
            for line in f:
                if 'linear_acceleration' in line:
                    writing = True
                    continue
                if writing:
                    x = re.search(r'x:\s([-\.\d]+)', line)
                    y = re.search(r'y:\s([-\.\d]+)', line)
                    z = re.search(r'z:\s([-\.\d]+)', line)
                    if x: fout.write(x.group(1)+',')
                    if y: fout.write(y.group(1)+',')
                    if z: 
                        fout.write(z.group(1))
                        writing = False
                
                if 'header' in line:
                    fout.write('\n')
                    writing = False
        fout.close()
        
    def makeGyroscope(self):
        fout = open(os.path.join(self.out_path, 'gyro_'+self.identifier+'.mat'),'w')
        with open(self.imu_file) as f:
            writing = False
            for line in f:
                if 'angular_velocity' in line:
                    writing = True
                    continue
                if writing:
                    x = re.search(r'x:\s([-\.\d]+)', line)
                    y = re.search(r'y:\s([-\.\d]+)', line)
                    z = re.search(r'z:\s([-\.\d]+)', line)
                    if x: fout.write(x.group(1)+',')
                    if y: fout.write(y.group(1)+',')
                    if z: 
                        fout.write(z.group(1))
                        writing = False
                
                if 'header' in line:
                    fout.write('\n')
                    writing = False
        fout.close()
        
    def makeOrientation(self):
        fout = open(os.path.join(self.out_path, 'orientation_'+self.identifier+'.mat'),'w')
        with open(self.imu_file) as f:
            writing = False
            for line in f:
                if 'orientation' in line:
                    writing = True
                    continue
                if writing:
                    x = re.search(r'x:\s([-\.\d]+)', line)
                    y = re.search(r'y:\s([-\.\d]+)', line)
                    z = re.search(r'z:\s([-\.\d]+)', line)
                    w = re.search(r'w:\s([-\.\d]+)', line)
                    if x: fout.write(x.group(1)+',')
                    if y: fout.write(y.group(1)+',')
                    if z: fout.write(z.group(1)+',')
                    if w: 
                        fout.write(w.group(1))
                        writing = False
                
                if 'angular_velocity' in line:
                    fout.write('\n')
                    writing = False
        fout.close()

#    def makeOrientation(self):
#        fout = open(os.path.join(self.out_path, 'orientation.mat'),'w')
#        with open(self.ort_file) as f:
#            for line in f:
#                x = re.search(r'x:\s([-\.\d]+)', line)
#                y = re.search(r'y:\s([-\.\d]+)', line)
#                z = re.search(r'z:\s([-\.\d]+)', line)
#                w = re.search(r'w:\s([-\.\d]+)', line)
#                if x: fout.write(x.group(1)+',')
#                if y: fout.write(y.group(1)+',')
#                if z: fout.write(z.group(1)+',')
#                if w: fout.write(w.group(1)+'\n')
#        fout.close()

def readBag(bagFile, dest_path):
    imu_u = open(os.path.join(dest_path, 'imu_u.dat'),'w')
    emg_u = open(os.path.join(dest_path, 'emg_u.dat'),'w')
    imu_l = open(os.path.join(dest_path, 'imu_l.dat'),'w')
    emg_l = open(os.path.join(dest_path, 'emg_l.dat'),'w')
    states = open(os.path.join(dest_path, 'raw_states.dat'),'w')
    state = 0
    for topic, msg, t in rosbag.Bag(bagFile).read_messages():
        if topic == '/myo/u/imu':
            imu_u.write(str(msg))
            states.write(str(state)+'\n')
        if topic == '/myo/u/emg':
            emg_u.write(str(msg))
        if topic == '/myo/l/imu':
            imu_l.write(str(msg))
        if topic == '/myo/l/emg':
            emg_l.write(str(msg))
            
        if topic == '/exercise/detected_state':
            state += 1
    imu_u.close()
    emg_u.close()
    imu_l.close()
    emg_l.close()
#    ort.close()


def process_states(dest_path):
    width = 20
    ps = 0
    with open(os.path.join(dest_path, 'raw_states.dat')) as f:
        with open(os.path.join(dest_path, 'states.dat'), 'w') as fout:
            i = 0
            for line in f:
                s = int(line.strip())
                if width > 0:
                    fout.write(line)
                    width -= 1
                else:
                    fout.write('-1\n')

                if s-ps==1:
                    fout.write(line)
                    width = 20
                ps = s



        
if __name__ == "__main__":
    import sys
    if len(sys.argv) >=2:
        src_path = sys.argv[1]
    else:
        src_path = 'work'
    bagFiles = glob.glob(os.path.join(src_path, '*.bag'))
    
    for i,item in enumerate(bagFiles):
        dest_path = os.path.join(src_path, str(i))
        if not os.path.exists(dest_path):
            os.makedirs(dest_path)
        readBag(item, dest_path)
        maker = makeData(os.path.join(dest_path,'emg_u.dat'), os.path.join(dest_path,'imu_u.dat'), os.path.join(dest_path,'ort_u.dat'), 'u', dest_path)
        maker.makeEMG()
        maker.makeACC()
        maker.makeGyroscope()
        maker.makeOrientation()
        
        maker = makeData(os.path.join(dest_path,'emg_l.dat'), os.path.join(dest_path,'imu_l.dat'), os.path.join(dest_path,'ort_l.dat'), 'l', dest_path)
        maker.makeEMG()
        maker.makeACC()
        maker.makeGyroscope()
        maker.makeOrientation()

        process_states(dest_path)
