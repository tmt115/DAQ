import threading
from daqsampling import sample_safety_check
from daqservos import 
# HIST = 
sample_lock = threading.Lock()
abort_event = threading.Event()

Class SensorSnapshot(enum):
    def __init__(self):
        self.tc1 = 0.0
        self.tc2 = 0.0
        self.pt1 = 0.0
        self.pt2 = 0.0
        self.pt3 = 0.0
        self.pt4 = 0.0
        self.load = 0.0
        self.timestamp = 0.0
        self.pastt1 = np.zeros(HIST)
        self.pastt2 = np.zeros(HIST)
        self.pastp1 = np.zeros(HIST)
        self.pastp2 = np.zeros(HIST)
        self.pastp3 = np.zeros(HIST)
        self.pastp4 = np.zeros(HIST)
        self.pastl = np.zeros(HIST)

curr_sample = SensorSnapshot()

def sample_thread():
    while not abort:
        with sample_lock:
            tc1, tc2, pt1, pt2, pt3, pt4, load, pastt1, pastt2, pastp1, pastp2, pastp3, pastp4, pastl = sample_safety_check()
            curr_sample.tc1, curr_sample.tc2, curr_sample.pt1, curr_sample.pt2, 
                curr_sample.pt3, curr_sample.pt4, curr_sample.load, curr_sample.pastt1, 
                curr_sample.pastt2, curr_sample.pastp1, curr_sample.pastp2, curr_sample.pastp3, 
                curr_sample.pastp4, curr_sample.pastl = tc1, tc2, pt1, pt2, pt3, pt4, load, 
                pastt1, pastt2, pastp1, pastp2, pastp3, pastp4, pastl
            time.sleep(0.1)
            
def servo_thread():
    while not abort
        with sample_lock:
            copy_sample = curr_sample
        




